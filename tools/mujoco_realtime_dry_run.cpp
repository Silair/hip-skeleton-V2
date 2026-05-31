// MuJoCo realtime dry-run runner for hs_exoskeleton_v2.
//
// This tool steps a single-leg MuJoCo model, samples the hip joint at the V2
// controller rate, runs the V2 control modules in-process, and writes the same
// runtime CSV as the hardware controller. V2 torque is recorded but never
// applied back into MuJoCo.

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "config/ControlConfig.h"
#include "config/LoggingConfig.h"
#include "control/AssistStateMachine.h"
#include "control/FreezeManager.h"
#include "control/GaitFeatureExtractor.h"
#include "control/GaitFeatures.h"
#include "control/IntentDetector.h"
#include "control/PhaseEstimator.h"
#include "control/StopDetector.h"
#include "control/StopTorqueLimiter.h"
#include "control/TorqueProfile.h"
#include "hardware/JointTypes.h"
#include "logging/ExoLogger.h"

namespace exo {
namespace {

constexpr double kPi = 3.14159265358979323846;

enum class VirtualRightMode {
    Zero,
    Copy,
    Inverted,
};

enum class DriverScenario {
    Sine,
    StopGo,
    AbruptStop,
};

struct Options {
    std::string mjcf_path;
    std::string output_dir = "/tmp/hsx_mujoco_realtime_out/logs";
    std::string prefix = "mujoco_realtime_dry_run";
    std::string joint_name = "hip";
    std::string driver_actuator_name = "hip_human_motor";
    std::string exo_actuator_name = "hip_exo_motor";
    double duration_s = 12.0;
    double sample_rate_hz = 50.0;
    double amplitude_deg = 40.0;
    double frequency_hz = 0.8;
    double driver_kp = 80.0;
    double driver_kd = 8.0;
    double driver_torque_limit = 80.0;
    double apply_v2_torque_scale = 0.0;
    double stop_start_s = 4.0;
    double stop_duration_s = 3.0;
    VirtualRightMode virtual_right = VirtualRightMode::Copy;
    DriverScenario driver_scenario = DriverScenario::Sine;
};

struct SimIds {
    int hip_qpos_addr = -1;
    int hip_qvel_addr = -1;
    int driver_actuator_id = -1;
    int exo_actuator_id = -1;
};

struct ControlModules {
    explicit ControlModules(const ControlConfig& config)
        : feature_extractor(config.phase),
          phase_estimator(config.phase),
          intent_detector(config.intent),
          stop_detector(config.stop),
          freeze_manager(config.freeze),
          assist_state_machine(config.assist),
          torque_profile(config.torque),
          stop_torque_limiter(config.stop) {}

    GaitFeatureExtractor feature_extractor;
    PhaseEstimator phase_estimator;
    IntentDetector intent_detector;
    StopDetector stop_detector;
    FreezeManager freeze_manager;
    AssistStateMachine assist_state_machine;
    TorqueProfile torque_profile;
    StopTorqueLimiter stop_torque_limiter;
    FreezeDecision freeze{};
    TorqueCommand previous_torque{};
    AssistState last_assist_state = AssistState::Transparent;
    double last_stop_probability = 0.0;
    double last_motion_confidence = 0.0;
};

class MjModelHandle {
public:
    explicit MjModelHandle(const std::string& path) {
        char error[1024] = {0};
        model_ = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
        if (!model_) {
            throw std::runtime_error(std::string("failed to load MJCF: ") + error);
        }
        data_ = mj_makeData(model_);
        if (!data_) {
            mj_deleteModel(model_);
            model_ = nullptr;
            throw std::runtime_error("failed to allocate MuJoCo data");
        }
    }

    ~MjModelHandle() {
        if (data_) {
            mj_deleteData(data_);
        }
        if (model_) {
            mj_deleteModel(model_);
        }
    }

    mjModel* model() const { return model_; }
    mjData* data() const { return data_; }

private:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
};

double clip(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

double parseDouble(const std::string& value, double fallback) {
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str()) {
        return fallback;
    }
    return parsed;
}

int requiredId(const mjModel* model, int object_type, const std::string& name) {
    const int id = mj_name2id(model, object_type, name.c_str());
    if (id < 0) {
        throw std::runtime_error("missing MuJoCo object: " + name);
    }
    return id;
}

VirtualRightMode parseVirtualRight(const std::string& value) {
    if (value == "zero") {
        return VirtualRightMode::Zero;
    }
    if (value == "copy") {
        return VirtualRightMode::Copy;
    }
    if (value == "inverted") {
        return VirtualRightMode::Inverted;
    }
    throw std::runtime_error("--virtual-right must be one of zero, copy, inverted");
}

DriverScenario parseDriverScenario(const std::string& value) {
    if (value == "sine") {
        return DriverScenario::Sine;
    }
    if (value == "stop_go") {
        return DriverScenario::StopGo;
    }
    if (value == "abrupt_stop") {
        return DriverScenario::AbruptStop;
    }
    throw std::runtime_error("--driver-scenario must be one of sine, stop_go, abrupt_stop");
}

void printUsage(const char* argv0) {
    std::cout
        << "MuJoCo realtime dry-run for hs_exoskeleton_v2\n\n"
        << "Usage: " << argv0 << " --mjcf model.xml [options]\n\n"
        << "Options:\n"
        << "  --output-dir DIR              Runtime CSV output root\n"
        << "  --prefix NAME                 Runtime CSV filename prefix\n"
        << "  --joint NAME                  Joint sampled as left hip (default: hip)\n"
        << "  --driver-actuator NAME        MuJoCo actuator used as human driver\n"
        << "  --exo-actuator NAME           MuJoCo actuator held at zero torque\n"
        << "  --duration-s SECONDS          Simulation duration (default: 12)\n"
        << "  --sample-rate-hz HZ           V2 controller sample rate (default: 50)\n"
        << "  --amplitude-deg DEG           Human driver sine amplitude (default: 40)\n"
        << "  --frequency-hz HZ             Human driver sine frequency (default: 0.8)\n"
        << "  --driver-kp VALUE             Human driver PD kp (default: 80)\n"
        << "  --driver-kd VALUE             Human driver PD kd (default: 8)\n"
        << "  --driver-torque-limit NM      Human driver torque limit (default: 80)\n"
        << "  --apply-v2-torque-scale S     Apply S * left V2 torque to exo actuator (default: 0)\n"
        << "  --driver-scenario NAME        sine, stop_go, abrupt_stop (default: sine)\n"
        << "  --stop-start-s SECONDS        Stop window start time (default: 4)\n"
        << "  --stop-duration-s SECONDS     Stop window duration for stop_go (default: 3)\n"
        << "  --virtual-right zero|copy|inverted (default: copy)\n";
}

Options parseArgs(int argc, char** argv) {
    Options options{};
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto need_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for " + name);
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--mjcf") {
            options.mjcf_path = need_value(arg);
        } else if (arg == "--output-dir") {
            options.output_dir = need_value(arg);
        } else if (arg == "--prefix") {
            options.prefix = need_value(arg);
        } else if (arg == "--joint") {
            options.joint_name = need_value(arg);
        } else if (arg == "--driver-actuator") {
            options.driver_actuator_name = need_value(arg);
        } else if (arg == "--exo-actuator") {
            options.exo_actuator_name = need_value(arg);
        } else if (arg == "--duration-s") {
            options.duration_s = parseDouble(need_value(arg), options.duration_s);
        } else if (arg == "--sample-rate-hz") {
            options.sample_rate_hz = parseDouble(need_value(arg), options.sample_rate_hz);
        } else if (arg == "--amplitude-deg") {
            options.amplitude_deg = parseDouble(need_value(arg), options.amplitude_deg);
        } else if (arg == "--frequency-hz") {
            options.frequency_hz = parseDouble(need_value(arg), options.frequency_hz);
        } else if (arg == "--driver-kp") {
            options.driver_kp = parseDouble(need_value(arg), options.driver_kp);
        } else if (arg == "--driver-kd") {
            options.driver_kd = parseDouble(need_value(arg), options.driver_kd);
        } else if (arg == "--driver-torque-limit") {
            options.driver_torque_limit = parseDouble(need_value(arg), options.driver_torque_limit);
        } else if (arg == "--apply-v2-torque-scale") {
            options.apply_v2_torque_scale = parseDouble(need_value(arg), options.apply_v2_torque_scale);
        } else if (arg == "--driver-scenario") {
            options.driver_scenario = parseDriverScenario(need_value(arg));
        } else if (arg == "--stop-start-s") {
            options.stop_start_s = parseDouble(need_value(arg), options.stop_start_s);
        } else if (arg == "--stop-duration-s") {
            options.stop_duration_s = parseDouble(need_value(arg), options.stop_duration_s);
        } else if (arg == "--virtual-right") {
            options.virtual_right = parseVirtualRight(need_value(arg));
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (options.mjcf_path.empty()) {
        throw std::runtime_error("--mjcf is required");
    }
    if (options.duration_s <= 0.0 || options.sample_rate_hz <= 0.0 || options.frequency_hz <= 0.0) {
        throw std::runtime_error("duration, sample rate, and frequency must be positive");
    }
    if (options.driver_torque_limit <= 0.0) {
        throw std::runtime_error("--driver-torque-limit must be positive");
    }
    if (options.apply_v2_torque_scale < 0.0) {
        throw std::runtime_error("--apply-v2-torque-scale must be non-negative");
    }
    if (options.stop_start_s < 0.0 || options.stop_duration_s < 0.0) {
        throw std::runtime_error("stop start and stop duration must be non-negative");
    }
    return options;
}

SimIds getSimIds(const mjModel* model, const Options& options) {
    const int joint_id = requiredId(model, mjOBJ_JOINT, options.joint_name);
    SimIds ids{};
    ids.hip_qpos_addr = model->jnt_qposadr[joint_id];
    ids.hip_qvel_addr = model->jnt_dofadr[joint_id];
    ids.driver_actuator_id = requiredId(model, mjOBJ_ACTUATOR, options.driver_actuator_name);
    ids.exo_actuator_id = requiredId(model, mjOBJ_ACTUATOR, options.exo_actuator_name);
    return ids;
}

struct DriverTarget {
    double q_des = 0.0;
    double qvel_des = 0.0;
};

DriverTarget sineTarget(double time_s, const Options& options) {
    const double amplitude_rad = options.amplitude_deg * kPi / 180.0;
    const double omega = 2.0 * kPi * options.frequency_hz;
    return {
        amplitude_rad * std::sin(omega * time_s),
        amplitude_rad * omega * std::cos(omega * time_s),
    };
}

DriverTarget driverTarget(double time_s, const Options& options) {
    const double stop_end_s = options.stop_start_s + options.stop_duration_s;
    if (options.driver_scenario == DriverScenario::StopGo &&
        time_s >= options.stop_start_s &&
        time_s < stop_end_s) {
        return {sineTarget(options.stop_start_s, options).q_des, 0.0};
    }
    if (options.driver_scenario == DriverScenario::AbruptStop &&
        time_s >= options.stop_start_s) {
        return {sineTarget(options.stop_start_s, options).q_des, 0.0};
    }
    return sineTarget(time_s, options);
}

double computeDriverTorque(double qpos, double qvel, double time_s, const Options& options) {
    const DriverTarget target = driverTarget(time_s, options);
    const double torque = options.driver_kp * (target.q_des - qpos) + options.driver_kd * (target.qvel_des - qvel);
    return clip(torque, -options.driver_torque_limit, options.driver_torque_limit);
}

double clipActuatorCtrl(const mjModel* model, int actuator_id, double value) {
    if (model->actuator_ctrllimited[actuator_id]) {
        const double lower = model->actuator_ctrlrange[2 * actuator_id];
        const double upper = model->actuator_ctrlrange[2 * actuator_id + 1];
        return clip(value, lower, upper);
    }
    return value;
}

ExoState makeState(const mjData* data, const SimIds& ids, const Options& options, uint64_t loop_seq, double dt_s) {
    const double left_pos = data->qpos[ids.hip_qpos_addr];
    const double left_vel = data->qvel[ids.hip_qvel_addr];
    double right_pos = 0.0;
    double right_vel = 0.0;
    if (options.virtual_right == VirtualRightMode::Copy) {
        right_pos = left_pos;
        right_vel = left_vel;
    } else if (options.virtual_right == VirtualRightMode::Inverted) {
        right_pos = -left_pos;
        right_vel = -left_vel;
    }

    ExoState state{};
    state.time_s = data->time;
    state.monotonic_time_s = data->time;
    state.epoch_ms = static_cast<int64_t>(std::llround(data->time * 1000.0));
    state.loop_seq = loop_seq;
    state.dt_s = dt_s;
    state.left.position_rad = left_pos;
    state.left.velocity_rad_s = left_vel;
    state.left.raw_motor_position_rad = left_pos;
    state.left.raw_motor_velocity_rad_s = left_vel;
    state.right.position_rad = right_pos;
    state.right.velocity_rad_s = right_vel;
    state.right.raw_motor_position_rad = right_pos;
    state.right.raw_motor_velocity_rad_s = right_vel;
    state.enabled = true;
    state.healthy = true;
    state.health = ExoHealth::Nominal;
    return state;
}

TorqueCommand updateController(ControlModules& modules, const ControlConfig& config, const ExoState& state, double dt_s, ExoLogger& logger) {
    StopDecision stop = modules.stop_detector.update(state, dt_s);
    GaitFeatures features = modules.feature_extractor.update(state, dt_s);
    const bool phase_tracking_enabled = modules.freeze.phase_tracking_enabled && stop.phase_tracking_enabled;
    PhaseEstimate phase = modules.phase_estimator.update(
        features,
        dt_s,
        phase_tracking_enabled,
        modules.last_assist_state,
        modules.last_stop_probability,
        modules.last_motion_confidence,
        modules.freeze.freeze_requested,
        stop.stop_requested);
    features.amplitude_rad = phase.amplitude_rad;
    features.frequency_hz = phase.frequency_hz;

    IntentEstimate intent = modules.intent_detector.update(features, dt_s);
    modules.last_stop_probability = intent.stop_probability;
    modules.last_motion_confidence = intent.motion_confidence;
    modules.freeze = stop.stop_requested ? modules.freeze_manager.resetToLive() : modules.freeze_manager.update(intent, dt_s);

    AssistInputs assist_inputs{};
    assist_inputs.motion_confidence = intent.motion_confidence;
    assist_inputs.phase_valid = phase.valid && !modules.freeze.recovery_active && !stop.stop_requested;
    assist_inputs.anchor_detected = phase.anchor_detected;
    assist_inputs.stop_requested = stop.stop_requested;
    assist_inputs.freeze_requested = modules.freeze.freeze_requested;
    assist_inputs.faulted = !state.healthy;

    AssistOutput assist = modules.assist_state_machine.update(assist_inputs, dt_s);
    modules.last_assist_state = assist.state;
    TorqueCommand torque = assist.state == AssistState::Stopping
        ? modules.stop_torque_limiter.update(modules.previous_torque, dt_s)
        : modules.torque_profile.compute(phase.phase_rad, phase.frequency_hz, assist.torque_scale, assist.allow_output);
    modules.previous_torque = torque;

    logger.write(state, features, phase, intent, modules.freeze, assist, torque);
    return torque;
}

int run(const Options& options) {
    MjModelHandle sim(options.mjcf_path);
    const SimIds ids = getSimIds(sim.model(), options);

    ControlConfig control_config{};
    control_config.loop_frequency_hz = options.sample_rate_hz;
    control_config.run_duration_s = options.duration_s;
    ControlModules modules(control_config);

    LoggingConfig logging{};
    logging.base_folder = options.output_dir;
    logging.filename_prefix = options.prefix;
    logging.stream_id = "exo_hs_v2_mujoco_realtime_dry_run";
    ExoLogger logger(logging);
    if (!logger.open()) {
        throw std::runtime_error("failed to open runtime logger");
    }

    const double sample_dt = 1.0 / options.sample_rate_hz;
    double next_sample_s = 0.0;
    double last_sample_s = 0.0;
    uint64_t loop_seq = 0;
    int samples = 0;
    double applied_exo_torque = 0.0;

    while (sim.data()->time <= options.duration_s + 1e-12) {
        const double qpos = sim.data()->qpos[ids.hip_qpos_addr];
        const double qvel = sim.data()->qvel[ids.hip_qvel_addr];
        sim.data()->ctrl[ids.driver_actuator_id] = computeDriverTorque(qpos, qvel, sim.data()->time, options);
        sim.data()->ctrl[ids.exo_actuator_id] = applied_exo_torque;

        if (sim.data()->time + 1e-12 >= next_sample_s) {
            const double dt_s = loop_seq == 0 ? sample_dt : std::max(1e-6, sim.data()->time - last_sample_s);
            last_sample_s = sim.data()->time;
            ExoState state = makeState(sim.data(), ids, options, ++loop_seq, dt_s);
            const TorqueCommand torque = updateController(modules, control_config, state, std::clamp(dt_s, 0.0001, 0.1), logger);
            applied_exo_torque = clipActuatorCtrl(
                sim.model(),
                ids.exo_actuator_id,
                options.apply_v2_torque_scale * torque.left_nm);
            sim.data()->ctrl[ids.exo_actuator_id] = applied_exo_torque;
            ++samples;
            next_sample_s += sample_dt;
        }

        mj_step(sim.model(), sim.data());
    }

    logger.close();
    std::cout << "MuJoCo realtime dry-run log: " << logger.outputPath() << "\n";
    std::cout << "Samples: " << samples << "\n";
    if (options.apply_v2_torque_scale > 0.0) {
        std::cout << "V2 torque scale applied to MuJoCo exo actuator: " << options.apply_v2_torque_scale << "\n";
    } else {
        std::cout << "V2 torque was recorded only; MuJoCo exo actuator was held at zero.\n";
    }
    return 0;
}

} // namespace
} // namespace exo

int main(int argc, char** argv) {
    try {
        const auto options = exo::parseArgs(argc, argv);
        return exo::run(options);
    } catch (const std::exception& exc) {
        std::cerr << "mujoco realtime dry-run error: " << exc.what() << "\n";
        return 1;
    }
}
