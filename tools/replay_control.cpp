// Offline replay runner for hs_exoskeleton_v2.
// It feeds a designed or recorded joint-angle CSV through the same V2 control
// modules used by the hardware loop, but never touches CAN/Kvaser hardware.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/ControlConfig.h"
#include "config/LoggingConfig.h"
#include "control/AssistStateMachine.h"
#include "control/FreezeManager.h"
#include "control/GaitFeatureExtractor.h"
#include "control/GaitFeatures.h"
#include "control/IntentDetector.h"
#include "control/PhaseEstimator.h"
#include "control/StopDetector.h"
#include "control/TorqueProfile.h"
#include "hardware/JointTypes.h"
#include "logging/ExoLogger.h"

namespace exo {
namespace {

struct ReplayOptions {
    std::string input_path;
    std::string output_dir = "ReplayData";
    std::string prefix = "offline_replay";
    double max_duration_s = 0.0;
    double fallback_rate_hz = 50.0;
};

struct ReplaySample {
    double time_s = 0.0;
    double left_pos_rad = 0.0;
    double right_pos_rad = 0.0;
    double left_vel_rad_s = std::numeric_limits<double>::quiet_NaN();
    double right_vel_rad_s = std::numeric_limits<double>::quiet_NaN();
    bool healthy = true;
    bool enabled = true;
};

std::string trim(const std::string& value) {
    size_t first = 0;
    while (first < value.size() && std::isspace(static_cast<unsigned char>(value[first]))) {
        ++first;
    }
    size_t last = value.size();
    while (last > first && std::isspace(static_cast<unsigned char>(value[last - 1]))) {
        --last;
    }
    return value.substr(first, last - first);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> cells;
    std::string cell;
    bool in_quotes = false;
    for (size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '"') {
            in_quotes = !in_quotes;
        } else if (ch == ',' && !in_quotes) {
            cells.push_back(trim(cell));
            cell.clear();
        } else {
            cell.push_back(ch);
        }
    }
    cells.push_back(trim(cell));
    return cells;
}

std::map<std::string, size_t> headerIndex(const std::vector<std::string>& header) {
    std::map<std::string, size_t> index;
    for (size_t i = 0; i < header.size(); ++i) {
        index[trim(header[i])] = i;
    }
    return index;
}

bool hasColumn(const std::map<std::string, size_t>& index, const std::string& key) {
    return index.find(key) != index.end();
}

std::string cellFor(const std::vector<std::string>& row, const std::map<std::string, size_t>& index, const std::vector<std::string>& keys) {
    for (const auto& key : keys) {
        const auto found = index.find(key);
        if (found != index.end() && found->second < row.size()) {
            return row[found->second];
        }
    }
    return "";
}

double parseDouble(const std::string& value, double fallback = 0.0) {
    if (value.empty()) {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str()) {
        return fallback;
    }
    return parsed;
}

bool parseBool(const std::string& value, bool fallback = true) {
    if (value.empty()) {
        return fallback;
    }
    std::string lowered;
    lowered.reserve(value.size());
    for (char ch : value) {
        lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    if (lowered == "1" || lowered == "true" || lowered == "yes") {
        return true;
    }
    if (lowered == "0" || lowered == "false" || lowered == "no") {
        return false;
    }
    return parseDouble(value, fallback ? 1.0 : 0.0) >= 0.5;
}

void computeMissingVelocities(std::vector<ReplaySample>& samples, double fallback_rate_hz) {
    if (samples.empty()) {
        return;
    }
    for (size_t i = 0; i < samples.size(); ++i) {
        auto diff = [&](double ReplaySample::*field) -> double {
            if (samples.size() == 1) {
                return 0.0;
            }
            size_t lo = i == 0 ? 0 : i - 1;
            size_t hi = i + 1 < samples.size() ? i + 1 : i;
            if (hi == lo) {
                hi = std::min(samples.size() - 1, lo + 1);
            }
            double dt = samples[hi].time_s - samples[lo].time_s;
            if (dt <= 1e-9) {
                dt = static_cast<double>(hi - lo) / std::max(1e-9, fallback_rate_hz);
            }
            return (samples[hi].*field - samples[lo].*field) / std::max(1e-9, dt);
        };
        if (std::isnan(samples[i].left_vel_rad_s)) {
            samples[i].left_vel_rad_s = diff(&ReplaySample::left_pos_rad);
        }
        if (std::isnan(samples[i].right_vel_rad_s)) {
            samples[i].right_vel_rad_s = diff(&ReplaySample::right_pos_rad);
        }
    }
}

std::vector<ReplaySample> readReplayCsv(const std::string& path, double fallback_rate_hz) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("failed to open replay input: " + path);
    }

    std::string line;
    if (!std::getline(input, line)) {
        throw std::runtime_error("replay input is empty: " + path);
    }
    const auto header = splitCsvLine(line);
    const auto index = headerIndex(header);
    const bool has_time = hasColumn(index, "TimeS") || hasColumn(index, "MonoTimeS") || hasColumn(index, "Time");
    if (!has_time) {
        throw std::runtime_error("replay input must contain TimeS, MonoTimeS, or Time column");
    }

    std::vector<ReplaySample> samples;
    while (std::getline(input, line)) {
        if (trim(line).empty()) {
            continue;
        }
        const auto row = splitCsvLine(line);
        ReplaySample sample{};
        sample.time_s = parseDouble(cellFor(row, index, {"TimeS", "MonoTimeS", "Time"}));
        sample.left_pos_rad = parseDouble(cellFor(row, index, {"LeftJointPosRad", "LeftJointPos", "LeftHipRad", "LeftPosRad"}));
        sample.right_pos_rad = parseDouble(cellFor(row, index, {"RightJointPosRad", "RightJointPos", "RightHipRad", "RightPosRad"}));
        const std::string left_vel = cellFor(row, index, {"LeftJointVelRadS", "LeftJointVel", "LeftHipVelRadS", "LeftVelRadS"});
        const std::string right_vel = cellFor(row, index, {"RightJointVelRadS", "RightJointVel", "RightHipVelRadS", "RightVelRadS"});
        if (!left_vel.empty()) {
            sample.left_vel_rad_s = parseDouble(left_vel);
        }
        if (!right_vel.empty()) {
            sample.right_vel_rad_s = parseDouble(right_vel);
        }
        sample.healthy = parseBool(cellFor(row, index, {"Healthy"}), true);
        sample.enabled = parseBool(cellFor(row, index, {"Enabled"}), true);
        samples.push_back(sample);
    }

    if (samples.empty()) {
        throw std::runtime_error("replay input has no samples: " + path);
    }
    std::sort(samples.begin(), samples.end(), [](const ReplaySample& a, const ReplaySample& b) {
        return a.time_s < b.time_s;
    });
    computeMissingVelocities(samples, fallback_rate_hz);
    return samples;
}

void printUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " --input curve.csv [--output-dir ReplayData] [--prefix offline_replay]\n"
              << "       [--max-duration-s seconds] [--fallback-rate-hz hz]\n\n"
              << "Input CSV columns:\n"
              << "  TimeS, LeftJointPosRad, RightJointPosRad\n"
              << "Optional columns:\n"
              << "  LeftJointVelRadS, RightJointVelRadS, Healthy, Enabled\n";
}

ReplayOptions parseArgs(int argc, char** argv) {
    ReplayOptions options{};
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
        } else if (arg == "--input") {
            options.input_path = need_value(arg);
        } else if (arg == "--output-dir") {
            options.output_dir = need_value(arg);
        } else if (arg == "--prefix") {
            options.prefix = need_value(arg);
        } else if (arg == "--max-duration-s") {
            options.max_duration_s = parseDouble(need_value(arg));
        } else if (arg == "--fallback-rate-hz") {
            options.fallback_rate_hz = parseDouble(need_value(arg), options.fallback_rate_hz);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
    if (options.input_path.empty()) {
        throw std::runtime_error("--input is required");
    }
    if (options.fallback_rate_hz <= 0.0) {
        throw std::runtime_error("--fallback-rate-hz must be positive");
    }
    return options;
}

int runReplay(const ReplayOptions& options) {
    const auto samples = readReplayCsv(options.input_path, options.fallback_rate_hz);

    ControlConfig config{};
    config.loop_frequency_hz = options.fallback_rate_hz;
    if (samples.size() >= 2) {
        const double duration = samples.back().time_s - samples.front().time_s;
        config.run_duration_s = duration > 0.0 ? duration : config.run_duration_s;
    }
    if (options.max_duration_s > 0.0) {
        config.run_duration_s = std::min(config.run_duration_s, options.max_duration_s);
    }

    LoggingConfig logging{};
    logging.base_folder = options.output_dir;
    logging.filename_prefix = options.prefix;
    logging.stream_id = "exo_hs_v2_offline_replay";

    ExoLogger logger(logging);
    if (!logger.open()) {
        throw std::runtime_error("failed to open replay output log");
    }

    GaitFeatureExtractor feature_extractor(config.phase);
    PhaseEstimator phase_estimator(config.phase);
    IntentDetector intent_detector(config.intent);
    StopDetector stop_detector(config.stop);
    FreezeManager freeze_manager(config.freeze);
    AssistStateMachine assist_state_machine(config.assist);
    TorqueProfile torque_profile(config.torque);

    FreezeDecision freeze{};
    uint64_t loop_seq = 0;
    double previous_time_s = samples.front().time_s;
    int64_t epoch_ms = 0;

    for (const auto& sample : samples) {
        const double relative_time_s = sample.time_s - samples.front().time_s;
        if (options.max_duration_s > 0.0 && relative_time_s > options.max_duration_s) {
            break;
        }

        double dt_s = sample.time_s - previous_time_s;
        if (loop_seq == 0 || dt_s <= 1e-9) {
            dt_s = 1.0 / options.fallback_rate_hz;
        }
        const double controller_dt_s = std::clamp(dt_s, 0.0001, 0.1);
        previous_time_s = sample.time_s;

        ExoState state{};
        state.time_s = relative_time_s;
        state.loop_seq = ++loop_seq;
        state.epoch_ms = epoch_ms;
        state.monotonic_time_s = relative_time_s;
        state.dt_s = dt_s;
        state.left.position_rad = sample.left_pos_rad;
        state.right.position_rad = sample.right_pos_rad;
        state.left.velocity_rad_s = sample.left_vel_rad_s;
        state.right.velocity_rad_s = sample.right_vel_rad_s;
        state.left.raw_motor_position_rad = sample.left_pos_rad;
        state.right.raw_motor_position_rad = sample.right_pos_rad;
        state.left.raw_motor_velocity_rad_s = sample.left_vel_rad_s;
        state.right.raw_motor_velocity_rad_s = sample.right_vel_rad_s;
        state.enabled = sample.enabled;
        state.healthy = sample.healthy;
        state.health = sample.healthy ? ExoHealth::Nominal : ExoHealth::CommunicationFault;
        epoch_ms += static_cast<int64_t>(std::llround(dt_s * 1000.0));

        StopDecision stop = stop_detector.update(state, controller_dt_s);
        GaitFeatures features = feature_extractor.update(state, controller_dt_s);
        const bool phase_tracking_enabled = freeze.phase_tracking_enabled && stop.phase_tracking_enabled;
        PhaseEstimate phase = phase_estimator.update(features, controller_dt_s, phase_tracking_enabled);
        features.amplitude_rad = phase.amplitude_rad;
        features.frequency_hz = phase.frequency_hz;

        IntentEstimate intent = intent_detector.update(features, controller_dt_s);
        freeze = stop.stop_requested ? freeze_manager.resetToLive() : freeze_manager.update(intent, controller_dt_s);

        AssistInputs assist_inputs{};
        assist_inputs.motion_confidence = intent.motion_confidence;
        assist_inputs.phase_valid = phase.valid && !freeze.recovery_active && !stop.stop_requested;
        assist_inputs.anchor_detected = phase.anchor_detected;
        assist_inputs.stop_requested = stop.stop_requested;
        assist_inputs.freeze_requested = freeze.freeze_requested;
        assist_inputs.faulted = !state.healthy;

        AssistOutput assist = assist_state_machine.update(assist_inputs, controller_dt_s);
        TorqueCommand torque = torque_profile.compute(phase.phase_rad, phase.frequency_hz, assist.torque_scale, assist.allow_output);

        logger.write(state, features, phase, intent, freeze, assist, torque);
    }

    logger.close();
    std::cout << "hs_exoskeleton_v2 offline replay log: " << logger.outputPath() << "\n";
    std::cout << "Analyze with: python3 hs_exoskeleton_v2/tools/analyze_run.py " << logger.outputPath() << "\n";
    return 0;
}

} // namespace
} // namespace exo

int main(int argc, char** argv) {
    try {
        const auto options = exo::parseArgs(argc, argv);
        return exo::runReplay(options);
    } catch (const std::exception& e) {
        std::cerr << "replay error: " << e.what() << "\n";
        return 1;
    }
}
