#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "config/LoggingConfig.h"
#include "control/AssistStateMachine.h"
#include "control/FreezeManager.h"
#include "control/GaitFeatures.h"
#include "control/TorqueProfile.h"
#include "hardware/JointTypes.h"
#include "logging/ExoLogger.h"

int main() {
    namespace fs = std::filesystem;
    const fs::path base = fs::temp_directory_path() / "hsx_v2_logger_test";
    fs::remove_all(base);

    exo::LoggingConfig config{};
    config.base_folder = base.string();
    config.filename_prefix = "logger_test";
    config.sync_session_id = "session";
    config.stream_id = "stream";

    exo::ExoLogger logger(config);
    assert(logger.open());

    exo::ExoState state{};
    state.healthy = true;
    state.loop_seq = 7;
    state.monotonic_time_s = 1.5;
    state.dt_s = 0.02;
    state.left.position_rad = 0.1;
    state.right.position_rad = -0.1;
    state.left.velocity_rad_s = 0.2;
    state.right.velocity_rad_s = -0.2;

    exo::GaitFeatures features{};
    features.phase_signal_rad = 0.25;
    features.filtered_phase_signal_rad = 0.2;
    features.spread_deg = 11.0;
    features.phase_velocity_deg_s = 22.0;
    features.signed_phase_velocity_deg_s = -22.0;

    exo::PhaseEstimate phase{};
    phase.phase_rad = 1.0;
    phase.phi_gp_rad = 0.95;
    phase.phi_e_rad = 0.05;
    phase.phi_final_rad = 1.0;
    phase.frequency_hz = 0.8;
    phase.amplitude_rad = 0.3;
    phase.ao_signal_estimate_rad = 0.19;
    phase.ao_signal_error_rad = 0.01;
    phase.valid = true;
    phase.anchor_detected = true;

    exo::IntentEstimate intent{};
    intent.motion_confidence = 0.9;
    intent.stop_probability = 0.1;

    exo::FreezeDecision freeze{};
    freeze.freeze_requested = false;
    freeze.phase_tracking_enabled = true;
    freeze.recovery_active = false;

    exo::AssistOutput assist{};
    assist.state = exo::AssistState::Active;
    assist.torque_scale = 0.75;
    assist.allow_output = true;

    exo::TorqueCommand torque{};
    torque.left_nm = 1.2;
    torque.right_nm = 0.0;

    logger.write(state, features, phase, intent, freeze, assist, torque);
    const std::string output_path = logger.outputPath();
    logger.close();

    assert(!output_path.empty());
    std::ifstream in(output_path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    const std::string csv = buffer.str();
    assert(csv.find("AoSignalEstimateRad,AoSignalErrorRad") != std::string::npos);
    assert(csv.find("PhaseSignalRad,FilteredPhaseSignalRad") != std::string::npos);
    assert(csv.find("SignedPhaseVelocityDegS") != std::string::npos);
    assert(csv.find("PhiGpRad,PhiERad,PhiFinalRad") != std::string::npos);
    assert(csv.find("CeLatchRad,TargetPhiRad") != std::string::npos);
    assert(csv.find("TorqueScale,AllowOutput") != std::string::npos);
    assert(csv.find("0.25,0.2") != std::string::npos);
    assert(csv.find("-22") != std::string::npos);
    return 0;
}
