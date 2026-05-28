#include "logging/ExoLogger.h"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "control/AssistStateMachine.h"
#include "control/FreezeManager.h"
#include "control/GaitFeatures.h"
#include "control/TorqueProfile.h"
#include "hardware/JointTypes.h"

namespace exo {

ExoLogger::ExoLogger(const LoggingConfig& config)
    : config_(config) {}

ExoLogger::~ExoLogger() {
    close();
}

std::string ExoLogger::buildPath() const {
    namespace fs = std::filesystem;
    const auto now = std::chrono::system_clock::now();
    const std::time_t raw = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &raw);
#else
    localtime_r(&raw, &tm);
#endif

    std::ostringstream date_folder;
    date_folder << std::put_time(&tm, "%Y.%m.%d");
    const fs::path dir = fs::path(config_.base_folder) / date_folder.str();
    fs::create_directories(dir);

    // 同日同前缀下递增序号，避免覆盖已有文件。
    int index = 1;
    while (true) {
        std::ostringstream file_name;
        file_name << config_.filename_prefix << '_' << index << '_' << date_folder.str() << ".csv";
        const fs::path candidate = dir / file_name.str();
        if (!fs::exists(candidate)) {
            return candidate.string();
        }
        ++index;
    }
}

bool ExoLogger::open() {
    output_path_ = buildPath();
    stream_.open(output_path_);
    if (!stream_.is_open()) {
        return false;
    }

    // 表头与 write() 中字段顺序一致，便于后处理脚本解析。
    stream_ << "SyncSessionId,StreamId,LoopSeq,EpochMs,MonoTimeS,DtS,Healthy,AssistState,FreezeState,"
            << "MotionConfidence,StopProbability,Phase,PhaseValid,AnchorDetected,Frequency,Amplitude,"
            << "AoSignalEstimateRad,AoSignalErrorRad,PhaseSignalRad,FilteredPhaseSignalRad,"
            << "SpreadDeg,PhaseVelocityDegS,SignedPhaseVelocityDegS,"
            << "FreezeRequested,PhaseTrackingEnabled,RecoveryActive,TorqueScale,AllowOutput,"
            << "LeftJointPos,RightJointPos,LeftJointVel,RightJointVel,LeftRawMotorPos,RightRawMotorPos,"
            << "LeftRawMotorVel,RightRawMotorVel,LeftTorqueCmd,RightTorqueCmd\n";
    return true;
}

const std::string& ExoLogger::outputPath() const {
    return output_path_;
}

void ExoLogger::write(const ExoState& state,
    const GaitFeatures& features,
    const PhaseEstimate& phase,
    const IntentEstimate& intent,
    const FreezeDecision& freeze,
    const AssistOutput& assist,
    const TorqueCommand& torque) {
    if (!stream_.is_open()) {
        return;
    }

    stream_ << config_.sync_session_id << ','
            << config_.stream_id << ','
            << state.loop_seq << ','
            << state.epoch_ms << ','
            << state.monotonic_time_s << ','
            << state.dt_s << ','
            << (state.healthy ? 1 : 0) << ','
            << static_cast<int>(assist.state) << ','
            << static_cast<int>(freeze.state) << ','
            << intent.motion_confidence << ','
            << intent.stop_probability << ','
            << phase.phase_rad << ','
            << (phase.valid ? 1 : 0) << ','
            << (phase.anchor_detected ? 1 : 0) << ','
            << phase.frequency_hz << ','
            << phase.amplitude_rad << ','
            << phase.ao_signal_estimate_rad << ','
            << phase.ao_signal_error_rad << ','
            << features.phase_signal_rad << ','
            << features.filtered_phase_signal_rad << ','
            << features.spread_deg << ','
            << features.phase_velocity_deg_s << ','
            << features.signed_phase_velocity_deg_s << ','
            << (freeze.freeze_requested ? 1 : 0) << ','
            << (freeze.phase_tracking_enabled ? 1 : 0) << ','
            << (freeze.recovery_active ? 1 : 0) << ','
            << assist.torque_scale << ','
            << (assist.allow_output ? 1 : 0) << ','
            << state.left.position_rad << ','
            << state.right.position_rad << ','
            << state.left.velocity_rad_s << ','
            << state.right.velocity_rad_s << ','
            << state.left.raw_motor_position_rad << ','
            << state.right.raw_motor_position_rad << ','
            << state.left.raw_motor_velocity_rad_s << ','
            << state.right.raw_motor_velocity_rad_s << ','
            << torque.left_nm << ','
            << torque.right_nm << '\n';
}

void ExoLogger::close() {
    if (stream_.is_open()) {
        stream_.close();
    }
}

} // namespace exo
