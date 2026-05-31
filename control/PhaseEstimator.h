// 用 MultiHarmonicAO 跟踪滤波后的步态信号，输出连续相位；可靠步态 anchor 离散校正 AO 频率。

#pragma once

#include "MultiHarmonicAO.h"
#include "config/ControlConfig.h"
#include "control/AssistStateMachine.h"

namespace exo {

struct GaitFeatures;
struct PhaseEstimate;

enum class AnchorRejectReason : int {
    None = 0,
    UnreliableSignal = 1,
    ConfirmFailed = 2,
    Refractory = 3,
    StopIntent = 4,
    AssistState = 5,
    Warmup = 6,
    Interval = 7,
    AnchorType = 8,
    LowConfidence = 9,
    FrequencyRange = 10,
};

class PhaseEstimator {
public:
    explicit PhaseEstimator(const PhaseConfig& config);

    PhaseEstimate update(const GaitFeatures& features,
                         double dt_s,
                         bool tracking_enabled,
                         AssistState assist_state,
                         double stop_probability = 0.0,
                         double motion_confidence = 0.0,
                         bool freeze_requested = false,
                         bool stop_requested = false);

private:
    enum class AnchorType {
        Peak,
        Valley,
    };

    struct AnchorEventContext {
        const GaitFeatures& features;
        double dt_s;
        AssistState assist_state;
        double stop_probability;
        double prev_velocity_deg_s;
        double curr_velocity_deg_s;
        double previous_velocity_abs_deg_s;
        double current_velocity_abs_deg_s;
        bool tracking_enabled;
    };

    static double wrapAngle(double angle_rad);
    static double clamp01(double value);
    static double moveToward(double current, double target, double max_step);
    static double anchorFrequencyGainForState(const PhaseConfig& config, AssistState assist_state);
    static bool confirmPendingAnchor(const PhaseConfig& config, AnchorType type, double curr_velocity_deg_s, double spread_deg);

    void applyRateLimitedOmega(double dt_s);
    void clampOmegaToConfigLimits();
    struct AnchorFrequencyMeasurement {
        double measured_frequency_hz = 0.0;
        double confidence = 0.0;
        bool ready = false;
        AnchorRejectReason rejection = AnchorRejectReason::None;
    };

    AnchorFrequencyMeasurement measureAnchorFrequency(const AnchorEventContext& ctx,
                                                      AnchorType anchor_type) const;
    bool applyAnchorFrequencyCorrection(const AnchorEventContext& ctx,
                                        PhaseEstimate& estimate,
                                        double omega_before_rad_s,
                                        double measured_frequency_hz,
                                        double confidence);
    void tryApplyDeferredFrequencyCorrection(const AnchorEventContext& ctx,
                                             PhaseEstimate& estimate,
                                             double omega_before_rad_s);
    void processAnchorEvent(const AnchorEventContext& ctx,
                            AnchorType anchor_type,
                            PhaseEstimate& estimate,
                            double omega_before_rad_s);
    void resetStartupPriorState();
    void updateStartupPrior(const GaitFeatures& features,
                            AssistState assist_state,
                            double stop_probability,
                            double motion_confidence,
                            bool tracking_enabled,
                            bool positive_peak,
                            bool negative_peak,
                            PhaseEstimate& estimate);
    void tryApplyStartupPrior(const AnchorEventContext& ctx,
                              PhaseEstimate& estimate,
                              double omega_before_rad_s);

    static double wrapToPi(double angle_rad);
    double targetPhiFor(AnchorType anchor_type) const;
    bool allowsPhiEAssistState(AssistState assist_state) const;
    bool computePhiEGate(const GaitFeatures& features,
                         AssistState assist_state,
                         double stop_probability,
                         bool freeze_requested,
                         bool stop_requested) const;
    bool allowsPhiELatch(AssistState assist_state,
                         double stop_probability,
                         double anchor_confidence,
                         const GaitFeatures& features,
                         bool freeze_requested,
                         bool stop_requested) const;
    void latchPhiE(AnchorType anchor_type, double phi_gp, double anchor_confidence, PhaseEstimate& estimate);
    void decayPhiE(double dt_s);
    void updatePhiEOverlay(const GaitFeatures& features,
                           AssistState assist_state,
                           double stop_probability,
                           bool freeze_requested,
                           bool stop_requested,
                           double dt_s,
                           PhaseEstimate& estimate);

    PhaseConfig config_;
    MultiHarmonicAO oscillator_;
    double last_signal_rad_ = 0.0;
    double last_phase_velocity_deg_s_ = 0.0;
    double last_anchor_time_s_ = -10.0;
    double last_frequency_anchor_time_s_ = -10.0;
    AnchorType last_frequency_anchor_type_ = AnchorType::Peak;
    bool has_frequency_anchor_ = false;
    int reliable_anchor_count_since_tracking_enable_ = 0;
    bool previous_tracking_enabled_ = true;
    double omega_target_rad_s_ = 0.0;
    bool omega_target_tracking_active_ = false;
    bool has_pending_anchor_ = false;
    AnchorType pending_anchor_type_ = AnchorType::Peak;
    double pending_prev_velocity_deg_s_ = 0.0;
    double pending_peak_velocity_abs_deg_s_ = 0.0;
    bool has_deferred_frequency_correction_ = false;
    double deferred_measured_frequency_hz_ = 0.0;
    double deferred_confidence_ = 0.0;
    double last_motion_confidence_ = 0.0;
    double last_spread_deg_ = 0.0;
    int same_sign_velocity_frames_ = 0;
    int spread_increase_frames_ = 0;
    int velocity_sign_ = 0;
    double last_zero_cross_time_s_ = -10.0;
    double startup_frequency_prior_hz_ = 0.0;
    double startup_confidence_ = 0.0;
    bool has_startup_prior_ = false;
    bool startup_prior_from_zero_cross_ = false;
    bool startup_prior_applied_ = false;
    double startup_tracking_enter_time_s_ = -10.0;
    AssistState previous_assist_state_ = AssistState::Transparent;
    double now_s_ = 0.0;

    double phi_e_rad_ = 0.0;
    double ce_latch_rad_ = 0.0;
    double t_anchor_s_ = -10.0;
    double pe_latch_rad_ = 0.0;
    double target_phi_at_anchor_rad_ = 0.0;
    bool phi_e_active_ = false;
    AnchorType last_phi_e_anchor_type_ = AnchorType::Peak;
};

} // namespace exo
