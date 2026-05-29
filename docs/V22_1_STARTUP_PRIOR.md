# V2.2.1 Startup Frequency Prior

## Purpose

Initialize `omega_target` during walk startup when formal anchor candidates are blocked by
reliable-cross gating, without mixing startup logic into steady-state anchor updates.

## Trigger (Tracking / Ramp only)

- `stop_probability < startup_prior_max_stop_probability` (default **0.55**)
- `motion_confidence ≥ startup_prior_min_motion_confidence` (default **0.48**)
- Same signed velocity for `startup_prior_same_sign_frames` (default **4**)
- `spread_deg` increasing for `startup_prior_spread_increase_frames` (default **3**)

## Frequency estimate

**Only** from zero-cross half-period timing, clamped to `[0.45, 1.0] Hz` (no velocity fallback).

## Apply once

On the first frame after a prior is ready:

- **Ramp / Active:** full `startup_prior_gain`
- **Tracking (optional, default on):** `startup_prior_gain * startup_prior_tracking_gain_scale` so
  low-frequency startup segments that never reach Ramp within 4s can still initialize `omega_target`

```text
omega_target ← blend(current, startup_prior_hz · 2π, startup_prior_gain · confidence)
```

Then rate-limited via existing `max_omega_rate_rad_s2` tracking.

## Not in scope

- No formal `AnchorDetected` / phase offset
- Disabled during Stopping / high stop intent
- No stride memory (V2.3)
