# Metrics comparison

Status: **MIXED**

| Metric | Before | After | Δ | Δ% | Verdict |
| --- | ---: | ---: | ---: | ---: | --- |
| Phase RMSE (% gait) | 9.9503 | 11.5693 | +1.6190 | +16.3% | WORSE |
| Phase error P95 (% gait) | 24.7093 | 28.3419 | +3.6327 | +14.7% | WORSE |
| Frequency RMSE (Hz) | 0.2532 | 0.2063 | -0.0469 | -18.5% | IMPROVED |
| Frequency MAE (Hz) | 0.1769 | 0.1414 | -0.0355 | -20.1% | IMPROVED |
| Frequency relock mean (s) | 1.7300 | 1.2733 | -0.4567 | -26.4% | IMPROVED |
| Frequency relock max (s) | 3.0000 | 1.3800 | -1.6200 | -54.0% | IMPROVED |
| Phase relock mean (s) | 0.7350 | 1.4067 | +0.6717 | +91.4% | WORSE |
| Phase relock max (s) | 0.9800 | 1.6200 | +0.6400 | +65.3% | WORSE |
| Combined relock mean (s) | 1.7300 | 1.4067 | -0.3233 | -18.7% | IMPROVED |
| Combined relock max (s) | 3.0000 | 1.6200 | -1.3800 | -46.0% | IMPROVED |
| Max torque rate (Nm/s) | 87.5799 | 64.0280 | -23.5519 | -26.9% | IMPROVED |
| Torque rate P95 (Nm/s) | 41.3992 | 38.8088 | -2.5905 | -6.3% | IMPROVED |
| Peak torque phase MAE (deg) | 7.4050 | 3.5610 | -3.8441 | -51.9% | IMPROVED |
| Omega jump P95 (Hz) | N/A | 0.0255 | N/A | N/A | SKIP |
| Omega jump max (Hz) | N/A | 0.0255 | N/A | N/A | SKIP |
| False anchor during stop (count) | 0.0000 | 0.0000 | +0.0000 | N/A | UNCHANGED |
| Anchor updates (count) | 0.0000 | 15.0000 | +15.0000 | N/A | IMPROVED |
