# hs_exoskeleton_v2

与旧版 `hs_exoskeleton.cpp` 控制链路并行的 V2 控制管线，旧路径保持不变。

## 目标

- 保留旧版控制循环，便于对照实验与回滚。
- 将 V2 控制逻辑拆成更小模块。
- 在关节空间运行控制器，不把 CAN/Kvaser 细节暴露给控制层。
- 用以下能力替代旧版里硬编码的 GMM / 冻结 / 力矩窗口耦合：
  - 基于归一化运动证据的启发式意图检测；
  - 冻结滞回与恢复；
  - 对称力矩生成。

## 目录结构

- `hardware/`：关节空间硬件抽象，以及基于 Kvaser 的实现
- `control/`：特征提取、相位估计、意图检测、冻结处理、状态机、力矩曲线、主控制器
- `logging/`：V2 信号的 CSV 记录
- `app/`：V2 应用程序入口
- `tests/`：纯控制逻辑回归测试

## 构建目标

- `hs_exoskeleton`：旧版目标
- `hs_exoskeleton_v2`：新控制器目标
- `hs_exoskeleton_v2_tests`：纯控制回归测试
## 运行数据分析

运行时 CSV 由 `logging/ExoLogger` 写入 `Data/<date>/`。运行后可用标准库脚本生成指标和 HTML/SVG 图表：

```bash
python3 hs_exoskeleton_v2/tools/analyze_run.py <runtime.csv>
```

分析脚本同时会输出控制效果评价判定：CLI 中显示 `evaluation_status`，报告目录中生成 `evaluation.json`，HTML 中增加“控制效果判定”区。需要让评价失败时返回非零退出码，可加 `--fail-on-evaluation`。

详见 `RUNTIME_ANALYSIS.md`。

## 离线曲线回放

新增 `tools/make_replay_curve.py` 与 C++ 目标 `hs_exoskeleton_v2_replay`，可先用人工设计/录制的关节角 CSV 离线跑 V2 控制层，再用 `tools/analyze_run.py` 生成指标和 HTML 图表。该路径不接触 Kvaser/CAN/底层电机控制。

```bash
python3 hs_exoskeleton_v2/tools/make_replay_curve.py --output /tmp/curve.csv --scenario sine
/tmp/hsx_v2_replay --input /tmp/curve.csv --output-dir /tmp/replay_logs
python3 hs_exoskeleton_v2/tools/analyze_run.py /tmp/replay_logs/<date>/<csv>
```

详见 `OFFLINE_REPLAY.md`。
