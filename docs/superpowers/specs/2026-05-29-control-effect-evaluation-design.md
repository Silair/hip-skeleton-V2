# 控制效果评价判定设计

## 背景

`hs_exoskeleton_v2/tools/analyze_run.py` 已能从运行 CSV 生成相位、频率、力矩、安全门控等指标，但当前只能“看指标”，不能直接判断本次控制效果是否达标。新增功能应在不接触实时控制、CAN/Kvaser、底层电机路径的前提下，对离线运行日志给出可复现的 PASS/FAIL 结论。

## 推荐方案

采用“离线分析脚本内置评价门控”方案：在 `analyze_run.py` 的指标计算后增加评价层，将关键指标与默认阈值比较，输出总体状态、逐项检查结果和失败原因。该方案复用已有 CSV、HTML、metrics.json 和测试框架，风险最低，最适合当前代码结构。

## 方案取舍

1. **离线评价门控（采用）**：修改 `tools/analyze_run.py`，输出 `evaluation.json`，HTML 增加“控制效果判定”区，CLI 打印 PASS/FAIL。优点是无需硬件、无需新增依赖、可用现有合成日志测试。
2. **实时控制环内判定**：在 `ExoController` 中在线累计指标并判定。优点是运行时可见；缺点是会增加实时路径复杂度，且许多参考相位指标需要事后插值，不适合作为第一步。
3. **单独新增评价脚本**：例如 `tools/evaluate_run.py`。优点是职责独立；缺点是会重复读取/解析逻辑，用户需要多跑一步。

## 已确认决策

- 第一阶段只做单 CSV 离线日志判定，不做实时在线判定，不做批量评估。
- 评价采用综合判定，分为相位/频率跟踪、助力力矩效果、安全门控三组。
- 阈值采用内置默认值，并允许 CLI 覆盖关键阈值。
- 输出 CLI 摘要、`evaluation.json` 和 HTML “控制效果判定”区。
- 安全项一票否决；性能项按阈值失败；无意义的条件指标可 `SKIP`。
- 参考相位沿用现有峰/谷事件构造的近似参考，不引入外部 ground truth。
- 默认评价 `FAIL` 仍 exit 0；仅在 `--fail-on-evaluation` 下返回 exit 2。
- 第一阶段不修改实时日志字段、不修改 C++ 控制链路。
- 报告和文档必须明确：PASS 不代表临床安全性、人体实验许可或硬件实际输出验证。

## 功能范围

- 新增默认阈值，覆盖：参考事件数量、相位 RMSE、相位 P95、频率 RMSE、AO 重构误差、收敛时间、最大力矩、力矩变化率、安全门控违规、静止误助力、双侧高力矩比例、峰值力矩相位误差。
- 支持 CLI 覆盖核心阈值：`--max-torque-nm`、`--max-torque-rate-nm-s`、`--phase-rmse-threshold-percent`、`--frequency-rmse-threshold-hz`、`--stationary-false-assist-threshold-s`、`--min-reference-events`、`--phase-p95-threshold-percent`、`--peak-torque-phase-threshold-deg`。
- 输出 `evaluation.json`，结构化保存总体 `status` 与每项 `checks`。
- HTML 报告增加判定摘要和逐项表格。
- CLI 打印 `evaluation_status`，可选 `--fail-on-evaluation` 用非零退出码接入 CI/实验批处理。

## 非目标

- 不修改控制算法、状态机、力矩曲线。
- 不新增第三方依赖。
- 不把近似参考相位当作实验 ground truth；报告中仍明确其来源。
- 不证明临床安全性、人体实验许可或硬件实际输出。

## 测试计划

- 扩展 `tests/test_analyze_run.py`：合成稳定步态应 PASS，并生成 `evaluation.json` / HTML 判定区。
- 新增失败用例：构造超过力矩上限、冻结时仍有力矩等指标，评价应 FAIL 并列出失败项。
- 运行 Python 单元测试：`python3 -m unittest tests/test_analyze_run.py`。
- 运行测试发现并验证 `--fail-on-evaluation` 的 PASS/FAIL 退出码路径。
