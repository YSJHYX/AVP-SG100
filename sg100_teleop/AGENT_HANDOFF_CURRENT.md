# AGENT HANDOFF — MODE7 ACTIVE MICRO-MOTION PREP（2026-09-03）

本轮只完成代码审计、工具实现、离线测试和文档更新。**没有执行真机，没有
commit，没有 push**；所有既有 recording artifact 原样保留。

## 当前真机基础

- N0 MODE_NULL realtime：PASS — REAL HARDWARE。
- N1 full-hand MODE7 entry，J1–J11 RX mode=7，zero-output/no-visible-motion，
  fault=0：PASS — REAL HARDWARE。
- N2 MODE7 feedback finite/stable/non-multiturn/joint-angle-scale：SUPPORTED —
  REAL HARDWARE。
- exact canonical identity/sign/zero/range：PENDING。

## 映射边界

- `F_cmd: q_sdk_cmd = F_cmd(q_canonical)` 仍是 outbound 候选。
- `F_fb: q_canonical = F_fb(q_mode7_feedback)` 由独立
  `SG100FeedbackMap` 表示；真实 feedback path 不再默认调用 outbound
  `hardware_map.actual_to_canonical()`。
- 未经逐关节真机 delta 验证：`feedback_map_valid=False`、
  `canonical_valid=False`、dataset=`QUARANTINED`。禁止从 `F_cmd` 逆推 `F_fb`。

## N3 工具

路径：`sg100_teleop/tools/sg100_mode7_active_micro_motion.py`。

- 默认 dry-run，不构造真机 driver、不打开串口。
- 先全手 MODE7/position=0/PID=0/output=0 并等待稳定 fresh frames。
- 首轴自动筛选 J3/J6/J8/J11 的 identity/simple candidates，并按 live
  minimum headroom 排序；拒绝 J2/J7 和任何非 identity/边界不足轴。
- 最新 artifact 的 dry-run 选择 J6/index_j3：current `-0.725093°`，SDK/
  canonical range 约 `[-60,90]°`，负/正 headroom `59.274907/90.725093°`，
  选择正向 0.5°；真机时重新按 fresh MODE7 baseline 计算。
- Stage A：只给首轴 `p=0.1,i=0,d=0,output_limit=0.5` 当前位置 active hold；
  其他十轴 MODE7 且零输出。该参数由当前 SDK realtime example 与 vendor
  production log（328/0/0/1638）共同支持。
- Stage B：ACTIVE HOLD 数值/人工双 PASS 后必须再输入 `MOVE`；只发一次默认
  0.5°，硬上限 1.0°，无 silent clamp，无第二 delta，无自动返回/homing。
- hard guards：hold 0.5°；micro target 1.5°；其他任一轴 0.75°；fault、
  unexpected mode、wrong dominant/direction、stall、NaN、异常/中断均
  MODE_NULL cleanup。
- artifact：`recording/sg100_mode7_active_<timestamp>_<joint>/`，包含
  `metadata.json`、`samples.csv`、`summary.json` 和每次 API call audit。
- native path 保持：`Hand_Control_t -> sg100_set_hand_control ->
  SG100Device::SetHandControl`；vendor scheduler 负责缓存重发。

## Gate

| 阶段 | 状态 |
|---|---|
| N0 MODE_NULL realtime | PASS |
| N1 full-hand MODE7 safe entry | PASS |
| N2 MODE7 joint-angle-scale | SUPPORTED |
| N3 active hold + micro-motion | TOOL PREP / HARDWARE PENDING |
| N4 per-joint F_cmd/F_fb | PENDING |
| N5 full-hand active integration | BLOCKED |
| N6 AVP | BLOCKED |
| N7 dataset | BLOCKED / QUARANTINED |

## 离线验证

- active-micro-motion focused：30 passed。
- 全部 `test_sg100_*.py`：221 passed。
- hand/data safety selected：46 passed。
- 模块 dry-run：PASS，明确未构造真机 driver、未打开串口。
- `compileall`：PASS；`git diff --check`：PASS。

## 下一步

软件验证全部 PASS 后，只允许用户未来执行文档中的一条 N3 命令。Agent 不得
代执行。硬件证据通过人工复核前，不得更新 production map、启用 canonical
feedback、接 AVP 或解除 dataset quarantine。
