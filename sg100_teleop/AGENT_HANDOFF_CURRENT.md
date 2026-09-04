# AGENT HANDOFF — J6 PASS / J7 + RESTRICTED AVP PREP（2026-09-03）

本轮只读复核用户已完成的 J6 真机 artifact，并完成 J7 与 restricted AVP
软件准备。**Agent 没有执行真机，没有 commit，没有 push**；所有既有
recording artifact 原样保留。

## 当前真机基础

- N0 MODE_NULL realtime：PASS — REAL HARDWARE。
- N1 full-hand MODE7 entry，J1–J11 RX mode=7，zero-output/no-visible-motion，
  fault=0：PASS — REAL HARDWARE。
- N2 MODE7 feedback finite/stable/non-multiturn/joint-angle-scale：SUPPORTED —
  REAL HARDWARE。
- N3a J6/index_j3 active hold、+0.5° micro motion、identity command channel：
  PASS — REAL HARDWARE。artifact：
  `recording/sg100_mode7_active_20260903_202121_J6`。
- exact canonical identity/sign/zero/range：PENDING。

## 映射边界

- `F_cmd: q_sdk_cmd = F_cmd(q_canonical)`：J6 identity 已获真机支持；J7
  软件映射为 `q_sdk_J7=-q_canonical_middle_j1`，真机待验证。
- `F_fb: q_canonical = F_fb(q_mode7_feedback)` 由独立
  `SG100FeedbackMap` 表示；真实 feedback path 不再默认调用 outbound
  `hardware_map.actual_to_canonical()`。
- 未经逐关节真机 delta 验证：`feedback_map_valid=False`、
  `canonical_valid=False`、dataset=`QUARANTINED`。禁止从 `F_cmd` 逆推 `F_fb`。

## J6 artifact 与 CRC 审计

- active hold：J6 settled/peak 均 0°；其他轴 hold 最大峰值 0.000437°。
- +0.5° command 后 J6 settled delta +0.252690°、peak 0.367403°；J6 为
  dominant，第二通道 J10=0°（十轴并列），fault=0，五次 SetHandControl
  用途依次为 MODE_NULL seed / MODE7 zero-output / active hold / micro / cleanup。
- 所有 active hold 与 micro RX 均为全 11 轴 mode 7；各 phase 内 sequence
  连续、最大间隔 19.998 ms、无 NaN/stall/guard trip。cleanup MODE_NULL API
  接受成功；旧 artifact 的 30 ms cleanup 采样仍显示传播前 mode 7，不能把它
  夸大为 RX 已回显 mode 0。
- SDK binary 的 `RS485Adapter::DrainRxBuf` 在 RX CRC mismatch 时不调用
  `rx_cb_`，丢弃坏帧头并继续重同步，无自动 retry。终端单次 CRC 因而分类为
  `CRC_TRANSIENT_RECOVERED`，不影响 J6 verdict。旧 artifact 无 CRC timestamp；
  新 C wrapper 只旁路监听 SDK 原输出，记录 `crc_error_count/timestamps_ns`，
  没有重写协议。

## J7 工具

路径：`sg100_teleop/tools/sg100_mode7_active_micro_motion.py`。

- 默认 dry-run，不构造真机 driver、不打开串口。
- 先全手 MODE7/position=0/PID=0/output=0 并等待稳定 fresh frames。
- 支持 `--joint J7` / `--joint middle_j1 --canonical-delta-deg -0.5`。
- J7 从 canonical 0° / 已缓存 SDK zero-output target 0° 起步，经 F_cmd
  `gain=-1` 恰好一次得到 canonical −0.5° → SDK +0.5°；raw MODE7 feedback
  只做 excursion/dominance 观测，绝不作为 command seed。
- 启用 hold 前新增 `ZERO` 机械零位确认，再沿用 `HOLD` / `MOVE` 两阶段门。
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
| N3a J6 identity active command | PASS — REAL HARDWARE |
| N3b J7 sign-flip active command | SOFTWARE READY / HARDWARE PENDING |
| N4 restricted AVP SG100 command | SOFTWARE PREP（J6 allowlist） |
| N5 full 11D AVP | BLOCKED |
| F_fb canonical measured state | PENDING |
| N7 dataset | BLOCKED / QUARANTINED |

## 离线验证

- active/restricted focused：45 passed。
- 全部 SG100：236 passed；AVP/MuJoCo：57 passed；USDZ contract：5 passed；
  data safety：47 passed。
- 模块 dry-run：PASS，明确未构造真机 driver、未打开串口。
- `compileall`：PASS；`git diff --check`：PASS。

## 下一步

下一次只允许用户执行一条 J7 sign-flip 命令，Agent 不代执行。J7 真机 PASS
前不能加入 real allowlist；full 11D、canonical feedback 和 dataset quarantine
继续阻断。
