# DATA CONTRACT V1 真机验收指南

## Phase A — 仅验收 RM65 真机

本阶段只验收 Apple Vision Pro → IK → RM65 发送器 → RM65 实测反馈 → 数据采集器 → 离线检查工具这一条真实因果链。当前没有 SG100 真机、头部 RealSense 或腕部 RealSense；它们的缺失不影响 RM65 Phase A 判定，也不得被描述为已经通过硬件验收。

数据仍采用 `rm65_sg100_joint17_v1` 的 17 维结构。前 6 维是 RM65 真机数据，后 11 维是 SG100 遥操作软件 qcmd/command proxy；后 11 维不是 SG100 实测反馈，也不是硬件执行确认。

## 1. 首次测试安全边界

首次真机测试仅运行 10–20 秒，必须满足：

- RM65 空载、低速、小范围运动；
- 工作空间内无人员，操作者能够立即使用物理急停；
- 机械臂远离桌面、工装、障碍物、奇异位姿和关节限位；
- 不执行抓取、接触或大范围动作；
- 不要求 SG100 张开、闭合或产生任何真实运动；
- 本流程不能替代电子围栏、碰撞保护和正式安全验收。

## 2. 精确启动命令

当前脚本的 `--help` 已确认存在 `--collect-arm-hand-data`、`--recording-dir`、`--data-stale-pre-ms` 和 `--data-stale-post-ms`。在仓库根目录和包含 MuJoCo、AVP streamer、RM65 SDK 的项目环境中运行：

```bash
PYTHONPATH=. python sg100_teleop/sim/run_avp_arm_hand_sim.py \
  --ip 192.168.2.179 \
  --calibration sg100_teleop/config/avp_right_calibration.yaml \
  --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v8_compat.usdz \
  --wrist-roll-ik \
  --verbose \
  --arm-backend real \
  --arm-ip 192.168.1.18 \
  --arm-follow low \
  --arm-safety-profile 10 \
  --arm-verbose \
  --collect-arm-hand-data \
  --data-stale-pre-ms 100 \
  --data-stale-post-ms 250
```

将 `192.168.2.179` 替换为测试当天 Apple Vision Pro 的实际地址。RM65 控制器当前地址为 `192.168.1.18:8080`。如需改变输出根目录，只使用真实存在的 `--recording-dir PATH` 参数。

## 3. 启动与采集顺序

严格按以下顺序执行：

1. 确认物理工作空间已清空，急停可用。
2. 启动上述 runner 和数据采集。
3. 等待 RM65 backend ready，确认命令流仍为 `DISABLED`。
4. 确认收到 fresh、valid 的 RM65 实测关节反馈。
5. 等待有效右手跟踪，在中性姿态按 `C` 完成 AVP calibration/anchor。
6. 确认 collector 进入 `RECORDER_ARMED`，预采样反馈已开始。
7. 确认终端显示 `hand baseline : SERIALIZED COMMAND PROXY`；它是当前 final teleop qcmd 的软件基准，不是 SG100 实测状态。
8. 确认 `TELEOP_ACTIVE` 后按 `A` 启用 RM65，以极低速完成第一条有效小运动。
9. 确认 collector 已开始形成规范 transition。
10. 继续 10–20 秒空载、小范围安全运动；允许产生 hand tracking/software qcmd，但不要求 SG100 硬件动作。
11. 按 `A` 停止命令流，再用 `Ctrl-C` 正常结束。
12. 等待 episode 完整落盘；不要强制杀进程或手工修改文件。
13. 立即运行下节的 inspector。

第一条 RM65 transition 必须具有真实预采样反馈：

```text
t_arm_state < t_arm_action_sent < t_arm_next_state
```

第一条 hand proxy 必须能够从原始事件独立重建：

```text
serialized_baseline_timestamp < first_hand_action_timestamp
state[0, 6:17] == serialized_baseline_hand_qcmd
```

## 4. 运行检查工具

先运行完整摘要：

```bash
python -m sg100_teleop.data_collection.inspect_episode \
  recording/<session>/arm_hand_data
```

再检查第一条：

```bash
python -m sg100_teleop.data_collection.inspect_episode \
  recording/<session>/arm_hand_data \
  --dump-transition 0
```

从摘要读取 transition 总数 `N`，把下面占位符替换为实际整数，分别检查中间和末条：

```bash
python -m sg100_teleop.data_collection.inspect_episode \
  recording/<session>/arm_hand_data \
  --dump-transition <middle_index>

python -m sg100_teleop.data_collection.inspect_episode \
  recording/<session>/arm_hand_data \
  --dump-transition <N-1>
```

需要机器可读结果或全部错误时使用：

```bash
python -m sg100_teleop.data_collection.inspect_episode \
  recording/<session>/arm_hand_data \
  --json

python -m sg100_teleop.data_collection.inspect_episode \
  recording/<session>/arm_hand_data \
  --dump-failures \
  --show-all-errors
```

## 5. Phase A 通过标准

必须同时满足：

- `state/action/next_state` 均为 `(N, 17)`，并且 `N > 0`；
- `RM65 REAL CAUSAL CHAIN: PASS`；
- `DATA CONTRACT V1: PASS`；
- RM65 的 measured feedback、post-limiter successful sent command 均存在；
- 严格 `state < action < next_state` 为 `N / N`；
- RM65 raw/canonical action match 和 state match 均为 `PASS`；
- first transition pre-roll 为 `PASS`；
- command bundle 完整且无重复、缺失或错误关联；
- hand proxy baseline 显示 `SERIALIZED`，baseline/raw match 为 `PASS`；
- hand command proxy consistency 为 `PASS`；
- state/action/next_state 中 NaN、`+Inf`、`-Inf` 数量均为 0；
- 已接受 transition 中 tracking loss 和 safety fault 均为 0；
- `full_arm_hand_hardware_validated` 始终为 `false`。

当前缺少权威关节限位时，range check 可以显示 `NOT AVAILABLE`；检查工具不会猜测限位，这本身不是 Phase A 阻塞项。

## 6. 本阶段不要求通过的项目

以下项目统一为 `NOT TESTED` 或 `NOT AVAILABLE`，不会仅因缺失而使 RM65 real causal chain 失败：

- SG100 measured feedback；
- SG100 real hardware motion/execution；
- 头部或腕部相机帧；
- 相机内参、外参和时间同步；
- Pi0.5 multimodal dataset readiness。

即使 Phase A 全部通过，也只能声明“RM65 REAL CAUSAL DATA PATH: PASS”和“DATA CONTRACT V1 LOW-DIM STRUCTURE: PASS”。不得声明 full Arm-Hand hardware、SG100 hardware、multimodal 或 Pi0.5 数据集已经通过或 ready。

## 7. 后续阶段

- Phase A：RM65 real hardware causal path（本轮准备和下一次用户真机测试）。
- Phase B：SG100 real feedback/execution（未来硬件）。
- Phase C：头部与腕部 RealSense（未来硬件）。
- Phase D：多传感器时间与空间标定。
- Phase E：OpenPI / 最终 AHFE 数据集集成。

当前 production runtime 仍未调用 `IKContinuityGate`；原始审计必须继续如实记录 `not_applied_in_current_runtime`。本轮不以检查工具或文档改动替代该控制策略，也不把它误报为已应用。
