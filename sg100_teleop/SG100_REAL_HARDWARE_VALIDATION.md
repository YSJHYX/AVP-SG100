# SG100 Real Hardware Validation Guide

> **CURRENT (2026-09-03): J6 N3a PASS; J7 N3b SOFTWARE READY / HARDWARE
> PENDING; THIS AGENT RAN NO HARDWARE.** Real N1
> tested only thumb_j1 with all joints commanded MODE_NULL and exposed that mode's
> raw/accumulated scale mismatch (~15–30° physical motion -> +1854 callback
> units). Vendor guidance says mode-7 feedback should instead be joint angle;
> A later preserved v4 probe artifact reached stable all-11 MODE7 with no
> visible movement and fault 0. This is a real-hardware PASS for safe MODE7
> entry/all-11 status/no-motion entry and supports a bounded joint-angle-scale
> feedback domain. The old AMBIGUOUS verdict incorrectly compared feedback to
> SDK command ranges. Exact feedback-to-canonical identity/sign/zero/range
> mapping is still pending. The user has since completed the guarded J6 active
> hold and +0.5° micro test; its preserved artifact passed read-only review.
> The next user-operated stage is exactly one J7 sign-flip test. Restricted
> J6-only AVP is software-prepared; full-hand/data remain blocked. See
> [SG100_LATEST_SDK_INTEGRATION.md](SG100_LATEST_SDK_INTEGRATION.md).

> **HISTORICAL (2026-09-02, OLD SDK architecture).**  The H0–H5 flow and
> every command below were written for the deleted CANFD-passive / raw
> 0x25/0x27/audited-0x05 backend (`sg100_realtime_hold_test.py`,
> `sg100_joint_smoke_test.py`, `SG100_SAFE_REALTIME_BOOTSTRAP.md` — all
> removed by the 2026-09-03 migration to the V1.5_NO_TACTILE SDK).
> The results recorded here remain valid **historical evidence** that the
> physical link, ~100 Hz realtime exchange, and measured feedback work.
> They are NOT a validation of the new backend.  Current architecture,
> pending hardware steps, and the probe command live in
> [SG100_LATEST_SDK_INTEGRATION.md](SG100_LATEST_SDK_INTEGRATION.md);
> new-SDK status: N0 link/cadence validated; physical mapping **BLOCKED**.

> 当前阶段定义已更新：H1 是 **SAFE REALTIME BOOTSTRAP / current-pose
> hold**，原文中的单关节 smoke 顺延为 H2。H1 的 SDK cache 审计、安全序列、
> PASS 条件和唯一执行命令见
> [SG100_SAFE_REALTIME_BOOTSTRAP.md](SG100_SAFE_REALTIME_BOOTSTRAP.md)（已随旧架构删除）。

第一次 SG100 真机接入的分阶段验证指南。**逐阶段推进，前一阶段未 PASS
不进入下一阶段。** 所有真机命令都由用户手动执行；所有运动 CLI 默认
READ-ONLY 或 DRY-RUN，真实运动必须显式 `--execute`。

硬件前提：SG100 空载、不连 RM65、不抓物体、不碰桌面。手臂保持关闭。

---

## 阶段总览

| 阶段 | 内容 | 状态 |
|------|------|------|
| N0 | MODE_NULL realtime | PASS — REAL HARDWARE |
| N1 | full-hand MODE7 safe entry | PASS — REAL HARDWARE |
| N2 | MODE7 joint-angle-scale feedback | SUPPORTED — REAL HARDWARE |
| N3a | J6 identity active command | PASS — REAL HARDWARE |
| N3b | J7 sign-flip active command | SOFTWARE READY / HARDWARE PENDING |
| N4 | restricted AVP SG100 command | SOFTWARE PREP (J6 allowlist) |
| N5 | full 11D AVP | BLOCKED |
| F_fb | canonical measured state | PENDING |
| N7 | dataset enable | BLOCKED / QUARANTINED |

## 当前下一阶段（J6 已完成；只准备 J7）

不再要求人工掰动关节。`tools/sg100_mode7_mapping_characterization.py` 保留历史
代码和证据，但退出主验证路径。新工具
`tools/sg100_mode7_active_micro_motion.py` 分两阶段。J6 使用的 fresh MODE7
feedback seed 已有真机证据；J7 明确禁止这样做。J7 Stage A 使用先前下发的
MODE7 zero-output SDK target 0°，并要求用户确认 middle_j1 位于 canonical
mechanical zero；canonical −0.5° 经 `gain=-1` 一次变为 SDK +0.5°。数值无
跳变且用户回答无可见运动后，Stage B 再输入 `MOVE`，只发送一次 micro delta。结束直接
MODE_NULL，不发送反向回位、不 homing、不 set-zero、不改 production map。

J6 artifact `sg100_mode7_active_20260903_202121_J6`：active hold J6
settled/peak 0°；+0.5° 后 settled +0.252690°、peak 0.367403°，其余十轴
micro excursion 0°，J6 dominant，fault 0，cleanup accepted。故 J6 不再重复。

安全参数来自当前 SDK example 和 vendor production log：`p=0.1, i=0, d=0,
output_limit=0.5`。delta 硬上限 1.0°；目标轴 excursion guard 1.5°，其他轴
0.75°，ACTIVE HOLD jump guard 0.5°。fault、mode loss、wrong dominant joint、
反向、过大运动、stall、NaN、Ctrl-C/EOF/exception 均触发 MODE_NULL cleanup 并
保留 artifact。1.5° 是默认命令的 3 倍：给 0.5° 微动留出反馈噪声/瞬态余量，
同时在进入多度运动前 fail-closed；单帧目标轴跳变超过 0.75° 或持续反向摆动
同样判为 rapid motion/oscillation。

---

## 最终运行行为要求（H3 实现时必须满足）

- **startup measured pose（当前未满足，BLOCKED）**：只有全部关节返回 mode 7
  且 mode-7 到 canonical 的逐关节 identity/sign/zero/range 映射经真机验证后，MuJoCo 中的 SG100 才能显示真机
  当前 **measured** 姿态（`--hand-backend real` 时 hand qpos 来自
  `read_feedback()`）。真机在 0 位则显示 0 位；真机不在 0 位则必须显示
  实际非 0 姿态。**禁止 startup 自动拉回 0 / home。**
- **C = teleop anchor**：按 C 之前，SG100 保持当前 real measured pose，
  MuJoCo hand 持续跟随 measured q，AVP 手部动作不得驱动真机。按 C 瞬间
  记录 `human_anchor`（当时 CanonicalHandState/retarget reference）与
  `robot_anchor`（当时 SG100 measured q[11]），并保证
  `q_target ≈ q_measured`（无 teleop jump）。
- **C 后 relative teleop**：
  `q_target(t) = q_robot_anchor + (F(human_t) − F(human_anchor))`，F 使用
  production SG100 semantics（base retarget → pinch coordination →
  power-grasp thumb lock），之后 clamp → filter/rate limit → realtime 11D
  sender（`update_realtime_target`）。
- **real-mode digital twin**：`--hand-backend sim` 时 MuJoCo physics qpos
  为显示真值；`--hand-backend real` 时 MuJoCo SG100 qpos 必须来自 measured
  feedback（不是 `hand_target_q` / `hardware_sent_qcmd`）。系统同时保留
  `hand_target_q` 与 `hand_measured_q`。

---

## H0 — Read-only probe

```bash
python -m sg100_teleop.tools.sg100_hardware_probe \
    --port /dev/ttyUSB0
```

- 该工具**不发送任何 position 命令**。
- 检查项：connection、joint mapping 表、feedback 语义证据、
  timestamp 单调、有效采样率（期望 ~100Hz）、duplicate、NaN、
  range 合理性、sample-to-sample 跳变。
- `changing: NO` 时用手指轻碰手（外力扰动）确认编码器在动。
- PASS 标准见输出 `RESULT: PASS (read-only)`。
- **注意**：PASS 只说明反馈流健全，方向/映射仍需 H1 验证。

无硬件冒烟（不需要设备）：

```bash
python -m sg100_teleop.tools.sg100_hardware_probe --mock
```

## N3 — J6 已 PASS；J7 sign-flip 待用户执行

DRY RUN（默认，不连接串口）：

```bash
.venv/bin/python -m sg100_teleop.tools.sg100_mode7_active_micro_motion
```

未来唯一获准的下一条真机命令（必须由用户操作，本轮 Agent 未执行）：

```bash
.venv/bin/python -m sg100_teleop.tools.sg100_mode7_active_micro_motion \
    --port /dev/ttyUSB0 --joint middle_j1 \
    --canonical-delta-deg -0.5 --execute
```

运行中必须依次输入 `ZERO`、`HOLD`、确认无可见 hold jump，再输入 `MOVE`。
默认回答均 ABORT。J7 PASS 前不得加入真实 allowlist。

## 旧 H2 smoke 方案（已废止）

旧方案从 `index_j2` 开始并连续使用 0.5°/1°/2°，不满足当前“两阶段、首轴
自动筛选、第一次只发一个 delta”的门禁，禁止执行。N3 首次真机证据经人工
复核后，再另行审核 N4 的逐关节验证顺序；任何单次结果都不得自动写入 map。

## N4 — restricted AVP（软件准备；本轮禁止执行）

现有 `SG100Retargeter` 同时向 MuJoCo 与真实后端提供同一 canonical qcmd；
`SG100RestrictedAVPBackend` 在后端内执行 canonical range、一次 F_cmd、SDK
range、allowlist、delta/spike/slew、stale/nonfinite、C/A gate、mode/fault/
CRC/stall 与 E-stop。当前 real allowlist 只接受 `index_j3`（J6）；J7 仍会被
构造期拒绝，full 11D 不可用。C 只缓存零输出 target，A 前无 active command。

以下旧 whole-hand H3 内容仅作历史说明，当前禁止执行。

## H3 — AVP + C anchor + whole-hand teleop（历史/已阻断）

> **前置：H1/H2 全 PASS。** C anchor（按 C 时记录 human/robot anchor、
> 无 teleop jump、C 后 relative teleop）与 startup measured pose 渲染
> 属于 H3 必做项，见上方"最终运行行为要求"。

空手完成五个动作：OPEN → PARTIAL GRASP → POWER GRASP → RELEASE
→ PINCH。不要抓物体。

```bash
python -m sg100_teleop.sim.run_avp_arm_hand_sim \
    --ip <AVP_IP> \
    --arm-backend sim \
    --hand-backend real \
    --hand-port /dev/ttyUSB0 \
    --thumb-retarget-debug
```

（`--arm-backend sim` = RM65 不动，仅 SG100 真机 + 仿真预览。）

**首次 power grasp 必须分级限制 thumb lock bias**（staged 10 → 20 →
30 → 40 deg，每级单独一条命令、单独确认，禁止跳级）：

```bash
# 第 1 级：lock bias 上限 10°
python -m sg100_teleop.sim.run_avp_arm_hand_sim \
    --ip <AVP_IP> --arm-backend sim --hand-backend real \
    --hand-port /dev/ttyUSB0 \
    --thumb-lock-hardware-cap-deg 10 --thumb-retarget-debug

# 观察无干涉后再依次 20 / 30 / 40（40 = production 配置原值）
```

每级检查（`--thumb-retarget-debug` 的 5Hz 输出）：
- POWER GRASP 时 `lock_weight ≈ 1`，thumb_j1 朝 index/middle 闭合区；
- 若机械上限或干涉出现在仿真目标角（~100°）之前：**立即停止**，
  不要强行追仿真目标，记录实际安全角并回报；
- PINCH 时 `pinch_gate ≈ 1`、`lock_weight ≈ 0`（pinch 不被叠加 +bias）；
- RELEASE 时 lock_weight 平滑回 0、拇指能重新外展。

## H4 — SG100 feedback + DATA CONTRACT

mock 链路自检（无硬件）：

```bash
python -m pytest sg100_teleop.tests.test_hand_data_contract_measured -q
```

真机采集（RM65 仍可关闭；arm 段按现有 REAL_DATA_V1_VALIDATION 流程，
hand 段为新增）：

```bash
python -m sg100_teleop.sim.run_avp_arm_hand_sim \
    --ip <AVP_IP> --arm-backend real --arm-ip <RM65_IP> \
    --hand-backend real --hand-port /dev/ttyUSB0 \
    --collect-arm-hand-data
```

检查 episode：

```bash
python -m sg100_teleop.data_collection.inspect_episode \
    recording/<session>/arm_hand_data
```

期望输出包含：

```text
HAND REAL VALIDATION (SG100 measured feedback)
feedback samples     : ...
commands sent        : ...
canonical actions    : ...
canonical state measured: YES
command/raw match    : PASS
feedback/raw match   : PASS
intervening command  : 0
first pre-roll       : PASS
SG100 REAL CAUSAL CHAIN: PASS
```

语义要点（见 ARM_HAND_DATA_CONTRACT_V1.md）：
- `state_hand` = action 之前**最新实测** SG100 反馈；
- `action_hand` = **真正发送成功**的 final qcmd（post-lock、post-filter）；
- `next_state_hand` = action 之后**第一条新实测**反馈；
- action 与 next_state 之间若有其它成功发送的 hand 命令 →
  `INTERVENING_HAND_COMMAND` 硬拒绝；
- 旧 proxy episode 会标注 `LEGACY HAND PROXY EPISODE`，照常可读。

## H5 — RM65 + SG100 combined（仅 H0–H4 全 PASS 后）

H4 的完整命令即 H5（arm real + hand real + collect）。首次运行请
先空手、低速，确认 coordinated 行为后再进入正式任务数据采集。

---

## 状态记录（由用户填写）

- [x] H0 probe PASS（2026-09-02，真机 /dev/ttyUSB0：serial PASS；
      RequestHandStatus 0x26 120 TX / 120 RX = 100%；Hand_Status decoded
      120；valid measured 11D 120；timestamps monotonic；motion commands 0）
- [x] N1 full-hand MODE7 safe entry PASS（真机）
- [x] N2 MODE7 joint-angle-scale feedback SUPPORTED（真机）
- [ ] N3 single-joint MODE7 active hold + one micro delta PASS
- [ ] N4 F_cmd/F_fb 逐关节映射审核完成
- [ ] H3 AVP + C anchor + whole-hand teleop PASS
      （OPEN/PARTIAL/POWER/RELEASE/PINCH）
- [ ] H3 thumb lock 分级 10/20/30/40 PASS（每级日期）
- [ ] H4 DATA CONTRACT PASS（episode 路径）
- [ ] H5 combined PASS

相机（Head/Wrist RealSense）未到位：`CAMERA: NOT TESTED`、
`PI0.5 MULTIMODAL: NOT READY`，与 SG100 进度无关。
