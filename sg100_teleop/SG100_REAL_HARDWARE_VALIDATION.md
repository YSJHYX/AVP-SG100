# SG100 Real Hardware Validation Guide

> **STOP (2026-09-03): DO NOT RUN ANY COMMAND IN THIS GUIDE.**  Real N1
> tested only thumb_j1 with all joints commanded MODE_NULL and exposed that mode's
> raw/accumulated scale mismatch (~15–30° physical motion -> +1854 callback
> units). Vendor guidance says mode-7 feedback should instead be joint angle;
> A later preserved v4 probe artifact reached stable all-11 mode7 with no
> visible movement and fault 0, but returned signs disagreed with configured
> ranges on J2/J5/J7/J10, so its physical position-domain result is AMBIGUOUS.
> Canonical physical-angle semantics therefore remain unverified. H2 and all
> later hardware stages are blocked. See
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

| 阶段 | 内容 | 风险 |
|------|------|------|
| H0 | 只读 probe（无任何命令）— **PASS（真机）** | 零 |
| H1 | safe realtime bootstrap，保持当前实测姿态 | 低 |
| H2 | realtime 11D 单轴小幅 smoke（逐关节验证） | 低 |
| H3 | AVP + C anchor + whole-hand teleop（OPEN/PARTIAL/POWER/RELEASE/PINCH） | 中 |
| H4 | SG100 feedback + DATA CONTRACT 采集验证 | 低 |
| H5 | RM65 + SG100 combined（仅前面全 PASS 后） | 中 |

---

## 最终运行行为要求（H3 实现时必须满足）

- **startup measured pose（当前未满足，BLOCKED）**：只有全部关节返回 mode 7
  且 mode-7 物理角语义经真机验证后，MuJoCo 中的 SG100 才能显示真机
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

## H1 — Safe realtime bootstrap

DRY RUN（默认，不连接串口）：

```bash
.venv/bin/python -m sg100_teleop.tools.sg100_realtime_hold_test
```

真实 H1 只保持当前 measured pose，不接受 AVP 或 delta：

```bash
.venv/bin/python -m sg100_teleop.tools.sg100_realtime_hold_test \
    --port /dev/ttyUSB0 --duration-s 4 --execute
```

只有输出 `SAFE REALTIME BOOTSTRAP: PASS` 才能准备 H2。

## H2 — realtime 11D 单轴 smoke（H1 PASS 后）

> **DO NOT RUN UNTIL H1 PASS.** 当前只准备了 11D single-axis target builder，
> 尚未提供 H2 真机执行命令。不得用 passthrough `SetJointPosition` 作为
> production H2 路径。

H2 将以 H1 的 11D hold 为基准，10 轴保持不变，只给 `index_j2` 依次增加
0.5°、1°、2°，并始终通过 production realtime whole-hand sender 发送。

判定：
- `measured delta` 与 commanded 同号且量级接近（>20%）→ 方向正确；
- 方向相反 → **停止**，按实测修 `sg100_teleop/config/sg100_hardware_map.yaml`
  的 `gain`（该关节 -1），再重测；
- 几乎不动 → 停止并调查状态；不要猜测或重复调用 enable；
- 任何 fault/反馈失效 → 工具自动停止发送后续命令。

### 后续其余关节逐一验证

推荐顺序（先非拇指、拇指 opposition 最后）：

```text
index_j2 → middle_j1 → little_j2 → index_j3 → middle_j2
→ little_j1 → index_j1 → little_j3 → thumb_j3 → thumb_j2 → thumb_j1
```

thumb_j1（最后测）：确认正方向确实是拇指**朝 index/middle 闭合区**
对掌、无自碰撞、无指连杆干涉、无异响/异常电流、能顺畅返回。

逐关节把结论（方向 OK / 需翻 gain / 需 offset）记录到
`sg100_hardware_map.yaml` 的注释里。

## H3 — AVP + C anchor + whole-hand teleop（RM65 保持关闭）

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
- [ ] H1 safe realtime current-pose hold PASS
- [ ] H2 11D realtime 单轴 smoke / 11 关节逐一 PASS
- [ ] H3 AVP + C anchor + whole-hand teleop PASS
      （OPEN/PARTIAL/POWER/RELEASE/PINCH）
- [ ] H3 thumb lock 分级 10/20/30/40 PASS（每级日期）
- [ ] H4 DATA CONTRACT PASS（episode 路径）
- [ ] H5 combined PASS

相机（Head/Wrist RealSense）未到位：`CAMERA: NOT TESTED`、
`PI0.5 MULTIMODAL: NOT READY`，与 SG100 进度无关。
