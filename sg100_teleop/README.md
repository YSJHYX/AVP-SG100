# SG100 + RM65 Apple Vision Pro Teleoperation

本文档是 `sg100_teleop` 的权威运行者手册与架构入口。根目录 README 是上游
VisionProTeleop 总览；本文件负责 RM65 + SG100 遥操作和 raw VLA 采集的实际
命令、热键、安全边界与数据语义。

## 1. What This Project Does

Apple Vision Pro 控制 RM65 六轴机械臂与 SG100 11-DoF 灵巧手；系统支持
simulation、受限真实硬件控制，以及相机/telemetry 在固定 30 Hz 时间轴上的 raw
VLA 数据采集。

## 2. Safety Status

> **存在 real backend 不表示 full-hand hardware 已验证，也不表示数据可直接训练。**

| 项目 | 当前状态 | 运行含义 |
|---|---|---|
| RM65 real | staged hardware validation 已有证据 | 仍需现场急停、tool profile、围栏和初始位检查 |
| SG100 real | restricted validation only | 不是 full-hand production ready |
| SG100 J6 / `index_j3` | VALIDATED | 当前 real allowlist 唯一允许的轴 |
| SG100 J7 | **QUARANTINED** | 不得加入 AVP real allowlist；command offset/zero equivalence 尚未解决 |
| SG100 `F_fb` | **PENDING** | MODE7 raw feedback 还不能解释为 canonical qpos |
| `training_ready` | **FALSE** | 当前 raw episode 继续隔离，不表示 recorder 损坏 |
| `dataset_status` | `RAW_CAPTURE_QUARANTINED` | 必须通过后续 mapping/hardware readiness gate |

真实硬件运行必须有人在物理急停旁，清空工作区并使用最低安全 profile。J7 的准确
结论是 active hold safety 未通过且 offset/zero equivalence unresolved，不是 sign
已被证伪。

## SG100 Command Safety

真实手位置命令的固定顺序是：AVP desired canonical 11D → canonical command
limit clamp → `hand_qcmd_effective_canonical_rad` → 单次 `F_cmd` → SDK
user-degree safety clamp/guard → `Hand_Control_t` → `SetHandControl()`。有限的越界
位置现在是 **CLAMP, NOT REJECT**；非有限值、反馈失效、mode 异常以及独立的
tracking spike 等安全故障仍然 fail closed。第二次 SDK-domain clamp 正常应为
no-op；若变化超过浮点比较 epsilon，会记录 `SDK_LIMIT_MAPPING_MISMATCH`。相同的
`JOINT_LIMIT_CLAMP` 只在进入 clamp 或请求值显著变化时报告，避免 500 Hz 刷屏。

命令边界不再复制 `sg100_joint_semantics.yaml` 中用于 morphology/retargeting 的
近似值，而是在启动时由实际加载的 SDK `joint_angle_map.json` user-degree ranges
和 validated `sg100_hardware_map.yaml` F_cmd 反推：

| joint | canonical min/max (rad) | SDK min/max (deg) | gain | offset (rad) |
|---|---:|---:|---:|---:|
| thumb_j1 | 0 / 2.181661565 | 0 / 125 | +1 | 0 |
| thumb_j2 | -2.617993878 / 0 | -150 / 0 | +1 | 0 |
| thumb_j3 | -1.047197551 / 1.570796327 | -60 / 90 | +1 | 0 |
| index_j1 | 0 / 1.570796327 | 0 / 90 | +1 | 0 |
| index_j2 | 0 / 2.181661565 | 0 / 125 | +1 | 0 |
| index_j3 | -1.047197551 / 1.570796327 | -60 / 90 | +1 | 0 |
| middle_j1 | -2.181661565 / 0 | 0 / 125 | **-1** | 0 |
| middle_j2 | -1.047197551 / 1.570796327 | -60 / 90 | +1 | 0 |
| little_j1 | -3.141592654 / 0 | -180 / 0 | +1 | 0 |
| little_j2 | 0 / 2.181661565 | 0 / 125 | +1 | 0 |
| little_j3 | -1.047197551 / 1.570796327 | -60 / 90 | +1 | 0 |

只有 `middle_j1/J7` 使用反号：canonical `[-125°,0°]` 经
`sdk=-canonical` 映射到 SDK `[0°,125°]`；`middle_j2/J8` 仍为 `gain=+1`。
例如 `thumb_j3=-61°` 和浮点边界 `-60.000447°` 最终都发送严格的 `-60°`，
但 desired/intent 字段仍保留原请求，训练 action 继续使用 final effective 字段。

当前 SDK 没有已验证的逐关节 0.5 A 运行时命令限制，因此本项目不把通用
`output_limit` 解释为安培，也不猜测或自动写入未注明单位的 Flash 参数。

## 3. Environment

```bash
cd ~/VisionProTeleop
source .venv/bin/activate
python -m pip install -e .
```

主 teleop 与 recorder 都推荐使用项目 `.venv`；本文命令显式写
`.venv/bin/python`，避免误用系统 Python。D455 `333422300281` 只是当前工作站
示例 serial，不是唯一合法设备。D405 到位后使用其实际 serial。

启动前确认 AVP 与工作站网络互通、calibration YAML 与 USDZ 存在、MuJoCo viewer
可启动。相机模式还应确认 `pyrealsense2` 能在同一个 `.venv` 中导入。

## 4. Architecture at a Glance

```text
AVP
 ├─ wrist pose → mapper/filter/IK → arm_q_goal_rad ──────────┐
 └─ hand tracking → retarget/filter → hand desired qcmd ────┤
                                                             ▼
                                                        TELEMETRY
                                                             ▼
D455 head ─┐                                      causal 30 Hz recorder
D405 wrist ┴─────────────────────────────────────────────────┤
                                                             ▼
                                                        NPZ + media

DESIRED / INTENT
       ↓
backend safety / limiter / allowlist
       ↓
EFFECTIVE / FINAL COMMAND
       ↓
hardware
       ↓
MEASURED / FEEDBACK
```

Control plane 与 data plane 相互独立。Recorder 只接收 UDP cache 和相机数据；它
不创建 robot backend、不发送 command，也不把 `On` 当成 robot enable。

## 5. Quick Start

IP `192.168.2.175` 是示例，必须替换为 Tracking Streamer 显示的实际 IP。Bash
多行命令中的反斜杠 `\` 必须是该行最后一个字符。

### A. AVP simulation only

需要 AVP，不需要 RealSense 或真实机器人；不会动真机。输出 MuJoCo/AVP 仿真，
并在 `recording/` 下产生 runner 自身的 lightweight simulation state recording。
只要 `--arm-backend real` 或 `--hand-backend real`，对应会话目录还会即时写入
`real_hardware_errors.jsonl`：逐条保存真机 safe-stop、追踪异常/超时、RM65
后台错误、操作门状态、未捕获异常和 traceback，并在正常退出时追加统计摘要。

```bash
cd ~/VisionProTeleop
.venv/bin/python sg100_teleop/sim/run_avp_arm_hand_sim.py \
  --ip 192.168.2.175 \
  --calibration sg100_teleop/config/avp_right_calibration.yaml \
  --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v8_compat.usdz \
  --wrist-roll-ik \
  --arm-backend sim \
  --hand-backend sim \
  --verbose
```

等待 valid tracking，在 MuJoCo viewer 按 `C` 建立 AVP relative anchor。`C` 不是
mechanical zero，也不修改 robot base axes。

### B. H1 — D455 camera-only recorder

需要 D455；不需要 AVP、RM65 或 SG100，不会动真机。输出 head RGB raw episode；
robot arrays 全 NaN、robot valid 全 False 是正确行为。

```bash
cd ~/VisionProTeleop
.venv/bin/python -m \
  sg100_teleop.data_collection.run_vla_raw_recorder \
  --telemetry optional \
  --required-cameras head \
  --camera-modalities rgb \
  --head-serial 333422300281 \
  --instruction "Validate D455 head camera raw capture" \
  --output-dir recording/vla_raw \
  --hz 30
```

在 recorder terminal 输入 `On` 开始，输入 `Off` 停止并检查 summary，最后输入
`Y` 保存或 `N` 丢弃。

### C. H2 — D455 + AVP simulation telemetry

需要 AVP + D455；RM65/SG100 backend 都是 sim，不会动真机。H2 验证 AVP desired
command chain、telemetry、相机与 dataset synchronization，不验证真实 hardware
effective/executed command。

Terminal 1 — AVP sim + telemetry：

```bash
cd ~/VisionProTeleop
.venv/bin/python sg100_teleop/sim/run_avp_arm_hand_sim.py \
  --ip 192.168.2.175 \
  --calibration sg100_teleop/config/avp_right_calibration.yaml \
  --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v8_compat.usdz \
  --wrist-roll-ik \
  --arm-backend sim \
  --hand-backend sim \
  --telemetry-publish 127.0.0.1:5560 \
  --telemetry-publish-hz 100 \
  --verbose
```

Terminal 2 — head RGB recorder：

```bash
cd ~/VisionProTeleop
.venv/bin/python -m \
  sg100_teleop.data_collection.run_vla_raw_recorder \
  --telemetry 127.0.0.1:5560 \
  --required-cameras head \
  --camera-modalities rgb \
  --head-serial 333422300281 \
  --instruction \
  "Validate AVP arm-hand command and D455 RGB time alignment" \
  --output-dir recording/vla_raw \
  --hz 30
```

推荐先启动 Terminal 2 使相机和 UDP subscriber READY，再启动 Terminal 1；UDP 是
best-effort。Terminal 1 tracking 稳定后按 `C`，等待 1–2 秒，再在 Terminal 2
输入 `On`。

预期 `robot_desired_qcmd_17d_valid` 接近 1。sim path 没有正式 real-backend
effective cache 时，`robot_qcmd_17d_valid` 可以为 0，effective17 保持 NaN。

### D. Future real RM65 + restricted SG100 collection

> **这不是 full-hand production 命令。只有完成现场 safety checklist 且明确决定进行
> 真实硬件验证时才可改成 real。启动仍不授权运动；`A` 才授权，`D` 只撤销。
> 当前 SG100 只允许 `index_j3`（J6），J7/full-11D 禁止。**

Terminal 1 受限模板（必须替换占位符并先核对 tool、急停、围栏与初始姿态）：

```bash
.venv/bin/python sg100_teleop/sim/run_avp_arm_hand_sim.py \
  --ip <AVP_IP> \
  --calibration sg100_teleop/config/avp_right_calibration.yaml \
  --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v8_compat.usdz \
  --wrist-roll-ik \
  --arm-backend real \
  --arm-ip <RM65_IP> \
  --arm-safety-profile 10 \
  --arm-tool-profile sg100_attached \
  --hand-backend real \
  --hand-port /dev/ttyUSB0 \
  --sg100-real-joints index_j3 \
  --telemetry-publish 127.0.0.1:5560 \
  --telemetry-publish-hz 100 \
  --verbose
```

Terminal 2 的未来 π0.5 recorder：

```bash
.venv/bin/python -m \
  sg100_teleop.data_collection.run_vla_raw_recorder \
  --telemetry 127.0.0.1:5560 \
  --required-cameras head,right_wrist \
  --camera-modalities rgb \
  --head-serial <D455_SERIAL> \
  --wrist-serial <D405_SERIAL> \
  --instruction "<TASK INSTRUCTION>" \
  --output-dir recording/vla_raw \
  --hz 30
```

即使 validation PASS，`F_fb` 未验证期间仍必须保持 `training_ready=False` 与
`RAW_CAPTURE_QUARANTINED`。

## 6. Standard Teleoperation Data Collection Procedure

### System startup

1. 清空工作区；真实运行确认物理急停、tool profile、围栏和 allowlist。非真实任务
   确保 backend 明确为 `sim`。
2. 启动 recorder，使 required cameras READY，并让 UDP subscriber 先绑定端口。
3. 启动 AVP runner；确认 backend、telemetry endpoint 和 requested max rate。
4. 确认 tracking stable；把 simulation/robot 放到 episode initial pose。

### Episode start/stop

1. Viewer 按 `C`，用下一帧 NEW + VALID tracking 建立/re-anchor AVP reference。
2. 等待 1–2 秒，确认 tracking、IK 和 desired command 稳定。
3. Recorder 输入 `On`；它只启动 dataset episode，不授权机器人。
4. 仅 real backend 且现场确认安全后，viewer 按 `A` 授权 active command。
5. 执行完整任务。
6. 任务完成后，real backend 按 `D` 撤销 authority；SG100 passive hold，RM65
   sender disable 并 slow-stop。`D` 永远不会 enable。
7. 等待 0.5–1 秒记录最终稳定状态，然后 recorder 输入 `Off`。
8. 查看 quality、rate、age、gap、writer error；输入 `Y` 保存或 `N` 丢弃。
   `Reset` 会丢弃 pending 并清 source rings/timing/validation state。

> **正式 training episode：`C → stable → On → A → task → D → stable → Off → Y`。**

Sim/H2 不需要 `A`/`D`。不要 `On → C`，因为 calibration/re-anchor 不应进入正式
task trajectory；正式 SOP 禁止在 episode 进行中按 `C`。`N`/`Reset` 都不消耗
final episode index。

**On/Off 不得与其他事件混淆**：`C` 不是录制开始，`A` 不是录制开始，`D` 不是
录制结束，`E` 不是正常录制结束。只有 `On`（被 state machine 接受）开始
episode；只有 `Off` 正常停止 episode。On 前允许 camera/telemetry warmup、AVP
tracking、`C` 标定，但 pre-On 阶段 0 dataset row、0 media enqueue；On 后第一个
正式 source sample 必须晚于 `episode_start_monotonic_ns`（pre-On ring 数据只能
以 invalid 出现，不静默使用）。Off 被接受后立即固定
`episode_stop_request_monotonic_ns`，此后禁止新 tick / row / media enqueue，
writer 只允许 flush Off 前已 enqueue 的内容。`A` 前少量 stationary state 与 `D`
后少量 final stationary state 都允许保留；`A`/`D` 不是 segment 边界。

## 7. Hotkey Reference

| 输入 | 位置 | 实际语义 |
|---|---|---|
| `C` | MuJoCo viewer | AVP relative anchor/re-anchor；不是 mechanical zero |
| `A` | MuJoCo viewer | real backend ARM/DISARM toggle |
| `D` | MuJoCo viewer | real backend **DISARM-only**；不会 enable，RM65 slow-stop、SG100 passive hold |
| `E` | MuJoCo viewer | restricted SG100 software E-stop latch + MODE_NULL；不能替代物理急停，也不是 RM65 E-stop |
| `On` | recorder | READY→RECORDING；不是 robot enable |
| `Off` | recorder | 停止、flush、validate |
| `Y` | recorder | 保存 pending，随后 index +1 |
| `N` | recorder | 丢弃 pending，sources 保持 warm，index 不变 |
| `Reset` | recorder | 丢弃并清 source rings/timing/validation，index 不变 |

### Reserved Safety / Control Keys

`C` / `A` / `D` / `E` 属于现有 teleop authority/safety contract，语义固定：

- `C`：AVP relative anchor/re-anchor（calibration）；
- `A`：real backend ARM/DISARM toggle（E-stop latched 时无法 arm）；
- `D`：**DISARM-only**，不是 toggle；`D`×2 仍 disarmed，只有 `A` 重新 armed；
- `E`：SG100 software E-stop latch；`D`/`A` 都不能清除 `E`。

后续任何新功能（包括夹爪形态/morphology switching）**禁止复用这四个键**，也
禁止改变它们的语义。

## 8. NPZ Data Semantics

- **DESIRED / INTENT**：AVP/policy/retargeter 想执行什么，尚未经过 backend safety。
- **EFFECTIVE / FINAL COMMAND**：经过 limiter/allowlist/safety 后真正提交到 backend
  或 hardware 边界的 command。
- **MEASURED / FEEDBACK**：backend 的真实反馈；不得由 desired、qcmd 或 MuJoCo
  qpos 冒充。

| Field | Shape | Unit | Layer | Meaning | Training use | Valid condition |
|---|---:|---|---|---|---|---|
| `arm_q_goal_rad` | `(N,6)` | rad | DESIRED | AVP wrist mapping + IK 的 absolute RM65 target | provenance/debug | telemetry fresh、tracking valid/anchored、finite |
| `arm_qcmd_sent_rad` | `(N,6)` | rad | EFFECTIVE | continuity/safety limiter 与 sender timing 后成功提交给 backend/CANFD 的 final command | future real action component | `arm_command_valid` |
| `arm_qpos_rad` | `(N,6)` | rad | MEASURED | RM65 backend feedback；不是 goal 或上一帧 qcmd | future real state component | `arm_feedback_valid` |
| `hand_qcmd_retargeted_canonical_rad` | `(N,11)` | rad | DESIRED | AVP hand → canonical retargeter → `JointCommandFilter` 后 target | provenance/debug | telemetry fresh、tracking valid/anchored、finite |
| `hand_qcmd_effective_canonical_rad` | `(N,11)` | rad | EFFECTIVE | restricted safety/allowlist/slew 后 final canonical command | future real action component | `hand_command_valid` |
| `hand_qcmd_sdk_deg` | `(N,11)` | deg | WIRE/SDK | `F_cmd` + rad→degree 后送到 SG100 SDK 边界的数据 | wire debug | `hand_command_valid` |
| `hand_feedback_sdk_rad` | `(N,11)` | rad | RAW FEEDBACK | MODE7 feedback 基础 degree→rad；**不是 canonical** | SG100 raw debug | `hand_feedback_valid` |
| `hand_qpos_canonical_rad` | `(N,11)` | rad | MEASURED CANONICAL | 只有 validated `F_fb` 后才成立 | future real state | `hand_feedback_canonical_valid` |
| `robot_desired_qcmd_17d_rad` | `(N,17)` | rad | DESIRED | arm goal + hand retargeted target | intent/debug/H2 | `robot_desired_qcmd_17d_valid` |
| `robot_qcmd_17d_rad` | `(N,17)` | rad | EFFECTIVE | arm sent + hand effective；**绝不是 desired alias** | validated real action | `robot_qcmd_17d_valid` |
| `robot_qpos_17d_rad` | `(N,17)` | rad | MEASURED | RM65 feedback + canonical SG100 feedback | validated real state | `robot_qpos_17d_valid` |

Desired17 只有 arm/hand desired 都来自 fresh、valid、anchored telemetry 且全部
finite 时才 valid；否则整行 NaN，不 zero/forward/previous-fill。Effective17 只拼接
effective components，缺失时绝不回退到 desired。Limiter 存在时 desired 与
effective 不同是正常行为。Episode 级 provenance 字符串为：

```text
robot_desired_qcmd_semantics
robot_effective_qcmd_semantics
robot_qpos_semantics
```

### Which field should I use?

> - Debug AVP intent：`robot_desired_qcmd_17d_rad`
> - Validated real demonstration training action：`robot_qcmd_17d_rad`
> - Training robot state：`robot_qpos_17d_rad`
> - SG100 raw SDK debugging：`hand_feedback_sdk_rad`
> - `F_fb` 验证前，**绝不能**把 raw MODE7 feedback 当作 canonical state。

当前 π0.5 目标 contract：observation = head RGB + right wrist RGB +
`robot_qpos_17d_rad`；action = `robot_qcmd_17d_rad`；language = episode
instruction。这是 future training-ready real episode 的目标。若未来决定用 desired
action，必须形成显式 dataset policy，不能静默切换。

## 9. 17D Ordering

`robot_joint_names` 是 ordering authority：

```text
 0 rm65_joint1
 1 rm65_joint2
 2 rm65_joint3
 3 rm65_joint4
 4 rm65_joint5
 5 rm65_joint6
 6 thumb_j1
 7 thumb_j2
 8 thumb_j3
 9 index_j1
10 index_j2
11 index_j3
12 middle_j1
13 middle_j2
14 little_j1
15 little_j2
16 little_j3
```

## 10. Camera Modalities

正式 π0.5 默认 `--camera-modalities rgb`：head D455 RGB + right wrist D405 RGB。
Depth 不直接进入当前 π0.5 image encoder，但能力保留；研发/几何 debug 使用
`--camera-modalities rgb,depth`，保存 raw aligned Z16 uint16 PNG。

`--required-cameras head` 是 D455 bring-up；production 默认仍为
`head,right_wrist`。RGB-only 不创建 depth directory，NPZ depth 字段写 disabled
sentinel。详见 `data_collection/VLA_RAW_DATA_COLLECTION.md`。

## 11. Dataset Clock and Telemetry

**CONFIGURED RATE / OBSERVED RATE / DATASET RATE 三者绝对不能混写。**
configured 是配置请求值，observed 必须来自 RAW 证据（sequence/timestamp
推导），dataset 是 30 Hz master timeline。

Frequency map（configured 值；observed 以 RAW 为准）：

| Source | CONFIGURED RATE | OBSERVED RATE 依据 | DATASET RATE |
|---|---|---|---|
| AVP tracking | ~60 Hz（`--tracking-hz`） | caller/network 实际率可变化；查 RAW | 30 Hz causal sample |
| Arm IK / target update | ~tracking rate | 随 AVP tracking | — |
| RM65 sender（low / filter / high profile） | 100 / 125 / 200 Hz | RAW `arm_qcmd_sequence` + source ts 推导 | 30 Hz causal sample |
| RM65 feedback（TCP polling / UDP realtime） | ~50 Hz / 配置最高 ~200 Hz | actual 必须 per-episode 用 RAW 实测 | 30 Hz causal sample |
| SG100 desired hand update | ~AVP/retarget rate | 随 AVP tracking | 30 Hz causal sample |
| SG100 vendor scheduler | ~100 Hz cached retransmission | **scheduler 100 Hz ≠ 100 Hz unique action** | 30 Hz causal sample |
| SG100 feedback | historically ~100 Hz | actual episode 必须用 RAW observed 数据 | 30 Hz causal sample |
| D455 / D405 | 30 Hz | RAW frame number/host ts | 30 Hz causal sample |
| External telemetry | requested 例如 100 Hz | caller-loop limited；H2 实测 ~50.8 Hz | 30 Hz causal sample |
| Dataset | — | — | 固定 30 Hz |

30 Hz sampler 每个 tick 选择 `source_timestamp <= dataset_tick` 且
`>= episode_start`（On 边界）的最新 causal sample，并记录 timestamp、age、
sequence、valid；不同 source timestamp 不会完全相同。pre-On 产生的 source 只能
以 invalid 出现在 tick 0，不进入正式样本。

`--telemetry-publish-hz 100` 是 **requested maximum / throttle target**。Publisher
没有独立定时线程，而由 runner main loop 达到 period 时调用，所以 actual rate 受
caller loop 限制。质量应查看：

```text
telemetry_effective_hz
telemetry_age_mean_ms / p95 / p99 / max
telemetry sequence gaps
```

Effective Hz 使用 UDP publish sequence 与 `publish_monotonic_ns` 推导；即使 30 Hz
sampler 跳过中间 packet，也用 sequence delta 估计 publish-call rate。Episode 保存
`telemetry_publish_hz_requested` 与 `telemetry_publish_hz_effective`。实测 H2 v2：
requested 100 Hz、actual 约 50.82 Hz；无需为追求 100 改造 control thread。

### Native rate 与 30 Hz dataset 的关系

**RAW = native rate evidence；CURATED = physical-time aligned training
trajectory。** 30 Hz causal sampling 会跳过中间 native event，因此 Curated
**无法**完整恢复 100/125/200 Hz RM65 command、50/200 Hz RM65 feedback、~100 Hz
SG100 feedback 等 native event rate。Inspector 对 RAW 输出
sequence-delta 估计的 native observed Hz（证据不足时明确
`NOT EXACTLY OBSERVABLE`），对 Curated 明确打印：

```text
NATIVE EVENT RATE: NOT EXACTLY RECOVERABLE FROM CURATED DATA
```

禁止用 30 Hz dataset tick 冒充 native Hz，也禁止为 Curated 恢复 native rate
而重新加入 sequence 字段。

## 12. Dataset Status Metadata

| Field | Meaning |
|---|---|
| `hardware_execution` | 当前 passive recorder 固定写 False；没有可信 backend identity 时绝不从图像/telemetry 猜成 True |
| `robot_telemetry_present` | episode 开始时 publisher 是否 alive |
| `training_ready` | 是否通过完整 training contract gate |
| `dataset_status` | 当前隔离/发布状态；现为 `RAW_CAPTURE_QUARANTINED` |

Camera-only 中 robot arrays NaN 是预期。`training_ready=False` 可能只是 `F_fb`
尚未验证，不代表 recorder broken。

## 13. Directory Layout and Inspection

```text
recording/vla_raw/                 # RAW：完整原始采集证据（不可变）
  episode_000000.npz
  episode_000000_media/
    head/rgb/000000.jpg
    right_wrist/rgb/000000.jpg   # 仅启用 wrist 时

recording/vla_curated/             # CURATED：清洗后的 30Hz 训练轨迹
  episode_000000/
    metadata.json
    trajectory.npz
    cleaning_report.json
    media/head/rgb/000000.jpg
```

RGB-D 在启用 role 下另有 `depth/000000.png`（仅 RAW）。只有 `Y` 后 pending 才
永久保存；curated 由 builder 独立生成。目录、字段与训练就绪状态的完整契约见
第 17 节「π0.5-AHFE Training Data Contract」。

```bash
.venv/bin/python -m \
  sg100_teleop.data_collection.inspect_vla_episode \
  recording/vla_raw/episode_000001.npz

.venv/bin/python -m \
  sg100_teleop.data_collection.build_curated_episode \
  recording/vla_raw/episode_000001.npz \
  --output-dir recording/vla_curated

.venv/bin/python -m \
  sg100_teleop.data_collection.inspect_vla_episode \
  recording/vla_curated/episode_000001
```

RAW inspector 输出 schema、N、duration/Hz、camera、telemetry rate/age、
desired / effective / qpos 17D valid ratio、native Hz、arm/hand skew、
desired/effective 差异、training status 与 validator result；curated 目录或
trajectory.npz 会被自动识别并输出 transition 级派生 timing 指标。

## 14. Operator Checklist

### PRE-RUN

- [ ] correct Python environment (`~/VisionProTeleop/.venv`)
- [ ] D455 detected
- [ ] D405 detected if required
- [ ] telemetry endpoint available
- [ ] backend is SIM unless real hardware intentionally requested
- [ ] physical E-stop/workspace/tool profile verified for real run
- [ ] SG100 real allowlist is exactly validated set; no J7
- [ ] recording output directory and next index checked

### PRE-EPISODE

- [ ] task initial pose ready
- [ ] AVP tracking stable
- [ ] press `C`
- [ ] calibration/IK/desired stable for 1–2 s
- [ ] recorder `On`
- [ ] press `A` only if real authority is intentionally required

### POST-EPISODE

- [ ] task finished
- [ ] press `D` if real authority was active
- [ ] robot stable for 0.5–1 s
- [ ] recorder `Off`
- [ ] inspect quality, telemetry rate/age/gaps and valid ratios
- [ ] `Y` save or `N` discard

## 15. Troubleshooting

### `rs-enumerate-devices: command not found`

先运行 `command -v rs-enumerate-devices`，再检查 `/data/realsense/install/bin` 是否
在 PATH。不要因为 PATH 问题直接重装 SDK。

### `pyrealsense2` import error

确认使用 `.venv/bin/python`，检查 `PYTHONPATH`、模块安装位置与 Python ABI。

### argparse: `unrecognized arguments: --telemetry-publish`

该参数属于 teleop runner，不属于 recorder；同时检查 Bash `\` 后是否有空格或又
跟参数。错误写法如 `--verbose \ --telemetry-publish ...`。

### Camera-only 的 robot valid ratio 为 0

EXPECTED。`--telemetry optional` 要求 robot arrays NaN、valid False。

### H2 的 effective17 全 NaN

Sim backend 没有正式 effective cache 时 EXPECTED。看 desired17；禁止
desired→effective fallback。

### Hand canonical qpos invalid

`F_fb` pending 时 EXPECTED。可检查 raw `hand_feedback_sdk_rad`，但不能用于
canonical training state。

### RGB-only 缺少 depth

EXPECTED。RGB-only 不启动 depth stream，也不创建 depth media。

### Requested telemetry 100 Hz，actual 只有 50–60 Hz

Publisher 被 caller loop 限制。查看 effective Hz、age 和 gaps；30 Hz dataset 不
要求另建 100 Hz control thread。

### Recorder `On` 被拒绝

检查 required camera READY；非 `telemetry optional` 还要求 alive publisher。
Production 默认要求 head + right wrist；D405 未到位时显式使用 head-only。

### Validator FAIL

不要直接 `Y`。查看 future timestamp、media count、writer drop/error、schema
component mismatch 和 sentinel；可用只读 inspector 复查。

## 16. Safety Warnings and References

- `C` 是 AVP reference anchor，不是 mechanical zero，也不授予 real command。
- `On`/`Off` 只控制 dataset episode。
- `A` 是 real authority toggle；`D` 是 disarm-only。
- `E` 是 restricted SG100 software latch，不能代替物理急停。
- J7/full-hand real AVP control 继续阻断，不得扩大 `index_j3` allowlist。
- `F_fb` 验证前，real SG100 canonical qpos 与完整 training dataset 不可发布。
- 不创建自动 `C`、自动 `A` 或自动 real motion 的一键脚本。

深入依据：

- `data_collection/VLA_RAW_DATA_COLLECTION.md`：raw VLA schema/validator contract
- `RM65_real_进度与测试指南.md`：RM65 staged hardware 状态与现场前置条件
- `SG100_REAL_HARDWARE_VALIDATION.md`：SG100 restricted validation evidence
- `config/sg100_joint_semantics.yaml`：canonical SG100 ordering
- `config/avp_to_tcp_identity.yaml`：AVP→TCP basis/filter

## 17. π0.5-AHFE Training Data Contract

本节是训练数据的 **authoritative index**：目录、训练字段、字段语义、数据就绪
状态都以此为准。任何修改 raw/curated/training 目录、训练字段、state/action
语义、17D ordering 或 camera modality 的代码变更，必须同步更新本节与
`data_collection/VLA_RAW_DATA_COLLECTION.md`。README 是 operator 与 training
engineer 的第一入口。

### Data Used by π0.5-AHFE

```text
RAW
recording/vla_raw/
│
│  full audit / replay provenance（不可变原始证据）
│
▼
CLEANING
│  build_curated_episode：validation + cleaning + causal transition build
▼
CURATED
recording/vla_curated/
│
│  Observation: head RGB + right wrist RGB + robot_qpos_17d_rad
│  Action:      robot_qcmd_17d_rad
│  Language:    language_instruction（metadata.json）
│
▼
ADAPTER — NOT IMPLEMENTED YET
│
▼
π0.5-AHFE DATASET
datasets/pi05_ahfe/ — NOT IMPLEMENTED YET
```

### Training Data Index

| Layer | Directory | Status |
|---|---|---|
| Raw source | `recording/vla_raw/` | IMPLEMENTED（schema v1–v3） |
| Cleaning / Curated | `recording/vla_curated/` | IMPLEMENTED（curated v1，FINALIZED） |
| π0.5-AHFE Adapter | — | NOT IMPLEMENTED YET |
| Model-ready dataset | `datasets/pi05_ahfe/` | NOT GENERATED YET |

- **RAW = 完整原始采集证据**：recorder 落盘的全部 native 字段与 media，包括
  telemetry/sequence/age/writer diagnostics 与 depth（可选）。RAW 只增不改，
  是 replay/audit 的唯一追溯源。
- **CURATED = 清洗后数据**：经 validation、cleaning、causal transition build
  的 30 Hz 训练轨迹；只含 training-valid transition（见下文 layout）。
- **π0.5-AHFE dataset = model-ready**：未来由 Adapter 从 Curated 生成，直接供
  π0.5-AHFE / OpenPI training。当前不保存任何 depth、SDK 域或 telemetry
  diagnostics。

### 当前训练字段（contract）

Observation：

```text
{
  head_rgb,               # D455 RGB
  right_wrist_rgb,        # D405 RGB
  robot_qpos_17d_rad,     # observation.state
  language_instruction,   # episode 级任务指令
}
```

Target/Action：

```text
robot_qcmd_17d_rad
```

- **STATE = REAL MEASURED CANONICAL STATE**：RM65 real feedback + SG100 real
  feedback 经 validated `F_fb` 得到的 canonical state。
- **ACTION = FINAL EFFECTIVE BACKEND COMMAND**：`arm_qcmd_sent_rad` +
  `hand_qcmd_effective_canonical_rad` 的 17D 拼接。**禁止**默认用
  `robot_desired_qcmd_17d_rad`（intent/provenance）替代 action。
- `robot_desired_qcmd_17d_rad` 不进入 π0.5-AHFE 训练；RAW 必须保留，Curated
  第一版可选保留（`--include-desired`）。

### AHFE Tensor Contract

```text
Observation state: (17,)
Arm slice:  [0:6]      # robot_qpos_17d_rad[..., 0:6]
Hand slice: [6:17]     # robot_qpos_17d_rad[..., 6:17]

Action: (17,)
AHFE arm target: action[..., 0:6]
AHFE hand target: action[..., 6:17]

Action horizon: 50（以最终 OpenPI π0.5 config 为准）
Adapter 输出: (50, 17)
AHFE 运行时切片: arm (50, 6) / hand (50, 11)
```

AHFE Arm/Hand factorization 由以上 slice（代码常量 `ARM_SLICE=(0,6)`、
`HAND_SLICE=(6,17)`）确定性完成；**不**在 Curated 或 model-ready dataset 中
同时保存 17D 与拆分的 arm 6D / hand 11D 数组，也不为 AHFE 在磁盘重复保存
三份 tensor——Adapter 运行时切片即可。

### Curated v1 Layout

```text
recording/vla_curated/episode_XXXXXX/
  metadata.json         # 训练契约元数据（一次性，不逐 tick 重复）
  trajectory.npz        # 仅 accepted transition 的核心数组
  media/
    head/rgb/000000.jpg         # 密集 0..M-1，与 trajectory 行对齐
    right_wrist/rgb/000000.jpg  # 仅 dual-camera source
  cleaning_report.json  # 拒绝统计 / segment / readiness blockers
```

`metadata.json`：`episode_id`、`language_instruction`、`dataset_hz`、
`joint_names[17]`、`arm_slice`、`hand_slice`、`camera_roles`、
`schema_name/schema_version`（curated）、`source_schema_name/version`、
`source_episode_path`、`hardware_execution`、`training_ready`、
`dataset_status`、`action_representation`、`action_unit`、
`state_representation`、`state_unit`、`segment_offsets`。

`trajectory.npz`（每行一个 causal transition `s_k, a_k, s_{k+1}`，v1 FINAL）：

| Field | Meaning |
|---|---|
| `source_tick_index` / `next_state_tick_index` | RAW tick 溯源（next=source+1） |
| `state_timestamp_ns` | `max(arm_qpos_source, hand_qpos_source)`：unified state 时刻 |
| `action_timestamp_ns` | `max(arm_qcmd_source, hand_qcmd_source)`：unified action 时刻 |
| `next_state_timestamp_ns` | `max(next arm qpos ts, next hand qpos ts)`（从 RAW k+1 行取，不落盘 next 组件数组） |
| `arm/hand_qpos_source_timestamp_ns` | state 两侧 native feedback 时刻（唯一 skew 依据） |
| `arm/hand_qcmd_source_timestamp_ns` | action 两侧 native command 时刻 |
| `robot_qpos_17d_rad` / `robot_qcmd_17d_rad` | 唯一 state/action 存储（rad） |
| `head_rgb_frame_index` / `right_wrist_rgb_frame_index` | curated media index；允许重复引用同一 media（RGB 去重），未启用 wrist 时恒 -1 |
| `robot_desired_qcmd_17d_rad` | OPTIONAL provenance（`--include-desired`） |

### Component-wise causality（v1 freeze 核心）

transition 接受不允许只看 unified `state <= action < next_state`；Cleaner 必须对
每侧组件分别验证：

```text
ARM:  arm_qpos_ts[k]  <= arm_qcmd_ts[k]  < arm_qpos_ts[next_state_tick]
HAND: hand_qpos_ts[k] <= hand_qcmd_ts[k] < hand_qpos_ts[next_state_tick]
```

ARM causal PASS **AND** HAND causal PASS 才接受该 transition。next 侧组件
timestamp 通过 `next_state_tick_index` 从 RAW 读取，验证后**不**在 Curated 重复
保存。unified timestamp（三者的 max）是 trajectory 的展示/对齐层，不替代
component-wise causality。有效 action 不存在时 transition invalid，**绝不**
fallback 到 desired。

### Segment 与 action horizon

`metadata.segment_offsets`（如 `[0, 101, 300]`）：segment 0 = rows `[0,101)`，
segment 1 = rows `[101,300)`；首项必须 0、末项必须 N、严格递增。同一 segment 内
相邻 rows 必须满足 `source_tick_index[k+1] == next_state_tick_index[k]`，否则切
segment——π0.5-AHFE Adapter 取 `actions[i:i+50]` 时**禁止跨 segment**，否则
horizon 会把 tracking loss / telemetry interruption / invalid feedback 等两侧
不同连续物理轨迹拼在一起。

切分规则（使用 RAW 已可观察信号）：

- tick 不连续（被拒 transition 造成）→ SPLIT；
- `estop_latched` → 结束当前 segment（E-stop：SPLIT，不删除有效数据）；
- `tracking_valid` / `telemetry_valid` 丢失 → SPLIT；
- `A` / `D` **不是** segment 边界（`A` 前与 `D` 后的少量 stationary state 保留）。

**Known limitation**：RAW 当前没有显式记录 AVP re-anchor（`C`）generation
event。Curated v1 不为此新增 event schema；正式 SOP 禁止在 episode 中按 `C`
（标准流程 `C → stable → On`），正常 training episode 本就不应包含 re-anchor。

`cleaning_report.json`：`source_episode_path`、`raw_tick_count`、
`candidate/accepted/rejected_transition_count`、`invalid_reason_counts`（bit 原因
统计，含 `ARM/HAND/UNIFIED_CAUSALITY_INVALID`）、`rejected_tick_ranges`、
`segment_count`、`segment_offsets`、`segment_event_split_count`、
`time_range_*`、`training_blockers`、`quality_summary`。Curated 主 trajectory
**不**保存 `transition_valid` / `invalid_reason_mask`——存在于 Curated 即
valid；被拒 transition 的明细回 RAW 查。

### Which Data Is Used?

| Field | Raw | Curated | π0.5-AHFE | Purpose |
|---|---|---|---|---|
| `robot_qpos_17d_rad` | YES | YES | YES | observation.state（measured） |
| `robot_qcmd_17d_rad` | YES | YES | YES | action（effective backend command） |
| `head/wrist RGB media` | YES | YES（accepted only） | YES | observation.image |
| `language_instruction` | YES | YES | YES | language conditioning |
| 4×native source timestamps | YES | YES | NO | timing/causality 审计 |
| `robot_desired_qcmd_17d_rad` | YES | OPTIONAL | NO | provenance / safety audit |
| `arm/hand` 拆分数组（6D/11D） | YES（components） | NO | runtime slice | AHFE factorization |
| `hand_qcmd_sdk_deg` | YES | NO | NO | SDK wire debug |
| `hand_feedback_sdk_rad` / `hand_feedback_modes` | YES | NO | NO | SG100 raw feedback debug |
| telemetry sequence/age/rate | YES | NO | NO | 链路诊断 |
| writer backlog/drop、sampler jitter | YES | NO | NO | runtime diagnostics |
| camera device ts/frame number/age | YES | NO | NO | RAW validation |
| Depth | optional | NO | NO | RGB-only baseline |
| derived skew/dt/Hz/norm 数组 | NO | NO | NO | inspector 运行时计算 |

### 存储精简：不保存，运行时推导

以下派生量**不落盘**，由 `inspect_vla_episode` 从保留 timestamp/sequence 实时
计算并输出 mean/p50/p95/p99/max：

```text
arm_hand_state_skew_ns  = abs(arm_qpos_source_ts - hand_qpos_source_ts)
arm_hand_action_skew_ns = abs(arm_qcmd_source_ts - hand_qcmd_source_ts)
state_action_dt_ns      = action_timestamp_ns - state_timestamp_ns
action_next_state_dt_ns = next_state_timestamp_ns - action_timestamp_ns
desired/effective L2 与 max abs（rad）
arm command / arm feedback / hand command / hand feedback native Hz（RAW only）
```

**当前不设最终 skew rejection threshold**：还没有完整真实 RM65+SG100
training-ready episode，10 ms / 20 ms 之类的阈值没有依据。v1 只计算、报告并
保存 source timestamp；第一批 real episode 后再按真实分布决定阈值。

`timestamp_relative_ns`、desired/effective norm、重复 arm/hand 数组、
replay 专用文件等同样不进入 Curated（完整 NO SAVE 列表见
`vla_curated_schema.FORBIDDEN_TRAJECTORY_KEYS`，并有测试守护）。

### Training Readiness（以当前代码与实际数据为准）

| Episode / Phase | head RGB | wrist RGB | Desired17 | Effective17 | Measured17 | Status |
|---|---|---|---|---|---|---|
| `episode_000000`（H1 D455-only） | YES | NO | NO | NO | NO | NOT READY |
| `episode_000001`（H2 AVP sim，v2） | YES | NO | components YES；unified v3 field ABSENT | NO | NO | NOT READY |
| Real Arm-Hand（未采集） | TBD | TBD | TBD | TBD | TBD | NOT READY until validated |

说明：H2 episode_000001 的 arm/hand desired **组件字段存在**（`arm_q_goal_rad`、
`hand_qcmd_retargeted_canonical_rad`），但它是 schema v2，缺 v3 统一字段
`robot_desired_qcmd_17d_rad`——不是"H2 没有 desired"。有效 desired17 需要 v3
recorder 重采。

当前 blocker（全部 episode 均 NOT TRAINING READY 的原因）：

1. SG100 `F_fb = PENDING` → `hand_qpos_canonical_rad` 无法成立 →
   `robot_qpos_17d_valid=0` → Curated cleaning 拒绝全部 transition；
2. `hardware_execution=False`（无可信 backend attestation 前 conservatively）；
3. H1/H2 episode 为 head-only、v1/v2 schema，且 effective17 无 real-backend
   cache 来源（sim path 无 effective cache 是预期行为）；
4. D405 wrist 数据尚未采集。

NPZ 能成功生成不代表可训练；上表只在对应条件真实满足后更新。Real Arm-Hand
在 `F_fb` validated + 完整 real chain 验证前保持 NOT TRAINING READY。

### Replay 导出（软嵌）

Curated 只保存 17D，replay 轨迹由 exporter 临时切片，不产生 permanent
duplicate：

```bash
.venv/bin/python -m \
  sg100_teleop.data_collection.export_replay_commands \
  recording/vla_curated/episode_000001/trajectory.npz \
  --target hand \
  --output hand_qcmd_canonical_rad.npy
```

`--target {arm,hand,arm_hand}` 切 `(N,6)/(N,11)/(N,17)`，默认
`--source effective`（hardware executed 语义）；`--source desired` 导出 AVP
intent 时，文件名/sidecar metadata 强制标记 desired，防止软嵌误认。输出
`.npy` 附带 `.json` sidecar，`command_semantics` 字段使用精确语义标签：

```text
effective → command_semantics: FINAL_EFFECTIVE_BACKEND_COMMAND
desired   → command_semantics: DESIRED_INTENT_NOT_HARDWARE_EXECUTED
```

exporter 只导出文件，不驱动硬件；真实 replay 必须经过既有 backend safety /
limiter / allowlist / hardware interface（SG100：canonical rad → safety /
allowlist → `F_cmd` → rad→deg → `SetHandControl`；J7 限制继续保持）。

Curated media 采用 frame index reference + 去重：多个 accepted transition 引用
同一 RAW RGB source frame 时只保存一份图像，`head_rgb_frame_index` /
`right_wrist_rgb_frame_index` 允许重复引用同一 curated media index。

### Native timing 与 native event log

30 Hz Curated trajectory **不是** native robot event log。100/125/200 Hz RM65
command、RM65 feedback、~60 Hz hand desired、~100 Hz SG100 feedback 等 native
event 的完整证据只存在于 RAW（每 source 的 timestamp/sequence/payload）；
Curated 只保留 30 Hz transition 级的 4 个 native source timestamp，因此：

```text
NATIVE EVENT RATE: NOT EXACTLY RECOVERABLE FROM CURATED DATA
```

（中间 native event 可能已被 30 Hz sampler 跳过；native Hz 只能从 RAW 的完整
sequence/timestamp 证据估计，且不向 Curated 增加 sequence 字段。）Curated 提供
的是 physical-time 对齐的训练轨迹与 state/action skew 分布。若未来需要独立
native event stream 文件（每 source 只存 `timestamp_ns/sequence/payload`），
必须与 30 Hz Curated 分开设计，不得扩张进每行 transition。

### Recording boundary 与安全语义（不变）

- 只有 `On` 被 state machine 接受后才开始 dataset tick / RGB enqueue /
  action-state row；`Off` 接受后立即停止创建新 row/enqueue，writer 只 flush
  Off 前已 enqueue 的内容。
- `D` 是 **DISARM-only**（不是 toggle，`D×2` 仍 disarmed），只有 `A` 重新
  armed；`E` 是 software E-stop latch，`D`/`A` 不能清除 `E`。

### 变更纪律

修改以下任何一项的 PR 必须同步更新本节：raw/curated/training 目录、训练字段、
state/action 语义、17D ordering、camera modality、AHFE slice、readiness 表。
