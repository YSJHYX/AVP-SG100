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
task trajectory。`N`/`Reset` 都不消耗 final episode index。

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

正式 π0.5 推荐 `--camera-modalities rgb`：head D455 RGB + right wrist D405 RGB。
Depth 不直接进入当前 π0.5 image encoder，但能力保留；研发/几何 debug 使用
`--camera-modalities rgb,depth`，保存 raw aligned Z16 uint16 PNG。

`--required-cameras head` 是 D455 bring-up；production 默认仍为
`head,right_wrist`。RGB-only 不创建 depth directory，NPZ depth 字段写 disabled
sentinel。详见 `data_collection/VLA_RAW_DATA_COLLECTION.md`。

## 11. Dataset Clock and Telemetry

| Source | 典型/配置频率 | 语义 |
|---|---:|---|
| AVP tracking | 约 60 Hz | caller/network 实际率可变化 |
| RM65 command | 100/125/200 Hz，依 backend/profile | real sender 自有时钟 |
| SG100 vendor scheduler | 约 100 Hz | retransmit cached SDK command |
| RealSense | 约 30 Hz | device clock + host receive time |
| External telemetry | caller-loop limited | requested max，不是 hard realtime |
| Dataset | 固定 30 Hz | learning/data master clock |

30 Hz sampler 每个 tick 选择 `source_timestamp <= dataset_tick` 的最新 causal
sample，并记录 timestamp、age、sequence、valid；不同 source timestamp 不会完全
相同。

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
recording/vla_raw/
  episode_000000.npz
  episode_000000_media/
    head/rgb/000000.jpg
    right_wrist/rgb/000000.jpg   # 仅启用 wrist 时
```

RGB-D 在启用 role 下另有 `depth/000000.png`。只有 `Y` 后 pending 才永久保存。

```bash
.venv/bin/python -m \
  sg100_teleop.data_collection.inspect_vla_episode \
  recording/vla_raw/episode_000001.npz
```

该只读工具输出 schema、N、duration/Hz、camera、telemetry rate/age、desired /
effective / qpos 17D valid ratio、training status 与 validator result。

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
