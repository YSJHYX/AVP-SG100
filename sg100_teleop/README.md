# SG100 + RM65 + Apple Vision Pro Teleoperation

本子项目位于 `sg100_teleop/`,在 VisionProTeleop 仓库内。

当前目标是完整验证以下 tabletop teleoperation 链路:

```text
Apple Vision Pro
    ↓ real-time wrist + hand tracking
RM65 TCP translation/orientation + SG100 natural finger motion
    ↓
MuJoCo robot-only / tabletop scene
    ↓
manual approach, grasp and object manipulation
```

目标交互包括 open、fist、pinch、opposition、arm-hand simultaneous teleoperation 和人工接近桌面物体。当前项目验证的是仿真遥操作基础链路,不是 MoveIt planner、VLA data collection pipeline、Manus 正式数据采集、π0.5 training 或 autonomous scripted grasp controller。

本 README 面向新加入项目、但未参与之前 SG100 URDF / MuJoCo / retargeting / RM65 IK 调试的工程人员。所有命令、路径、参数、默认值、joint 顺序、验证状态均以当前实际源码为准。

---

## 0. TL;DR

**当前阶段:**

- ✅ 已实现并单元测试:AVP hand skeleton → canonical semantic hand state → SG100 retargeting → MuJoCo position control
- ✅ 已实现并单元测试:AVP 几何特征提取的 rigid-transform / scale 不变性
- ✅ 已实现并单元测试:多帧中位数标定 + 异常帧鲁棒性 + YAML 序列化
- ✅ 已实现并验证:RM65 + SG100 17DoF combined MuJoCo 模型(`nq=nv=nu=17`)
- ✅ 已实现并验证:RM65 teleop TCP 定义、FK、analytical Jacobian、DLS IK
- ✅ 已实现并验证:AVP wrist relative-pose mapper(synthetic)
- ✅ 已实现并验证:Fake AVP arm teleop、Fake AVP 17DoF arm-hand teleop、continuous TCP teleop regression
- ✅ 已实现:tracking freshness/right-hand validity、显式 neutral arming、configurable AVP→TCP local basis
- ✅ 已验证:production-format `TrackingData → q17 → robot-only/tabletop MuJoCo` 完整 synthetic integration
- ✅ 已验证:SG100 11/11 direct physical kinematics 与 11/11 canonical-to-physical semantics,无 sign mismatch
- ✅ 已实现:`run_avp_arm_hand_sim.py`(真实 AVP → RM65 + SG100 combined 主入口,CLI / architecture / synthetic integration 已验证)
- ⚠️ 尚未完成:**真实 Apple Vision Pro 端到端验证**(live tracking、wrist axis mapping、arm/hand/combined live teleop)
- ⚠️ 尚未完成:真实 AVP tabletop contact、pickup 与 grasp quality 验收
- ❌ 不在当前项目范围:RM65 + SG100 真机遥操作、正式 VLA 数据采集系统

**当前阶段不是:**

- RM65 + SG100 真机遥操作项目
- 正式 VLA 数据采集 / LeRobot dataset recorder / π0.5 training pipeline
- MoveIt planner 或 autonomous scripted grasp controller
- 生产级 grasp / collision physics

---

## 1. 项目范围与定位

本项目用于验证 **Apple Vision Pro 到 RM65 + SG100 MuJoCo tabletop scene 的臂手遥操作链路**,包括 AVP tracking freshness、显式 arming、wrist relative-pose mapping、RM65 Cartesian IK、SG100 semantic/morphology-aware retargeting、17DoF 联合控制和 extra free-object scene compatibility。

> 本项目 **不作为正式 VLA 数据采集系统**。后续 SG100 正式示教数据采集将使用 Manus 数据手套,并在独立的数据采集 Pipeline 中完成。

> 本项目也 **不以 RM65 + SG100 真机遥操作为目标**。当前的控制策略、tracking-loss 策略和 IK-failure 策略仅面向 MuJoCo 仿真验证,不可直接迁移到真机。真机后续需要独立的安全状态机(watchdog / timeout / e-stop / hardware limits)。

**不要把下列概念混为一谈:**

| 概念 | 是否成立 |
|---|---|
| 「代码已实现 + 仿真/synthetic 已验证」 | 多数模块成立 |
| 「真实 AVP 已端到端验证」 | **不成立,尚未执行** |
| 「真实机器人已验证」 | **不在项目范围** |

---

## 2. 与上游 VisionProTeleop 的关系

本项目 **没有重新搭建新的 AVP teleoperation framework**,也没有新建独立的通信系统。

整体仍基于原 [`Improbable-AI/VisionProTeleop`](https://github.com/Improbable-AI/VisionProTeleop) 的运行框架。上游核心 API 仍是:

```text
VisionProStreamer
    ↓
configure_mujoco()
    ↓
start_webrtc()
    ↓
get_latest()
    ↓
MuJoCo simulation
    ↓
update_sim()
```

近期新增内容属于 **RM65 + SG100 robot-specific adaptation layer**:

> 在原 VisionProTeleop 的 VisionProStreamer、tracking、WebRTC 与 MuJoCo streaming 框架基础上,增加 SG100 手部语义重定向、RM65 Cartesian IK 以及 arm-hand combined simulation adaptation。

代码边界:

```text
avp_stream/                      ← 上游,负责 AVP 通信 / tracking / WebRTC / MuJoCo↔AVP streaming
    VisionProStreamer

sg100_teleop/                    ← 本子项目
    tracking/
        avp_source.py            HandData / TrackingData → Nx4x4 skeleton
        avp_adapter.py           skeleton → CanonicalHandState
        avp_hand_adapter.py      AVP joint index + geometry feature extractor
        avp_wrist_mapper.py      AVP wrist relative-pose → TCP target

    calibration/                 ← hand 标定采集 / 序列化
    hand/
        canonical_hand.py        设备无关语义中间表示
    retargeting/
        sg100_retargeter.py      CanonicalHandState → SG100 q[11]
    kinematics/
        rm65_ik.py               RM65 FK / Jacobian / DLS IK
    control/
        joint_command_filter.py  低通 + velocity limit
    sim/
        run_avp_sg100_sim.py     hand-only debug / regression runner
        run_fake_avp_arm_teleop.py
        run_fake_avp_arm_hand_teleop.py
        run_avp_arm_hand_sim.py  真实 AVP → RM65+SG100 combined 主入口
```

`avp_stream/` 视为 upstream-sensitive,默认不修改(见 [§26](#26-development-rules--architecture-constraints))。

---

## 3. 系统架构与数据流

### 3.1 Hand-only(已实现,hand debug / regression)

```text
Apple Vision Pro
      ↓
VisionProStreamer.get_latest()
      ↓
right HandData  (Nx4x4, world frame)
      ↓
AVPHandAdapter
      ↓
CanonicalHandState
      ↓
SG100Retargeter
      ↓
q_hand_goal[11]
      ↓
JointCommandFilter
      ↓
q_hand_cmd[11]
      ↓
SG100 MuJoCo
```

对应 runner:`sim/run_avp_sg100_sim.py`。**保留不废弃**,作为 hand-only debug / regression 入口。

### 3.2 Arm branch(已实现)

```text
AVP right wrist SE(3)   (= hand[0])
      ↓
AVPWristRelativeMapper
      ↓
relative wrist motion  →  target TCP SE(3)
      ↓
RM65IKSolver
      ↓
q_arm_goal[6]
      ↓
RM65 MuJoCo
```

### 3.3 Combined arm-hand(最终主链)

```text
visionOS HandTrackingProvider
        ↓
VisionProStreamer
        ↓
TrackingData
        ↓
TrackingFrameGate
        ↓
TeleopArmingController
        ↓ same accepted right-hand frame
        ├──────── wrist ─────────────────┐
        │                                │
        │  AVPWristRelativeMapper        │
        │           ↓                    │
        │      TCP target                │
        │           ↓                    │
        │      RM65IKSolver              │
        │           ↓                    │
        │       q_arm[6]                 │
        │                                │
        └──────── hand ──────────────────┤
                                         │
             AVPHandAdapter              │
                    ↓                    │
             HandCalibration             │
                    ↓                    │
             CanonicalHandState          │
                    ↓                    │
             SG100Retargeter             │
                    ↓                    │
              q_hand[11]                 │
                    ↓                    │
             JointCommandFilter          │
                                         │
q_arm[6] + q_hand_cmd[11] ───────────────┘
                    ↓
         CombinedRobotLayout
                    ↓
          MuJoCo robot controls
                    ↓
RM65 + SG100 + optional tabletop/free objects
                    ↓
VisionProStreamer.update_sim()
```

> **arm 与 hand 必须使用同一次 `streamer.get_latest()` 获取的同一帧 right-hand tracking 数据。** 代码中 `hand[0]`(AVP wrist SE(3))用于 arm 分支,`whole hand` 用于 hand 分支。禁止描述成 arm / hand 分别读取两个独立 tracking frame。

进入分支之前,frame 必须是 **NEW + right-hand tracked + fresh + parseable + finite**;并且 session 必须经过 `TeleopArmingController` 的显式 neutral confirmation。duplicate、untracked、stale 或 malformed frame 不更新 robot goals。

**重要 shape / unit:**

```text
q_arm_goal:
    shape = (6,)
    unit  = rad
    representation = absolute joint position

q_hand_goal / q_hand_cmd:
    shape = (11,)
    unit  = rad
    representation = absolute joint position

data.ctrl:
    shape  = (17,)
    layout = ctrl[0:6] RM65, ctrl[6:17] SG100
```

---

## 4. 目录结构

```text
sg100_teleop/
├── __init__.py
├── assets/
│   ├── sg100/                       # SG100 standalone assets
│   │   ├── config/
│   │   ├── meshes/                  # 原始 CAD STL(不直接喂 MuJoCo)
│   │   ├── meshes_mujoco/           # MuJoCo-friendly 简化 STL
│   │   ├── mjcf/
│   │   │   ├── sg100.xml            # SG100 canonical kinematic reference
│   │   │   └── sg100_sim.xml        # SG100 standalone sim model
│   │   └── urdf/                    # 原始 / mujoco URDF
│   └── rm65_sg100/
│       └── mjcf/
│           ├── rm65_sg100_sim.xml   # RM65 + SG100 robot-only model
│           └── rm65_sg100_grasp_scene.xml # tabletop/free-object validation scene
├── calibration/
│   ├── calibrate_avp_hand.py        # AVP live hand calibration
│   ├── io.py                        # YAML save / load
│   └── session.py                   # multi-frame 采集 + median build
├── config/
│   ├── sg100_joint_semantics.yaml   # SG100 joint 顺序 / 范围 / 语义(权威)
│   ├── avp_to_tcp_identity.yaml     # 未标定的 development basis default
│   └── avp_right_calibration.yaml   # hand 标定产物(运行 calibration 后生成)
├── control/
│   ├── joint_command_filter.py      # 低通 + velocity limit
│   └── teleop_arming.py             # 显式 neutral confirmation state machine
├── hand/
│   ├── canonical_hand.py            # CanonicalHandState
│   └── hand_calibration.py          # raw → canonical normalization
├── kinematics/
│   └── rm65_ik.py                   # RM65 FK / Jacobian / DLS IK
├── retargeting/
│   └── sg100_retargeter.py          # CanonicalHandState → SG100 q[11]
├── sim/
│   ├── build_sg100_sim.py           # 生成 sg100_sim.xml
│   ├── demo_mock_retargeting.py     # 不接 AVP 的 hand retarget 演示
│   ├── run_avp_sg100_sim.py         # hand-only AVP teleop
│   ├── run_fake_avp_arm_teleop.py   # fake AVP arm-only
│   ├── run_fake_avp_arm_hand_teleop.py  # fake AVP 17DoF
│   ├── run_avp_arm_hand_sim.py      # 真实 AVP → RM65+SG100 combined 主入口
│   ├── sweep_sg100_joint.py         # SG100 单关节 sweep
│   └── view_sg100_controlled.py     # SG100 controlled viewer
├── tests/
│   ├── test_sg100_retargeter.py
│   ├── test_sg100_physical_joint_semantics.py
│   ├── test_avp_feature_extractor.py
│   ├── test_avp_tracking_freshness.py
│   ├── test_teleop_arming.py
│   ├── test_avp_to_tcp_config.py
│   ├── test_avp_to_tcp_basis_six_axis.py
│   ├── test_avp_to_tcp_basis_ik.py
│   ├── test_raw_avp_to_combined_mujoco.py
│   ├── test_combined_scene_compatibility.py
│   ├── test_hand_calibration.py
│   ├── test_calibration_session.py
│   ├── test_avp_wrist_mapper.py
│   ├── test_rm65_fk_tcp.py
│   ├── test_rm65_tcp_jacobian.py
│   ├── test_rm65_ik.py              # 诊断型
│   ├── test_rm65_ik_z_scan.py       # 诊断型
│   ├── test_rm65_ik_reachable_targets.py
│   └── test_rm65_continuous_tcp_teleop.py
└── tracking/
    ├── avp_adapter.py               # HandData → raw → canonical
    ├── avp_hand_adapter.py          # AVP_JOINT_INDEX + feature extractor
    ├── avp_source.py                # TrackingData → Nx4x4 skeleton
    ├── avp_wrist_mapper.py          # AVP wrist relative-pose mapper
    ├── avp_to_tcp_config.py         # basis YAML load/validation
    └── frame_gate.py                # freshness/right-hand validity gate
```

---

## 5. SG100 hand 子系统

### 5.1 SG100 joint 顺序与语义(权威源:`config/sg100_joint_semantics.yaml`)

11 DoF,**`q[11]` 即此顺序:**

| idx | joint | range (rad) | axis | semantic feature | mapping type |
|----|-------|-------------|------|------------------|--------------|
| 0 | `thumb_j1`  | `[0.0, 2.182]`    | `0 0 -1` | `thumb_opposition`              | positive |
| 1 | `thumb_j2`  | `[-2.61, 0.0]`    | `0 0 1`  | `thumb_abduction`               | negative |
| 2 | `thumb_j3`  | `[-1.048, 1.57]`  | `0 0 1`  | `thumb_distal` (bidirectional)  | bidirectional |
| 3 | `index_j1`  | `[0.0, 1.57]`     | `0 0 -1` | `index_adduction`               | positive |
| 4 | `index_j2`  | `[0.0, 2.18]`     | `0 0 1`  | `index_flexion`                 | positive |
| 5 | `index_j3`  | `[-1.05, 1.57]`   | `0 0 1`  | `index_distal` (bidirectional)  | bidirectional |
| 6 | `middle_j1` | `[-2.18, 0.0]`    | `0 0 -1` | `middle_flexion`                | negative |
| 7 | `middle_j2` | `[-1.05, 1.57]`   | `0 0 1`  | `middle_distal` (bidirectional) | bidirectional |
| 8 | `little_j1` | `[-3.14, 0.0]`    | `0 0 -1` | `little_outward`                | negative |
| 9 | `little_j2` | `[0.0, 2.18]`     | `0 0 1`  | `little_flexion`                | positive |
| 10| `little_j3` | `[-1.05, 1.57]`   | `0 0 1`  | `little_distal` (bidirectional) | bidirectional |

> 以下方向均已经过 SG100 joint semantic sweep 视觉确认,与真机运动方向一致。**不要因为「人体解剖命名不标准」自行重命名或反转 joint direction。**

Phase 6 又增加了 automated actual-geometry regression:`11/11 direct joint physical kinematics PASS`,`11/11 CanonicalHandState → SG100Retargeter → MuJoCo physical semantics PASS`,没有 mapping sign mismatch。其准确边界是:

- **Software/MuJoCo retargeting identity、order、range、sign、monotonic physical response: VALIDATED**。
- **真实 AVP human-motion fidelity: PENDING HARDWARE VALIDATION**。

这里执行的是 **semantic / morphology-aware motion retargeting**,不是 human anatomical joint angle 的 1:1 copying。人手与 SG100 的结构、自由度和活动范围不同;真实用户的 open/fist/pinch/opposition fidelity 必须在 AVP hardware 与用户 hand calibration 后评估。

#### Thumb
- `thumb_j1`:`0 → positive`,大拇指绕掌心方向内旋。
- `thumb_j2`:`0 → negative`,大拇指向外、向下展开。
- `thumb_j3`:双向。`negative → 远端向外伸展`;`positive → 远端向掌心弯曲`。

#### Index
- `index_j1`:`0 → positive`,食指向拇指方向横向偏摆。
- `index_j2`:`0 → positive`,食指朝掌心弯曲。
- `index_j3`:双向。`negative → 远端朝手背伸展`;`positive → 远端朝掌心弯曲`。

#### Middle
- `middle_j1`:`0 → negative`,中指朝掌心弯曲。
- `middle_j2`:双向。`negative → 朝手背伸展`;`positive → 朝掌心弯曲`。

#### Little
- `little_j1`:`0 → negative`,小指向外、向下。
- `little_j2`:`0 → positive`,小指朝掌心弯曲。
- `little_j3`:双向。`negative → 朝手背伸展`;`positive → 朝掌心弯曲`。

### 5.2 CanonicalHandState(`hand/canonical_hand.py`)

11 个设备无关语义 feature。约定:

- **Unidirectional:** `0.0 = neutral`,`1.0 = calibrated active endpoint`。
  成员:`thumb_opposition`, `thumb_abduction`, `index_adduction`, `index_flexion`, `middle_flexion`, `little_outward`, `little_flexion`。
- **Bidirectional:** `-1.0 = calibrated negative`,`0.0 = neutral`,`+1.0 = calibrated positive`。
  成员:`thumb_distal`, `index_distal`, `middle_distal`, `little_distal`(均为 distal flexion/extension)。

`CanonicalHandState.neutral()` 返回全 0。`clipped()` 把每个 feature 裁到其约定区间。该层 **不含** Vision Pro / Manus / MediaPipe / MuJoCo / SG100 特定数据结构。

### 5.3 SG100Retargeter(`retargeting/sg100_retargeter.py`)

- 输入:`CanonicalHandState`
- 输出:`q_target.shape == (11,)`,单位 **rad**,表示 **absolute joint position**

Joint 顺序与 range 从 `sg100_joint_semantics.yaml` 加载;feature → joint 由 `FEATURE_MAPPING` 固定,3 种 mapping type:

- `positive`:canonical `[0,1]` → SG100 `[0, upper]`
- `negative`:canonical `[0,1]` → SG100 `[0, lower]`(`value * lower`,lower 为负)
- `bidirectional`:canonical `[-1,1]` → 不对称 SG100 range。`value >= 0` → `value * upper`;`value < 0` → `(-value) * lower`

最后统一 `np.clip(q, lower, upper)`。该层 **不依赖** AVP joint index 或 tracking backend,可被未来 `Manus → CanonicalHandState` 直接复用。

### 5.4 AVP feature extraction(`tracking/avp_hand_adapter.py` / `avp_source.py` / `avp_adapter.py`)

`avp_source.get_hand_from_tracking()` 把 `VisionProStreamer.get_latest()` 输出统一为 world-frame `Nx4x4` skeleton(兼容当前 `TrackingData.right` 与旧 dict API)。`AVP_JOINT_INDEX` 定义 25/27-joint 命名映射。`AVPHandFeatureExtractor` 核心:

```text
world skeleton
    ↓  (× inverse wrist_world)
wrist-local skeleton
    ↓  (palm frame: x = little→index, y = wrist→middle_knuckle, z = palm normal)
palm-relative geometric features  →  AVPRawHandFeatures
```

由此保证:**手整体平移 / 手腕旋转 / 手掌尺寸(经 palm_width 归一化)** 不直接改变 finger feature。raw feature **未归一化**(例如 `thumb_opposition = 1.3`、`index_flexion = 0.8 rad` 都是合法 raw 值),归一化由 `HandCalibration` 完成。

### 5.5 JointCommandFilter(`control/joint_command_filter.py`)

低频 tracking goal(~60 Hz)→ 高频 MuJoCo command(500 Hz)的一阶低通 + velocity 限幅:

```text
alpha = 1 - exp(-dt / tau)        # 连续时间形式,timestep 变化时行为不漂移
filtered = q + alpha * (target - q)
delta    = clip(filtered - q, ±max_velocity * dt)
```

默认 `tau=0.06`,`max_velocity=4.0`。纯通用,不含 SG100-specific mapping。

### 5.6 Hand calibration

详细 pose guide 与 CLI 见 [§16](#16-apple-vision-pro-hand-calibration-guide)。Calibration 负责 `raw human geometry → normalized semantic space`,**不是** robot joint mapping。

---

## 6. RM65 + SG100 combined MuJoCo 模型

文件:`assets/rm65_sg100/mjcf/rm65_sg100_sim.xml`

- model name:`RM65-SG100-sim`
- `<option timestep="0.002" integrator="implicitfast" gravity="0 0 0" />`(500 Hz;combined model gravity 为 0)
- `<compiler angle="radian" />`

**Validated dimensions:**

```text
nq = 17
nv = 17
nu = 17
```

**Joint order(`q[17]` / `qpos` / `ctrl` 均按此顺序):**

```text
[00] joint_1        ← RM65
[01] joint_2
[02] joint_3
[03] joint_4
[04] joint_5
[05] joint_6

[06] thumb_j1       ← SG100
[07] thumb_j2
[08] thumb_j3
[09] index_j1
[10] index_j2
[11] index_j3
[12] middle_j1
[13] middle_j2
[14] little_j1
[15] little_j2
[16] little_j3
```

**Control layout:**

```text
ctrl[0:6]   = RM65
ctrl[6:17]  = SG100
```

> 已通过 `run_avp_arm_hand_sim.py` 启动校验 + `test_rm65_fk_tcp` 验证:**qpos order、DoF order、actuator ctrl order 三者一致**。运行时若 `nq/nv/nu != 17` 或三者顺序不一致,runner 直接 `RuntimeError`。

**Actuator / dynamics 参数(当前 validation 用,不代表真机 motor dynamics):**

| 部位 | actuator | joint damping | joint armature |
|---|---|---|---|
| RM65 (`joint_*_pos`) | position,`kp=50 dampratio=1 inheritrange=1` | `0.1` | `0.01` |
| SG100 (`*_j*_pos`) | position,`kp=2.0 dampratio=1.0 inheritrange=1` | `0.002` | `0.001` |

RM65 各 joint range(来自 XML):`joint_1 ±3.106`、`joint_2 ±2.2689`、`joint_3 ±2.356`、`joint_4 ±3.106`、`joint_5 ±2.234`、`joint_6 ±6.28`。SG100 各 joint range 与 `sg100_joint_semantics.yaml` 一致。

### 6.1 Robot-only 与 tabletop scene

| Model | 定位 |
|---|---|
| `rm65_sg100_sim.xml` | robot-only 17DoF regression / default live runner model |
| `rm65_sg100_grasp_scene.xml` | table、dynamic bottle、collision proxy、workspace/grasp-reference sites 的 tabletop teleoperation validation environment |

`CombinedRobotLayout` 按名称解析且只初始化/控制 6 RM65 + 11 SG100 = **17 robot joints/actuators**。tabletop scene 可以包含额外 freejoint qpos/qvel;`initialize_robot_state()` 不会用 robot q17 覆盖 bottle 等 scene state。Phase 1/5 已验证 robot-only 与 tabletop model 都能接受同一 active q17 update,bottle freejoint 保持 finite 且 quaternion valid。

table、bottle、collision proxy、workspace mask 与 grasp-reference site **不是 automatic grasp controller**。当前已验证 scene load/init/control compatibility;尚未以真实 AVP 完成最终 contact、gravity、pickup 或 grasp-quality 验收。早期固定 bottle/q pose 实验不得描述成 production algorithm。

---

## 7. SG100 与 RM65 安装关系 / Teleoperation TCP

### 7.1 安装关系(无额外法兰 offset)

已确认:SG100 原 `base_link` mesh **已包含转接法兰几何**。因此 combined simulation 中 **不再使用额外 STEP adapter**。

> 禁止在文档中继续描述旧的额外 `HM_SG100V3_P003转接法兰.STEP`,或此前猜测的 `+18.5 mm` offset。当前 accepted transform 为:

```text
T_link6_to_sg100_base
    translation : [0, 0, 0]
    rotation    : Identity
```

XML 中 `<body name="sg100_base_link" pos="0 0 0" quat="1 0 0 0">` 印证此关系。Body hierarchy:

```text
world
└─ RM65
   └─ link_1 → link_2 → link_3 → link_4 → link_5 → link_6
      └─ sg100_base_link
         ├─ thumb  (thumb_link1/2/3)
         ├─ index  (index_link1/2/3)
         ├─ middle (middle_link1/2)
         └─ little (little_link1/2/3)
```

### 7.2 Teleoperation TCP

```text
TELEOP_TCP_FRAME = sg100_base_link
```

选它作为 teleop TCP 的原因:

- 与 AVP wrist / root 语义一致(都是「腕/根」)。
- 是 SG100 的固定 root。
- SG100 finger articulation **不会改变** 该 TCP(已由 `test_rm65_fk_tcp` 的 hand-independence 测试验证)。
- 已通过 FK / TCP regression。

> 不要把 palm grasp point、fingertip、`link_6` 误写为当前 teleop TCP。未来若增加 grasp TCP,应作为 **独立概念**,不替换当前 teleop TCP。

---

## 8. RM65 teleop-ready pose

当前 Cartesian teleoperation 默认起始姿态(`run_avp_arm_hand_sim.py` 中 `RM65_TELEOP_READY_Q`):

```python
RM65_TELEOP_READY_Q = [
    0.32488,
   -0.76769,
    0.97621,
    0.22463,
   -1.80649,
   -0.12430,
]
```

> **这不是 RM65 mechanical zero pose,也不是最终抓取最优 q0。** 它是当前 repository authoritative robot-side teleop calibration / ready anchor candidate,用于 combined runner 的稳定起点、IK seed 与首次 session wrist-neutral 的 TCP anchor。初始位姿和抓取水平优化属于另一工作流。

```text
RM65_ZERO_Q  ≠  RM65_TELEOP_READY_Q
```

不要混淆两者。旧 diagnostic pose `[0.30, -0.40, 0.35, -0.25, 0.30, 0.20]` **不是** 当前 authoritative teleop-ready anchor。

---

## 9. RM65 FK / Jacobian / IK(`kinematics/rm65_ik.py`)

`RM65IKSolver`:6D Cartesian IK。

- **TCP** = `sg100_base_link`
- **只使用 RM65 六个 DoF**(`joint_1..joint_6`)。SG100 11 个 joint **故意排除**——Jacobian 仅取 RM65 对应列。
- 误差定义:**world-frame position error** + **world-frame orientation error**(后者用 SO(3) logarithm,与 MuJoCo `jacr` 的 world-frame 角速度表达一致)。
- Jacobian:MuJoCo 解析 `mj_jac`(TCP body 原点)。
- 求解:**adaptive Damped Least Squares (DLS)**,SVD 提供诊断(`sigma_min` / condition number)。
- 保护:joint limit clipping + per-joint step protection + full-vector step protection。
- 种子:**previous valid solution 用作下一帧 seed**(由调用者维护 `q_arm_goal` 并传入)。

**`IKResult` 字段:** `converged`, `iterations`, `position_error`, `rotation_error`, `sigma_min`, `condition_number`, `damping`, `reason`。

**Adaptive damping 与 solver 参数(按代码实际值,勿改):**

```text
base damping        = 0.002
sigma threshold     = 0.05
maximum damping     = 0.05
orientation weight  = 0.20

position_tolerance  = 1e-4
rotation_tolerance  = 1e-3
max_iterations      = 200
step_gain           = 0.8
max_joint_step      = 0.08    # per-joint
max_step_norm       = 0.15    # full-vector
joint_limit_margin  = 1e-4
```

> Adaptive damping 在 `sigma_min` 接近 0.05 阈值前就开始增大阻尼(因为多个代表位形的 `sigma_min` 落在 0.015–0.03),从而在奇异区附近维持数值稳定。

---

## 10. IK failure 策略

当前是 **MuJoCo simulation validation**。IK 失败时:

```text
IK success → accept q_arm_candidate(更新 q_arm_goal)
IK failure → keep last valid q_arm_goal(HOLD)
```

即 **IK fail → HOLD last valid arm command**。失败帧的 `q_candidate` **不被采用**。

> 这不是真机生产安全机制。真机需要独立的安全状态机。

---

## 11. Legacy +Z singularity diagnostic(非当前 ready anchor)

`test_rm65_ik_z_scan.py` 保留旧 diagnostic seed `[0.30, -0.40, 0.35, -0.25, 0.30, 0.20]`,固定 orientation 对 world +Z 做局部 scan。其历史观察是:

```text
+5 mm        PASS
约 +7.5 mm   逼近 singular region
+10 mm       FAIL
+15 mm       FAIL
+20 mm       FAIL
```

表现:`q3 → 0`、`sigma_min → 0`、condition number 急剧上升。

> 结论:这是该 **legacy diagnostic pose branch** 在固定 orientation 条件下的局部 workspace/singularity 记录,不代表当前 authoritative `RM65_TELEOP_READY_Q`,也不作为 `RM65IKSolver` 实现错误。该脚本属于 [§20](#20-extended--experimental-validation)。

正式 regression 应优先使用 **guaranteed-reachable FK-generated targets**(见 `test_rm65_ik_reachable_targets.py`)。

---

## 12. AVP Wrist Relative Pose Mapper(`tracking/avp_wrist_mapper.py`)

`AVPWristRelativeMapper`:把 AVP wrist 的 **相对** SE(3) 运动映射到 robot TCP target。

核心思想 —— **不**直接建立 `AVP absolute world pose == MuJoCo absolute world pose`,而是在启动时建立 relative calibration:

```text
initial AVP wrist  : T_avp0
initial robot TCP  : T_tcp0
```

之后每帧:

```text
T_avp(t)
   ↓
relative ΔT_avp  = inverse(T_avp0) @ T_avp(t)
   ↓
axis mapping (R_tcp_from_avp_local)
   ↓
translation scale
   ↓
relative ΔT_tcp
   ↓
T_tcp_target = T_tcp0 @ ΔT_tcp        (world frame)
```

该类 **故意独立于 `VisionProStreamer`**,只接受普通 4×4 homogeneous transform。

**当前默认:**

```text
translation_scale = 0.5        # 见下方说明
```

> `translation_scale = 0.5` 是 checked-in `avp_to_tcp_identity.yaml` 的 validation 默认值;real runner 通过 `--avp-to-tcp-config` 加载,basis 与 scale 不再是分散的 CLI 数值。含义:`10 cm` 人手运动 → `5 cm` 机器人运动。**这不代表** 真实 AVP 标定后的唯一正确 scale。`AVPWristRelativeMapper.__init__` 的类默认 `1.0` 仅是通用回退。

辅助函数:`make_transform`、`invert_transform`、`rotation_angle`。

### 12.1 AVP wrist axis mapping

必须区分两个坐标转换:

1. `YUP2ZUP`:VisionProStreamer 对 AVP/ARKit **world representation** 的转换。
2. `avp_to_tcp_local_rotation = A`:初始 AVP wrist-local relative motion 到 robot TCP-local control motion 的 basis change。

二者作用域不同,不能用 `YUP2ZUP` 代替 wrist-local control calibration。mapper 数学为:

```text
p_robot_local = A @ p_avp_local
R_robot_local = A @ R_avp_local @ A.T

delta_p_tcp = translation_scale * A @ delta_p_avp
T_tcp_target = T_tcp_runtime_0 @ delta_T_tcp
```

`config/avp_to_tcp_identity.yaml` 明确标记为:

```text
IDENTITY — UNCALIBRATED DEVELOPMENT DEFAULT
```

它不是已验证真实 AVP mapping。Phase 4 已验证 nonidentity proper-rotation config、六轴 basis mapping 与 basis→IK;真实 AVP 到手后仍必须通过 forward/back、left/right、up/down 和 local Rx/Ry/Rz 确定并写入 hardware-validated YAML。

**禁止** 写成「AVP 与 RM65 坐标轴已经完成真机标定」。目前没有。

---

## 13. 三种 calibration 必须区分

| Calibration | 数据/文件 | 作用 | 生命周期 |
|---|---|---|---|
| User `HandCalibration` | `avp_right_calibration.yaml` | real human raw hand features → canonical ranges | 用户预先采集,可跨 session 使用 |
| Session wrist-neutral | `T_avp_wrist_0 ↔ T_tcp_runtime_0` | 建立 relative Cartesian teleoperation anchor | 每次 arming/recalibration runtime 建立 |
| AVP→TCP coordinate basis | `avp_to_tcp_local_rotation=A` + `translation_scale` YAML | 决定真实 forward/left/up 与 roll/pitch/yaw 控制方向 | hardware 六轴标定后持久化 |

不要把三者统称为一个模糊的 calibration。`HandCalibration` 不决定 wrist axis;session neutral 不学习 user hand range;coordinate basis 不保存 session absolute pose。

### 13.1 `q_hand0_runtime`

显式 neutral confirmation 后的下一帧同时经过:

```text
AVP skeleton
→ AVPHandAdapter
→ HandCalibration
→ CanonicalHandState
→ SG100Retargeter
→ q_hand0_runtime
```

因此 `q_hand0_runtime` 是 calibration frame 的真实 11DoF retarget output,**不是** `zeros(11)`、`POSE_220` 或 fixed grasp pose。代码只把 mathematical zero 用作未 arming startup placeholder;Phase 5 test 中的 zero reference 也只属于 test-only kinematic reference。

---

## 14. 多频率 runtime 与 command 语义

当前运行频率设计:

```text
AVP tracking query       ≈ 60 Hz   (real runner --tracking-hz 默认 60)
MuJoCo physics           = 500 Hz   (model.opt.timestep = 0.002)
MuJoCo → AVP sim stream  ≈ 60 Hz   (--stream-hz 默认 60)
```

physics timestep 从 `model.opt.timestep` 读取。正式 live runner(`run_avp_arm_hand_sim.py`)沿用 `run_avp_sg100_sim.py` 的固定结构式时钟:

```text
wall_dt → min(wall_dt, 0.05) → physics_accumulator
while accumulator >= physics_dt:
    mj_step()
```

> 这不是新建 scheduler,是沿用原 runner 的 wall-clock accumulator 模式,避免窗口拖动 / 调试暂停后产生巨大追赶步。

fake runner 使用固定 `PHYSICS_DT=0.002`、`PHYSICS_PER_TRACKING_FRAME=8` → target 更新 62.5 Hz(8 × 0.002 = 0.016 s),便于确定性复现。

**Hand goal / command / state 概念区分:**

```text
q_hand_goal   ≈ 60 Hz AVP tracking 输出的 SG100 target
q_hand_cmd    JointCommandFilter 在 500 Hz physics loop 中生成的 command
hand qpos     MuJoCo 实际 simulated joint position

q_arm_goal    IK 输出(~60 Hz),直接送 ctrl[0:6]
```

> **RM65 当前第一版不增加额外 arm JointCommandFilter。** `q_arm_goal` 直接写入 `ctrl[0:6]`(注释明确:IK 输出已是连续量)。仅 SG100 hand 走 `JointCommandFilter`。

---

## 15. Tracking freshness、validity 与 arming

visionOS `HandTrackingProvider` 提供 `HandAnchor.isTracked` 和 `sourceFrameSequence`;Python `TrackingData` 暴露 `frame_sequence`、`received_monotonic_ns` 与 `right_hand_valid`。`TrackingFrameGate` 只把以下 frame 交给 combined pipeline:

```text
NEW + right-hand tracked + fresh + parseable + finite
```

- duplicate cached frame:不更新 arm/hand goals,也不刷新 timeout。
- new but untracked frame:不更新 goals,也不刷新 timeout。
- stale/missing-metadata/malformed frame:拒绝。
- accepted parseable frame:只处理一次并刷新 valid-tracking time。

`TeleopArmingController` state model:

```text
WAITING_FOR_TRACKING
    ↓ first NEW+VALID frame
WAITING_FOR_NEUTRAL_CONFIRM
    ↓ operator presses C in current MuJoCo viewer
CALIBRATION_REQUESTED
    ↓ next NEW+VALID frame received after request
TELEOP_ACTIVE

tracking timeout → TRACKING_TIMEOUT_DISARMED
new valid tracking → WAITING_FOR_NEUTRAL_CONFIRM
```

tracking reacquisition **不会自动恢复旧 teleop anchor**;必须重新按 C 并用下一张有效新帧建立 wrist/hand runtime anchors。C key 是当前 bring-up trigger,未来可以换成 AVP UI command,但 arming/calibration core 本身 source-agnostic。

Tracking failure 与 IK failure 必须区分:前者没有 usable frame,后者有有效 frame 但 TCP target 未可靠求解。两者都 HOLD last valid robot targets,绝不 snap 回 q=0;tracking timeout 还会丢弃旧 mapper calibration 并进入上述 re-arming flow。

---

## 16. Apple Vision Pro hand calibration guide

### 为什么需要 calibration

不同用户存在手掌尺寸 / 舒适活动范围 / 动作风格 / AVP 几何差异,因此不能把 raw 角度直接映射到 SG100 full range:

```text
AVP Raw Features → user calibration → Canonical [-1,1]/[0,1] → SG100 joint range
```

### Calibration command

```bash
cd ~/VisionProTeleop
source .venv/bin/activate

python -m sg100_teleop.calibration.calibrate_avp_hand \
  --ip <VISION_PRO_IP>
```

CLI(以源码为准):

| 参数 | required | 默认值 | 含义 |
|---|---|---|---|
| `--ip` | ✅ | — | Apple Vision Pro IP |
| `--hand` | ❌ | `right` | 仅支持 `right` |
| `--duration` | ❌ | `1.5` | 每个 pose 采样时长(秒) |
| `--hz` | ❌ | `60.0` | 采样频率(Hz) |
| `--countdown` | ❌ | `2` | 每个 pose 前倒数秒数 |
| `--output` | ❌ | `sg100_teleop/config/avp_right_calibration.yaml` | 输出 YAML |
| `--verbose` | ❌ | off | 详细日志 |

有效帧不足 `max(20, int(duration*hz*0.5))`(默认 45)时抛 `RuntimeError`。

### 10 个 calibration poses(执行顺序)

1. `neutral` — 自然张开手掌。建立所有 feature neutral baseline。
2. `thumb_opposition` — 拇指内移向掌心。标定 `thumb_opposition` → `thumb_j1`(positive)。
3. `thumb_abduction` — 拇指外/下展。标定 `thumb_abduction` → `thumb_j2`(negative)。
4. `thumb_distal_negative` — 拇指远端外伸。`thumb_distal = -1`。
5. `thumb_distal_positive` — 拇指远端内弯。`thumb_distal = +1`。
6. `index_adduction` — 食指横向偏摆向拇指。`index_adduction = 1` → `index_j1`。
7. `finger_flexion` — index/middle/little 同时弯曲。同时标定 `index_flexion`/`middle_flexion`/`little_flexion`。
8. `finger_distal_negative` — 远端朝手背伸展。`index/middle/little_distal = -1`。
9. `finger_distal_positive` — 远端朝掌心弯曲。`index/middle/little_distal = +1`。
10. `little_outward` — 小指外/下展。`little_outward = 1` → `little_j1`(negative)。

每个 pose 采集多帧(默认 1.5 s × 60 Hz ≈ 90 帧),对每个 raw feature 取 **median**(`session.py: pose_median`)以抑制 jitter / outlier。

> raw 数值方向可能与 semantic 方向不一致(例如 `thumb_opposition` 的 `active` raw 值可能 **小于** `neutral`),calibration 仍输出 canonical `+1`。这是设计预期,不是 bug。

### Loading during teleoperation

`run_avp_sg100_sim.py` 与 `run_avp_arm_hand_sim.py` 均通过 `load_calibration(...)` 加载该 YAML,默认路径一致。该产物不是 checked-in universal user calibration;**运行 live teleop 前必须先生成或通过 `--calibration` 指定**。若不存在会直接 `FileNotFoundError`,不会偷偷使用 fake/default hand mapping。

---

## 17. Runners

### 17.1 `run_avp_sg100_sim.py` — hand-only(保留)

hand-only debug / regression 入口,**不废弃**。

```bash
python -m sg100_teleop.sim.run_avp_sg100_sim \
  --ip <VISION_PRO_IP> \
  --verbose
```

加载 `sg100_sim.xml`(SG100 standalone),`nq=nu=11` 校验,hand pipeline。tracking-loss 时 hold last `q_goal`。

### 17.2 `run_fake_avp_arm_teleop.py` — fake arm-only(已验证)

不接 AVP,用 synthetic wrist 运动验证 `Fake AVP wrist → AVPWristRelativeMapper → RM65 IK → combined MuJoCo`。

```bash
cd ~/VisionProTeleop
source .venv/bin/activate
python -m sg100_teleop.sim.run_fake_avp_arm_teleop
```

- 固定 500 Hz physics / 62.5 Hz target;`translation_scale=0.5`,`R_tcp_from_avp_local=eye(3)`。
- 19 s 序列:`NEUTRAL → LOCAL_X → LOCAL_Y → LOCAL_RZ → COMBINED`。
- PASS 条件:`IK failures == 0`,打印 `FAKE AVP ARM TELEOP: PASS`。
- 此前运行记录的输出:

```text
IK failures = 0
max q_goal step/frame             ≈ 0.004619 rad
min sigma_min                     ≈ 0.016954
max condition                     ≈ 45.17
max simulated TCP error           ≈ 2.60 mm
max simulated orientation error   ≈ 0.324 deg
FAKE AVP ARM TELEOP: PASS
```

### 17.3 `run_fake_avp_arm_hand_teleop.py` — fake 17DoF(已验证)

不接 AVP,验证 `fake wrist → RM65 q[6]` 与 `fake CanonicalHandState → SG100 q[11]` 的 simultaneous 17DoF 控制。

```bash
python -m sg100_teleop.sim.run_fake_avp_arm_hand_teleop
```

- 20 s 序列:`NEUTRAL → ARM_ONLY → HAND_ONLY → ARM_AND_HAND → WRIST_AND_HAND`。
- 代码内回归阈值:`arm step/frame < 0.05`、`hand step/frame < 0.10`、`TCP pos error < 5 mm`、`TCP rot error < 1 deg`、`SG100 tracking error < 0.15 rad`、`IK failures == 0`,全过打印 `FAKE AVP 17DOF TELEOP: PASS`。
- 此前运行记录的输出:

```text
IK failures = 0
max arm q_goal step/frame          ≈ 0.002817 rad
max hand q_goal step/frame         ≈ 0.046562 rad
min sigma_min                      ≈ 0.061328
max condition                      ≈ 9.884
max simulated TCP position error   ≈ 4.446 mm
max simulated TCP rotation error   ≈ 0.310 deg
max SG100 tracking error           ≈ 0.122686 rad
FAKE AVP 17DOF TELEOP: PASS
```

### 17.4 `run_avp_arm_hand_sim.py` — 真实 AVP combined 主入口

**定位:最终 AVP → RM65 + SG100 combined MuJoCo simulation 主入口。**

> ⚠️ 该文件已 **完成代码结构、CLI 与静态架构验证**(启动时校验 `nq/nv/nu==17`、qpos/DoF/actuator order 一致、TCP parent==`link_6`、retargeter order、neutral→q=0、MuJoCo TCP 与 IK FK 一致性等)。但 **当前尚未拿到真实 Apple Vision Pro**,Real AVP live teleoperation **尚未完成验证**。**不要写成「真实 AVP 17DoF teleoperation PASS」。目前没有这个结果。**

基本运行:

```bash
cd ~/VisionProTeleop
source .venv/bin/activate

python -m sg100_teleop.sim.run_avp_arm_hand_sim \
  --ip <APPLE_VISION_PRO_IP>
```

Verbose / 查看 CLI:

```bash
python -m sg100_teleop.sim.run_avp_arm_hand_sim --verbose
python -m sg100_teleop.sim.run_avp_arm_hand_sim --help
```

主要参数(以源码为准):

| 参数 | required | 默认值 | 含义 |
|---|---|---|---|
| `--ip` | ✅ | — | Apple Vision Pro IP |
| `--model` | ❌ | `sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_sim.xml` | combined MuJoCo XML |
| `--calibration` | ❌ | `sg100_teleop/config/avp_right_calibration.yaml` | AVP right-hand calibration |
| `--port` | ❌ | `50051` | AVP MuJoCo streaming gRPC port |
| `--tracking-hz` | ❌ | `60.0` | AVP tracking 查询频率 |
| `--stream-hz` | ❌ | `60.0` | MuJoCo state 回传 AVP 频率 |
| `--avp-to-tcp-config` | ❌ | `sg100_teleop/config/avp_to_tcp_identity.yaml` | proper local basis + translation scale YAML |
| `--filter-tau` | ❌ | `0.06` | SG100 hand 低通时间常数 |
| `--max-hand-velocity` | ❌ | `4.0` | SG100 hand joint 速度上限 |
| `--tracking-timeout` | ❌ | `0.5` | 报告 tracking loss 的超时(秒) |
| `--attach` | ❌ | `0.0 0.7 0.8 0.0` | MuJoCo scene 在 AVP AR 的放置 `x y z yaw_deg` |
| `--verbose` | ❌ | off | 低频 runtime 诊断 |

运行时要点(与 [§3.3](#33-combined-arm-hand最终主链) 一致):

- `TrackingFrameGate` 先拒绝 duplicate/untracked/stale/malformed frame;同一张 accepted right-hand frame 的 `hand[0]` → arm,`whole hand` → hand。
- 第一张有效帧只使 state 进入 `WAITING_FOR_NEUTRAL_CONFIRM`,不会 calibration 或 active-follow。
- 用户在 neutral pose 按 C 后,**下一张** NEW+VALID frame 同时建立 `T_avp_wrist_0`,`T_tcp_runtime_0`,`q_hand0_runtime`,state → `TELEOP_ACTIVE`。
- Active frame:`AVPWristRelativeMapper.map()` → TCP target → `RM65IKSolver.solve(q_seed=q_arm_goal, ...)`。
- IK success → 更新 `q_arm_goal`;IK fail → HOLD。
- hand:`q_hand_goal` → `JointCommandFilter@500Hz` → `q_hand_cmd`;arm:`q_arm_goal` 直送 `ctrl[0:6]`(无额外 arm filter)。
- physics 500 Hz;`viewer.sync()` 在 physics loop 之外;`streamer.update_sim()` 按 `--stream-hz` 回传。
- tracking loss / IK failure 均 HOLD;tracking timeout 还会 disarm 并要求重新 C confirmation(见 §15)。

---

## 18. 测试与运行命令

当前仓库没有配置 pytest runner;核心 tests 是可直接执行的独立 `main()` scripts。使用实际存在的命令 `python -m sg100_teleop.tests.<name>`,打印 `PASS` 即通过。不要在文档中写一个不存在的 pytest workflow。

### 18.0 Tracking / arming / coordinate mapping

| Category | Test | Purpose | When to run |
|---|---|---|---|
| Tracking | `test_avp_streamer_frame_metadata` | source sequence、receive timestamp、right-hand validity transport metadata | streamer/protocol change |
| Tracking | `test_avp_tracking_freshness` | duplicate/untracked/stale/malformed gate behavior | combined tracking change |
| Arming | `test_teleop_arming` | explicit C-style request、same-frame anchors、timeout/re-arm | arming/runtime change |
| Coordinate | `test_avp_to_tcp_config` | YAML schema、proper rotation、identity warning | config loader change |
| Coordinate | `test_avp_to_tcp_basis_six_axis` | nonidentity ±translation/rotation axes | basis semantics change |
| Coordinate/IK | `test_avp_to_tcp_basis_ik` | configured six-axis target reaches RM65 IK/FK | mapper or IK change |

```bash
python -m sg100_teleop.tests.test_avp_streamer_frame_metadata
python -m sg100_teleop.tests.test_avp_tracking_freshness
python -m sg100_teleop.tests.test_teleop_arming
python -m sg100_teleop.tests.test_avp_to_tcp_config
python -m sg100_teleop.tests.test_avp_to_tcp_basis_six_axis
python -m sg100_teleop.tests.test_avp_to_tcp_basis_ik
```

### 18.1 SG100 retargeting

```bash
python -m sg100_teleop.tests.test_sg100_retargeter
```

验证 `CanonicalHandState → SG100 q[11]`:neutral→0、full activation→不对称 joint limit(精确 `1e-6`)、4 个 distal `-1`→lower 端。

### 18.1.1 SG100 physical semantics

```bash
python -m sg100_teleop.tests.test_sg100_physical_joint_semantics
```

验证 11/11 direct parent-local physical joint axis/sign 和 11/11 canonical-to-distal-geometry semantics,包括 flexion、opposition、abduction/adduction、outward 和 distal signed monotonic response。修改 SG100 MJCF、semantic YAML 或 retargeter 时必跑。

### 18.2 AVP hand feature extractor

```bash
python -m sg100_teleop.tests.test_avp_feature_extractor
```

验证 rigid-transform 不变性(`<1e-8`)、scale 不变性(`<1e-8`)、synthetic index flexion 响应(`>0.3`)。

### 18.3 Hand calibration

```bash
python -m sg100_teleop.tests.test_hand_calibration
```

验证 neutral→0、active→1、negative distal→-1,含 raw 数值方向与 semantic 方向不一致用例。

### 18.4 Calibration session

```bash
python -m sg100_teleop.tests.test_calibration_session
```

验证 10 poses、100 噪声帧 + 1 outlier 的 median 鲁棒性、YAML round-trip 一致。

### 18.5 AVP wrist mapper

```bash
python -m sg100_teleop.tests.test_avp_wrist_mapper
```

5 个子测试(容差 `pos 1e-10 / rot 1e-7`):

```text
CALIBRATION_IDENTITY
LOCAL_X_TRANSLATION
LOCAL_Z_ROTATION
TRANSLATION_SCALE
AXIS_ALIGNMENT
AVP WRIST RELATIVE MAPPER: PASS
```

### 18.6 RM65 FK / TCP

```bash
python -m sg100_teleop.tests.test_rm65_fk_tcp
```

验证:`sg100_base_link` 是 `link_6` 直接子节点;多个 RM65 位形下 `link_6 → sg100_base_link` 相对位姿 ≈ 零(translation tol `1e-10`,rotation tol `1e-7`);SG100 articulation 不改变 teleop TCP(hand-independence)。输出 `FK/TCP VALIDATION: PASS`(含 `Hierarchy check: PASS`)。

### 18.7 RM65 TCP Jacobian

```bash
python -m sg100_teleop.tests.test_rm65_tcp_jacobian
```

验证 MuJoCo 解析 Jacobian 与 central finite-difference Jacobian 一致(`JAC_TOL=1e-5`),以及 SG100 11 列对 TCP Jacobian 贡献严格为 0(`HAND_TOL=1e-12`)。此前最大 analytic-vs-FD error ≈ `2.985e-10`。输出 `TCP JACOBIAN VALIDATION: PASS`。

### 18.8 RM65 guaranteed-reachable IK

```bash
python -m sg100_teleop.tests.test_rm65_ik_reachable_targets
```

4 个由 FK 生成的 guaranteed-reachable 目标(`TARGET_A..D`,每个要求 converged、pos `<2e-4`、rot `<2e-3`、within limits)。代表 FK / Jacobian / orientation convention / DLS / joint ordering 整体一致。输出:

```text
TARGET_A PASS
TARGET_B PASS
TARGET_C PASS
TARGET_D PASS
GUARANTEED-REACHABLE IK: PASS
```

### 18.9 RM65 continuous TCP teleoperation

```bash
python -m sg100_teleop.tests.test_rm65_continuous_tcp_teleop
```

8 s / 500 帧 Lissajous 轨迹(X 10 mm / Y 8 mm),IK fail→hold。回归阈值:IK failures `0/500`、IK pos `<0.2 mm`、IK rot `<0.002 rad`、sim TCP pos `<5 mm`、sim TCP rot `<1 deg`、joint step/frame `<0.05 rad`、hand 动态偏转 `<1e-3 rad`。此前运行记录:

```text
IK failures                       = 0 / 500
IK position error max             ≈ 0.05944 mm
IK rotation error max             ≈ 0.000061 deg
SIM TCP position error max        ≈ 2.065 mm
SIM TCP rotation error max        ≈ 0.250 deg
max q-goal step/frame             ≈ 0.001753 rad
min sigma_min                     ≈ 0.058077
max condition                     ≈ 10.418
max hand |qpos|                   ≈ 0.000211 rad
CONTINUOUS TCP TELEOP: PASS
```

### 18.10 Raw AVP-format combined integration

```bash
python -m sg100_teleop.tests.test_raw_avp_to_combined_mujoco
```

从 production-format `TrackingData` 开始,真实经过 FrameGate、Arming、adapter/calibration/retargeter、nonidentity wrist basis、IK、q17 layout/control。覆盖 wrist-only、hand-only、same-frame arm+hand、duplicate/invalid frame、robot-only 和 tabletop scene;不从 `CanonicalHandState` 中途起跑。

### 18.11 Tabletop scene compatibility

```bash
python -m sg100_teleop.tests.test_combined_scene_compatibility
```

验证 robot-only `nq=nv=nu=17`,tabletop scene 仍只有 17 robot actuators,extra bottle freejoint 未被 robot initialization 覆盖且 q17 control 后保持 finite/valid。它不验证自动抓取或 pickup。

---

## 19. FAST CORE REGRESSION

```bash
cd ~/VisionProTeleop
source .venv/bin/activate

python -m compileall -q sg100_teleop avp_stream
python -m sg100_teleop.tests.test_avp_tracking_freshness
python -m sg100_teleop.tests.test_teleop_arming
python -m sg100_teleop.tests.test_avp_to_tcp_config
python -m sg100_teleop.tests.test_avp_to_tcp_basis_six_axis
python -m sg100_teleop.tests.test_avp_to_tcp_basis_ik
python -m sg100_teleop.tests.test_avp_wrist_mapper
python -m sg100_teleop.tests.test_sg100_retargeter
python -m sg100_teleop.tests.test_sg100_physical_joint_semantics
python -m sg100_teleop.tests.test_avp_feature_extractor
python -m sg100_teleop.tests.test_hand_calibration
python -m sg100_teleop.tests.test_rm65_continuous_tcp_teleop
python -m sg100_teleop.tests.test_raw_avp_to_combined_mujoco
python -m sg100_teleop.tests.test_combined_scene_compatibility
```

需要确认 interactive combined dynamics 时再运行 20 s viewer regression:

```bash
python -m sg100_teleop.sim.run_fake_avp_arm_hand_teleop
```

修改 tracking、arming、mapping、calibration、IK、retargeting、robot layout 或 scene compatibility 后必须重新运行相应 core tests;跨模块改动运行整个列表。

---

## 20. EXTENDED / EXPERIMENTAL VALIDATION

以下测试保留用于 calibration robustness、FK/Jacobian 深层验证、workspace/singularity 和 grasp experiments,但不要求每次 FAST CORE 都运行:

```text
test_calibration_session.py
test_rm65_fk_tcp.py
test_rm65_tcp_jacobian.py
test_rm65_ik_reachable_targets.py
test_rm65_ik.py
test_rm65_ik_z_scan.py
test_rm65_sg100_workspace_sweep.py
test_rm65_sg100_grasp_pose_edges.py
```

- `test_rm65_ik.py`:保留其自身 diagnostic seed,测 6 个 world-frame 平移/旋转目标(±20 mm/±5 deg)。用于分析局部 6D 行为,不定义当前 teleop-ready anchor,也不覆盖完整 workspace。
- `test_rm65_ik_z_scan.py`:**纯诊断,无 PASS/FAIL 断言**。包含 intentionally near-singular / unreachable 的 +Z 目标(DIRECT scan + +Z continuation)。用于分析 workspace limit / singularity / `sigma_min` / condition number。
- `test_rm65_sg100_workspace_sweep.py` 与 `test_rm65_sg100_grasp_pose_edges.py`:耗时 workspace/grasp-reference experiments,不是日常 regression,也不是 autonomous grasp controller。

> **不要要求这两个 local IK diagnostic 的所有目标都 PASS。** 尤其 `test_rm65_ik_z_scan.py` 的部分目标按设计就会 FAIL(见 §11)。

运行(可选,分析用):

```bash
python -m sg100_teleop.tests.test_rm65_ik
python -m sg100_teleop.tests.test_rm65_ik_z_scan
```

---

## 21. Phase 1–6 Validation Summary

| Phase | Purpose | Status | Key result |
|---|---|---|---|
| 1 | Tabletop scene compatibility | PASS | extra freejoint-safe robot initialization/control |
| 2 | Tracking freshness + right-hand validity | PASS | duplicate/untracked frames do not update or refresh timeout |
| 3 | Explicit neutral calibration / arming | PASS | confirmation-gated same-frame wrist/hand anchors; timeout requires re-arm |
| 4 | Configurable AVP-to-TCP basis | PASS | strict proper-rotation YAML, nonidentity six-axis mapper→IK |
| 5 | Raw AVP-format → q17 → MuJoCo | PASS | production-format TrackingData drives robot-only/tabletop arm+hand pipeline |
| 6 | SG100 physical semantic regression | PASS | 11/11 direct + 11/11 canonical physical semantics; no sign mismatch |

Phase 5 also established:wrist-only motion does not materially drive hand articulation;hand-only articulation does not materially drive arm motion;same-frame arm+hand update PASS;robot-only and tabletop MuJoCo PASS;bottle freejoint finite/valid. **NO REAL AVP HARDWARE WAS USED.**

---

## 22. Readiness Matrix

| Capability | Status |
|---|---|
| Transport protocol | PASS |
| Frame freshness | PASS |
| Right-hand validity | PASS |
| Teleop arming | PASS |
| Relative wrist SE(3) | PASS |
| Configurable local basis | PASS |
| RM65 6D IK | PASS |
| Raw AVP hand parsing | PASS |
| Hand calibration framework | PASS |
| Canonical hand representation | PASS |
| SG100 retargeting semantics | PASS |
| SG100 physical MuJoCo semantics | PASS |
| Raw-format q17 integration | PASS |
| Robot-only MuJoCo | PASS |
| Tabletop scene integration | PASS |
| Real AVP axis calibration | **PENDING HARDWARE** |
| Real-user hand calibration | **PENDING HARDWARE** |
| Real AVP latency/jitter | **PENDING HARDWARE** |
| Real combined teleoperation | **PENDING HARDWARE** |
| Real tabletop object grasp | **PENDING HARDWARE** |

PASS 表示当前 repository implementation 的 software/synthetic/MuJoCo regression 通过,不等于 hardware acceptance。

---

## 23. AVP Hardware Bring-up Checklist

```text
 1. Build/install the current visionOS app.
 2. Verify protobuf compatibility, right_hand_valid and sourceFrameSequence.
 3. Check streaming freshness; cached reads must retain sequence/timestamp.
 4. Perform the current user's 10-pose HandCalibration.
 5. Run AVP→TCP six-axis calibration:forward/back, left/right, up/down,
    local Rx/Ry/Rz.
 6. Write and review a hardware-validated basis/translation-scale YAML.
 7. Move RM65 simulation to RM65_TELEOP_READY_Q.
 8. Acquire NEW+VALID+fresh right-hand tracking.
 9. User assumes a comfortable neutral wrist/hand pose.
10. Press C (or future source-agnostic UI trigger) for neutral calibration.
11. Validate hand-only:open, fist, pinch, opposition.
12. Validate small arm translations one axis at a time.
13. Validate small wrist rotations one axis at a time.
14. Validate combined free-space arm-hand teleoperation.
15. Deliberately lose tracking and confirm timeout/disarm/HOLD.
16. Reacquire tracking, confirm no auto-rearm, then recalibrate.
17. Load rm65_sg100_grasp_scene.xml.
18. Approach the object manually without scripted grasp logic.
19. Close fingers manually and evaluate contact/grasp.
20. Lift/manipulate only after all previous acceptance steps pass.
```

> 禁止跳过 user calibration、six-axis basis、tracking-loss/disarm 验证就直接宣称 real AVP combined control 或 tabletop grasp 完成。

### 23.1 Future real-AVP acceptance metrics

Hardware session 应记录但当前尚无真实数值:

| Metric | Suggested measurement |
|---|---|
| Tracking update rate | accepted NEW+VALID frames/s |
| Tracking-loss rate | invalid/timeout events per minute |
| End-to-end latency | sensed motion → visible MuJoCo response |
| Hand command jitter | stationary-pose q_hand RMS/peak-to-peak |
| Repeatability | repeated pose endpoint dispersion |
| Range utilization | per-joint fraction of safe SG100 range |
| Monotonicity | human motion progression vs semantic/q response |

Acceptance motions:`OPEN`,`FIST`,`INDEX FLEX`,`THUMB OPPOSITION`,`PINCH`,`SPREAD`,`COMBINED GRASP`。这些是 future hardware metrics;不得填入 fake/synthetic PASS numbers。

---

## 24. Known Limitations

- 真实 AVP 端到端验证尚未执行(synthetic / fake / sim 已通过)。
- checked-in identity basis/`translation_scale=0.5` 是 **UNCALIBRATED DEVELOPMENT DEFAULT**;真实 AVP wrist-local axis/scale 尚未完成 hardware six-axis calibration。
- thumb opposition / abduction 由 palm-local geometric projection 建模,真实 thumb 3D coupling 可能在真机数据中显现。
- robot/tabletop validation scene 已有明确 collision proxies,但尚无真实 AVP contact、pickup 或 grasp-quality acceptance。
- 只做 right SG100 hand(`--hand` 仅 `choices=["right"]`)。
- simulation 参数(`damping / armature / kp`)不代表真实 motor dynamics,用于稳定 validation backend。
- RM65 当前第一版无额外 arm command filter;IK-failure / tracking-loss 的 HOLD 仅是 simulation validation 策略,非真机安全机制。
- combined model `gravity = 0 0 0`(当前 validation 设定)。

---

## 25. Troubleshooting

### `avp_stream` import fails
检查 `.venv` 是否激活(`source .venv/bin/activate`),以及 `avp_stream` 是否已 `pip install -e .` 安装。

### MuJoCo model fails to load mesh(SG100)
**不要** 改回原始 high-poly STL(超 ~200k faces 限制),也 **不要** 改回 `package://` ROS URI。使用 `meshes_mujoco/` 下简化 mesh。文件名保留厂商原拼写(如 `little_llink2.STL`、`little_linik3.STL`),勿重命名,否则 MJCF mesh 引用失效。

### Calibration file not found
先运行 `python -m sg100_teleop.calibration.calibrate_avp_hand --ip <VISION_PRO_IP>`,或用 `--calibration path/to/your.yaml` 显式指定。

### Tracking returns no hand
检查 AVP IP / 网络连通 / AVP app 与 hand tracking 启动 / **右手** 在 tracking 范围。先用 `python examples/00_hand_streaming.py --ip <VISION_PRO_IP>` 确认 stream。

### Combined model dimension / order error
若 runner 报 `nq/nv/nu != 17` 或 qpos/DoF/actuator order 不一致,检查 `rm65_sg100_sim.xml` 是否被改动。joint order 必须为 `joint_1..6` + SG100 11 joints。

### IK hold 频繁 / TCP 跟踪误差大
按 §11 的限制理解 diagnostic seed;优先检查目标是否接近当前位形奇异区或超出 workspace,再用 `test_rm65_ik_z_scan.py` 分析。**不要** 首先改 IK 参数或 MuJoCo joint axis。

### Hand moves but SG100 direction is wrong
按顺序排查:calibration pose 是否做反 → YAML endpoint 是否合理 → raw feature(`--verbose`)→ `sg100_joint_semantics.yaml` mapping type。**不要** 首先改 MuJoCo joint axis(joint direction 已 sweep 验证)。

---

## 26. Development Rules / Architecture Constraints

1. **不要** 向 `avp_stream/` 写 SG100 / RM65-specific logic。`avp_stream` 是通用 AVP tracking 库,属上游。
2. **不要** 让 `SG100Retargeter` 依赖 AVP joint index。Retargeter 只接受 `CanonicalHandState`。
3. tracking backend 应转换成 `CanonicalHandState`,而不是直接产出 robot joint command。
4. robot-specific joint semantics 放在 SG100 retargeting / config 层(`sg100_joint_semantics.yaml` + `SG100Retargeter.FEATURE_MAPPING`)。
5. **不要** 修改 canonical `sg100.xml` 来解决 simulation-specific physics。simulation 改动应进入 `*_sim.xml`,或经 build pipeline 生成。
6. RM65 IK 只用 RM65 六个 DoF;`RM65IKSolver` 不应感知 SG100 joint。
7. `AVPWristRelativeMapper` 故意独立于 `VisionProStreamer`,只接受 4×4 transform。
8. future Manus(或其它 glove)集成应实现 `Manus → CanonicalHandState`,**而不是** 另复制一套 SG100 retargeter。
9. 不要随意更改:SG100 joint order / range / direction semantics、RM65 IK adaptive damping 参数、combined model joint order、`CanonicalHandState` 的 `[0,1]/[-1,1]` 约定。
10. 不要重新开启 CAD full mesh self-collision(q=0 历史 penetration 由 CAD overlap 引起,非 joint origin 错误)。
11. 修改核心模块后必须运行 [§19](#19-fast-core-regression) 全部 regression。
12. **不允许「顺手重构」**——可追溯性与 regression stability 优先于代码风格统一。

> **项目范围说明**
>
> 本项目中的 `q_goal / q_cmd / qpos / timestamp` 仅用于 teleoperation 调试、仿真验证与 runtime 分析。本项目 **不作为正式 VLA 数据采集系统**;后续 SG100 正式示教数据采集将使用 Manus 数据手套并在独立 Pipeline 完成。本项目也 **不以 RM65 + SG100 真机遥操作为目标**,当前控制 / tracking-loss / IK-failure 策略仅面向 MuJoCo 仿真验证。
