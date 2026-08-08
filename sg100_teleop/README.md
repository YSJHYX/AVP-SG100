# SG100 + Apple Vision Pro Teleoperation

本子项目位于 `sg100_teleop/`,在 VisionProTeleop 仓库内,目标是:

**验证 Apple Vision Pro → SG100 retargeting → MuJoCo simulation teleoperation pipeline。**

本 README 面向新加入项目、但没有参与之前 SG100 URDF/MuJoCo/retargeting 调试的工程人员。
所有命令、路径、参数、默认值均以当前实际源码为准。

---

## 0. TL;DR

**当前阶段:**
- ✅ 已实现并单元测试:AVP hand skeleton → canonical semantic hand state → SG100 retargeting → MuJoCo position control
- ✅ 已实现并单元测试:AVP 几何特征提取的 rigid-transform / scale 不变性
- ✅ 已实现并单元测试:多帧中位数标定 + 异常帧鲁棒性 + YAML 序列化
- ✅ 已实现:AVP live calibration 程序、AVP → SG100 MuJoCo live 程序
- ⚠️ 尚未完成:SG100 真机 backend、RM65 臂、最终 grasp collision physics、真实设备验证

**当前阶段不是:**
- RM65 + SG100 完整臂手 teleoperation
- SG100 真机 teleoperation
- 最终生产级 collision/grasp physics
- VLA 数据采集系统

这些属于后续阶段(见 [§17 Next Development Stages](#17-next-development-stages))。

---

## 1. Overview

SG100 是一个 11 DoF 灵巧手。本子项目使用 Apple Vision Pro 的手部 tracking,将人手动作转换成 **设备无关的 canonical semantic hand representation**,再 retarget 成 SG100 的 absolute joint position command。

关键设计原则:

**本子项目不是:**

```
AVP joint index
    ↓
SG100 joint index
```

**而是:**

```
AVP skeleton
    ↓
human hand semantic features
    ↓
CanonicalHandState
    ↓
SG100 morphology
```

这样未来 Manus glove 或其它 tracking backend 可以直接复用后半段:

```
CanonicalHandState
    ↓
SG100Retargeter
```

完整 pipeline:

```
Apple Vision Pro
        ↓
VisionProStreamer.get_latest()
        ↓
AVP Hand Skeleton  (Nx4x4, world frame)
        ↓
AVPHandFeatureExtractor
        ↓
AVPRawHandFeatures  (geometric, 未归一化)
        ↓
User-specific Hand Calibration
        ↓
CanonicalHandState  (semantic, [-1,1] / [0,1])
        ↓
SG100Retargeter
        ↓
q_goal[11]  (radians, absolute joint position)
        ↓
JointCommandFilter
        ↓
q_cmd[11]
        ↓
MuJoCo Position Actuators
        ↓
SG100 Simulation
        ↓
VisionProStreamer.update_sim()
        ↓
Vision Pro AR Visualization
```

---

## 2. Current Project Status

| 模块 | 状态 |
|---|---|
| SG100 URDF 静态审计 | ✅ 完成 |
| URDF → MuJoCo-compatible URDF 转换 | ✅ 完成 |
| 高面数 STL 简化 | ✅ 完成(`meshes_mujoco/`) |
| Canonical MJCF 生成 | ✅ 完成(`mjcf/sg100.xml`) |
| Simulation MJCF 生成 | ✅ 完成(`mjcf/sg100_sim.xml`) |
| 11 个 position actuators | ✅ 完成 |
| CAD visual mesh 关闭 collision(`contype=0 conaffinity=0`) | ✅ 完成 |
| q=0 collision 问题诊断(CAD overlap,非 joint origin 问题) | ✅ 完成 |
| Controlled dynamics 验证 | ✅ 完成 |
| 11 DoF semantic motion audit(对照真机运动方向) | ✅ 完成 |
| `CanonicalHandState` | ✅ 完成 + 单元测试 |
| `SG100Retargeter` | ✅ 完成 + 单元测试 |
| Mock retargeting 演示 | ✅ 完成 |
| AVP raw geometry feature extraction | ✅ 完成 + 单元测试 |
| Rigid-transform 不变性测试 | ✅ 完成 |
| Hand-scale 不变性测试 | ✅ 完成 |
| Calibration normalization | ✅ 完成 + 单元测试 |
| Multi-frame median calibration | ✅ 完成 + 单元测试 |
| Outlier robustness | ✅ 完成 + 单元测试 |
| YAML calibration serialization | ✅ 完成 + 单元测试 |
| AVP live calibration 程序 | ✅ 完成(尚未真实设备验证) |
| AVP → SG100 MuJoCo live 程序 | ✅ 完成(尚未真实设备验证) |

**尚未完成:**

| 模块 | 状态 |
|---|---|
| 最终简化 collision geometries | ❌ 未完成 |
| Object grasp physics | ❌ 未完成 |
| SG100 hardware backend | ❌ 未完成 |
| RM65 集成 | ❌ 未完成 |
| Real robot 安全状态机 | ❌ 未完成 |
| AVP 真实设备验证 | ❌ 未执行 |

> 注意:不要把「代码已经实现」和「真实设备已经验证」混为一谈。AVP live 程序虽已实现并仿真验证,但尚未在真实 SG100 / 真实 AVP 环境下完成验证。

---

## 3. Directory Structure

当前 `sg100_teleop/` 真实目录树:

```
sg100_teleop/
├── __init__.py
├── assets/
│   └── sg100/
│       ├── config/
│       │   └── joint_names_SG100V3.0_R_URDF_3D.yaml
│       ├── meshes/                    # 原始 CAD STL
│       ├── meshes_mujoco/             # MuJoCo-friendly 简化 STL
│       ├── mjcf/
│       │   ├── sg100.xml              # canonical kinematic reference
│       │   └── sg100_sim.xml          # 实际仿真 / AVP teleop 模型
│       ├── urdf/
│       │   ├── SG100V3.0_R_URDF_3D.urdf       # 原始 URDF(不改)
│       │   └── SG100V3.0_R_URDF_3D_mujoco.urdf # MuJoCo 专用 URDF
│       ├── CMakeLists.txt
│       ├── launch/
│       ├── package.xml
│       └── export.log
├── calibration/
│   ├── __init__.py
│   ├── calibrate_avp_hand.py          # AVP live calibration 程序
│   ├── io.py                          # YAML save/load
│   └── session.py                     # multi-frame 采集 + median build
├── config/
│   ├── sg100_joint_semantics.yaml     # SG100 joint 顺序/范围/语义
│   └── avp_right_calibration.yaml     # 标定后生成(默认输出)
├── control/
│   ├── __init__.py
│   └── joint_command_filter.py        # low-pass + velocity limit
├── hand/
│   ├── __init__.py
│   ├── canonical_hand.py              # CanonicalHandState
│   └── hand_calibration.py            # raw → canonical normalization
├── retargeting/
│   ├── __init__.py
│   └── sg100_retargeter.py            # canonical → SG100 q_target
├── sim/
│   ├── __init__.py
│   ├── build_sg100_sim.py             # 生成 sg100_sim.xml
│   ├── demo_mock_retargeting.py       # 不接 AVP 的 retarget 演示
│   ├── run_avp_sg100_sim.py           # AVP live teleop 主程序
│   ├── sweep_sg100_joint.py           # 单关节方向/限位 sweep
│   └── view_sg100_controlled.py       # controlled viewer
├── tracking/
│   ├── __init__.py
│   ├── avp_adapter.py                 # HandData → raw → canonical
│   ├── avp_hand_adapter.py            # AVP_JOINT_INDEX + feature extractor
│   └── avp_source.py                  # TrackingData → Nx4x4 skeleton
└── tests/
    ├── test_avp_feature_extractor.py
    ├── test_calibration_session.py
    ├── test_hand_calibration.py
    └── test_sg100_retargeter.py
```

### `assets/sg100/urdf/`

- `SG100V3.0_R_URDF_3D.urdf`:**原始 SG100 URDF**,直接来自厂商。**不要修改**,作为审计与回溯基准。
- `SG100V3.0_R_URDF_3D_mujoco.urdf`:MuJoCo 专用副本。之所以单独保留一份,是因为原始 URDF 使用 `package://...` ROS URI,MuJoCo 无法解析,需要把 mesh 路径改成本地相对路径。
- 原始 URDF 保留不动的理由:任何 vendor 后续更新可以 diff 对照,避免 MuJoCo hack 污染上游定义。

### `assets/sg100/meshes/`

原始 CAD / 视觉 STL。**不要**把这些 mesh 直接喂给 MuJoCo。

### `assets/sg100/meshes_mujoco/`

MuJoCo-friendly 简化 mesh。历史问题:原始 `base_link.STL` 约有 **481k faces**,超过 MuJoCo STL decoder 约 200k faces 的限制,加载会失败。因此生成简化版本。

> 注:文件名保留了厂商原始命名(包括原拼写 `little_llink2.STL`、`little_linik3.STL`),不要重命名,否则 MJCF 里的 `mesh file=` 引用会失效。

当前该目录主要服务于 **visualization**(`sg100_sim.xml` 中 CAD mesh 不参与 collision,见 [§14 Simulation Model Notes](#14-simulation-model-notes))。

### `assets/sg100/mjcf/sg100.xml`

**Canonical kinematic reference MJCF。**

- 模型名:`SG100V3.0_R_URDF_3D`
- 11 个 joint + inertial + CAD visual mesh
- **不**含 actuators、damping、armature、timestep、gravity 等 simulation-specific 参数
- **不要**把 simulation-specific hack 加回这个文件(见 [§19 Development Rules](#19-development-rules--architecture-constraints))

### `assets/sg100/mjcf/sg100_sim.xml`

**目前真正用于 controlled simulation / AVP teleoperation 的模型。** 由 `sim/build_sg100_sim.py` 从 `sg100.xml` 自动生成,不要手改。

主要参数:

| 参数 | 值 |
|---|---|
| model name | `SG100V3_sim` |
| timestep | `0.002`(500 Hz) |
| integrator | `implicitfast` |
| gravity | `0 0 -9.81` |
| joint damping | `0.002` |
| joint armature | `0.001` |
| position kp | `2.0` |
| position dampratio | `1.0`(近似 critical damping) |
| joints | 11 |
| actuators | 11 position(actuator 名 `<joint>_pos`) |
| CAD visual mesh | `contype=0 conaffinity=0`,**仅可视化** |
| grasp collision geometry | **尚未完成** |

---

## 4. SG100 Joint Order and Semantics

下表来自 `config/sg100_joint_semantics.yaml`(权威源),joint axis 来自 `sg100.xml`/`sg100_sim.xml`。

**Joint order(数组索引顺序,`q[11]` 即此顺序):**

| idx | joint | range (rad) | axis | semantic feature | mapping type |
|----|-------|-------------|------|------------------|--------------|
| 0 | `thumb_j1`  | `[0.0, 2.182]`    | `0 0 -1` | `thumb_opposition`               | positive |
| 1 | `thumb_j2`  | `[-2.61, 0.0]`    | `0 0 1`  | `thumb_abduction`                | negative |
| 2 | `thumb_j3`  | `[-1.048, 1.57]`  | `0 0 1`  | `thumb_distal` (bidirectional)   | bidirectional |
| 3 | `index_j1`  | `[0.0, 1.57]`     | `0 0 -1` | `index_adduction`                | positive |
| 4 | `index_j2`  | `[0.0, 2.18]`     | `0 0 1`  | `index_flexion`                  | positive |
| 5 | `index_j3`  | `[-1.05, 1.57]`   | `0 0 1`  | `index_distal` (bidirectional)   | bidirectional |
| 6 | `middle_j1` | `[-2.18, 0.0]`    | `0 0 -1` | `middle_flexion`                 | negative |
| 7 | `middle_j2` | `[-1.05, 1.57]`   | `0 0 1`  | `middle_distal` (bidirectional)  | bidirectional |
| 8 | `little_j1` | `[-3.14, 0.0]`    | `0 0 -1` | `little_outward`                 | negative |
| 9 | `little_j2` | `[0.0, 2.18]`     | `0 0 1`  | `little_flexion`                 | positive |
| 10| `little_j3` | `[-1.05, 1.57]`   | `0 0 1`  | `little_distal` (bidirectional)  | bidirectional |

### 已确认的运动方向语义

> 以下方向均已经过 SG100 joint semantic sweep 视觉确认,与真机运动方向一致。

#### Thumb
- `thumb_j1`:`0 → positive`,大拇指绕掌心方向内旋(`positive_direction: rotate_toward_palm`)。
- `thumb_j2`:`0 → negative`,大拇指向外、向下展(`negative_direction: rotate_outward_and_downward`)。
- `thumb_j3`:双向。
  - `negative → extension_outward`:拇指远端向外伸展。
  - `positive → flexion_toward_palm`:拇指远端向掌心弯曲。

#### Index
- `index_j1`:`0 → positive`,食指向拇指方向横向偏摆(`positive_direction: toward_thumb`)。
- `index_j2`:`0 → positive`,食指朝掌心弯曲(`positive_direction: flexion_toward_palm`)。
- `index_j3`:双向。
  - `negative → extension_toward_dorsal_side`:远端朝手背伸展。
  - `positive → flexion_toward_palm`:远端朝掌心弯曲。

#### Middle
- `middle_j1`:`0 → negative`,中指朝掌心弯曲(`negative_direction: flexion_toward_palm`)。
- `middle_j2`:双向。
  - `negative → extension_toward_dorsal_side`
  - `positive → flexion_toward_palm`

#### Little
- `little_j1`:`0 → negative`,小指向外、向下(`negative_direction: rotate_outward_and_downward`)。
- `little_j2`:`0 → positive`,小指朝掌心弯曲。
- `little_j3`:双向。
  - `negative → extension_toward_dorsal_side`
  - `positive → flexion_toward_palm`

### 为什么用 semantic naming,而不是简单 MCP/PIP/DIP

把所有 SG100 joint 强行对应到人体 MCP/PIP/DIP 会出现问题:不同 finger 的 SG100 正方向并不统一(有些是 flexion 在正方向,有些在负方向),而且 thumb 与 little 还有 adduction/abduction/opposition 这种非典型 flexion 的 DoF。

因此本子项目对每个 SG100 joint 单独标注:
- 它驱动哪个 canonical feature
- 该 feature 的 raw 值往哪个方向映射到 SG100 joint range

这些信息全部集中在 `config/sg100_joint_semantics.yaml` 与 `retargeting/sg100_retargeter.py` 的 `FEATURE_MAPPING`,而 **不 scatter/hardcode 到多个 Python 文件**。

---

## 5. Architecture

### Tracking layer — `tracking/avp_source.py`

职责:把 `VisionProStreamer.get_latest()` 的返回值统一转换成 world-frame `Nx4x4` hand skeleton。

`get_hand_from_tracking(tracking, side="right")` 同时兼容两套 API:

1. **当前 `TrackingData` API**:`tracking.right`(`avp_stream.HandData`,`(27, 4, 4)`)。要求 `shape[0] >= 25` 且 `shape[1:] == (4, 4)`。
2. **旧 dict API**:`tracking["right_wrist"]` + `tracking["right_arm"]`,组合成 `wrist @ arm`。

`avp_stream` 的 HandData 至少包含前 25 个 hand joints(index 0–24),后两个(index 25 `forearmWrist`、26 `forearmArm`)为 forearm joints,retargeting 不直接使用。

### AVP Feature Extraction — `tracking/avp_hand_adapter.py`

包含:

- `AVP_JOINT_INDEX`:25/27-joint 命名 → index 映射。
- `AVPRawHandFeatures`:11 个 raw 几何 feature 的 dataclass。
- `AVPHandFeatureExtractor`:从 `Nx4x4` skeleton 提取 raw features。

核心设计:

```
world-frame skeleton
    ↓  (乘以 wrist_world 的逆)
wrist-local skeleton
    ↓  (构建 palm frame: x=little→index, y=wrist→middle_knuckle, z=palm normal)
palm-relative geometric features
```

这样保证:

- **手整体平移** 不会改变 finger commands
- **手腕旋转** 不会改变 finger shape representation
- **手掌尺寸变化**(通过 `palm_width` 归一化)尽可能不影响 retargeting

> 注意:raw features **不是** `[0,1]`。例如 `thumb_opposition = 1.3`、`index_flexion = 0.8 rad` 都是完全合法的 raw 值。只有经过 calibration 后才变成 canonical 值。

---

## 6. CanonicalHandState

文件:`hand/canonical_hand.py`。

11 个 canonical semantic features:

| Finger | Features |
|--------|----------|
| Thumb  | `thumb_opposition`, `thumb_abduction`, `thumb_distal` |
| Index  | `index_adduction`, `index_flexion`, `index_distal` |
| Middle | `middle_flexion`, `middle_distal` |
| Little | `little_outward`, `little_flexion`, `little_distal` |

### 约定

**Unidirectional:**

```
0.0 = neutral
1.0 = calibrated active endpoint
```

unidirectional features:`thumb_opposition`, `thumb_abduction`, `index_adduction`, `index_flexion`, `middle_flexion`, `little_outward`, `little_flexion`。

**Bidirectional:**

```
-1.0 = calibrated negative semantic direction
 0.0 = neutral
+1.0 = calibrated positive semantic direction
```

bidirectional features:`thumb_distal`, `index_distal`, `middle_distal`, `little_distal`(均为 distal flexion/extension)。

> `CanonicalHandState` **不属于** Vision Pro、Manus、MuJoCo 或 SG100。它是 tracking-backend independent 的中间表示。`CanonicalHandState.neutral()` 返回全 0。

---

## 7. SG100 Retargeter

文件:`retargeting/sg100_retargeter.py`。

- **输入:** `CanonicalHandState`
- **输出:** `q_target.shape == (11,)`,单位 **radians**,表示 **absolute joint position**。

Joint 顺序与 range 全部从 `config/sg100_joint_semantics.yaml` 加载;feature → joint 的映射由 `FEATURE_MAPPING` 固定,包含 3 种 mapping type:

- `positive`:canonical `[0,1]` → SG100 `[0, upper]`
- `negative`:canonical `[0,1]` → SG100 `[lower, 0]`(即 `value * lower`,lower 为负)
- `bidirectional`:canonical `[-1,1]` → 不对称 SG100 range
  - `value >= 0`:`value * upper`
  - `value < 0`:`(-value) * lower`

最终再 `np.clip(q, lower, upper)` 保证 joint limit。

这层 **不依赖** 任何 AVP joint index 或 tracking backend。未来只要实现:

```
Manus → CanonicalHandState
```

就可以直接复用 `SG100Retargeter`。同理,`SG100Retargeter` 也可以直接接入:

```
SG100Retargeter → MuJoCoBackend
SG100Retargeter → SG100HardwareBackend
```

而无需改动上层 human hand representation。

---

## 8. JointCommandFilter

文件:`control/joint_command_filter.py`。

动机:AVP tracking 是低频更新,而 MuJoCo physics 当前是高频运行。需要把低频 `q_goal` 平滑送到高频 `q_cmd`。

```
AVP tracking (~60 Hz)
       ↓
q_goal
       ↓
first-order low-pass + velocity limit
       ↓
q_cmd
       ↓
MuJoCo (~500 Hz)
```

参数:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `dof` | — | 自由度(SG100 = 11) |
| `tau` | `0.06` | 低通时间常数(秒) |
| `max_velocity` | `4.0` | 最大 command slew(rad/s) |

低通系数使用连续时间形式:

```
alpha = 1 - exp(-dt / tau)
```

相对于固定 `alpha`,这种形式在不同 timestep 下具有一致的时间常数;若 physics timestep 改变,filter 行为不会漂移。随后再做 `max_velocity * dt` 的速度限幅。

本 filter **不含** SG100-specific mapping 逻辑,纯通用。

---

## 9. Configuration

### `config/sg100_joint_semantics.yaml`

权威的 SG100 joint 语义源。包含:

- `joint_order`(11 个 joint 的固定顺序)
- 每个 joint 的 `range`、`zero`、`semantic`、`positive_direction` / `negative_direction`

> 原则:robot-specific 的 semantic truth 应放在这个 config 里,而 **不 scatter/hardcode 到多个 Python 文件**。`SG100Retargeter` 在 `__init__` 时加载它。

### `config/avp_right_calibration.yaml`

⚠️ **这个文件不是仓库默认的「通用参数」。**

它是 **特定 user + 特定 tracking device + 特定 hand** 的 calibration result,默认由 calibration 命令生成:

```
sg100_teleop/config/avp_right_calibration.yaml
```

默认 output 路径(`calibrate_avp_hand.py` 的 `--output` 默认值)与 live runner 默认加载路径(`run_avp_sg100_sim.py` 的 `--calibration` 默认值)**完全一致**,都是 `sg100_teleop/config/avp_right_calibration.yaml`。

文件内容包含:

- `schema_version`(当前 `1`)
- `metadata`:创建时间戳、`device`、`tracking_library`、`hand`、`sampling_hz`、`duration_per_pose_sec`、`samples_per_pose`
- `unidirectional`:每个 feature 的 `neutral` / `active` endpoint
- `bidirectional`:每个 feature 的 `negative` / `neutral` / `positive` endpoint

> **不要手工修改 calibration value**,除非正在调试 feature extraction。重新标定比手工编辑可靠。

---

## 10. Apple Vision Pro Hand Calibration Guide

### 为什么需要 calibration

不同用户存在:

- 手掌尺寸差异
- 舒适活动范围差异
- 动作风格差异
- AVP tracking 几何差异

因此不能直接把 raw angle 或 raw thumb position 映射到 SG100 full range。Pipeline 是:

```
AVP Raw Features
       ↓
user calibration
       ↓
Canonical [-1,1] / [0,1]
       ↓
SG100 joint range
```

### Calibration Command

```
cd ~/VisionProTeleop
source .venv/bin/activate

python -m sg100_teleop.calibration.calibrate_avp_hand \
  --ip <VISION_PRO_IP>
```

完整 CLI 参数(以 `calibrate_avp_hand.py` 源码为准):

| 参数 | required | 默认值 | 含义 |
|------|----------|--------|------|
| `--ip` | ✅ | — | Apple Vision Pro IP 地址 |
| `--hand` | ❌ | `right` | 标定用手。当前仅支持 `right`(与已验证的 SG100 morphology 对应) |
| `--duration` | ❌ | `1.5` | 每个 pose 采样时长(秒) |
| `--hz` | ❌ | `60.0` | 采样频率(Hz) |
| `--countdown` | ❌ | `2` | 每个 pose 前的倒数秒数 |
| `--output` | ❌ | `sg100_teleop/config/avp_right_calibration.yaml` | 输出 YAML 路径 |
| `--verbose` | ❌ | off | 输出详细日志 |

程序会对每个 pose 检查有效帧数:若 `valid < max(20, int(duration * hz * 0.5))`(默认 45 帧),会抛 `RuntimeError` 要求重做。

### Calibration Pose Guide

按程序实际执行顺序(`calibrate_avp_hand.py` 中 `POSE_INSTRUCTIONS`)介绍每个 pose。

每个 pose 共同要求:
1. 按描述摆出手势
2. 该动作用于标定某个/某些 canonical feature
3. 对应驱动哪个 SG100 semantic DoF 见下文
4. **不需要做到生理极限**,只需舒适、稳定、可重复
5. 程序会采集多帧(默认 1.5 s × 60 Hz ≈ 90 帧),不是单帧

#### 1. `neutral`
- 动作:自然张开手掌,手指舒适伸直。
- 目标:建立 **所有 feature** 的 neutral baseline。

#### 2. `thumb_opposition`
- 动作:大拇指向掌心内部移动。
- 标定:`thumb_opposition`(unidirectional)。
- 主要驱动:`thumb_j1`(positive direction)。

#### 3. `thumb_abduction`
- 动作:大拇指向外、向下展开。
- 标定:`thumb_abduction`(unidirectional)。
- 主要驱动:`thumb_j2`(negative direction)。

#### 4. `thumb_distal_negative`
- 动作:拇指远端节段向外伸展。
- 标定:`thumb_distal` 的 `negative` endpoint → canonical `-1`。
- 主要驱动:`thumb_j3` 的 negative direction。

#### 5. `thumb_distal_positive`
- 动作:拇指远端节段向内弯曲。
- 标定:`thumb_distal` 的 `positive` endpoint → canonical `+1`。

#### 6. `index_adduction`
- 动作:食指横向向拇指方向偏摆。
- 标定:`index_adduction = 1`。
- 主要驱动:`index_j1`(positive direction)。

#### 7. `finger_flexion`
- 动作:index / middle / little 同时朝掌心弯曲。
- 同时标定:
  - `index_flexion` → `index_j2`
  - `middle_flexion` → `middle_j1`
  - `little_flexion` → `little_j2`

#### 8. `finger_distal_negative`
- 动作:远端关节朝手背方向伸展。
- 标定:
  - `index_distal = -1`
  - `middle_distal = -1`
  - `little_distal = -1`

#### 9. `finger_distal_positive`
- 动作:远端关节向掌心方向弯曲。
- 标定:
  - `index_distal = +1`
  - `middle_distal = +1`
  - `little_distal = +1`

#### 10. `little_outward`
- 动作:小指向外/向下展开。
- 标定:`little_outward = 1`。
- 主要驱动:`little_j1`(negative direction)。

### Sampling Behavior

- 每个 pose 不是只读取单帧,而是采集一段时间(默认 `--duration 1.5` s × `--hz 60` ≈ 90 帧)。
- 对每个 raw feature 取 **median**(`session.py` 中 `pose_median`)。
- median 用于降低 tracking jitter 与偶发异常帧的影响。
- outlier 鲁棒性已在 `test_calibration_session.py` 中验证(每 pose 100 帧 + 1 个极端 outlier,median 结果不受影响)。

### Calibration Output

输出文件默认:`sg100_teleop/config/avp_right_calibration.yaml`。

示例结构(数值为占位,**不要**当作真实标定值):

```yaml
schema_version: 1

metadata:
  created_at_utc: "2026-08-08T12:00:00+00:00"
  device: apple_vision_pro
  tracking_library: avp_stream
  hand: right
  sampling_hz: 60.0
  duration_per_pose_sec: 1.5
  samples_per_pose:
    neutral: 88
    thumb_opposition: 90
    # ...

unidirectional:
  thumb_opposition:
    neutral: ...
    active: ...
  index_flexion:
    neutral: ...
    active: ...
  # ...

bidirectional:
  index_distal:
    negative: ...
    neutral: ...
    positive: ...
  # ...
```

> raw 数值方向可能与 semantic 方向不一致。例如 `thumb_opposition` 的 `active` raw 值可能 **小于** `neutral`(见 `test_hand_calibration.py`),calibration 仍会正确输出 canonical `+1`。这是设计预期,不是 bug。

### How Calibration Is Loaded During Teleoperation

Live runner `sim/run_avp_sg100_sim.py` 在启动时通过 `load_calibration(...)` 加载 calibration YAML。

默认运行:

```
python -m sg100_teleop.sim.run_avp_sg100_sim \
  --ip <VISION_PRO_IP>
```

使用自定义 calibration:

```
python -m sg100_teleop.sim.run_avp_sg100_sim \
  --ip <VISION_PRO_IP> \
  --calibration path/to/custom_calibration.yaml
```

> **运行 live AVP teleoperation 前必须存在 calibration 文件。** 若 `--calibration` 指定文件不存在,程序直接抛 `FileNotFoundError` 并提示 "Run calibrate_avp_hand first.",**不会** 偷偷使用 fake/default mapping。

---

## 11. Run Order / Quick Start

从新 clone 或当前环境开始的推荐执行顺序。**不要** 随机跑 tests。

### Step 0 — Activate environment

```
cd ~/VisionProTeleop
source .venv/bin/activate
```

### Step 1 — Verify MuJoCo and avp_stream

```
python - <<'PY'
import mujoco
import avp_stream

print("MuJoCo:", mujoco.__version__)
print("avp_stream imported successfully")
PY
```

### Step 2 — Build SG100 simulation MJCF

从 canonical `sg100.xml` 生成 `sg100_sim.xml`(添加 timestep、damping、armature、11 position actuators,关闭 CAD collision)。

```
python sg100_teleop/sim/build_sg100_sim.py
```

预期输出会列出 joints=11、actuators=11,以及写入路径 `sg100_teleop/assets/sg100/mjcf/sg100_sim.xml`。

> 这一步只生成 simulation-specific 参数,**不** 修改 canonical `sg100.xml`。`sg100_sim.xml` 已在仓库中,通常不需要重新生成,除非改了 `sg100.xml` 或 build 脚本。

### Step 3 — Run SG100 controlled viewer

仓库目前没有独立的 model validation script,controlled viewer 用于检查 11 DoF position control 是否稳定:

```
python sg100_teleop/sim/view_sg100_controlled.py
```

该脚本加载 `sg100_sim.xml`,发送 `q_target = zeros(11)`,在 MuJoCo passive viewer 中验证 model 加载与零位稳定性。

### Step 4 — Optional joint semantic sweep

视觉验证单个 joint 的方向与限位是否与真机 SG100 一致:

```
python sg100_teleop/sim/sweep_sg100_joint.py \
  --joint index_j1
```

可用 joint(11 个):`thumb_j1`、`thumb_j2`、`thumb_j3`、`index_j1`、`index_j2`、`index_j3`、`middle_j1`、`middle_j2`、`little_j1`、`little_j2`、`little_j3`。

CLI 参数:

| 参数 | required | 默认值 | 含义 |
|------|----------|--------|------|
| `--joint` | ✅ | — | 单个 joint 名 |
| `--segment-time` | ❌ | `2.0` | 每段插值时长(秒) |

> 该 sweep 不是日常 teleop 必跑项,主要用来确认 joint axis 已正确(`0 → lower → 0 → upper → 0`)。

### Step 5 — Run regression tests

```
python -m sg100_teleop.tests.test_sg100_retargeter
python -m sg100_teleop.tests.test_avp_feature_extractor
python -m sg100_teleop.tests.test_hand_calibration
python -m sg100_teleop.tests.test_calibration_session
```

全部预期 `PASS`。详细说明见 [§13 Tests](#13-tests)。

### Step 6 — Optional mock retargeting

不连接 Vision Pro,验证 `CanonicalHandState → SG100Retargeter → MuJoCo` 全链路:

```
python -m sg100_teleop.sim.demo_mock_retargeting
```

会循环演示 4 个 pose:`neutral` → `open` → `fist` → `pinch`。

### Step 7 — Connect Apple Vision Pro

确认 PC 与 AVP 网络可通信。用户自行提供 IP,替换 `<VISION_PRO_IP>`。

先用 upstream tracking example 确认 stream 正常:

```
python examples/00_hand_streaming.py \
  --ip <VISION_PRO_IP>
```

若该 example 能持续打印 tracking 数据,说明 AVP 连接正常。

### Step 8 — Perform calibration

运行 calibration 命令,详细流程见 [§10 Calibration Guide](#10-apple-vision-pro-hand-calibration-guide)。

```
python -m sg100_teleop.calibration.calibrate_avp_hand \
  --ip <VISION_PRO_IP>
```

### Step 9 — Verify generated calibration

```
cat sg100_teleop/config/avp_right_calibration.yaml
```

确认每个 feature 都有合理的 neutral/active(或 negative/neutral/positive)endpoint。

### Step 10 — Run AVP → SG100 MuJoCo teleoperation

```
python -m sg100_teleop.sim.run_avp_sg100_sim \
  --ip <VISION_PRO_IP> \
  --verbose
```

运行时 pipeline 见 [§1 Overview](#1-overview)。完整 CLI 参数见 [§12 Live Teleoperation CLI Parameters](#12-live-teleoperation-cli-parameters)。

---

## 12. Live Teleoperation CLI Parameters

`sim/run_avp_sg100_sim.py` 的完整 CLI(以源码为准):

| 参数 | required | 默认值 | 含义 |
|------|----------|--------|------|
| `--ip` | ✅ | — | Apple Vision Pro IP |
| `--model` | ❌ | `sg100_teleop/assets/sg100/mjcf/sg100_sim.xml` | MuJoCo SG100 仿真 MJCF |
| `--calibration` | ❌ | `sg100_teleop/config/avp_right_calibration.yaml` | 用户 calibration YAML |
| `--port` | ❌ | `50051` | MuJoCo / AVP streaming 相关 gRPC port |
| `--tracking-hz` | ❌ | `60.0` | AVP goal 更新频率(程序 polling/update target) |
| `--stream-hz` | ❌ | `60.0` | MuJoCo state 回传 AVP 的频率 |
| `--filter-tau` | ❌ | `0.06` | SG100 q command low-pass 时间常数(秒) |
| `--max-velocity` | ❌ | `4.0` | SG100 模拟 command slew 限速(rad/s) |
| `--tracking-timeout` | ❌ | `0.5` | 多少秒未收到有效 hand tracking 后报告 tracking lost |
| `--attach` | ❌ | `0.0 0.7 0.8 0.0` | MuJoCo scene 在 AVP AR 中的放置 `X Y Z YAW_DEG` |
| `--verbose` | ❌ | off | 低频打印 `q_goal` / `q_cmd` / `qpos` |

参数详解:

### `--ip`
Apple Vision Pro 的 IP。

### `--model`
MuJoCo SG100 仿真 MJCF。runner 启动时会检查 `model.nq == 11` 且 `model.nu == 11`,否则抛 `RuntimeError`。

### `--calibration`
启动时加载的用户 calibration YAML。文件不存在则拒绝启动。

### `--port`
MuJoCo AR streaming 的 gRPC port,传给 `VisionProStreamer.configure_mujoco(grpc_port=...)`。

### `--tracking-hz`
程序 polling `streamer.get_latest()` 的频率。**不保证** AVP 必然以该频率产生数据,这只是 runner 的目标更新频率。

### `--stream-hz`
runner 调用 `streamer.update_sim()` 把 MuJoCo 状态回传 AVP 的频率。

### `--filter-tau`
SG100 q command 低通时间常数,直接传给 `JointCommandFilter(tau=...)`。

### `--max-velocity`
模拟 command 的 slew/velocity 限速,单位 rad/s,传给 `JointCommandFilter(max_velocity=...)`。

### `--tracking-timeout`
超过该秒数未收到有效 hand tracking 时报告 lost。**当前 simulation 策略:hold last `q_goal`,不回零**(见 [§15 Tracking Loss Behavior](#15-tracking-loss-behavior))。

> ⚠️ 这只是 simulation validation behavior。真实 SG100 backend 后续必须升级为安全状态机(例如 `ACTIVE → HOLD → SAFE_STOP`)。

### `--attach`
MuJoCo scene 在 Vision Pro AR 中的放置,格式 `x y z yaw_deg`(4 元素)。runner 内部转成 7 元素 quaternion 传给 `configure_mujoco(relative_to=...)`。

> **不要** 把 `--attach` 误认为 SG100 joint configuration。它只影响 AR 场景摆放。

---

## 13. Tests

所有 test 以 `python -m sg100_teleop.tests.<name>` 运行,每个 test 都是独立 `main()` 脚本,`PASS` 即通过。

| 测试 | command |
|------|---------|
| retargeter 一致性 | `python -m sg100_teleop.tests.test_sg100_retargeter` |
| feature extractor 不变性 | `python -m sg100_teleop.tests.test_avp_feature_extractor` |
| calibration normalization | `python -m sg100_teleop.tests.test_hand_calibration` |
| calibration session 鲁棒性 | `python -m sg100_teleop.tests.test_calibration_session` |

### `test_sg100_retargeter.py`

验证三件事:

1. **Neutral**:`CanonicalHandState.neutral()` → `q_target == zeros(11)`。
2. **Full semantic activations**:所有 feature 取最大值时,canonical full motion 正确映射到 SG100 不对称 joint limit(对照硬编码 expected 数组,精确到 `1e-6`)。
3. **Bidirectional distal**:4 个 distal feature 取 `-1` 时,对应 joint 到达 `lower` 端(如 `thumb_j3 → -1.048`、`index_j3 → -1.05` 等)。

PASS 表示:`CanonicalHandState → SG100Retargeter` 的 mapping 内部一致,joint range 与 config 严格匹配。

### `test_avp_feature_extractor.py`

用 synthetic right-hand-like 几何验证三个核心不变性:

1. **Rigid-transform invariance**:对整只手施加旋转 `(37, -21, 64)°` + 平移 `(0.8, -1.2, 0.5)`,raw features max diff `< 1e-8`。
   - 意义:用户移动手臂或转动手腕,不会意外改变 SG100 finger posture。
2. **Scale invariance**:整只手放大 `1.7×`,raw features max diff `< 1e-8`(角度天然 scale-invariant;thumb 坐标已除以 `palm_width`)。
   - 意义:不同用户手掌尺寸不会直接造成完全不同 SG100 posture。
3. **Finger motion response**:synthetic index 弯曲后,`index_flexion` 增量 `> 0.3`。
   - 意义:feature extractor 对真实 finger motion 有响应。

PASS 表示:AVP geometric feature extractor 的核心几何性质成立。

### `test_hand_calibration.py`

验证:

- neutral raw → canonical `0`
- active endpoint → canonical `1`
- negative distal endpoint → canonical `-1`

包含 raw 数值方向与 semantic 方向不一致的用例(例如 `thumb_opposition` 的 `active=0.40` 比 `neutral=1.40` 小,但 calibration 仍输出 `+1`)。bidirectional 还会校验 `negative` 与 `positive` 必须在 `neutral` 两侧(否则 `BidirectionalCalibration.__post_init__` 抛错)。

PASS 表示:raw features → `CanonicalHandState` 的 normalization 正确。

### `test_calibration_session.py`

详细介绍:

- 全部 10 个 required poses 都被采集
- 每个 pose 注入 **100 帧带噪声(`σ=0.01`)样本 + 1 个极端 outlier(全部 +100)**
- `pose_median` 聚合后,outlier 不影响结果
- `session.build()` 生成 `HandCalibration`
- 中性 pose normalize 后 `abs(value) < 0.05`
- finger_flexion pose normalize 后 `index_flexion` / `middle_flexion` 接近 `1.0`
- YAML `save_calibration` → `load_calibration` roundtrip 后,对同一 raw input 的 normalize 结果完全一致(`as_dict() == as_dict()`)

PASS 表示:multi-frame median calibration 对异常帧具有基础鲁棒性,且 serialization 不改变 mapping result。

> 注:这些是 **synthetic / mock** 测试,不是 AVP hardware validation。

---

## 14. Simulation Model Notes

历史问题:原始 URDF 中 **visual mesh == collision mesh**。在 `q=0` 时 MuJoCo 检测到多处深度 penetration。

经定位,问题出在:

```
base_link / palm CAD
        ↔
finger root CAD
```

存在大量 CAD overlap。**这不是 joint origin / zero pose 错误**,而是厂商 CAD 本身存在的 mesh 互相穿插。

因此当前 `sg100_sim.xml` 的处理:

```
CAD STL = visual only
contype = 0
conaffinity = 0
```

即所有 CAD mesh **不参与任何 contact**。

> ⚠️ 当前 `sg100_sim.xml` **不用于** 最终 grasp/contact physics。若你看到「没有 collision」,这是 **设计如此**,不是 bug。

后续需要补:

```
visual mesh
    +
simplified collision geometry
```

建议(尚未实现):

- palm:box / convex
- finger segment:capsule / box / low-poly convex
- fingertip:sphere / capsule / convex

---

## 15. Tracking Loss Behavior

当前 simulation 在 tracking lost 时的策略:**hold last `q_goal`**,不立即 `q = 0`。

原因:直接 snap-to-zero 会造成明显、不自然的 simulation movement。

但必须醒目标注:

> **⚠️ Do not reuse this policy directly on real SG100 hardware.**

真机后续应增加:

- watchdog
- command timeout
- velocity limits
- hardware limits
- emergency stop
- `ACTIVE / HOLD / SAFE_STOP` 状态机

---

## 16. Known Limitations

- 当前 calibration 需要 **真实 AVP 数据** 最终验证(synthetic 测试已通过,但尚未在真实设备上完成端到端验证)。
- thumb opposition / abduction 目前由 **palm-local geometric projections** 建模。
- thumb motion 存在 **3D coupling**:若真实设备数据显示两个 thumb feature 高度耦合,后续可能改为 calibrated 2D/3D thumb basis。
- 当前 SG100 CAD mesh 是 **visual only**(无 collision)。
- 尚无 **final grasp collision geometry**。
- 尚无 **external object grasp validation**。
- 目前 **只做 right SG100 hand**(代码中 `--hand` 仅 `choices=["right"]`)。
- 还没有 **SG100 hardware backend**。
- 还没有 **RM65 arm mapping**。
- simulation 参数(`damping=0.002`、`armature=0.001`、`kp=2.0`)**不代表** 真实 SG100 motor dynamics,当前主要用于稳定的 validation backend。

---

## 17. Next Development Stages

- **Stage A** — 真实 Apple Vision Pro calibration validation。
- **Stage B** — AVP live SG100 simulation evaluation。
- **Stage C** — simplified collision geometries + object/table grasp scene。
- **Stage D** — SG100 HardwareBackend。
- **Stage E** — RM65 wrist/TCP mapping:
  ```
  AVP wrist relative pose
      ↓
  workspace mapping
      ↓
  RM65 IK
  ```
  同时:
  ```
  AVP finger geometry
      ↓
  SG100 hand q
  ```
- **Stage F** — RM65 + SG100 combined teleoperation。

---

## 18. Troubleshooting

### `avp_stream` import fails
检查 `.venv` 是否激活(`source .venv/bin/activate`),以及 `avp_stream` 是否已 `pip install -e .` 安装。

### MuJoCo model fails to load mesh
**不要** 直接改回原始 high-poly STL(会超过 ~200k faces 限制),也 **不要** 把 mesh 路径改回 `package://...` ROS URI(MuJoCo 无法解析)。应使用 `meshes_mujoco/` 下的简化 mesh。

### Calibration file not found
先运行:

```
python -m sg100_teleop.calibration.calibrate_avp_hand --ip <VISION_PRO_IP>
```

或通过 `--calibration path/to/your.yaml` 显式指定。

### Tracking returns no hand
- 检查 AVP IP 是否正确
- PC / AVP 网络连通性
- 先运行 `python examples/00_hand_streaming.py --ip <VISION_PRO_IP>` 确认 stream
- 确认 AVP app / hand tracking 已启动
- 确认 **右手** 在 tracking 范围内(当前 pipeline 固定 `side="right"`)

### Hand moves but SG100 direction is wrong
按以下顺序排查:

1. calibration pose 是否做反(例如 `thumb_distal_negative` 做成了 positive 方向)
2. calibration YAML 中的 endpoint 是否合理
3. raw feature 输出(`--verbose` 或在 calibration summary 里看 median)
4. `sg100_joint_semantics.yaml` 的 mapping type 是否正确

> **不要** 第一时间修改 MuJoCo joint axis —— joint axis 已经通过 SG100 semantic sweep 验证过。

### SG100 command appears noisy
检查:

- calibration 质量(重新标定)
- tracking 质量
- `--filter-tau`(增大 → 更平滑、但响应更慢)
- `--max-velocity`(减小 → 更温和、但跟踪 lag 增大)

---

## 19. Development Rules / Architecture Constraints

给 contributor 的硬性约束:

### Rule 1
**不要** 向 `avp_stream` 写 SG100-specific logic。`avp_stream` 是通用 AVP tracking 库,不属于本子项目。

### Rule 2
**不要** 让 `SG100Retargeter` 依赖 AVP joint index。Retargeter 只接受 `CanonicalHandState`。

### Rule 3
tracking backend 应该转换成 `CanonicalHandState`,而不是直接产出 SG100 joint command。

### Rule 4
robot-specific joint semantics 放在 SG100 retargeting / config 层(`sg100_joint_semantics.yaml` + `SG100Retargeter.FEATURE_MAPPING`)。

### Rule 5
**不要** 修改 canonical `sg100.xml` 来解决 simulation-specific physics 问题。simulation modifications 应进入 `sg100_sim.xml`,或通过 `sim/build_sg100_sim.py` build pipeline 生成。

### Rule 6
future Manus(或其它 glove)集成应实现:

```
Manus → CanonicalHandState
```

**而不是** 另外复制一套 SG100 retargeter。


> **项目范围说明**
>
> 本节所描述的时间戳与关节状态信息，仅用于遥操作链路的调试与验证。
>
> 本项目不作为正式的 VLA 数据采集系统。后续 SG100 正式示教数据采集将使用 Manus 数据手套，并在独立的数据采集 Pipeline 中完成。
