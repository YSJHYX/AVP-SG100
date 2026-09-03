# SG100 + RM65 + Apple Vision Pro Teleoperation

本文档是 `sg100_teleop` 当前实现的权威入口，内容以仓库现有源码、配置、场景和测试为准。

当前系统实现了 Apple Vision Pro 右手 tracking 驱动 MuJoCo 中 RM65 六轴机械臂与 SG100 11-DoF 灵巧手的联合遥操作，并将 MuJoCo 状态与预构建 USDZ 场景回传给 Vision Pro。真实 AVP 到 MuJoCo 的端到端控制链、桌面接触及抓取流程已经实际运行验证；这里的“真实 AVP 验证”指真实头显驱动仿真机器人，并不代表已经控制或验证实体 RM65、实体 SG100 或真实物体。

## 1. 当前状态

| 能力 | 状态 | 说明 |
|---|---|---|
| AVP 右手 tracking 接入 | 已实现并实机运行 | 使用 `VisionProStreamer` 获取带序号和接收时间的 tracking frame |
| RM65 wrist translation | 已实现 | calibration-relative、固定世界轴映射，可配置比例 |
| RM65 wrist rotation | 已实现 | 默认跟随 calibration-relative rotation，不复制 AVP 绝对姿态 |
| RM65 TCP target filtering | 已实现 | mapping 后、IK 前进行 SE(3) filtering 和速度限制 |
| SG100 hand retargeting | 已实现 | calibration feature → canonical hand → SG100 joint target |
| Combined MuJoCo runtime | 已实现并实机运行 | RM65 6 DoF + SG100 11 DoF + 可交互桌面物体 |
| AVP USDZ scene streaming | 已实现 | 支持预构建兼容 USDZ，避免运行时转换 |
| AVP 驱动仿真抓取 | 已实际运行 | 已覆盖桌面接触、拾取与抓取调试 |
| 实体 RM65 / SG100 控制 | 不在当前实现范围 | 当前 runner 只写 MuJoCo control |
| 定量延迟、抖动与跨用户统计 | 待系统测量 | 仓库尚无可复现的硬件统计报告 |

当前推荐主入口：

```bash
python sg100_teleop/sim/run_avp_arm_hand_sim.py \
  --ip 192.168.2.XXX \
  --calibration sg100_teleop/config/avp_right_calibration.yaml \
  --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v7_compat.usdz
```

V7 正式候选的单行命令（首次使用仍需完成下方 AVP live checklist）：

```bash
python sg100_teleop/sim/run_avp_arm_hand_sim.py --ip 192.168.2.XXX --calibration sg100_teleop/config/avp_right_calibration.yaml --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v7_compat.usdz
```

这条命令没有附加 IK mode flag，因此使用默认的 6-DoF full-pose IK：TCP xyz 跟随 wrist 相对平移，TCP rotation 跟随 wrist 相对旋转。

如果本轮需要 position-only IK，运行：

```bash
python sg100_teleop/sim/run_avp_arm_hand_sim.py \
  --ip 192.168.2.XXX \
  --calibration sg100_teleop/config/avp_right_calibration.yaml \
  --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v7_compat.usdz \
  --position-only-ik
```

## 2. 系统边界与技术路线

本目录负责：

- 将 AVP 右手 skeleton 转换为稳定、可验证的 tracking sample；
- 将 AVP wrist calibration-relative pose 映射为 RM65 TCP target；
- 对 raw TCP target 做 production filtering，再交给 IK；
- 将手部语义特征映射为 SG100 11-DoF joint target；
- 在 MuJoCo 中联合驱动 RM65、SG100 和桌面物体；
- 将 MuJoCo state 与 USDZ visual scene 流送到 AVP。

本目录目前不负责：

- 实体 RM65 控制器通信；
- 实体 SG100 驱动与电流/力矩闭环；
- 实体机械臂碰撞规划、安全 PLC 或急停集成；
- 自动抓取规划、物体识别或自主策略；
- AVP 相机/麦克风媒体采集；本项目只记录 MuJoCo visual state，并离线渲染无声 MP4。

系统没有把 AVP pose 直接当作 robot world pose。当前控制语义是“按 `C` 建立 controller-relative anchor，之后分别计算相对平移和相对旋转”，并在进入 IK 前过滤目标。

## 3. 运行时架构

```text
Vision Pro right-hand tracking
        │
        ├── TrackingFrameGate
        │     ├── frame sequence 去重
        │     ├── timestamp / freshness 检查
        │     └── malformed / stale / timeout 拒绝
        │
        ├── Arm branch
        │     AVP wrist SE(3)
        │       → AVPWristRelativeMapper
        │       → raw TCP target
        │       → TcpTargetFilter
        │       → selected RM65 IK mode
        │       → q_arm_goal
        │
        └── Hand branch
              AVP hand joints
                → AVPHandAdapter
                → CanonicalHandState
                → SG100Retargeter
                → q_hand_goal
                → JointCommandFilter @ physics rate
                → q_hand_cmd

q_arm_goal + q_hand_cmd
        → MuJoCo controls/physics/contact
        → VisionProStreamer.update_sim()
        → AVP scene
```

关键顺序必须保持：

```text
accepted tracking frame → relative pose mapping → TCP filtering → IK
```

不得把 raw wrist pose 直接送入 IK，也不得在正常路径中让固定 rotation 覆盖默认的 relative-rotation target。

## 4. 坐标系、robot base 与 wrist calibration

### 4.1 Robot base 如何确定

RM65 的运动学和 MuJoCo 控制在模型自身的 robot world/base frame 中计算。teleoperation TCP body 是：

```text
sg100_base_link
```

它在模型中直接挂接到 `link_6`，所以当前 TCP 是 SG100 base，不是 RM65 法兰中心或虚构末端点。

AVP 中机器人整体显示位置由 `--attach X Y Z YAW_DEG` 决定，默认：

```text
0.0 0.0 0.8 90
```

`--attach` 只是 AVP 场景中的 visual placement，不会重新标定 MuJoCo robot base，也不会改变 IK 坐标定义。

AVP world 到 robot world 的控制轴映射来自：

```text
sg100_teleop/config/avp_to_tcp_identity.yaml
```

当前 basis 是 identity，并标记为 `uncalibrated development default`。它已经用于当前实机运行链路，但不是经过独立六轴测量、适用于所有用户与佩戴姿态的正式标定。若现场发现方向反向或换轴，应修改固定 proper-rotation basis，而不是用 wrist calibration 临时补偿。

### 4.2 Translation semantics

按 `C` 时记录 `p_avp_anchor` 与 `p_tcp_anchor`。之后：

```text
delta_p_avp = p_avp_current - p_avp_anchor
delta_p_tcp = translation_scale * A @ delta_p_avp
p_tcp_raw   = p_tcp_anchor + delta_p_tcp
```

- `A` 是固定 AVP-world → robot-world axis basis；
- `translation_scale` 当前默认 `1.0`；
- 平移只使用 world-space position difference；
- calibration wrist orientation 不参与 translation delta。

因此两个不同 wrist calibration rotation，只要 `p_current - p_anchor` 相同，TCP translation delta 就相同。每次 wrist calibration 不会旋转或重定义 robot x/y/z axes，只会重置 position zero point。

### 4.3 Rotation semantics

按 `C` 时同时记录 `R_avp_anchor` 与 `R_tcp_anchor`。之后：

```text
R_rel_avp = R_avp_current @ R_avp_anchor.T
R_rel_tcp = A @ R_rel_avp @ A.T
R_tcp_raw = R_rel_tcp @ R_tcp_anchor
```

这意味着：

- calibration 瞬间 `R_rel_avp = I`，所以 `R_tcp_raw = R_tcp_anchor`；
- 只平移 wrist 时 TCP rotation 不变；
- 只旋转 wrist 时 TCP position 不变；
- rotation 是相对 calibration pose 的增量，不是 AVP absolute rotation；
- translation 和 rotation 独立计算，最后才组装为 `T_tcp_raw`。

### 4.4 每次按 C 会发生什么

`C` 不改变固定轴映射 `A`，也不修改 robot base axes。下一帧 `NEW + VALID` tracking 到达后，系统用同一帧：

- 保存当前 AVP wrist position/rotation；
- 读取当前 MuJoCo 实际 TCP position/rotation；
- 重建 wrist mapper anchor；
- 将 TCP filter reset 到当前实际 TCP pose；
- 保存当前 SG100 runtime hand neutral anchor；
- forearm-axis mode 下重新保存有效 forearm direction；
- 将 teleop 状态切换为 active。

此时：

```text
delta_p = 0
delta_R = I
filtered target = current actual TCP pose
```

所以重新 calibration 不应导致 TCP position 或 rotation 跳变。它改变的是后续相对增量的零点，而不是 robot base 的 x/y axes。

## 5. RM65 IK 模式与失败策略

Mode flags 互斥；默认 mode 不需要 flag。

| 启动方式 | Task | Rotation 行为 |
|---|---|---|
| 无 mode flag | 6-DoF full-pose IK | 跟随 filtered relative wrist rotation |
| `--lock-tcp-rotation` | 6-DoF full-pose IK | 固定为 calibration 时 TCP rotation |
| `--position-only-ik` | 3D DLS IK | orientation 不进入 IK task |
| `--forearm-axis-ik` | 5-DoF DLS IK | 跟随 TCP local +Z，绕 +Z roll 自由 |

默认 full-pose solver 使用 RealMan 官方离线闭式 IK，并按上一帧 joint target 选近邻解。position-only 与 forearm-axis 使用 MuJoCo Jacobian 的 damped least-squares。

| 参数 | Position-only | Forearm-axis |
|---|---:|---:|
| position tolerance | `1e-4 m` | `1e-4 m` |
| axis tolerance | — | `1e-3` |
| max iterations | `60` | `80` |
| damping | `0.05` | `0.05` |
| posture gain | `0.02` | `0.02` |
| max internal step | `0.2 rad` | `0.2 rad` |
| axis weight | — | `0.2` |

Forearm-axis mode 需要有效的额外 forearm points。legacy 25-joint sample 或无效 forearm axis 会让 arm task 自动退化为 position-only，SG100 hand retargeting 不受影响。

主 IK 成功且输出合法时更新 `q_arm_goal`。主 IK 不收敛时：

1. 增加 `ik_failure_count`；
2. 从上一 `q_arm_goal` 启动 position-only best-effort DLS；
3. fallback 合法时逐关节限制在上一 target `±0.1 rad` 后接受；
4. fallback 非法时保持上一有效 target。

这能在 orientation 暂时不可达时继续连续追踪位置，减少纯 HOLD 后的回跳。`--verbose` 会输出 failure/fallback、误差、最小奇异值、condition number 和较大的 accepted joint jump。

## 6. TCP target filtering

配置位于 `config/avp_to_tcp_identity.yaml`：

```yaml
filter:
  position_tau: 0.06
  rotation_tau: 0.06
  max_linear_velocity: 0.25
  max_angular_velocity: 1.5
```

```text
T_tcp_raw → TcpTargetFilter → T_tcp_target → IK
```

- position：一阶低通，再做三维向量范数线速度限制；
- rotation：SO(3) 最短 geodesic/log-exp 更新，再做角速度限制；
- `dt`：来自 accepted frame 的 `received_monotonic_ns`；
- duplicate/stale/malformed frame：不推进 filter time；
- calibration：reset 到当前实际 TCP pose；
- tracking timeout：clear，旧状态不跨 session 复用。

arm chain 不再叠加 joint-space EMA。SG100 hand 单独使用 `JointCommandFilter`，默认 `tau=0.06 s`、最大 joint velocity `4.0 rad/s`。旧 YAML 缺少 `filter` block 时会加载上述默认值。

## 7. SG100 hand pipeline

权威 joint 顺序来自：

```text
sg100_teleop/config/sg100_joint_semantics.yaml
```

运行时会检查 `SG100Retargeter.joint_order` 和 combined model named layout，不依赖“模型恰好只有 17 qpos”的假设。

当前 11 DoF 语义：

```text
thumb:  opposition, abduction, distal flexion/extension
index:  adduction, flexion, distal
middle: flexion, distal
little: outward, flexion, distal
```

AVP adapter 提取语义量，经 calibration YAML 归一化成 `CanonicalHandState`，再由 `SG100Retargeter` 转为受 joint limits 约束的目标。arm 与 hand 使用同一 accepted tracking frame。

retargeter 在基础映射之上有两个可选 config-driven 协调层（section 缺失或 disabled 时输出与基础映射 bit-exact 一致）：

1. `pinch_coordination`：pinch gate 平滑地把 thumb 指令 blend 到 pinch anchors/target；
2. `power_grasp_thumb_lock`：power grasp 时对 thumb 施加连续的 opposition bias（FK 实测基础映射 q1≈60° 时拇指悬在 index-middle closure region 拇指侧约 59 mm，未形成对掌锁扣；bias +40° 后进入 closure region，d_lock 62→31 mm）。lock weight = grasp 闭合度 × thumb engagement × (1 − pinch gate)，pinch/open hand/刻意张拇抓取均保持 weight 0。

thumb 诊断（单关节 sweep、operating point FK 几何、BEFORE/AFTER 对比）：

```bash
python -m sg100_teleop.sim.inspect_sg100_thumb_kinematics
```

两个 sim runner 均支持 `--thumb-retarget-debug`（约 5 Hz 打印 grasp/lock 权重、human thumb 特征、raw/bias/final q 与拇指尖几何）。

需要区分：

1. `avp_right_calibration.yaml`：长期保存的用户 hand feature ranges；
2. wrist/TCP anchor：每次按 `C` 建立的 relative-pose anchor；
3. `q_hand0_runtime`：同一 `C` frame 保存的 SG100 runtime neutral。

重新按 `C` 更新后两项，不重写 calibration YAML。

采集 hand calibration：

```bash
python -m sg100_teleop.calibration.calibrate_avp_hand \
  --ip 192.168.2.XXX \
  --hand right \
  --output sg100_teleop/config/avp_right_calibration.yaml
```

当前 YAML schema version 3，包含 neutral、thumb opposition/abduction/distal、index adduction/flexion/distal、middle flexion/distal 与 little outward/flexion/distal 等标定量。

## 8. Tracking、arming 与 timeout

主 runner 不会启动即跟手：

```text
waiting for valid right hand
  → valid tracking available
  → 用户保持 neutral 并按 C
  → 下一帧 NEW + VALID frame 建立 anchors
  → active teleoperation
```

`TrackingFrameGate` 检查 frame sequence、接收时间、freshness、right-hand shape/finite 及 timeout。默认 tracking 与 AVP state stream 均为 60 Hz，timeout 为 2 秒。

Timeout 后系统：

- 保持最后有效 RM65/SG100 target；
- disarm teleoperation；
- clear TCP filter；
- 清除 wrist/TCP/forearm/runtime-hand anchors；
- tracking 恢复后要求重新按 `C`。

Duplicate 或 malformed frame 不推进 command，也不刷新有效 tracking timeout。

## 9. MuJoCo 模型、场景与外观

默认 physics model：

```text
sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_avp_grasp_scene.xml
```

当前场景包含 RM65+SG100、`-9.81 m/s²` 重力、`0.002 s` timestep、抓取桌、可交互 water bottle、Coke/Red Bull/Pringles/apple/orange/mug、桌面筐子，以及 hand/palm/object/table/arm collision masks 和 collision proxies。

默认 scene 状态维度为 `nq=66, nv=59, nu=17`。17 个 actuator 对应 RM65 6 DoF 与 SG100 11 DoF，其余 state 来自 free objects，不能用全局 slice 假设 robot 占满整个 model state。

Robot-only model：

```text
sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_sim.xml
```

它用于基础回归，维度 `17/17/17`，不等于当前 tabletop production scene。

### Startup pose

主 runner 根据 scene 中 `water_bottle` 位置，用 full-pose IK 求距瓶子约 `0.25 m` 的 pre-grasp stance，避免一启动 hand collision proxy 就挤压物体。若 bottle 不存在或 IK 无解，才回退：

```text
[0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430]
```

按 `C` 时 TCP anchor 始终读取当时 MuJoCo 实际 pose，不硬编码为 ready pose。

### USDZ 选择

| 文件 | 用途 |
|---|---|
| `rm65_sg100_sim_compat.usdz` | 轻量 robot-only compatibility package，保留用于 rollback |
| `rm65_sg100_avp_grasp_scene_compat.usdz` | 包含桌面、筐子和纹理物体的兼容 visual scene |
| `rm65_sg100_avp_grasp_v7_compat.usdz` | 当前 V7 production visual scene：最终 RM65/SG100、腕部相机支架与外壳、中性灰桌面及纹理物体 |
| `rm65_sg100_avp_grasp_scene.usdz` | 抓取场景完整导出包 |

需要在 AVP 内看到带纹理桌面抓取物与筐子时：

```bash
python sg100_teleop/sim/run_avp_arm_hand_sim.py \
  --ip 192.168.2.XXX \
  --calibration sg100_teleop/config/avp_right_calibration.yaml \
  --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v7_compat.usdz
```

`--model` 决定本机 MuJoCo physics/contact；`--usdz` 决定发给 AVP 的 visual package。提供 `--usdz` 后会验证文件并绕过 runtime conversion。兼容包让每个 streamed body 在 USD `Geometry` 下有可唯一匹配的 visual ownership，避免 AVP 对嵌套 body/leaf 重复施加 transform。

## 10. Recording 与 MP4

主 runner 使用：

```python
VisionProStreamer(ip=args.ip, record=True)
```

这里的 `record=True` 仍只把已处理 tracking samples 追加到 Python 进程内存列表，供 `get_recording()` 读取。它与新增的本机 MuJoCo visual-state recorder 是两套独立机制。当前 runner：

- 没有调用 `configure_video()`；
- 没有配置 video track；
- 没有把 `get_recording()` 写入 JSON/JSONL；
- 不会仅凭 `record=True` 要求 AVP 生成 `video.mp4`。

因此 AVP metadata 中 `hasRecording=false` 时，下面目录只有 tracking、metadata JSON 和 scene XML、没有视频，是符合当前路径的：

```text
On My Vision Pro/
└── Tracking Streamer/
    └── Recordings/
        └── recording_日期_时间_UUID/
```

与此同时，主 runner 默认按 MuJoCo simulation time 以 30 FPS 将 `timestamps/qpos/mocap` 保存到：

```text
recording/YYYYMMDD_HHMMSS/mujoco_recording.npz
```

运行时不渲染 RGB，也不编码 MP4。退出后离线生成本机无声 H.264 视频：

```bash
python -m sg100_teleop.recording.render_mujoco_video \
  --input recording/<session>/mujoco_recording.npz
```

默认输出为同一 session 下的 `mujoco_view.mp4`，相机为 `workstation_front`，分辨率 `1280×720`，FPS 使用 recording metadata。AVP 目录中没有 `video.mp4` 与本机这条离线 pipeline 并不矛盾。

## 11. 安装与运行

从仓库根目录：

```bash
cd ~/VisionProTeleop
python -m pip install -e .
```

根目录 `setup.py` 包含 NumPy、SciPy、MuJoCo、gRPC、aiortc、PyYAML、OpenCV 等依赖。RM65 full-pose IK 还要求当前环境可导入 RealMan `Robotic_Arm.rm_inverse_kinematics`。

启动前确认 PC 与 AVP 网络可达、IP 正确、calibration YAML 存在、本机可启动 MuJoCo viewer、USDZ 可读，且 Tracking Streamer 已允许连接。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `--model` | grasp scene XML | 本机 physics model |
| `--calibration` | right calibration YAML | hand feature calibration |
| `--avp-to-tcp-config` | identity YAML | axis basis、scale、TCP filter |
| `--usdz` | 无 | 预构建 AVP visual package |
| `--port` | `50051` | simulation streaming gRPC port |
| `--tracking-hz` | `60` | tracking query rate |
| `--stream-hz` | `60` | state stream rate |
| `--tracking-timeout` | `2.0 s` | tracking loss/disarm threshold |
| `--recording-dir` | `recording` | 本机 MuJoCo state recording 根目录 |
| `--video-fps` | `30` | simulation-time state sampling rate |
| `--filter-tau` | `0.06 s` | SG100 joint filter tau |
| `--max-hand-velocity` | `4.0 rad/s` | SG100 command limit |
| `--attach` | `0 0 0.8 90` | AVP visual placement |
| `--verbose` | off | tracking/IK diagnostics |

启动后：等待 valid right hand → 保持 neutral → 在 MuJoCo viewer 按 `C` → 状态 active → 先做小幅单轴动作。Tracking timeout、用户换位或基准变化后重新按 `C`。

## 12. 验证与测试

测试脚本可直接作为 module 运行，不强制依赖 pytest。

### Fast core regression

```bash
python -m sg100_teleop.tests.test_avp_streamer_frame_metadata
python -m sg100_teleop.tests.test_avp_tracking_freshness
python -m sg100_teleop.tests.test_teleop_arming
python -m sg100_teleop.tests.test_avp_source_adapter
python -m sg100_teleop.tests.test_avp_to_tcp_config
python -m sg100_teleop.tests.test_avp_wrist_mapper
python -m sg100_teleop.tests.test_avp_wrist_controller_relative
python -m sg100_teleop.tests.test_tcp_target_filter
python -m sg100_teleop.tests.test_sg100_retargeter
python -m sg100_teleop.tests.test_sg100_retarget_semantics
python -m sg100_teleop.tests.test_sg100_physical_joint_semantics
python -m sg100_teleop.tests.test_sg100_driver
python -m sg100_teleop.tests.test_sg100_latest_sdk_driver
python -m sg100_teleop.tests.test_sg100_no_legacy_backend
python -m sg100_teleop.tests.test_sg100_sign_characterization
python -m sg100_teleop.tests.test_rm65_fk_tcp
python -m sg100_teleop.tests.test_rm65_tcp_jacobian
python -m sg100_teleop.tests.test_rm65_ik_position_only
python -m sg100_teleop.tests.test_forearm_axis_ik
python -m sg100_teleop.tests.test_raw_avp_to_combined_mujoco
MUJOCO_GL=egl python -m sg100_teleop.tests.test_mujoco_state_recording
```

重点覆盖 frame freshness、arming、translation/rotation 解耦、recalibration、rotation lock 隔离、TCP filter、SG100 semantics、RM65 IK 与 raw AVP→combined integration。

### Scene validation

```bash
python -m sg100_teleop.sim.validate_avp_grasp_scene
python -m sg100_teleop.tests.test_combined_scene_compatibility
python -m sg100_teleop.tests.test_rm65_sg100_grasp_pose_edges
python -m sg100_teleop.tests.test_rm65_sg100_workspace_sweep
```

Validator 检查 XML/mesh、重力、named layout、bottle freejoint、collision masks、hand force range、桌面接触和 1 秒动力学稳定性。本地查看：

```bash
python -m sg100_teleop.sim.view_avp_grasp_scene
```

### Extended IK diagnostics

```bash
python -m sg100_teleop.tests.test_rm65_ik_reachable_targets
python -m sg100_teleop.tests.test_rm65_continuous_tcp_teleop
python -m sg100_teleop.tests.test_avp_to_tcp_basis_ik
python -m sg100_teleop.tests.test_avp_to_tcp_basis_six_axis
python -m sg100_teleop.tests.test_rm65_ik_z_scan
```

`test_rm65_ik.py` 包含 legacy `+Z` singularity diagnostic，可能故意报告该历史 case 不可达；不能单独解释为当前 pre-grasp anchor 或 production chain 全面失败。

### USDZ contract

```bash
python -m sg100_teleop.tests.test_avp_mujoco_usdz_contract
python -m sg100_teleop.tests.test_prebuilt_usdz
```

V7 contract test 固定验证 `(nq, nv, nu) = (66, 59, 17)`、旧动态节点名称兼容、腕部支架/外壳层级、碰撞调试几何排除、SG100 高精度网格、桌面颜色、关键变换和包内引用。

需要 `pxr` 与 `mujoco-usd-converter` 的 export/audit 使用单独环境：

```bash
.venv_usdz/bin/python -m sg100_teleop.tools.export_avp_mujoco_usdz \
  --xml sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_avp_grasp_scene.xml \
  --output sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v7_compat.usdz
```

不要在未重新做 USDZ contract/audit 时手工替换包内层级、body names 或 visual ownership。

## 13. Live acceptance checklist

1. 未 calibration 时 robot 保持 startup pose；
2. valid 后按 `C`，calibration 瞬间 TCP/hand 不跳；
3. AVP 分别沿 x/y/z 移动，确认 robot world axis 与比例；
4. 只旋转 wrist 时默认 full-pose mode 的 position 不变；
5. 只平移 wrist 时 rotation 保持 anchor；
6. 同时运动时 translation/rotation 独立更新；
7. 再按 `C`，当前 pose 成为新零点且无跳变；
8. 遮挡右手，确认 HOLD、timeout disarm 和 filter clear；
9. tracking 恢复后必须重新按 `C`；
10. 检查 `--verbose` 中 rejected/duplicate/stale 和 IK fallback；
11. 验证 bottle/table/finger contact 无穿透或爆炸；
12. 对比 lightweight/grasp USDZ 的 visual/body transform。

“已实时运行”不等于“已经定量验收”。正式 performance 结论还应保存 frame interval、端到端 latency、stationary jitter、各轴 direction/scale、IK fallback rate、接触穿透、抓取成功率，以及跨用户/网络条件的重复性。

## 14. 已知限制

- 默认 identity basis 未经正式测量认证，现场仍需确认六轴方向。
- Hand calibration 与用户相关；换人或明显改变佩戴/neutral pose 后应重新采集或至少重新按 `C`。
- TCP filter 已限 target 速度，但主 IK 成功解目前只记录大 joint jump，未统一对所有成功解做 arm joint-step clamp。
- IK fallback 优先位置连续性，orientation 在不可达阶段可暂时偏离。
- 当前 contact 调试不等价于完整 self-collision planning 或实体安全保证。
- USDZ visual scene 与 MuJoCo physics model 是两个文件，修改任一侧都需检查 naming、层级、材质与几何一致性。
- USDZ 视觉包与 physics scene 是独立文件；每次 V7 MJCF 视觉变更后都必须重新导出并运行 contract test。
- AVP tracking samples 仍不自动落盘；本机 MP4 必须在 session 结束后由 offline renderer 生成。
- 实体 RM65/SG100、实体抓取和实体安全链尚未实现。

## 15. Troubleshooting

### `avp_stream` import 失败

从仓库根目录执行 `python -m pip install -e .`，并确认当前 interpreter 使用该环境。

### Full-pose IK import/solve 失败

确认 RealMan `Robotic_Arm.rm_inverse_kinematics` 可导入。只验证平移链时可先用 `--position-only-ik`，但这不替代修复依赖。

### MuJoCo model/mesh 加载失败

从仓库根目录运行，并先执行 `python -m sg100_teleop.sim.validate_avp_grasp_scene`。不要随意改变 MJCF include/mesh 相对路径。

### Tracking 有手但无法 active

必须先收到 valid right hand，再在 MuJoCo viewer 按 `C`；timeout 后也要重新按 `C`。用 `--verbose` 检查 frame metadata 与 teleop state。

### Robot 移动轴向错误

先排除 `--attach` 造成的纯显示旋转。控制轴由 `--avp-to-tcp-config` 的 basis 决定；wrist calibration 不会重定义 x/y axes。

### IK fallback 频繁

用 `--verbose` 检查 target、position/rotation error、`sigma_min` 与 condition number。依次排查 scale/basis、workspace、wrist rotation、startup pose 和 IK mode，不要靠绕过 TCP filter 或扩大 joint limits 掩盖问题。

### AVP 有 XML/JSON 但无 `video.mp4`

若 metadata `hasRecording=false`，当前 simulation-only WebRTC 配置不会在 AVP 目录产生视频；`record=True` 只是 Python 内存 tracking 开关。本机录像请使用 `recording/<session>/mujoco_recording.npz` 运行 offline renderer。

### AVP 中物体没有外观

使用 `rm65_sg100_avp_grasp_v7_compat.usdz`；轻量 `rm65_sg100_sim_compat.usdz` 仅保留作 robot-only rollback，physics `--model` 仍应指向 grasp scene XML。

## 16. 维护约束

1. SG100 joint order/direction 只由权威 config 与 named model layout 定义；
2. robot state/control 使用 named resolution，不假设 scene 只有 17 DoF；
3. arm 与 hand 使用同一 accepted tracking frame；
4. translation 使用固定 world-axis position difference，不与 calibration rotation 混合；
5. rotation 使用 calibration-relative delta，不复制 AVP absolute rotation；
6. mapper → TCP filter → IK 的顺序不可交换；
7. calibration 同时 reset mapper、filter 与 runtime anchors，target 等于当前实际 pose；
8. normal path 默认启用 relative rotation，rotation lock 只作显式 A/B mode；
9. duplicate/stale/malformed frame 不推进 command/filter；
10. timeout 后旧 anchor 不复用，必须重新按 `C`；
11. full-pose、position-only、forearm-axis task 定义不可混淆；
12. IK fallback、limits、solver 参数改动必须有 regression；
13. MuJoCo viewer sync 不放入 500 Hz physics inner loop；
14. USDZ ownership/层级改动必须通过 contract 与 AVP runtime 验证；
15. 文档必须区分“真实 AVP 驱动仿真”和“实体机器人验证”。

## 17. 关键文件索引

```text
sg100_teleop/
├── README.md
├── calibration/
│   ├── calibrate_avp_hand.py
│   └── io.py
├── config/
│   ├── avp_right_calibration.yaml
│   ├── avp_to_tcp_identity.yaml
│   └── sg100_joint_semantics.yaml
├── control/
│   ├── joint_command_filter.py
│   └── tcp_target_filter.py
├── hand/canonical_hand.py
├── kinematics/rm65_ik.py
├── retargeting/sg100_retargeter.py
├── recording/
│   ├── mujoco_state_recorder.py
│   └── render_mujoco_video.py
├── tracking/
│   ├── avp_hand_adapter.py
│   ├── avp_source.py
│   └── avp_wrist_mapper.py
├── sim/
│   ├── run_avp_arm_hand_sim.py
│   ├── combined_robot_layout.py
│   ├── validate_avp_grasp_scene.py
│   └── view_avp_grasp_scene.py
├── tools/export_avp_mujoco_usdz.py
├── assets/rm65_sg100/
│   ├── mjcf/
│   │   ├── rm65_sg100_sim.xml
│   │   └── rm65_sg100_avp_grasp_scene.xml
│   └── usdz/
│       ├── rm65_sg100_sim_compat.usdz
│       ├── rm65_sg100_avp_grasp_scene.usdz
│       ├── rm65_sg100_avp_grasp_scene_compat.usdz
│       └── rm65_sg100_avp_grasp_v7_compat.usdz
└── tests/test_*.py
```

主链关键配置分别是：hand feature calibration、AVP/TCP axis/scale/filter、SG100 joint semantics、MuJoCo physics/contact XML 与所选 AVP USDZ appearance。修改技术路线或能力状态时，应同步更新本文档和相应 regression/acceptance 说明，避免再次出现前后进度冲突。
