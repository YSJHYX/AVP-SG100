# SG100 AVP 真机控制层技术交接（面向嵌入式/机器人软件）

> 读者：接手 SG100 真机控制层、SDK 封装、实时通信、安全状态机与 AVP
> hand command 下发的软件工程师。
> 本文基于 2026-09-04 的实际代码与真机 artifact 审计；不要求你重新研究
> AVP retargeting 算法。看完即可开始接 SG100 AVP 真机 backend。
>
> 配套权威文档（细节冲突时以代码 + 本文为准）：
> `SG100_LATEST_SDK_INTEGRATION.md`（SDK 集成权威描述）、
> `SG100_MODE7_FEEDBACK_SEMANTICS_AUDIT.md`（反馈语义证据）、
> `SG100_REAL_HARDWARE_VALIDATION.md`（验证命令）、
> `AGENT_HANDOFF_CURRENT.md`（上一轮交接现状）。

---

## 1. 最终系统架构（必须复用，不要重新设计）

MuJoCo 中已经工作良好的链路是唯一命令源，直接复用：

```
AVP hand tracking (60 Hz)
        ↓
现有 SG100Retargeter（含 pinch coordination / power-grasp thumb lock /
        ↓  机械限位 clamp / JointCommandFilter 平滑）
canonical SG100 qcmd[11]（rad）
        ├────────→ MuJoCo backend（仿真，已长期运行）
        │
        └────────→ SG100 real backend（本文交接对象）
                         ↓
                       F_cmd（canonical rad → SDK user rad）
                         ↓
                  command safety envelope
                         ↓
              Hand_Control_t（per-joint mode7）
                         ↓
                SG100Device::SetHandControl()
                         ↓
                vendor CommScheduler（10 ms 重发 cache）
                         ↓
                      SG100 真机
```

反馈路径**独立**于命令路径：

```
SG100 Joint_Status（~100 Hz 回调）
        ↓
MODE7 raw feedback（SDK callback domain, deg）
        ↓
F_fb（独立 SG100FeedbackMap —— 当前未验证）
        ↓
canonical measured q（当前 canonical_valid=False）
        ↓
MuJoCo 反馈 / dataset（当前 QUARANTINED）
```

两条公式，写代码时必须分开成两个对象：

```
q_sdk_cmd    = F_cmd(q_canonical)          # 出站，SG100HardwareMap
q_canonical  = F_fb(q_mode7_feedback)      # 入站，SG100FeedbackMap
```

**禁止默认 `F_fb = inverse(F_cmd)`。** 历史版本曾用
`hardware_map.actual_to_canonical()` 解析真实反馈，隐式做了这个假设，
已修正：`actual_to_canonical` 仅保留给 mock/round-trip 测试，真实反馈
必须走独立的 `SG100FeedbackMap`（当前 `validated=False`，即没有任何
validated F_fb 存在）。

---

## 1A. AVP → SG100 canonical command data contract（2026-09-04 数据链审计）

命令链只有一个 authoritative 变量，两个 backend 消费同一对象：

| 项 | 值 | 代码证据（run_avp_arm_hand_sim.py） |
|---|---|---|
| authoritative variable | `q_hand_cmd`（canonical_hand_qcmd_rad） | — |
| 生产者 | `retargeter.retarget_with_stages(canonical)` → `q_hand_candidate` | L2291 |
| 60Hz goal | `q_hand_goal = q_hand_candidate.copy()`（同帧 sensory 输入） | L3011/2555 |
| filter 位置 | **retargeting 之后**：`q_hand_cmd = hand_command_filter.update(q_hand_goal, physics_dt)`（tau=0.06s、max 4 rad/s、低通+限速，无任何 SG100 映射逻辑） | real L3238 / sim L3371 |
| shape / dtype / unit | `(11,)` / `float64` / **rad** | JointCommandFilter |
| joint order | `SG100_JOINT_ORDER`（thumb_j1…little_j3，== retargeter.joint_order == SDK J1..J11 恒等） | 测试 test_canonical_order_binds_mujoco_actuators_by_name |
| limits | semantics.yaml 机械限位（retargeter 内已 clamp） | sg100_joint_semantics.yaml |

**MuJoCo consumer（sim path）**：`write_robot_controls()` →
`data.ctrl[layout.hand_actuator_ids] = q_hand_cmd`
（combined_robot_layout.py L197）。actuator 为 position servo、gear=1、
ctrlrange==jnt_range → **ctrl 数位 = joint rad，无额外 actuator
transform、无第二次 sign transform**（resolve_robot_layout 逐轴校验
`{joint}_pos` 驱动同名 joint）。**`qpos` 是仿真的响应状态，不是
command**：`q_hand_cmd != qpos`（即使数值通常接近）。real-arm
kinematic-twin 路径下 hand qpos 仅作可视化直写（L3296 附近，无硬件
driver 时），同样不是命令源。

**Real SG100 consumer**：`restricted_hand_backend.submit(q_hand_cmd, …)`
（real L3256 / sim L3385）——与 MuJoCo **同一个变量**（两个路径都在
filter 之后立即 `state_recorder.set_hand_command(q_hand_cmd)` 完成录制
latch）；backend 内部才做 F_cmd → SDK user deg → envelope →
Hand_Control_t。禁止第二个 retargeter，禁止在
retargeter/AVP/runner 层加 degree 或 J7 特殊逻辑。

**频率语义**：AVP tracking/retargeting ~60 Hz；physics filter ~500 Hz
（MuJoCo timestep）；recorder 采样 30 Hz（**只是记录频率**，不是命令
频率）；vendor scheduler ~10 ms 重发 cache。real backend 在新
authoritative command 到来时更新 SetHandControl 即可，勿绑定 30 Hz。

**Recording fields**（MujocoStateRecorder，向后兼容，仅 hand 模式）：

| 字段 | 含义 |
|---|---|
| `hand_qcmd_canonical_rad [N,11]` | **authoritative command**（backend 前、post-filter，set_hand_command 逐帧 latch，绝不从 qpos 反推） |
| `hand_qpos_mujoco_rad [N,11]` | MuJoCo 仿真响应（显式 slice） |
| `hand_joint_names` / `hand_command_unit='rad'` / `hand_command_domain='canonical'` | 布局与单位自描述 |
| metadata `hand_command_source` / `hand_qpos_source` / `hand_command_missing_frames` | provenance |

历史 recording（如 98.3s/2951 帧 session）只保存 `qpos`：
`qpos[:,6:17]` 是 **SG100 hand 仿真状态**（rad、canonical 约定一致，
J7 为负值域），**没有保存 original retargeted hand command**——本轮
新增字段后未来 session 才有。

**C gate 与 first-command latch**：C 之前 command 可计算/显示但
`submit` 返回 `C_ANCHOR_REQUIRED`（driver cache 也是 MODE_NULL，双层
挡）；C 时 anchor 缓存 F_cmd 后目标 + 零增益零输出；enable 后
velocity limiter 从 anchor 值起步 → 首个 active command ≈ 当前 safe
target，无瞬移。

**调试**：runner `--sg100-command-debug`（~5 Hz 打印 timestamp /
canonical rad / allowlist / post-F_cmd SDK deg / reject reason /
C-enabled 状态）。

回归锁定：`tests/test_canonical_command_data_chain.py`（same canonical
command 双投递 bit-exact、J7 映射不污染 shared command 且不泄露到
真机、recording 字段独立性）。

---

## 2. 11D canonical joint contract（从实际 config 读取）

来源：`sg100_teleop/config/sg100_joint_semantics.yaml`（canonical 限位）、
`sg100_teleop/config/sg100_hardware_map.yaml`（F_cmd gain/offset）、
`~/SG100_SDK_Publish/joint_angle_map.json`（SDK user 命令限位，md5
`8a0c1de4a7d60a47a016dd453fb49ee3`）。单位均为 deg（存储为 rad）。

| idx | canonical | SDK | canonical min | canonical max | SDK cmd min | SDK cmd max | F_cmd gain | F_cmd offset | F_cmd 验证 | F_fb 验证 |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | thumb_j1 | J1 | 0.0 | +125.0 | 0 | +125 | +1 | 0 | CANDIDATE（未真机） | 无 |
| 1 | thumb_j2 | J2 | −149.5 | 0.0 | −150 | 0 | +1 | 0 | CANDIDATE（未真机） | 无 |
| 2 | thumb_j3 | J3 | −60.0 | +90.0 | −60 | +90 | +1 | 0 | CANDIDATE（未真机） | 无 |
| 3 | index_j1 | J4 | 0.0 | +90.0 | 0 | +90 | +1 | 0 | CANDIDATE（未真机） | 无 |
| 4 | index_j2 | J5 | 0.0 | +124.9 | 0 | +125 | +1 | 0 | CANDIDATE（未真机） | 无 |
| 5 | index_j3 | J6 | −60.2 | +90.0 | −60 | +90 | +1 | 0 | **PASS — REAL（J6 active）** | 无 |
| 6 | middle_j1 | **J7** | −124.9 | 0.0 | 0 | +125 | **−1** | 0（待验） | CANDIDATE（HOLD gate 未过） | 无 |
| 7 | middle_j2 | J8 | −60.2 | +90.0 | −60 | +90 | +1 | 0 | CANDIDATE（未真机） | 无 |
| 8 | little_j1 | J9 | −179.9 | 0.0 | −180 | 0 | +1 | 0 | CANDIDATE（未真机） | 无 |
| 9 | little_j2 | J10 | 0.0 | +124.9 | 0 | +125 | +1 | 0 | CANDIDATE（未真机） | 无 |
| 10 | little_j3 | J11 | −60.2 | +90.0 | −60 | +90 | +1 | 0 | CANDIDATE（未真机） | 无 |

补充事实：

- **joint order 恒等**：canonical index 0..10 == SDK joint id 1..11
  （vendor 布局 thumb 1-3 / index 4-6 / middle 7-8 / little 9-11）。
  已由 `SG100HardwareMap` 加载时强校验，无需 runtime 重排。
- **offset=0 的证据**：N0 静态探针证明机械零位下每个 SDK joint 回读
  ≈0°（±0.055°）。但注意 §15：J7 的 "canonical 0 == SDK 0" 并未通过
  active hold gate 验证。
- **数值边界**：canonical 端点比 SDK user range 宽约 0.2°
  （125.002>125、−60.16<−60）。处理方式是 **reject，不是 clamp**：
  `SG100SDKDriver._user_deg_within_vendor_range()` 拒收越界目标，保证
  "记录的 action == 编码的 action"（vendor 侧 clamp 不往返安全）。
- `F_fb 验证` 列全为"无"：当前不存在 validated `SG100FeedbackMap`。

---

## 3. 重点：middle_j1 / J7（最容易写错的一轴）

| 域 | 范围 (deg) |
|---|---|
| canonical / MuJoCo | [−125, 0]（负方向 = flexion toward palm） |
| SDK outbound J7 user | [0, +125] |

当前 outbound candidate：

```
q_sdk_J7 = -1 * q_canonical_middle_j1 + offset    # offset 待真机验证
```

即 canonical −0.5° → SDK +0.5°。gain=−1 已写入
`sg100_hardware_map.yaml`（`gain[6] = -1.0`）。

**实现要求**：

- gain/offset 只放在 `sg100_hardware_map.yaml`（hardware command
  mapping），**不要**改 `SG100Retargeter`，**不要**在 AVP 层为 J7 写
  特殊 if/else；
- J7 当前 **不能视为 production validated**：2026-09-04 真机 run 在
  active hold 阶段 fail-closed（见 §16），micro-motion 未执行；
- 限制在 Python 侧（`SG100SDKDriver` reject）而不是依赖 SDK clamp。

---

## 4. SG100 SDK 权威版本

| item | value |
|---|---|
| path | `~/SG100_SDK_Publish` |
| branch | `V3.1` |
| commit | `f924b44475`（"feat(publish): docs: record Linux ARM64 export"） |

注意：

- `~/SG100_SDK_Publish_backup_wrong_branch` 是 14-joint pre-V1.5 分支，
  **不受支持**，勿用。
- 历史自研 raw RS485 packet / 自算 CRC sender 已**废弃**（§28）。
  production path 必须走 vendor SDK；本仓库 C wrapper 只旁路监听 SDK
  已有 console 输出统计 CRC，不重写协议。

SDK path 解析顺序（`sg100_sdk_driver._default_sdk_dir()`）：
`$SG100_SDK_DIR` → `<repo>/../SG100_SDK_Publish` → `~/SG100_SDK_Publish`。

---

## 5. 控制链路与权威 send API

```
Python (SG100SDKDriver)
    → ctypes → sg100_c_wrapper.cpp（extern "C"，本仓库维护）
    → vendor C++ SDK（SG100Device）
    → SG100 真机
```

具体 native send path（`sg100_teleop/robot/sg100_sdk_driver.py` 的
`send_positions_rad()` / `send_mode7_*()` → wrapper
`sg100_set_hand_control()`）：

```
SG100SDKDriver
  → sg100_set_hand_control(positions_deg[11], modes[11], p[11], i[11], d[11], output_limits[11])
  → SG100Device::SetHandControl(Hand_Control_t)     ← 唯一权威 API
```

**production realtime 控制只用 `SetHandControl(Hand_Control_t)`**。

明确不用：

- ❌ `SetJointPositions`：强制 `MODE_POSITION` + 继承 cache PID，
  clamp 不往返安全；wrapper 已**删除**其入口；
- ❌ 自己拼串口 packet / 算 CRC / 管理 0x05/0x25/0x26/0x27 命令字——
  这些是 vendor 内部细节，上层不可见；
- ❌ 自建第二个 100 Hz sender（见 §9）。

---

## 6. Hand_Control_t ABI 与 payload

本机实测（`sg100_test_struct_sizes`，与 `sg100_types.h` @
V1.5_NO_TACTILE 的 static_assert 一致）：

| struct | size |
|---|---|
| Joint_Control_t | 16 B |
| Joint_Status_t | 12 B |
| Hand_Control_t | **176 B** = 11 × 16 |
| Hand_Status_t | **132 B** = 11 × 12 |
| MotorDown_t | 14 B |
| MotorUp_t | 12 B |

`Joint_Control_t` 字段（u8 id, u8 mode, f32 position, u16 p/i/d/
output_limit, 2 B reserved）。`Hand_Control_t = 11 × Joint_Control_t`，
每个 joint 必须：`id = 1..11`、`mode`、`position`（**SDK user 度**）、
`p/i/d/output_limit`（`SG100_Quant` 量化，range 10，对称 int16）。

**所有 struct 必须 deterministic zero-init**。C wrapper 的写法：

```cpp
Hand_Control_t control{};          // zero-init 整个 176 B
for (int j = 0; j < JOINT_COUNT; ++j) {
    control.joints[j].id = ...;    // 再逐字段填充
    ...
}
```

禁止未初始化 payload 直接下发。ABI 大小由
`SG100SDKDriver._assert_struct_abi()` 在 connect 时校验，SDK 升级改变
layout 会 fail loudly 而不是发坏帧。

---

## 7. MODE7 控制语义（两种"模式"不要混淆）

**两个不同层面的模式**：

| 层面 | API | 值 | 含义 |
|---|---|---|---|
| 掌板 runtime mode | `SwitchToRealtimeMode()` / `SwitchToPassthroughMode()` | realtime / passthrough | 调度器是否跑 10 ms 实时交换 |
| per-joint control mode | `Hand_Control_t.joints[i].mode`（CtrlMode_e） | `MODE_NULL=0`、`MODE_POSITION=1`、**`MODE_JOINT_POSITION=7`** | 关节板控制律 |

当前真机主线 = **palm realtime + 全 11 关节 MODE7**：

```
1) connect()                     # 打开串口，回调注册
2) _send_null_control()          # cache 写入全 MODE_NULL（零输出，安全 seed）
3) SwitchToRealtimeMode()        # 进入掌板实时（0x25 → ACK → 调度器重启）
4) 等待 fresh status flow        # 实时反馈恢复的证据
5) SetHandControl(mode=7 × 11)   # 之后才是位置目标
```

要点：

- **先 MODE_NULL seed 再切 realtime**：进入 realtime 本身不可能动手
  （cache 无驱动力输出），这是 vendor teaching_mode_example 的
  production 范式；
- `send_positions_rad()` 在 `enter_realtime()` 成功前 **fail-closed
  返回 None**（cache 还是 MODE_NULL，位置目标不得覆盖它）；
- MODE7 语义（头文件注释）："关节位置控制（基于 joint_zero_rad 的
  绝对位置）"——绝对角参考关节板零点，与 canonical 零的关系见 §16。

---

## 8. vendor scheduler 行为（不要造第二个 sender）

vendor `CommScheduler`（`docs/Realtime_Performance_Analysis.md` §2.2，
二进制复核过）：实时模式下每 **10 ms** 一个周期——

1. FIFO 非空则取一条队列命令，否则**复制最新 `cached_control_`** 编码
   成 0x05 整手帧（clamp → a*x+b → deg2rad）；
2. 写串口（附 CRC32）→ 固定等待 5 ms → 收包校验 → 更新状态缓存并同步
   调用回调。

因此 application 层的正确模型：

```
AVP/control loop（60 Hz）
    ↓ 仅当 target 更新时
SetHandControl(new target)      ← 只更新线程安全的 cache，立即返回
    ↓
vendor cached_control_
    ↓
CommScheduler ~10 ms 自动重发
```

**嵌软禁止** `while True: send_serial()` 或任何自建 100Hz 循环——会与
vendor scheduler 形成双 sender。你调用 `SetHandControl` 的频率不需要
与 10 ms 对齐；60 Hz AVP 更新率是足够的（高频 flooding 没有收益，
cache 覆盖语义决定了只有最新值会被发送）。

---

## 9. MODE7 已完成的真机验证（N0–N3a）

| Gate | 内容 | 结论 | 证据 |
|---|---|---|---|
| N0 | MODE_NULL realtime transport | **PASS — REAL** | 静态探针：机械零位各 SDK joint 回读 ≈0°（±0.055°） |
| N1 | full-hand MODE7 zero-output entry | **PASS — REAL** | 无可见运动，fault=0 |
| N2 | all 11 RX mode == 7 | **PASS — REAL** | 回调持续全 7 |
| N2' | MODE7 反馈域 | **SUPPORTED — REAL** | finite / stable / non-multiturn / joint-angle-scale |
| N3a | J6/index_j3 identity active command | **PASS — REAL** | `recording/sg100_mode7_active_20260903_202121_J6` |

MODE7 进入 transition 时序（保留的真实 v4 artifact）：

- MODE_NULL 帧 0–2 @ 9.4 / 19.4 / 29.4 ms（**允许**，属于 transition，
  不能看到第一帧 mode0 就判失败）；
- first-any-mode7 == first-all-mode7 @ **39.4 ms**（历史多次运行约
  27–40 ms）；
- 第 5 个连续 all-11-mode7 帧标记 `MODE7_STABLE` @ **79.4 ms**（历史
  稳定约 67–80 ms）。

**稳定判据（工具已实现）**：5 个连续 fresh callback 且 all 11 mode==7。

---

## 10. MODE7 feedback 当前结论（数据侧红线）

| 反馈域 | 结论 |
|---|---|
| MODE_NULL feedback | **不能作为 canonical joint angle**。真机证明是 raw/accumulated-like 多圈坐标（真实 ~15–30° 运动 → 回调 0→1854° 扫描） |
| MODE7 feedback | 真机证明 finite / stable / non-multiturn / joint-angle-scale（vendor log 1676 次 mode0→7 转换后数值域收窄到 [−165, +111]°） |

但 **MODE7 → exact canonical 映射（F_fb）未完成**：

```
feedback_map_valid = False
canonical_valid    = False
```

代码现状（`sg100_sdk_driver.read_feedback()`）：

- 每个样本带 `joint_modes`（11 个硬件 mode 字节，真实关节板状态，非
  命令回显）、`feedback_domain`（all-7 → `MODE7_JOINT_POSITION_CANDIDATE`，
  否则 `NON_CANONICAL_DIAGNOSTIC`）、`canonical_valid`；
- `q_rad` 仅做 deg→rad 单位转换，**绝不经过 F_cmd 逆映射**；
- 只有 (all-7 + `mode7_hardware_validated` gate + validated
  `SG100FeedbackMap`) 三者齐备才会应用 F_fb；
- collector 侧：非 canonical 样本被
  `HAND_FEEDBACK_NON_CANONICAL_MODE` 硬拒（`collector.py:396`）。

**红线：raw MODE7 feedback 不得写入 training canonical state。**

---

## 11. 入站映射为什么独立（历史 bug）

历史错误：真实 feedback path 曾直接调用 outbound
`hardware_map.actual_to_canonical()` —— 隐式假设
`F_fb = F_cmd⁻¹`。

当前架构：`SG100FeedbackMap`（`sg100_driver.py`）独立表示入站映射：

```python
SG100FeedbackMap(
    sdk_callback_index_by_canonical,  # 通道 permutation
    sign,                              # 每轴 ±1
    zero_offset_rad,                   # 每轴零偏
    validated=False,                   # 必须真机逐轴 delta 验证后才能 True
)
```

- F_fb 未验证时：raw callback 保留（诊断用），`canonical_valid=False`；
- `callback_to_canonical()` 在 `validated=False` 时直接 raise；
- **验证方法**（P8 任务）：逐轴施加小 canonical delta，观测 dominant
  反馈通道、方向、零点，回填上表并置 `validated=True`。

---

## 12. 数据安全状态（dataset gate）

- 17D schema 不变：`rm65_sg100_joint17_v1` = RM65 6 + SG100 11，rad，
  absolute joint position；
- 但 MODE7 exact canonical measured hand 未验证 →
  **dataset = QUARANTINED，training-eligible measured-hand data 必须继续拒绝**；
- runner 里有硬 gate（`run_avp_arm_hand_sim.py:1884`）：
  `--collect-arm-hand-data` + `--hand-backend real` 直接 `RuntimeError`
  （"SG100 F_fb 尚未验证：real-hand dataset 必须保持 QUARANTINED"）；
- **command path 成功 ≠ measured canonical state 已验证**。J6 PASS 解锁
  的只是单轴命令能力，不解锁 dataset。

---

## 13. J6（index_j3）已完成的真机主动控制验证

这是目前**唯一**通过真实主动命令验证的轴。真实测试数据
（`recording/sg100_mode7_active_20260903_202121_J6`）：

| 项 | 值 |
|---|---|
| canonical | index_j3（SDK J6，identity candidate） |
| 命令 | active hold + 单次 **+0.5°** micro command |
| pre（hold 基线） | −0.659181535° |
| settled feedback delta | **+0.252689660°** |
| peak | +0.367402673° |
| dominant 反馈通道 | J6 |
| 第二通道 | 0°（其余十轴并列） |
| 全 11 轴 RX mode | 7 |
| fault | 0 |
| transport stall | NO |
| cleanup | MODE_NULL command accepted |
| 首个 fresh RX 延迟 | micro 命令后 4.255 ms |

结论：**J6 IDENTITY COMMAND PATH = SUPPORTED — REAL HARDWARE**。
这证明整条链路（`SetHandControl` + MODE7 active controller + SDK 通道
J6 ↔ 物理 index_j3 + 反馈响应）在真机上打通。

五次 `SetHandControl` 用途依次：MODE_NULL seed → MODE7 zero-output →
active hold → micro → cleanup。工具 contract 断言
`set_hand_control_counter_delta == recorded_command_events`。

---

## 14. J6 active controller 参数（仅限已验证范围）

历史真机验证过的参数：

```
p = 0.1,  i = 0,  d = 0,  output_limit = 0.5
```

来源：`examples/realtime_joint_control_example.cpp` 的
`InitHandCtrl(0.1, 0, 0, 0.5)` + vendor `sg100_control.log` MODE7 TX
（量化值 328/0/0/1638 ↔ 0.1/0/0/0.5，`SG100_Quant` range 10）。

注意：只证明了**该参数在 J6 小幅（±0.5°）测试中受控**。不要自动推断
所有关节 production 都用同参数。当前 AVP restricted allowlist 路径已配置为
Kp 10.0 / Ki 0 / Kd 0 / output_limit 0.5；直接 `send_positions_rad`
路径配置为 Kp 10.0 / Ki 0 / Kd 0.05 / output_limit 1.0。两条路径的
Kp 10.0 均未被逐轴真机验证。

---

## 15. J7 当前失败——准确结论（不要误传）

2026-09-04 真机 run：`recording/sg100_mode7_active_20260904_094530_J7/`
（`summary.json`：`status=HARD_FAIL`）。

计划：canonical 0 → −0.5°，经 F_cmd gain=−1 → SDK J7 0 → +0.5°。

实际流程：ZERO 确认 → MODE7 zero-output → **active hold 阶段**
（J7 携带 p=0.1/output=0.5、target=当前 SDK 0°，其余十轴零输出）→
`ActiveMicroMotionAbort: excessive target-joint excursion` →
fail-closed → MODE_NULL cleanup。

| 项 | 值 |
|---|---|
| micro-motion | **NOT EXECUTED**（`exactly_one_micro_delta_command: False`） |
| active hold | NOT_RUN（gate 未过） |
| fault | 0 |
| CRC | 0（NO_CRC_REPORTED） |
| cleanup | PASS |

准确结论（逐条）：

```
J7 F_cmd gain -1        : CANDIDATE（软件就绪，方向未真机证实）
J7 offset               : UNVERIFIED
canonical 0 == SDK 0    : NOT VALIDATED（本次 hold 就是想验证它，没过）
J7 micro-motion         : NOT EXECUTED
```

**不要写成 "J7 sign flip FAIL"** —— 失败的不是 sign flip 本身（根本没有
走到 micro 步），而是 **SDK target=0 作为 active hold seed 没有通过
excursion 安全 gate**（hold 期间 J7 反馈偏移超过 `HOLD_HARD_EXCURSION_DEG
= 0.5°` 硬限）。root cause 待查（候选：J7 的 MODE7 反馈零点/尺度与
SDK 命令域不同构 → active hold 一上输出关节就被推离 0°，excursion
guard 正确地 fail-closed 了）。

---

## 16. J7 之后：嵌软要注意的 F_cmd 设计

F_cmd 必须是完整的仿射形式：

```
q_sdk = gain * q_canonical + offset          # 不要只写 gain * q_canonical
```

- J7：`gain candidate = -1`，`offset = TBD`（不是 0 的已验证事实）；
- per-joint 配置必须支持：`gain / offset / min / max / validation flag`
  （`sg100_hardware_map.yaml` 已是此结构），**禁止 hardcode 特殊
  if/else 分支**；
- J7 的排查方向：先用只读方式比对 J7 的 MODE7 反馈零点（静止时回调
  值 vs SDK 命令域 0°），确认是否存在需要 offset 吸收的零点差；再
  决定 hold seed 用"当前反馈位置"还是 SDK 0°。

---

## 17. AVP real backend 当前实现状态

`sg100_teleop/robot/sg100_restricted_avp_backend.py`
（`SG100RestrictedAVPBackend`，217 行）+ `run_avp_arm_hand_sim.py` 集成：

原则已落实：

- simulation 与 real backend 消费**同一个** canonical
  `q_hand_cmd[11]`（retargeter 输出，唯一源）；
- 没有复制 `SG100Retargeter`，没有改 AVP hand geometry mapping；
- real 路径被 restricted envelope 包裹（allowlist / 限速 / fail-closed）。

runner 集成点（`run_avp_arm_hand_sim.py`）：

| 位置 | 行为 |
|---|---|
| L1884 | `--collect-arm-hand-data` + real hand → 拒绝（dataset quarantine） |
| L1892 | `SG100SDKDriver` + `SG100RestrictedAVPBackend(joint_allowlist=args.sg100_real_joints)` |
| L1926 | connect → start → **enter_realtime()**（MODE_NULL seed → SwitchToRealtimeMode → 等 fresh flow） |
| L2499 | **C 键**：`restricted.anchor(q_hand0_runtime)` —— 缓存 F_cmd 映射后的 MODE7 目标 + 零输出 |
| L2073 | **A 键**：`set_enabled(True)`；**D 键** disable → hold_passive |
| L2093 | **E 键**：`emergency_stop()` |
| L3184/3306 | physics 循环内 `restricted.submit(q_hand_cmd, tracking_received_ns, dt_s)`，rejected → safe stop 打印 |
| 参数 | `--sg100-real-joints`（默认 `("index_j3",)`，即 J6-only） |

软件状态：**SOFTWARE READY（J6-only allowlist）**，尚未有真机 AVP run。

---

## 18. 推荐的 production interface（嵌软接手面）

`SG100RealBackend` 推荐 contract（现有限制版已实现大部分，命名可调但
语义必须保留）：

```
输入
  canonical_q_cmd_rad[11]   # retargeter 输出（post-lock/filter）
  timestamp_ns              # 命令时间
  enabled / tracking_valid

输出 / 状态
  last_sdk_q_cmd            # 最近 F_cmd 后的 SDK 域目标
  joint_modes               # 最近反馈的 11 个硬件 mode
  fault                     # SDK fault 状态
  transport_health          # fresh RX 延迟 / sequence 连续性 / CRC 计数
  command_health            # SetHandControl 接受率 / reject 计数
  feedback_raw              # SDK callback 域（deg→rad 后原值）
  feedback_canonical_valid  # F_fb 是否已验证并应用（当前 False）

方法
  connect()                       # 打开串口 + 回调注册
  enter_realtime()                # MODE_NULL seed → Switch → 等 fresh flow
  enable() / disable()            # A/D；disable → hold_passive
  send_canonical_command(q, ts)   # §19 pipeline，失败 fail-closed
  hold()                          # 保持当前目标（被动）
  disable_to_mode_null()          # 全轴 MODE_NULL
  stop() / disconnect()           # 停调度、关串口
  get_status()                    # 上面的状态聚合
  emergency_stop()                # E 键路径
```

---

## 19. real backend command pipeline（顺序固定）

```
canonical q[11]
    ↓ validate finite                        # 非 finite → reject
    ↓ canonical range check                  # semantics.yaml 机械限位，越界 reject（无 clamp）
    ↓ F_cmd                                  # gain*q+offset（sg100_hardware_map.yaml）
    ↓ SDK range check                        # joint_angle_map.json user 限位，越界 reject（无 clamp）
    ↓ optional per-cycle delta gate          # None 时关闭；配置数值时超限 → reject
    ↓ velocity / slew limiter                # 限速后的值才可下发（limit 后仍需重过 range check）
    ↓ tracking-spike rejection               # 单帧跳变 > 0.20 rad → safe stop
    ↓ joint allowlist                        # 未解锁轴保持 anchor 目标 + 零输出
    ↓ Hand_Control_t（zero-init + 逐字段填充）
    ↓ SetHandControl()
```

**严禁 gross range error 后 silent clamp**。canonical 端点 vs SDK 限位
的 ~0.2° 数值边界差属于"small numerical boundary handling"范畴，当前
策略就是 reject（保守、可审计）；如要放宽必须单独定义，不得借此掩盖
mapping bug。

当前 restricted envelope 默认参数（`SG100RestrictedAVPBackend.__init__`）：

| 参数 | 默认值 |
|---|---|
| max_cycle_delta_rad | 0.05 |
| max_velocity_rad_s | 0.5 |
| tracking_spike_rad | 0.20 |
| stale_tracking_s | 0.10 |
| feedback_stall_s | 0.10 |
| crc_storm_delta | 3（相对 anchor 基线的增量） |

`run_avp_arm_hand_sim` 真机手入口当前显式传入
`max_cycle_delta_rad=0.20`；超过该值仍会触发
`PER_CYCLE_DELTA_EXCEEDED`。传入 `None` 可关闭该拒绝门，并让大的合法目标差
仅由 `max_velocity_rad_s` 逐步追踪。`tracking_spike_rad`、范围、反馈、CRC 和
速度限幅独立生效。

---

## 20. C-anchor / AVP enable 语义（重点实现/保留）

当前约定：

- **C 键** = AVP human anchor + teleop enable（同时捕获 wrist 与 hand
  runtime anchor），**不是 SG100 机械零点**；
- **C 之前不得发送任何 active SG100 command**（driver
  fail-closed：realtime 进入后 cache 是 MODE_NULL，`send` 也被
  envelope 的 `C_ANCHOR_REQUIRED` 挡住）；
- anchor 时只做：F_cmd 映射目标 → MODE7 + **零增益/零输出** 缓存
  （`send_mode7_zero_output_sdk_targets`），不用 F_fb；
- **enable 第一帧不得瞬移**：`anchor()` 缓存的 `_last_canonical` 成为
  first-command latch —— submit 的 velocity limiter 从 anchor 值起步，
  保证 first active command ≈ 当前 safe command target。

嵌软接手时必须保留这个 latch 语义（或实现 backend relative offset
anchoring，等价即可）。

---

## 21. joint allowlist（bring-up 分期）

`SG100RestrictedAVPBackend` 构造时强制：allowlist ⊆
`hardware_validated_joints`（当前 = `("index_j3",)`）。J7 未通过 →
**构造直接 raise**，不可能进入 production allowlist。

推荐 bring-up 顺序：

| Phase | 范围 | 前置 |
|---|---|---|
| 1 | **J6 / index_j3 only**（当前默认） | 已满足 |
| 2 | 多个 identity/simple 轴（thumb_j3 / index_j1 等行程小、无符号翻转的） | 每轴 micro-motion 真机 PASS |
| 3 | J7 等 sign/offset 特殊轴 | §16 的 zero/offset 问题解决 |
| 4 | full 11D | 全部轴验证 + F_fb 至少 angle-scale 全轴确认 |

**不要第一版就全开。**

---

## 22. safety envelope 总表（已实现 / 必须保持）

| gate | 实现 | 触发动作 |
|---|---|---|
| finite check | `_validate_and_map` | reject → safe stop |
| canonical range | 同上（无 clamp） | reject → safe stop |
| SDK user range | 同上 + driver `_user_deg_within_vendor_range` | reject → safe stop |
| per-cycle max delta | `max_cycle_delta_rad`（AVP 真机入口当前为 `0.20`） | `PER_CYCLE_DELTA_EXCEEDED`；`None` 时关闭 |
| velocity limit | `max_velocity_rad_s * dt`（slew） | 限幅（非 reject），结果重验 |
| tracking spike | `tracking_spike_rad` 单帧跳变 | `TRACKING_SPIKE` |
| AVP stale timeout | `stale_tracking_ns` | `STALE_AVP_TRACKING` |
| feedback stall | fresh RX > 100 ms | `TRANSPORT_STALL` |
| unexpected RX mode | 非 all-7 | `UNEXPECTED_RX_MODE` |
| fault | `driver.fault_active` | `SG100_FAULT` |
| CRC storm | 增量 ≥ 3 | `CRC_STORM` |
| serial/realtime health | `serial_open` / `vendor_work_mode != 0` | `TRANSPORT_CLOSED` / `REALTIME_MODE_LOSS` |
| allowlist | 构造期强校验 | 构造 raise |
| C gate | 未 anchor / 未 enable | `C_ANCHOR_REQUIRED` / `NOT_ENABLED_NO_ACTIVE_COMMAND` |
| emergency stop | E 键 → estop latch | `EMERGENCY_STOP`（latch 到进程结束） |
| MODE_NULL safe disable | `hold_passive()` / cleanup | 全轴 MODE_NULL |

fail-closed 语义：任何上述事件 → **停止 active command → MODE_NULL →
stop → disconnect**，并保留 artifact（recording/ 目录 summary/samples/
command audit）。E-stop 是 latch（`ESTOP_LATCHED`），不自动恢复。

---

## 23. CRC 行为

- CRC error 来自 `RS485Adapter::DrainRxBuf()` 的 **RX 帧校验**；坏帧
  **不进入** rx callback，adapter 丢帧头并继续重同步，无自动 retry
  （二进制控制流审计结论）；
- J6 真机测试出现过一次终端 CRC 行 → 分类
  `CRC_TRANSIENT_RECOVERED`（无 fault / 无 mode loss / 无 stall）；
- C wrapper 已暴露 `crc_error_count` + `crc_error_timestamps_ns`
  （只旁路监听 SDK 已有 console 输出，未实现协议代码）；
- production 需要 **CRC rate / storm 阈值**（当前
  `crc_storm_delta=3`），**不是单个 CRC 立即 emergency stop**。

---

## 24. fresh feedback vs cached feedback（历史 bug）

历史错误：cached pre-command RX 被误认为 post-command RX。

当前正确模型（必须保持）：

- wrapper 内部状态快照带 **callback sequence + receive timestamp**
  （CLOCK_MONOTONIC，与 Python `time.monotonic_ns()` 同源）；
- Python `read_feedback()` 按 sequence 去重：`sequence == last` →
  cache 未刷新，返回上次样本（不是新测量）；
- `driver._wait_fresh_sequence(timeout)` = fresh-frame watermark：
  只接受 `sequence > start_sequence` 的样本；
- **命令延迟测量只能用命令提交之后的 fresh RX**；禁止 poll 最新
  cached state 当作新反馈。

---

## 25. transport watchdog

当前 validation 工具与 restricted envelope 共用同一逻辑：

- **阈值：fresh RX 停止约 100 ms**（工具 `TRANSPORT_STALL_S = 0.1`；
  envelope `feedback_stall_s = 0.10`）→ hard failure；
- 行为：`ActiveMicroMotionAbort("transport stall")` /
  `TRANSPORT_STALL` safe stop → MODE_NULL cleanup；
- 恢复策略：**无自动恢复**——fail-closed 后需要人工重新走
  connect → enter_realtime → anchor → enable 流程。

嵌软 production 不要自行发明不同阈值/逻辑；如需调整先复用该常量
定义处并同步两处。

---

## 26. MODE_NULL cleanup（所有退出路径）

所有退出路径——PASS / FAIL / exception / KeyboardInterrupt / EOF /
fault / transport stall / mode loss——都必须尝试：

```
MODE_NULL 全轴命令 → stop() → disconnect()
```

- cleanup 每个 action 独立保护：一个 cleanup action 失败不能阻止后续
  cleanup（`try/finally` 链）；
- 已验证：J6/J7 真机 run 的 `cleanup_status: PASS`；
- 注意诚实性：cleanup 命令**被接受** ≠ RX 已回显 mode 0（J6 artifact
  的 30 ms cleanup 采样窗口内 RX 仍显示 mode 7，传播需要时间）。

---

## 27. validation 工具 vs production（不要复制当控制器）

| 工具 | 用途 | production? |
|---|---|---|
| `tools/sg100_hardware_probe.py` | transport / MODE_NULL 只读 probe | ❌ |
| `tools/sg100_mode7_feedback_probe.py` | MODE7 entry / 反馈域验证（v5 schema，含 transition 判据） | ❌ |
| `tools/sg100_mode7_active_micro_motion.py` | 单轴 active command smoke（dry-run 默认，ZERO→HOLD→MOVE 双人工门） | ❌ |
| `tools/sg100_mode7_mapping_characterization.py` | 反馈映射表征（复用 feedback_probe 基建） | ❌ |
| `tools/sg100_sign_characterization.py` | 被动符号表征（历史，见 §28） | ❌ historical |

production AVP loop = `SG100SDKDriver` + `SG100RestrictedAVPBackend`
+ safety envelope（§22）+ runner 集成（§17）。**禁止把 validation
script 复制为最终控制器**——它们没有 C-gate/allowlist/velocity
limiter 的完整组合，artifact schema 也是诊断用的。

---

## 28. 已废弃路径（DEPRECATED / HISTORICAL ONLY）

| 旧路径 | 状态 |
|---|---|
| 历史 raw RS485 packet / 自算 CRC sender | **DEPRECATED** —— production 必须走 vendor SDK |
| 历史 custom 协议 workaround（0x05/0x25 手工帧） | **DEPRECATED** —— vendor 内部细节 |
| MODE_NULL passive sign characterization（`sg100_sign_characterization.py`） | **HISTORICAL ONLY** —— MODE_NULL 反馈不可作关节角 |
| 用 outbound hardware map（`actual_to_canonical`）解析真实入站反馈 | **DEPRECATED（bug）** —— 已被独立 `SG100FeedbackMap` 取代 |
| 未验证 feedback 写 dataset | **禁止** —— quarantine gate 在 runner + collector 双层 |
| manual hand-movement characterization（被动掰手） | **HISTORICAL ONLY** —— 已被主动 J6/J7 测试取代 |
| `SG100_SDK_Publish_backup_wrong_branch`（14-joint pre-V1.5） | **NOT SUPPORTED** |
| `SetJointPositions` production 用法 | **DEPRECATED** —— wrapper 入口已删除 |

---

## 29. RM65 与 SG100 职责边界

| | RM65 | SG100 |
|---|---|---|
| 输入 | AVP arm target | AVP hand tracking |
| 链路 | arm mapping / IK / RM65 backend（CANFD，已有 v3 体系） | `SG100Retargeter` → canonical q → F_cmd → SG100 driver |
| 并行 | 同一个 teleop runner（`run_avp_arm_hand_sim.py`）中同步运行，但 **command backend 完全分离** | |

**不要把 SG100 joint command 塞进 RM65 SDK**，也不要让 SG100 driver
依赖 arm backend 的任何状态。两者唯一共享的是 teleop 的 C/A/D/E 键
状态机与 command_bundle_id（数据合同用）。

---

## 30. 实时频率

| 环节 | 频率 | 说明 |
|---|---|---|
| AVP hand tracking | 60 Hz | 上游输入 |
| SG100 command 更新 | 60 Hz（AVP loop） | 不必与 scheduler 对齐 |
| vendor CommScheduler | ~100 Hz（10 ms 固定周期，写死在 `CommScheduler::SerialTask()`） | 自动重发 cache |
| SG100 RX 回调 | 实测 ~100 Hz（最大帧间 19.998 ms） | `RegisterHandStatusCallback` |

嵌软注意：`SetHandControl` 高频 flooding 无收益（cache 覆盖语义，
只有最新值会被发）；也不要试图改 SDK 源码提频——
`Realtime_Performance_Analysis` 明确：不改协议/固件前提下重点是重构
CommScheduler 的固定 5 ms 等待，且"若目标只是稳定 100 Hz，不必推翻
程序框架"。

---

## 31. 单位表（防 deg↔rad 双重转换）

| 环节 | 单位 | 证据 |
|---|---|---|
| AVP / CanonicalHandState | 无量纲 canonical 特征 [0,1]/[−1,1] | `canonical_hand.py` |
| **retargeter 输出 / canonical q[11]** | **rad** | `sg100_joint_semantics.yaml` |
| MuJoCo | rad | mjcf |
| F_cmd 输入 | rad（canonical） | `SG100HardwareMap.canonical_to_actual` |
| F_cmd 输出 | rad（SDK user 域）→ driver 内 `np.rad2deg` 一次 | `sg100_sdk_driver.send_positions_rad` |
| **`Hand_Control_t.position`（SetHandControl 输入）** | **SDK user DEGREES** | vendor 文档 §2.2 + 新二进制复核（cache 原样存，发送时 clamp→a*x+b→×π/180） |
| SDK 内部 wire | rad（joint_angle_map 变换后） | scheduler encode lambda |
| `Joint_Status.position`（wire） | rad（accumulated，模式相关语义） | sg100_types.h |
| 回调给 Python 的值 | **user degrees**（SDK 已做逆映射 + rad2deg，无 receive clamp） | `Realtime_Performance_Analysis` §2.2 |
| Python `sdk_callback_positions_rad` | rad（×π/180 一次，仅单位转换，无映射） | `read_feedback` |
| dataset 存储 | rad | schema |

**最容易犯的错**：回调已经是 user degrees，Python 侧只需 ×π/180；
而 `SetHandControl` 输入是 user degrees，F_cmd 输出（rad）后必须
×180/π 一次。两个方向各恰好一次三角转换，多一次就是 double
conversion bug。

---

## 32. command event / dataset event

- validation 工具的 diagnostic send（`SG100Mode7ActiveSubmission` 等）
  **不产生** normal canonical command-event bookkeeping——它们与
  `SG100CommandSent` 是不同类型，明确不进 canonical provenance；
- production AVP 命令的 canonical 事件由 DATA CONTRACT 定义
  （`ARM_HAND_DATA_CONTRACT_V1.md`：`hand_hardware_sent` raw event +
  canonical action = final hardware-sent qcmd）；
- 但当前 **real measured hand canonical invalid → training-ready
  dataset 禁止**（§12 双层 gate）。command 事件链路已就绪，等 F_fb
  解锁。

---

## 33. 嵌软最小开发任务列表（P0–P8，按序）

| 阶段 | 任务 | 验收 |
|---|---|---|
| **P0** | 稳定 SDK lifecycle：connect / enter_realtime（NULL seed→switch→fresh flow）/ MODE7 / MODE_NULL / fault & watchdog | N0/N1/N2 复现稳定 |
| **P1** | production F_cmd adapter：per-joint gain/offset/min/max/validation flag（就是 `sg100_hardware_map.yaml` + `SG100HardwareMap`，补验证流程） | 每轴真机 micro PASS 后置 validated |
| **P2** | canonical q[11] 接入 `SG100RealBackend`（消费 retargeter 输出；现有 restricted backend 即雏形） | sim/real 同源命令 |
| **P3** | command safety envelope 完整化（§19 pipeline + §22 全 gate） | 单测 + 故障注入 |
| **P4** | AVP C-gate / enable / stale tracking 接入（保留 §20 latch 语义） | C 前 zero active command |
| **P5** | **restricted J6-only 真机 AVP run**（首个真机 teleop，空手） | 操作员视觉确认 + artifact |
| **P6** | 解决 J7 zero/offset（§16 排查方向），J7 micro PASS | J7 `exactly_one_micro_delta_command: True` |
| **P7** | 扩展至 full 11D（按 §21 Phase 2→4 分期） | 全轴 micro PASS |
| **P8** | F_fb 逐轴验证（`SG100FeedbackMap.validated=True`）→ measured-state dataset unlock | `canonical_valid=True` → quarantine 解除（需数据侧同意） |

---

## 34. 禁止事项（嵌软红线）

1. 不要重写 AVP retargeter（含 pinch/power-grasp lock——那是算法侧资产）；
2. 不要自己重新实现 RS485 协议 / CRC / 帧格式；
3. 不要自建第二个 100 Hz vendor sender（双 scheduler）；
4. 不要用 MODE_NULL feedback 做关节角；
5. 不要假设 `F_cmd == F_fb⁻¹`；
6. 不要把 raw MODE7 feedback 直接当 canonical measured q（含写
   MuJoCo 物理角 / dataset state）；
7. 不要把 J7 的 offset=0 / gain=−1 当成已验证事实；
8. 不要直接全 11 轴开放 AVP（allowlist 分期）；
9. 不要 silent clamp gross mapping error（reject，不 clamp）；
10. 不要在 C anchor 之前发 active command；
11. 不要自动 homing / 自动 set-zero / 自动回机械零点（vendor
    zero 语义未验证前禁止 `HomingZero`/`SetJointZero` 类调用）；
12. 不要解除 dataset quarantine（双层 gate 由数据侧解除）。

---

## 35. 快速启动

**environment**：repo `~/VisionProTeleop`；Python `.venv/bin/python`；
SDK `~/SG100_SDK_Publish`（§4 版本）；串口 `/dev/ttyUSB0`，1 000 000
baud。C wrapper `_sg100_c_wrapper.so` 随仓库（源码 mtime 变化自动
重编）。

**启动顺序（固定）**：先 READ-ONLY probe → 确认 → 单轴 validation →
才允许 restricted AVP。

```bash
cd ~/VisionProTeleop

# 1) READ-ONLY：transport / MODE_NULL probe（不发任何命令）
.venv/bin/python -m sg100_teleop.tools.sg100_hardware_probe --port /dev/ttyUSB0

# 2) REAL HARDWARE（用户手动）：MODE7 entry + 反馈域验证
.venv/bin/python -m sg100_teleop.tools.sg100_mode7_feedback_probe --port /dev/ttyUSB0

# 3) J6 历史已完成（reference，不要默认重跑）：
#    recording/sg100_mode7_active_20260903_202121_J6
#    如需复现（用户手动，dry-run 先行）：
.venv/bin/python -m sg100_teleop.tools.sg100_mode7_active_micro_motion \
    --joint J6            # DRY RUN：不打开串口
#   确认 plan 后加 --execute（ZERO→HOLD→MOVE 人工门）

# 4) restricted AVP（真机 J6-only；RM65 保持 sim/关闭）
.venv/bin/python -m sg100_teleop.sim.run_avp_arm_hand_sim \
    --ip <AVP_IP> --arm-backend sim \
    --hand-backend real --hand-port /dev/ttyUSB0 \
    --sg100-real-joints index_j3
#   流程：C（anchor，零输出缓存）→ A（enable active）→ 操作 → D/E
#   mock 后端（无硬件软件回归）：--hand-backend mock

# 5) 软件回归（只读，无硬件）
.venv/bin/python -m pytest sg100_teleop/tests/test_sg100_driver.py \
    sg100_teleop/tests/test_sg100_latest_sdk_driver.py \
    sg100_teleop/tests/test_sg100_mode7_active_micro_motion.py \
    sg100_teleop/tests/test_sg100_mode7_feedback_semantics.py \
    sg100_teleop/tests/test_sg100_mode7_mapping_characterization.py \
    sg100_teleop/tests/test_sg100_no_legacy_backend.py \
    sg100_teleop/tests/test_sg100_restricted_avp_backend.py \
    sg100_teleop/tests/test_sg100_sign_characterization.py \
    sg100_teleop/tests/test_hand_data_contract_measured.py -q
```

标注：1) READ-ONLY；2)3)4) REAL HARDWARE（用户手动执行）；5) 软件。

---

## 36. 当前状态表（2026-09-04 审计）

| milestone | 状态 |
|---|---|
| MODE_NULL transport（N0） | **PASS — REAL** |
| MODE7 entry（N1） | **PASS — REAL** |
| MODE7 all-11 mode（N2） | **PASS — REAL** |
| MODE7 angle-scale feedback | **SUPPORTED — REAL** |
| J6 identity active command（N3a） | **PASS — REAL** |
| J7 sign/offset（N3b） | **BLOCKED / ROOT CAUSE PENDING**（hold gate 未过，micro 未执行） |
| restricted AVP backend | **SOFTWARE READY（J6-only allowlist）** |
| full 11D AVP | **BLOCKED** |
| F_fb（exact canonical feedback map） | **PENDING**（`feedback_map_valid=False`） |
| canonical measured hand | **INVALID**（`canonical_valid=False`） |
| dataset | **QUARANTINED**（runner+collector 双层 gate） |

## 37. 代码 ownership map

| 文件 | 职责 | production | 嵌软维护 | 允许修改 | 备注 |
|---|---|---|---|---|---|
| `robot/sg100_sdk_driver.py` | SDK adapter（lifecycle/MODE7/反馈/诊断） | ✅ | **YES** | 是 | 本交接核心 |
| `robot/sg100_c_wrapper.cpp` + `_sg100_c_wrapper.so` | extern "C" ABI 桥 | ✅ | **YES** | 是 | zero-init、ABI 断言、CRC 监听 |
| `robot/sg100_restricted_avp_backend.py` | real command safety envelope | ✅ | **YES** | 是 | J6-only allowlist |
| `robot/sg100_driver.py` | 契约/类型/F_cmd/F_fb 定义 | ✅ | **YES** | 是（谨慎） | `SG100FeedbackMap` 在此 |
| `config/sg100_hardware_map.yaml` | F_cmd 权威映射 | ✅ | **YES** | 是 | J7 gain=−1 |
| `sim/run_avp_arm_hand_sim.py` | teleop 编排（sim/real 同源） | ✅ 集成 | 协作 | 是 | 算法侧共同维护 |
| `data_collection/collector.py` 等 | dataset gate / quarantine | ✅ | 协作 | 谨慎 | 数据侧主导 |
| `tools/sg100_hardware_probe.py` | validation | ❌ | 参考 | 尽量不动 | READ-ONLY |
| `tools/sg100_mode7_feedback_probe.py` | validation | ❌ | 参考 | 尽量不动 | |
| `tools/sg100_mode7_active_micro_motion.py` | validation | ❌ | 参考 | 尽量不动 | ZERO→HOLD→MOVE |
| `tools/sg100_mode7_mapping_characterization.py` | validation | ❌ | 参考 | 尽量不动 | |
| `tools/sg100_sign_characterization.py` | historical | ❌ | NO | 不动 | DEPRECATED |
| `retargeting/sg100_retargeter.py` | AVP 算法 | ✅ | **NO** | 算法侧 | 禁改 |
| `robot/sg100_mock_driver.py` | 测试 mock | 测试 | YES | 是 | 保持与 SDK driver 契约同步 |
| 测试 `test_sg100_*`（9 个文件） | 回归 | ✅ | YES | 是 | 205 passed 基线 |
