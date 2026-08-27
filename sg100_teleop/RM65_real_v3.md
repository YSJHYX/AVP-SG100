# RM65 真机迁移方案 v3(原理与实施教程)

> 本文取代 v1(`REAL_RM65_MIGRATION.md`)与 v2(`RM65_真机迁移方案_修订版_v2.md`),是 RM65-B 真机接入的**唯一实施依据**。
> 范围:实体 RM65-B 控制接入。实体 SG100 驱动不在范围内(装手只影响 tool profile 与安全包络)。
> SDK 事实均以本机安装的 `robotic-arm 1.1.6`(`mujoco-learn` conda env)源码为准,文中引用 `interface.py` = `Robotic_Arm/rm_robot_interface.py`,`wrap.py` = `Robotic_Arm/rm_ctypes_wrap.py`。
>
> 写作时间:2026-08-27。与实际不符时,以源码为准并回改本文。

## v3 相对 v2 的修订记录

1. **修复停止顺序竞态**(v2 §15/16/28 三处):改为"先原子停 sender → 排空在途 → 再慢停",并给出失败升级路径(§A3.5);
2. **安全参数给出数值初值表**(v2 全符号化):qdot/qddot/watchdog/follow-error/σ_min 等三档 profile 初值(§B3.1);
3. **回填 v1 丢失内容**:控制器具体 IP/端口/网段配置、冒烟逐行验收判据、网络拓扑隔离(有线 NIC 承载命令、AVP 走独立网卡)、Phase 2 无硬件单元验证;
4. **反馈分级策略**:PROFILE_10 + low-follow 阶段允许 50 Hz TCP polling(`qd=None`),high-follow 前才强制 UDP push(§A5.2);
5. **IK 连续性门给出数值语义**:与代码常量 `IK_FALLBACK_MAX_STEP_RAD=0.05` / `IK_ACCEPT_JUMP_LOG_RAD=0.2` 对齐,并明确 real 模式禁用 best-effort fallback 改 HOLD(§A3.3);
6. **状态机落到键位**:C=标定、A=ARM、空格=pause,定义 RUNNING 中按 C 的语义,给出与现有 `TeleopArmingController` 的映射(§A4);
7. **digital twin 给出实现分界**:flange-only 阶段单 kinematic MjData,SG100 装上后才拆双 MjData(§A5.4);
8. **修正链路图**:补上现有代码真实存在的 `ArmCommandFilter`(README.md:253 的"arm 链无 EMA"陈述与代码不符,以代码为准,见 §C5);
9. **新增**:奇异点 gate、断电行为、单位审计教程、T_base_table 测量教程、Go/No-Go 全项可溯源;
10. **修文档结构**:单一编号轴,架构图只保留一张;
11. **v3.1(同日)**:采纳仿真实测结论——HOLD 在间歇性不可达下卡顿;§A3.3 重写为失败分类处理(不可达贴边界跟、tracking 丢失才 HOLD、门降级为毛刺兜底);**wrist-roll-ik 定为真机首推 IK 模式**,联调第 4 级默认启用。

---

# 第一部分 原理篇

## A1. 链路总览与迁移原理

### A1.1 现有 sim 链路(以代码为准)

```text
AVP 右手 tracking (~60 Hz)
  → TrackingFrameGate            tracking/frame_gate.py
  → AVPWristRelativeMapper       tracking/avp_wrist_mapper.py(按 C 建 anchor)
  → TcpTargetFilter              control/tcp_target_filter.py(τ=0.06s, 0.25m/s, 1.5rad/s)
  → RM65IKSolver                 kinematics/rm65_ik.py(RealMan 官方闭式 IK,最近种子解)
  → q_arm_goal (~60 Hz)
  → ArmCommandFilter             JointCommandFilter @ 500 Hz 物理率
                                   (runner --arm-filter-tau 默认 0.03s,
                                    --max-arm-velocity 默认 3.0 rad/s)
  → q_arm_cmd
  → write_robot_controls()       sim/combined_robot_layout.py:189 → data.ctrl
  → MuJoCo physics → USDZ → AVP
```

注意:`README.md:253` 写"arm chain 不再叠加 joint-space EMA"是**过时的**——代码里 `arm_command_filter` 真实存在(runner:1149-1161 创建、:2436 在物理率上更新)。本文以代码为准。

### A1.2 为什么上游可以整体复用

三个已验证的事实决定了迁移量很小:

| 事实 | 依据 | 推论 |
|---|---|---|
| IK 就是 RealMan 官方算法 | `rm65_ik.py` 用 `Robotic_Arm` 包 `Algo`(interface.py:6147),rad/deg、四元数次序桥已写好 | IK 输出的关节角与控制器语义完全一致 |
| MuJoCo 世界系 = 官方基座系 | FK 一致性 ≤5µm / 0.022°(rm65_ik.py 内注) | 不需要任何 SE(3) 修正 |
| 命令链已有低通+限速 | `ArmCommandFilter` | 结构可直接搬到真机,只需换 dt 语义(§A3.2) |

**迁移 = 在 `write_robot_controls()` 这一个分叉点接入真机后端 + 补真机必备的安全层。** SDK 调用不得散落到 mapper/filter/IK/arming/recorder/streaming 里,全部收敛在 `sg100_teleop/robot/` 子包。

### A1.3 目标架构(全文唯一架构图)

```text
AVP tracking (~60 Hz)
  → TrackingFrameGate → WristRelativeMapper → TcpTargetFilter → 官方 IK
  → IKContinuityGate            @60 Hz,毛刺帧丢弃,失败分类处理(§A3.3)
  → LatestTargetBuffer          (target+timestamp+seq, 生产/消费解耦, §A3.4)
  → JointTrajectorySafetyLimiter @sender 实测周期
      ├ joint position soft limits
      ├ joint velocity limits
      └ joint acceleration limits
  → FixedPeriodSender (100/125/200 Hz)
  → rm_movej_canfd()  ──TCP──▶ RM65 控制器
                                │
  RM65 反馈(TCP polling 50 Hz 或 UDP push 5 ms,分级采用 §A5.2)
      → SafetyMonitor
      │   ├ target-age watchdog      ├ feedback-age watchdog
      │   ├ cmd-actual deviation     ├ controller error state
      │   └ fence/collision state
      → [任一触发] 原子停 sender → 排空 → rm_set_arm_slow_stop() → DISARM(§A3.5)
      → Digital Twin(MjData, kinematic, §A5.4)→ USDZ → AVP
      → Recorder(扩展 mujoco_recording.npz + sender 日志)
```

---

## A2. 透传控制原理(rm_movej_canfd)

### A2.1 规划式 vs 透传

RealMan SDK 有两类关节运动接口:

- `rm_movej(joint_deg, v, r, connect, block)`(interface.py:1460):**每次调用做一次完整轨迹规划**(梯形速度曲线,带加减速)。`v` 是速度百分比 1–100。适合点到点;**流式下发会顿挫**,因为每个目标都从零速规划。
- `rm_movej_canfd(joint, follow, expand, trajectory_mode, radio)`(interface.py:1736):**角度不经规划直接透传给关节执行**("若指令正确,机械臂立即执行")。遥操作的标准通道。

### A2.2 follow 与 trajectory_mode 的确切语义

```text
follow=True  高跟随:控制器以更紧的环路跟踪透传目标,要求发送周期 ≤10 ms(interface.py:1746)
follow=False 低跟随:周期要求宽松,适合 50–100 Hz
trajectory_mode 仅在高跟随模式生效(docstring:"高跟随模式下,0-完全透传、1-曲线拟合、2-滤波")
radio 是平滑系数:曲线拟合取 0~100,滤波取 0~999
```

**三种合法组合**(v1 的 `follow=False + trajectory_mode=2` 是无效搭配,v2 已纠正,v3 沿用):

| CLI 值 | 参数 | 用途 | 时机 |
|---|---|---|---|
| `low` | follow=False | 链路联调、首跑 | Phase 4 全程默认 |
| `filter` | follow=True, mode=2, radio 调参 | 真机遥操作首选稳定模式 | Phase 4 第 5 级后 |
| `high` | follow=True, mode=0 | 低延迟实验 | sender 周期实测达标后 |

### A2.3 核心安全原理:canfd 没有速度参数,软件限速就是真限速

`rm_movej_canfd` 的参数里**没有任何速度上限**(对比 `rm_movej` 的 `v` 百分比)。控制器执行的速度完全由你发送的相邻两个目标的差值除以周期决定:

```text
实际关节速度 ≈ |q_cmd[k] - q_cmd[k-1]| / T_send
```

因此 **`JointTrajectorySafetyLimiter` 不是"滤波美化",它就是真机唯一的速度/加速度约束**。任何绕过限幅器直发 canfd 的路径都是安全违规(这条要写进 README 维护约束)。

### A2.4 周期与 jitter

高跟随要求 ≤10 ms。100 Hz = 10 ms **没有任何余量**——一次 GC、一次线程调度抖动就超期。所以:

```text
low:    100 Hz 起步
filter: 125 Hz
high:   200 Hz(SDK 注:I 系列有线网口周期最快可达 2 ms)
```

且必须实测并记录五个指标:`mean / p95 / p99 / max period`、`missed deadline count`(测量方法见 §B4.4)。

---

## A3. 纵深防御安全层原理

### A3.1 防御层级总表

| 层 | 位置 | 机制 | 失效假设 |
|---|---|---|---|
| 1 | 软件 @60 Hz | IKContinuityGate 拒绝跳变解 | IK 返回坏解 |
| 2 | 软件 @sender 率 | q/qdot/qddot 限幅器 | 上游全部失守时的最后软件防线 |
| 3 | 软件 | target/feedback watchdog + 停止序列 | 数据流断/延迟 |
| 4 | 软件 | workspace guard(基于 T_base_table) | 目标合法但越出物理桌面 |
| 5 | 控制器 | electronic fence(whole-arm) | PC 侧软件整体失效 |
| 6 | 控制器 | 动力学碰撞保护(`rm_set_collision_state` 0–8) | 意外接触 |
| 7 | 物理 | 急停按钮 | 一切软件失效 |

上层失效下层兜底;第 5/6/7 层独立于 PC 存在。

### A3.2 限幅器:为什么 step clamp 不够,要 q/qdot/qddot 三层

CANFD 透传不做轨迹规划,所以软件侧必须自己构造一条有界的命令轨迹。离散形式:

```text
位置:  q_min + margin ≤ q_cmd ≤ q_max - margin
速度:  |q_cmd[k] - q_cmd[k-1]| ≤ qdot_max · dt
加速度: |v_cmd[k] - v_cmd[k-1]| ≤ qddot_max · dt    (v_cmd = (q_cmd[k]-q_cmd[k-1])/dt)
```

**为什么只有 position+velocity clamp 不够**:假设 IK 输出持续朝一个坏方向缓移,每帧差值都小于速度上限——速度约束全程合法,但加速度无界时,轨迹可以在两个周期内从 +qdot_max 翻转到 -qdot_max,对真机是冲击。加速度层保证命令轨迹本身是 C¹ 平滑的(第一版不强制 jerk 限制)。

**为什么 dt 必须用实测值**(v2 §6 的正确坚持):仿真里 `ArmCommandFilter` 用 `physics_dt=0.002s`;真机 sender 的实际周期受调度影响在 10ms 上下波动。若限幅器以为 dt=10ms 而实际两帧间隔 14ms,速度上限被放大约 40%。所以限幅器用 `time.monotonic_ns()` 差分出**本帧实际 dt** 参与计算。

### A3.3 IK 失败处理:分类策略与连续性门(★ v3.1 修订)

实测事实(2026-08-27 仿真验证):HOLD(冻结在最后目标)在间歇性不可达下产生明显的走走停停——**二值的 tracking↔frozen 切换本身就是卡顿源**。真机失败处理按三类分治,不再一刀切 REJECT+HOLD:

| 失败类型 | 处置 | 理由 |
|---|---|---|
| 位置不可达(手伸出工作空间) | **不 HOLD**:best-effort DLS 继续解,贴工作空间边界跟;IK 的 position error 只作监控指标上报,不作停止条件 | 边界滑移是连续运动;冻结-恢复才是跳变源 |
| tracking 丢失 / 坏帧 | HOLD(正确——没有信息);恢复后的目标跳变由 §A3.2 限幅器自动变成有界斜坡(qdot/qddot 层天然是 slew-rate limiter) | 这是 HOLD 该管的事 |
| 工作空间内不收敛(极罕见) | 用实际反馈 q 重播种重试一次 → 仍失败才 HOLD + 计数告警 | 真异常才停 |

**wrist-roll-ik 是真机首推 IK 模式**,它从结构上消灭了大部分失败震荡:关节 1-5 只解 position-only DLS(runner:1957-1986,`best_effort=True`,无二值失败语义,超界时自然贴边界);关节 6 是 anchor+滤波 roll 直映射(runner:2040+,τ 滤波 + ±MAX clip + 关节限位,无 IK 无 branch;且 joint 6 位置 Jacobian 列为零,roll 覆写不扰动位置解)。唯一内在奇异(180° 纯 swing)温和保持标定腕角(runner:166-170)。真机上需重验的是 `WRIST_ROLL_SIGN` 方向/比例(同 identity basis,逐轴实机确认)。

full-pose 模式保留给 sim 对照与后续真机实验——orientation 不可达的间歇失败是它特有的;其 fallback(退化 position-only 继续跟)语义正确,真机上保留,只需保证退化输出经过连续性门与限幅器。

连续性门降级为**毛刺兜底**(position-DLS 从上一解出发本身连续):

```text
单帧 |q_candidate - q_previous_goal| 最大分量 > 0.2 rad(对齐 IK_ACCEPT_JUMP_LOG_RAD)
  → 丢弃该帧,不重置链路,下一帧正常
连续 3 帧被拦 → HOLD + 以实际反馈 q 重播种     # 区分"毛刺帧"与"真跳变"
```

(原先"real 模式禁用 best-effort fallback 改 HOLD"的决策废止,以本节分类策略为准。)

### A3.4 LatestTargetBuffer:为什么带 timestamp+seq

遥操作只关心"最新目标"。sender(100–200 Hz)与 IK(60 Hz)速率不同步,sender 大多数周期没有新目标,此时**禁止重发旧目标假装是新目标**——若 AVP 断流、IK 停止产出,一个只存值的 buffer 会让 sender 永远重发最后一帧。所以:

```python
@dataclass
class ArmTarget:
    q_rad: np.ndarray
    source_timestamp_ns: int   # 产出时刻(sender 据此判过期)
    sequence: int              # 单调递增(sender 据此识别"已发过")
```

sender 每跳:若 `seq` 没变 → 不发(或按 HOLD 语义处理);若 age 超阈值 → 触发 watchdog(§A3.5)。

### A3.5 停止序列:竞态分析与正确顺序(★ v3 最重要修订)

**v2 的错误**:三处都写 `rm_set_arm_slow_stop() → sender disable → DISARM`。

**竞态**:slow-stop 是毫秒级 TCP 调用;独立的 watchdog 线程发出它时,100–200 Hz 的 sender 几乎必然还有至少一条 canfd 在途或紧随其后。而 canfd 的语义是"若指令正确,机械臂立即执行"——**慢停之后到达的透传指令行为未知**(可能被忽略,也可能重新驱动关节)。安全路径不能建立在未知行为上。

**正确顺序**(四个阶段,不可调换):

```text
(1) 原子禁用 sender
    —— sender 每跳检查的使能 flag 与 watchdog 写 flag 共用同一把锁;
       或直接把 watchdog 检查放进 sender 线程(停发的线程就是发送的线程,天然无竞态)
(2) 排空在途发送(join / 等待当前 rm_movej_canfd 调用返回)
(3) 调用 rm_set_arm_slow_stop();返回非 0 → 重试一次;仍失败 → 升级 rm_set_arm_stop()(立即停)
(4) 清空 LatestTargetBuffer → DISARM → 打印原因
```

同样的顺序适用于:tracking stale、feedback stale、follow error 超限、controller fault、手动空格/Ctrl-C。

**恢复一律手工**:重读实际 q → 重置限幅器/IK 种子 → 重新按 C → 重新 A。**禁止网络恢复后自动回到 RUNNING**。

### A3.6 "停发 ≠ 停机"

不要假设"sender 停止发送,机械臂自己会停"。透传模式下控制器对最后一条指令的行为(保持?缓停?)未经文档确认,必须显式调用慢停(§A3.5 第 3 步),并在 Phase 4 第 3 级用实验确认实际行为(§B4.3)。

### A3.7 奇异点

`RM65IKSolver` 每解都算 `sigma_min` / `condition_number`(rm65_ik.py:466-485,`--verbose` 可见)。near-singular 时最近解切换和 qdot 需求都趋于激烈,而 canfd 路径没有 avoid-singularity 开关(`rm_set_movev_canfd_init` 的避奇异只作用于速度透传)。处置:σ_min 低于下限(初值 0.05,实测校准)→ gate 拒绝 + HOLD + 计数上报。

---

## A4. 状态机原理

### A4.1 为什么标定(C)与使能(A)必须分离

- **C 是几何操作**:建立 AVP wrist 与 TCP 的相对 anchor,不涉及能量,重复执行无害;
- **ARM 是权限状态**:从此 sender 允许把目标送进真机。一次按键同时完成两件事,意味着"我想重新对零"这个高频、安全的操作,每次都附赠一次真实的运动授权——错键即动。

现有 sim 语义是"按 C 即 TELEOP_ACTIVE"(teleop_arming.py:79-85),real 模式改为两段,这是与 sim 的**行为分叉**,runner 需新键位。

### A4.2 状态机与键位

```text
DISCONNECTED
    ↓ connect + 读实际 q 校验
CONNECTED_SAFE ──(初始: twin/IK种子/限幅器 全部由实际 q 初始化)
    ↓ 按 C(实际 q → 软件 FK → wrist_mapper.calibrate)
CALIBRATED
    ↓ 按 A
ARMED ──(收到首个有效 tracking 目标)──▶ RUNNING
    ↑ 空格 = pause(rm_set_arm_pause)/ resume(rm_set_arm_continue)
    ↓ 按 A / 任一 watchdog
DISARMED → (手工恢复流程 §A3.5)
```

| 键 | 语义 | 合法状态 |
|---|---|---|
| `C` | 仅标定/重 anchor。**RUNNING 中按 C:重 anchor,不 disarm**(与 README §4.4 语义一致) | CONNECTED_SAFE 之后任意 |
| `A` | ARM / DISARM 切换 | CALIBRATED 之后 |
| `空格` | pause / resume | RUNNING |
| `Ctrl-C` | 停止序列(§A3.5)→ 断开 | 任意 |

首跑选项 `--arm-hold-to-run`:按住 A 才维持 ARMED,松开即 DISARM(deadman 语义,建议第一次 AVP 联调时启用)。

实现上**扩展而非重写** `TeleopArmingController`(teleop_arming.py):在 `CALIBRATION_REQUESTED→TELEOP_ACTIVE` 之间插入 `ARM_PENDING` 态;`disarm_for_tracking_timeout` 的"超时后不可自动重 arm"语义(teleop_arming.py:87-98)原样保留。

### A4.3 ARMED→RUNNING 的首目标门

ARM 瞬间的第一个目标若远离当前实际 q(比如用户手已移动),会造成起步跳变。首个目标必须通过接近门:与实际 q 的距离 < 0.15 rad,否则丢弃并等待下一帧(用户会被提示"回到 anchor 附近")。

---

## A5. 反馈与 Digital Twin 原理

### A5.1 两条反馈通道与 ArmState 数据来源

| 通道 | API | 周期 | 数据 | 单位 | 备注 |
|---|---|---|---|---|---|
| TCP polling | `rm_get_current_arm_state()`(interface.py:1150) | ~50 Hz | joint、pose(x,y,z,rx,ry,rz)、err | ° / m / ° | **结构里没有关节速度**(wrap.py:1853-1873,`to_dictionary` 返回扁平 `{"joint", "pose":[6], "err"}`) |
| UDP push | `rm_set_realtime_push` + `rm_realtime_arm_state_call_back`(interface.py:5492/5528) | 5ms 倍数(cycle 字段) | `rm_realtime_arm_joint_state_t`(wrap.py:3385):joint_status(**含 joint_speed**)、waypoint、err、arm_current_status、handState | joint_position °;joint_speed **0.01°/s** | 需 `RM_TRIPLE_MODE_E` + 正确 ip/port 配置 + **防火墙放行** |

```python
@dataclass
class ArmState:
    q_rad: np.ndarray                  # 两条通道都可得
    qd_rad_s: np.ndarray | None        # 仅 UDP push(TCP 结构无此字段,必须允许 None)
    received_monotonic_ns: int
    sequence: int
    controller_error: int | None
    arm_status: int | None             # 仅 UDP push 的 arm_current_status
    valid: bool
```

### A5.2 分级采用策略(为什么第一版不强制 UDP)

UDP push 引入三个新失败面:三线程模式、上报目标 IP/端口配置(`rm_realtime_push_config_t`:cycle 5ms 倍数、port 默认 8080、**ip 默认是 192.168.1.18,必须改成 PC 有线网卡 IP**,wrap.py:1181-1207)、PC 防火墙。给首跑的安全监控路径叠加失败面不划算,而 low-follow + PROFILE_10 下 50 Hz polling 完全够用:

```text
PROFILE_10 + low:   TCP polling 50 Hz 反馈(qd=None)
filter/high 之前:   切换 UDP push(cycle=1→5ms,勾选 joint_speed + arm_current_status),
                   并实测:push 到达率、周期抖动、防火墙放行
```

高跟随对反馈延迟敏感,那才是 UDP 的用武之地(5ms 周期)。

### A5.3 Kinematic Twin 原理

**为什么不跑 arm actuator 积分**:真机模式下 arm 的真值是反馈,不是 MuJoCo 动力学。若保留 actuator 积分又周期性覆写 qpos,contact solver 与 actuator 每次覆写都看到跳变(v2 §17 指出的风险是对的)。

**flange-only 阶段(Phase 2-4):单个 kinematic MjData 足够**

```text
每个反馈帧:
  data.qpos[arm] = q_feedback
  data.qvel[arm] = (q_fb[k] - q_fb[k-1]) / dt_fb        # 差分速度,给 contact/约束一致性用
  data.qpos[hand] = 手部 retarget 目标(照旧)
  不 mj_step;仅 mj_forward
  → viewer / state_recorder / streamer.update_sim 照用
```

**SG100 装上之后(需要手部接触物理时)才拆双 MjData**:一个 kinematic(如上,供可视/记录),一个 physics(arm 关节每步用 qpos+qvel 覆写后被 mj_step 积分手与物体)。分界写死:flange-only 不引入第二个 MjData。

### A5.4 网络:为什么命令路径必须有线

§A2.4 的高跟随要求周期 ≤10ms 且稳定。WLAN 的延迟抖动(几十 ms 级)会直接摧毁它,而 AVP 的 gRPC/aiortc 流量恰恰是大带宽 UDP。所以:

```text
有线 NIC(如 enp3s0):192.168.1.0/24,只承载 RM65 控制器(TCP 8080 + UDP push)
无线 NIC(如 wlp2s0):承载 AVP 流量
ip route 确认 192.168.1.0/24 走有线、AVP 网段走无线(命令路径零 WLAN 跳)
```

---

## A6. 坐标系、标定与单位原理

### A6.1 "基座系一致" ≠ "桌面一致"

已验证的只是**机器人运动学**:MuJoCo world ≡ 官方 base frame。**不**代表仿真里的桌面位置等于真实工位桌面。真机安装后必须实测 `T_base_table`(教程见 §B0.4),它决定:软件 workspace guard 的边界、电子围栏的 box、AVP 视觉场景里桌面的摆放置信度。

### A6.2 anchor 用实际 q + 软件 FK

按 C 时:**真机反馈关节角 → 项目自己的 FK(`RM65IKSolver.fk`)→ TCP pose → `wrist_mapper.calibrate()`**。不读控制器回报的 tool pose,理由:(1) sim/real 共享同一 TCP 定义与同一套运动学,anchor 语义完全一致;(2) 不依赖控制器 tool frame 配置是否正确——配置错误会在标定环节就暴露,而不是运动中。

### A6.3 tool profile

```text
flange_only    第一阶段:TCP=法兰中心,安全包络只含臂
sg100_attached 装手后:TCP=sg100_base(与仿真一致),包络、围栏 margin、tool length 全部重定
```

装手后**不得**沿用 flange_only 的任何安全参数(§B4 第 7 级会重新验证)。

### A6.4 单位桥:rad 内部 / deg 边界,转换只许一处

项目内部全 rad;SDK 边界全 deg(关节角、pose 欧拉角),UDP 的 joint_speed 还有 0.01°/s 缩放。漏乘或重乘 57.3 就是灾难。约束:**转换代码只存在于 `RM65RealBackend` 一处**,并以 +1° 单位审计(§B4.5)作为 Go/No-Go 硬项。

---

# 第二部分 教程篇

## B0. Phase 0 — 硬件/网络教程

### B0.1 网络配置(具体命令)

```bash
# 1. 找有线网卡名
ip addr                          # 假设为 enp3s0;无线为 wlp2s0

# 2. 配同网段(控制器默认静态 192.168.1.18)
sudo ip addr add 192.168.1.100/24 dev enp3s0
sudo ip link set enp3s0 up

# 3. 连通性
ping -c 4 192.168.1.18

# 4. 路由隔离检查:192.168.1.0/24 必须走有线;AVP 网段走无线
ip route
```

持久化用 netplan/nmcli 自行固化。若网卡名不确定,`ip link` 先确认。

### B0.2 物理检查表

- [ ] 底座刚性固定(台钳/打孔),摇不动
- [ ] 急停按钮在操作者手边,**演练**:按下→确认停→释放→复位→确认可动
- [ ] 真机 1 m 范围清空人员/易损物
- [ ] 第一阶段 flange_only:SG100 不装
- [ ] 线缆拖链不影响全工作空间运动

### B0.3 断电行为确认(上电前查规格书,上电后验证)

- [ ] 查 RealMan RM65-B 手册:急停/断电时关节是否带抱闸(free or lock)
- [ ] 记录结论写进本文 §C4;若断电关节自由,规定断电前必须先回到安全姿态

### B0.4 T_base_table 测量教程

方法(示教法,约 15 分钟):

1. 控制器上电,使能拖动示教;
2. 手动把法兰中心(或装尖笔尖工装)依次点到真实桌面的**四个角 + 前缘中点**,每点静止时用冒烟脚本读一次位姿:

```bash
conda activate mujoco-learn
python scripts/rm65_smoke.py --ip 192.168.1.18 state    # 每个点记录 [STATE] TCP position 行
```

3. 再点桌面上表面任意 3 点取 z 高度;
4. 填入 `sg100_teleop/config/T_base_table.yaml`(新文件):四角+前缘 xy、桌面 z、5 cm 安全 margin;
5. 该 YAML 供软件 workspace guard(§A3.1 第 4 层)与电子围栏(第 5 层)生成用。

**退出条件**:ping 通;急停演练 PASS;T_base_table.yaml 生成;断电行为记录。

## B1. Phase 1 — SDK 冒烟教程

环境:**必须** `conda activate mujoco-learn`(全机唯一装有 `Robotic_Arm` 的 env;base 3.13 没有)。

### B1.1 只读检查

```bash
python scripts/rm65_smoke.py --ip 192.168.1.18 state
```

逐行判读(**注意:SDK 返回的 pose 是扁平列表 [x,y,z,rx,ry,rz],欧拉角、单位度**——v3 已修脚本解析):

| 输出 | 通过判据 |
|---|---|
| `[CONN] arm_id=<正数>` | 连接成功;`-1` 即失败,查网线/IP |
| `[CONN] arm_dof=6` | 必须为 6,否则停下排查型号 |
| `[STATE] joint (deg)` | 六个有限值,量级正常(±170 内) |
| `[STATE] TCP position (m)` | 三个有限值 |
| `[STATE] TCP euler (deg)` | 三个有限值 |
| `[STATE] controller error code` | 无错误;有错先查手册清错 |
| 无任何 `[FAIL]` | 全程零失败 |

### B1.2 小幅运动(±3°)

```bash
python scripts/rm65_smoke.py --ip 192.168.1.18 wiggle --joint 1 --amplitude 3 --speed 5
```

| 输出 | 通过判据 |
|---|---|
| 两次 `[MOVE]` 返回码 | 都是 `0`(`_STATUS_TEXT` 里 `success`) |
| `following error` | < 1° |
| 结束位姿 | ≈ 起始位姿(肉眼+读数) |

**关节方向确认表**(逐关节做,填表留档;这项进 Go/No-Go):

| joint | 预期(RealMan 定义) | 实测正向 | 勾 |
|---|---|---|---|
| 1 | 底座旋转 | __ | [ ] |
| 2 | 肩 | __ | [ ] |
| 3 | 肘 | __ | [ ] |
| 4 | 腕滚 | __ | [ ] |
| 5 | 腕摆 | __ | [ ] |
| 6 | 法兰滚 | __ | [ ] |

### B1.3 随时可用的网络急停

```bash
python scripts/rm65_smoke.py --ip 192.168.1.18 stop    # 减速停
```

**退出条件**:state 全过;六关节方向表填完;wiggle 判据全过。

## B2. Phase 2 — Backend 实现教程(无硬件可完成)

### B2.1 代码结构

```text
sg100_teleop/robot/
├── __init__.py
├── arm_backend.py             抽象:connect/disconnect/send_joint_target/
│                              joint_state/stop_immediate/stop_smooth/enable
├── arm_state.py               ArmState / ArmTarget dataclass(§A5.1)
├── latest_target_buffer.py    线程安全单槽 buffer(seq+timestamp)
├── ik_continuity_gate.py      §A3.3 四道门
├── joint_trajectory_safety.py q/qdot/qddot 限幅器(实测 dt)
├── safety_monitor.py          watchdog 集合 + §A3.5 停止序列
├── rm65_sim.py                包 write_robot_controls,sim 语义零变化
└── rm65_real.py               RoboticArm 封装:deg 桥(唯一转换点)、
                              sender 线程、反馈线程、慢停/急停
```

### B2.2 无硬件单元验证(逐项写测试,v1 有、v2 丢了,v3 恢复并加细)

```bash
# 1. deg 桥往返:q_rad → 内部 → 度 → 回 rad,断言误差 < 1e-9
# 2. latest-target 语义:两个线程并发写/读,断言 (a) 读到的永远是最旧未发 seq;
#    (b) seq 不变时不重发; (c) 过期目标触发 watchdog 事件
# 3. 限幅器离线轨迹:喂斜坡/阶跃/正弦目标,断言输出序列的
#    max|Δq|/dt ≤ qdot_max、max|Δv|/dt ≤ qddot_max、全程在 soft limit 内
# 4. gate:构造 +20°→-150° 毛刺帧,断言该帧被丢弃且链路不重置;
#    构造 0.05 rad 合法步,断言 PASS;构造连续 3 帧毛刺,断言 HOLD+重播种
# 5. 停止序列:mock backend,断言调用顺序 = disable → drain → slow_stop,
#    slow_stop 返回 1 时断言重试+升级 stop_immediate
python -m sg100_teleop.tests.test_robot_backend_units   # 新测试模块
```

### B2.3 runner 接线(四处)

1. 参数:`--arm-backend {sim,real}`(默认 sim)、`--arm-ip/--arm-port(8080)`、`--arm-follow {low,filter,high}`、`--arm-radio`、`--arm-send-hz(100)`、`--arm-safety-profile {10,25,50}`、`--arm-tool-profile`、`--arm-target-timeout-ms(150)`、`--arm-feedback-timeout-ms(200)`、`--arm-max-follow-error-deg(5)`、`--arm-hold-to-run`、`--arm-verbose`;
2. 启动:real → `rm_get_joint_degree()` 读实际 q → twin/IK 种子/限幅器/gate 全部由实际 q 初始化;**跳过** bottle pre-grasp startup 逻辑;
3. 物理循环:`q_arm_cmd` 照旧算 → real 模式交 backend;sim 模式路径零变化;
4. 按 C:real 模式 anchor = 实际 q → 软件 FK(§A6.2);新增 A 键(§A4.2)。

**退出条件**:单元验证 5 项全绿;`--arm-backend sim` 现有回归全过;real 无硬件时报错清晰退出(不崩溃)。

## B3. Phase 3 — 安全层配置教程

### B3.1 数值初值表(真机上逐项标定后回填)

| 参数 | PROFILE_10 | PROFILE_25 | PROFILE_50 | 说明 |
|---|---:|---:|---:|---|
| qdot_max(rad/s) | 0.5 | 1.25 | 2.5 | 以 RealMan 规格书核对 |
| qddot_max(rad/s²) | 2 | 5 | 10 | |
| TCP 线速度上限(m/s) | 0.05 | 0.125 | 0.25 | TcpTargetFilter 同步调低 |
| joint soft margin(deg) | 2 | 2 | 2 | 距硬限位 |
| target age(ms) | 150 | 150 | 150 | 60Hz 源,容丢 8 帧 |
| feedback age(ms) | 200(50Hz polling) | 200 | 100(UDP) | 随通道切换 |
| follow error(deg) | 5 | 5 | 5 | cmd vs actual |
| σ_min 下限 | 0.05 | 0.05 | 0.05 | 实测校准 |
| 碰撞等级 rm_set_collision_state | 6 | 6 | 6 | 0-8,8 最灵敏(interface.py:585) |
| gate per-solve(rad) | 0.2 | 0.2 | 0.2 | 对齐 IK_ACCEPT_JUMP_LOG_RAD |
| 首目标接近门(rad) | 0.15 | 0.15 | 0.15 | §A4.3 |

### B3.2 控制器侧配置

```python
# 伪代码,连上后一次性配置(Python 侧经由 RM65RealBackend)
arm.rm_set_collision_state(6)                     # 动力学碰撞,interface.py:585
arm.rm_set_collision_detection(1)                 # 静态碰撞开关(语义以手册为准)
fence = rm_fence_config_t(...)                    # 由 T_base_table.yaml 生成 box
arm.rm_add_electronic_fence_config(fence)         # interface.py:5122,whole-arm
```

### B3.3 UDP push 配置(仅 filter/high 前需要,§A5.2)

```python
# 1. 防火墙(一次性):sudo ufw allow from 192.168.1.18 to any port 8081 proto udp
# 2. 配置(cycle 单位 5ms;ip 必须填 PC 有线网卡 IP!)
cfg = rm_realtime_push_config_t(cycle=1, enable=True, port=8081,
                                ip="192.168.1.100",          # PC 有线 IP
                                custom_config=rm_udp_custom_config_t(
                                    joint_speed=1, arm_current_status=1))
arm.rm_set_realtime_push(cfg)                      # interface.py:5492
arm.rm_realtime_arm_state_call_back(callback)      # :5528,载荷 wrap.py:3385
# joint_speed 单位 0.01°/s → rad/s 换算在 deg 桥统一处理
```

验证:收到回调周期 ≈5ms、丢包率 <1%,才允许切 filter 模式。

### B3.4 逐项验证方法

每项安全机制都要**主动触发一次**确认行为:拔 AVP(触发 target watchdog)、停反馈线程(feedback watchdog)、手推关节(follow error)、低速逼近围栏边界(fence 触发-恢复)。观察点:慢停是否按 §A3.5 顺序发生、DISARM 后是否要求完整恢复流程。

**退出条件**:§A3.1 七层每层至少一次主动触发验证;初值表回填实测值。

## B4. Phase 4 — 联调阶梯教程(逐级通过才进下一级)

| 级 | 内容 | 命令/方法 | 通过判据 |
|---|---|---|---|
| 1 | sim 回归 | README §12/§13 全量 | 全 PASS(基准) |
| 2 | SDK 冒烟 | §B1 | 同 §B1 退出条件 |
| 3 | backend 无 AVP | `--arm-backend real` + ±3° 单关节正弦 @100Hz 脚本 | sender 周期五指标达标;twin 跟随;<br>**停发实验**:停正弦源,确认停止序列按 §A3.5 执行、机械臂实际慢停(§A3.6 的实证) |
| 4 | AVP + PROFILE_10 | `--wrist-roll-ik --arm-follow low --arm-safety-profile 10 --arm-hold-to-run`,空载逐轴 | 六轴方向/比例确认(identity basis + `WRIST_ROLL_SIGN` 实机定案);不可达时贴边界滑移**无冻结卡顿**;watchdog 可用 |
| 5 | 安全机制联试 | §B3.4 全项 + fence 逼近 + 切 `--arm-follow filter`(先过 §B3.3) | 每项触发-恢复符合预期 |
| 6 | 提速 | PROFILE_10 → 25 → 50 | 每档重跑 follow-error/周期指标 |
| 7 | 装手 | `--arm-tool-profile sg100_attached`,重测 T_base_table 相关 margin | 围栏/包络/首目标门全部重验证 |

### B4.5 单位审计(Go/No-Go 硬项)

第 3 级中:发单关节 +1° 指令,读反馈应 +1°±0.05°;同时 twin 中该关节显示 +1°。三处(指令/反馈/显示)一致才算单位桥闭合。

### B4.4 sender timing 测量(第 3 级起持续记录)

sender 每跳记录 `send_monotonic_ns`;统计 mean/p95/p99/max period 与 missed deadline(超 10ms 计一次)。数据进 sender 日志(§B5)。

## B5. Phase 5 — 验收测量

记录机制:**扩展** `mujoco_recording.npz` 与 sender 日志,不另造轮子:

```text
每 tracking 帧:AVP timestamp、seq、q_arm_goal、gate 判定
每 sender 跳:  send ts、q_arm_cmd、实测 dt
每反馈帧:      ts、q_actual、(qd)、err、arm_status
事件流:        watchdog 触发、fence/collision、IK REJECT/HOLD 计数
```

指标:端到端 latency(AVP→cmd→反馈到位)、`q_cmd-q_actual` 分布、sender 五指标、watchdog 触发-恢复延迟、IK success/fallback(仅 sim)/reject 率、各 PROFILE 对比。

---

# 第三部分 参考

## C1. Go / No-Go(首次 AVP→真机连续遥操作,全项可溯源)

| # | 项 | 溯源 |
|---|---|---|
| 1 | state 冒烟 PASS | §B1.1 |
| 2 | wiggle ±3° PASS(返回码/跟随误差/归位) | §B1.2 |
| 3 | 六关节方向人工确认表已填 | §B1.2 |
| 4 | T_base_table 已测量并生成 YAML | §B0.4 |
| 5 | 断电行为已确认并记录 | §B0.3 |
| 6 | 实际 q 启动(twin/IK/限幅器/gate 初始化) | §B2.3-2 |
| 7 | 软件 FK anchor PASS | §A6.2 |
| 8 | C 与 A 分离 + 键位生效(+hold-to-run) | §A4.2 |
| 9 | 首目标接近门 PASS | §A4.3 |
| 10 | IK continuity gate PASS(含 §B2.2-4 用例) | §A3.3 |
| 11 | q 软限位 PASS | §A3.2 |
| 12 | qdot 限幅 PASS | §A3.2 |
| 13 | qddot 限幅 PASS | §A3.2 |
| 14 | target watchdog 主动触发 PASS | §B3.4 |
| 15 | feedback watchdog 主动触发 PASS | §B3.4 |
| 16 | 停止序列顺序正确(停发实验实证) | §A3.5/§B4-3 |
| 17 | cmd-feedback 监控 PASS | §B3.4 |
| 18 | 单位审计(+1°)PASS | §B4.5 |
| 19 | 软件 workspace guard PASS(基于实测桌面) | §A3.1-L4 |
| 20 | 控制器 whole-arm fence PASS(candfd 下触发实测) | §B3.2/§B4-5 |
| 21 | 碰撞保护已配置并触发验证 | §B3.2 |
| 22 | sender timing 五指标达标 | §B4.4 |
| 23 | twin 跟随反馈(不含 actuator 积分) | §A5.3 |
| 24 | 物理急停已演练 | §B0.2 |
| 25 | PROFILE_10 已选;1 m 区域清空 | §B4-4 |
| 26 | wrist-roll 通道方向/比例实机确认(`WRIST_ROLL_SIGN`);不可达贴边界滑移无卡顿 | §A3.3/§B4-4 |

任一未过 = NO-GO;回退 `--arm-backend sim` 永远可用。

## C2. SDK 速查(robotic-arm 1.1.6)

| 用途 | API | 位置 |
|---|---|---|
| 连接/断开 | `RoboticArm(RM_SINGLE_MODE_E)` + `rm_create_robot_arm(ip,8080)` / `rm_delete_robot_arm` | :7115 |
| 关节透传 | `rm_movej_canfd(joint_deg, follow, expand, trajectory_mode, radio)` | :1736 |
| 规划式关节运动 | `rm_movej(joint_deg, v_pct, r, connect, block)`(单线程 block=超时秒) | :1460 |
| 停止 | `rm_set_arm_stop` / `rm_set_arm_slow_stop` / `rm_set_arm_pause`+`continue` | :2069-2114 |
| 状态(TCP) | `rm_get_current_arm_state` / `rm_get_joint_degree`(°) | :1150/:1277 |
| UDP push | `rm_set_realtime_push` / `rm_realtime_arm_state_call_back` | :5492/:5528 |
| 动力学碰撞 | `rm_set_collision_state(0-8)` | :585 |
| 静态碰撞开关 | `rm_set_collision_detection` | :1423 |
| 电子围栏 | `rm_add/update/delete_electronic_fence_config` | :5122-5158 |
| home 位 | `rm_set_init_pose` / `rm_get_init_pose` | :1232/:1256 |
| 冒烟脚本 | `scripts/rm65_smoke.py`(state/wiggle/stop) | 本仓库 |

## C3. 与 v1/v2 的关系

- v1(`REAL_RM65_MIGRATION.md`):Phase 0-1 操作细节已全部并入本文 §B0/§B1,文件保留作历史,不再维护;
- v2(`RM65_真机迁移方案_修订版_v2.md`):架构修订已全部吸收(含本文对其停止顺序、参数数值等处的再修订),文件保留作历史,不再维护;
- 后续只改本文。

## C4. 风险与缓解

| 风险 | 缓解 | 溯源 |
|---|---|---|
| canfd 周期抖动 | 有线 NIC 独占命令路径;实测五指标;filter/high 用 125/200Hz | §A2.4/§A5.4 |
| 慢停后 canfd 到达行为未知 | 停止序列先停发;第 3 级停发实验实证 | §A3.5/§A3.6 |
| 旧目标被重发 | ArmTarget seq+timestamp;sender 只发未发过的最新目标 | §A3.4 |
| 反馈断流 | feedback watchdog → 停止序列 | §A3.5 |
| IK branch 跳变 | real 模式禁用 fallback + 四道 gate | §A3.3 |
| 命令跳变/冲击 | q/qdot/qddot 三层(实测 dt) | §A3.2 |
| 轴向标错 | PROFILE_10 + hold-to-run + 逐轴验证 | §B4-4 |
| 控制器/软件 TCP 定义不一致 | anchor=实际 q+软件 FK | §A6.2 |
| 真实桌面≠仿真桌面 | 实测 T_base_table,双层围栏 | §A6.1/§A3.1 |
| 装手后包络变化 | tool profile 强制重验证 | §A6.3 |
| 单位换算错误 | 单一转换点+审计 | §A6.4 |
| 软件整体失效 | whole-arm fence + 碰撞保护 + 物理急停 | §A3.1 |
| 断电行为未知 | §B0.3 事前确认记录 | §B0.3 |

## C5. README 同步事项(Phase 2 完成后执行)

1. 勘误 `README.md:253`:"arm chain 不再叠加 joint-space EMA"与代码不符(runner:1149 `arm_command_filter`,tau 0.03s/3rad/s),按代码改写;
2. 能力状态表更新"实体 RM65 控制"行;
3. §16 维护约束新增两条:
   - real 模式下任何 `q_arm_goal` 不得绕过 continuity gate → latest-target → limiter 直达 `rm_movej_canfd`;
   - rad↔deg 转换只许存在于 `RM65RealBackend`;
4. §17 文件索引加入 `sg100_teleop/robot/`。
