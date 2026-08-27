# RM65 真机迁移方案 v3.2（原理、实施与代码改造依据）

> 本文取代 v1(`REAL_RM65_MIGRATION.md`)与 v2(`RM65_真机迁移方案_修订版_v2.md`),是 RM65-B 真机接入的**唯一实施依据**。
> 范围:实体 RM65-B 控制接入。实体 SG100 驱动不在范围内(装手只影响 tool profile 与安全包络)。
> SDK 事实均以本机安装的 `robotic-arm 1.1.6`(`mujoco-learn` conda env)源码为准,文中引用 `interface.py` = `Robotic_Arm/rm_robot_interface.py`,`wrap.py` = `Robotic_Arm/rm_ctypes_wrap.py`。
>
> 写作时间:2026-08-27。与实际不符时,以源码为准并回改本文。

## v3.2 相对 v3.1 的修订记录

1. **修正 LatestTargetBuffer / sender 的核心时间语义**：buffer 是“最新目标单槽”，不是消息队列；sender 在 target 仍 fresh 时每个固定周期都要读取同一个 latest target，经 limiter 生成并发送本周期 `q_arm_cmd`。`seq` 只用于识别上游是否更新、日志和诊断，**不能用 `seq` 不变作为“本周期不发送”的条件**（§A3.4）。
2. **明确 sim / real 在 `q_arm_goal` 后分叉**：sim 保留现有 `ArmCommandFilter @500 Hz → data.ctrl`；real **不得**先经过 MuJoCo 500 Hz `ArmCommandFilter`，而是直接进入 `IKContinuityGate → LatestTargetBuffer → Real JointTrajectorySafetyLimiter @ sender clock`（§A1.3、§B2.3）。
3. **统一 v3.1 的 IK fallback 最终决策**：位置不可达继续使用 best-effort position DLS 贴工作空间边界滑移；full-pose 允许退化到 position-only；continuity gate 仅作为毛刺兜底。删除旧的“real 禁用 fallback”残留表述（§A3.3、§C4）。
4. **停止序列增加 bounded drain**：原子禁用 sender 后，只允许在配置的 `sender_drain_timeout_ms` 内等待当前 CANFD 调用结束；禁止无限 `join` 卡死安全路径。超时进入明确升级路径并记录 fault（§A3.5）。
5. **补齐 RUNNING 中重新按 C 的原子 re-anchor 语义**：先 inhibit sender / drain / 清 stale target，再以实际反馈 q 重置 TCP filter、IK seed、real limiter 和 mapper anchor，等待新的 post-C tracking frame 后恢复（§A4.2、§A4.4）。
6. **补齐 pause/resume 语义**：pause 不允许 sender 继续积累或追逐新 target；恢复时以实际 q 重置 limiter/IK 并只接受 fresh post-resume target。首跑可选择 pause 直接降级到 DISARMED（§A4.2、§A4.4）。
7. **首目标接近门定义为 per-joint max norm**：`max_i |q_target[i]-q_actual[i]| < threshold`，不再使用含糊的“距离 < 0.15 rad”（§A4.3）。
8. **tool profile 落到明确的固定变换合同**：`T_base_tcp = T_base_flange · T_flange_tcp(profile)`；`flange_only` 与 `sg100_attached` 必须分别有 FK/IK 回归测试，不能只改安全 margin（§A6.3）。
9. **T_base_table 测量统一 canonical FK 语义**：优先记录实际 q，再用软件 FK + 已知 probe tip transform 得到测量点；不再默认相信 controller 当前 tool pose（§B0.4）。
10. **qdot/qddot 从“统一标量”升级为 per-joint 向量合同**：B3 表中的标量仅作为首轮 smoke 的临时上限；production 必须以 RM65 各关节规格/实测值形成 6D limits，并由 PROFILE_10/25/50 缩放（§B3.1）。
11. **follow-error 采用 soft-persistent + hard 两级语义**，避免单帧反馈抖动误停，也避免长期接近阈值的偏差长期不处理（§B3.1、§B3.4）。
12. **修正 limiter 的 dt 解释**：使用 sender 实际 monotonic dt；短周期必须收紧 Δq，长周期不能因为 dt 变大而授权一次大跳步——超过 sender period 容忍范围应视为 timing fault / stop，而不是用超长 dt 放宽限幅（§A3.2）。
13. **SDK thread mode 速查分级**：low + TCP polling 可以使用 single mode；启用 UDP realtime push 时必须使用 `RM_TRIPLE_MODE_E`（§A5.2、§C2）。
14. **删除“手推 active 机械臂触发 follow-error”的标准测试**：改为 mock / fault injection + 极低速、受控的 cmd/feedback 偏差测试（§B3.4）。
15. **补充 flange-only digital twin 的显示免责声明**：若物理 SG100 尚未安装而 AVP 仍显示 SG100，则该手仅是 development visualization，不得作为真实碰撞包络依据（§A5.3）。
16. **新增 Agent 实施合同**：明确改代码顺序、必须新增的测试以及不得改变的 sim 行为，供 VSCode Agent 直接按阶段实施（§C6）。

### v3 / v3.1 已吸收的历史修订

1. 修复停止顺序竞态：从“先 slow-stop 再停 sender”改为“先停 sender → drain → slow-stop”。
2. 给出 qdot/qddot/watchdog/follow-error/σ_min 等安全参数初值。
3. 回填网络、冒烟、无硬件单测、Go/No-Go 等实施细节。
4. 首跑允许 PROFILE_10 + low-follow + 50 Hz TCP polling；filter/high 前切 UDP push。
5. IK continuity gate 与现有常量对齐，并在 v3.1 中进一步改为失败分类处理。
6. C=标定、A=ARM、空格=pause；real 模式与 sim 的 arming 行为分叉。
7. digital twin 区分 flange-only kinematic twin 与后续 SG100/contact physics。
8. 记录并纠正现有 sim 中真实存在的 `ArmCommandFilter`。
9. 增加奇异点、断电、单位审计、T_base_table 与 Go/No-Go。
10. v3.1 仿真实测确认：不可达时 HOLD 会卡顿；wrist-roll-ik 作为真机首推 IK 模式，位置不可达用 best-effort DLS 贴边界滑移，tracking 丢失才 HOLD。

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
| sim 命令链已有低通+限速 | `ArmCommandFilter` | **只保留给 sim**；real 必须在 `q_arm_goal` 后独立分叉，使用 sender 实际时钟的真机 limiter(§A1.3/§A3.2) |

**迁移 = 在 `write_robot_controls()` 这一个分叉点接入真机后端 + 补真机必备的安全层。** SDK 调用不得散落到 mapper/filter/IK/arming/recorder/streaming 里,全部收敛在 `sg100_teleop/robot/` 子包。

### A1.3 目标架构(全文唯一架构图)

```text
AVP tracking (~60 Hz)
  → TrackingFrameGate → WristRelativeMapper → TcpTargetFilter → 官方 IK
  → q_arm_goal
      │
      ├─ SIM path（现有语义原样保留）
      │    → ArmCommandFilter @ MuJoCo 500 Hz
      │    → q_arm_cmd_sim
      │    → write_robot_controls() → data.ctrl → MuJoCo physics
      │
      └─ REAL path（在 q_arm_goal 处独立分叉）
           → IKContinuityGate @~60 Hz
           → LatestTargetBuffer(target + source_timestamp + seq)
           → JointTrajectorySafetyLimiter @ sender 实测周期
               ├ joint position soft limits
               ├ per-joint velocity limits
               └ per-joint acceleration limits
           → FixedPeriodSender (100/125/200 Hz)
               ├ 每 tick 读取“最新且 fresh”的 target
               ├ 即使 source seq 未变化，也继续产生/发送本周期 q_arm_cmd_real
               └ target stale / timing fault → SafetyMonitor
           → rm_movej_canfd() ──TCP──▶ RM65 控制器
                                      │
RM65 反馈(TCP polling 50 Hz 或 UDP push 5 ms,分级采用 §A5.2)
  → SafetyMonitor
      ├ target-age watchdog       ├ feedback-age watchdog
      ├ soft/hard follow error    ├ controller error state
      └ fence/collision state
  → [任一安全触发]
      原子 inhibit sender
      → bounded drain
      → rm_set_arm_slow_stop()
      → clear target
      → DISARM
  → Digital Twin(MjData, kinematic, §A5.3) → USDZ → AVP
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

**为什么 dt 必须来自真实 sender 时钟**：仿真 `ArmCommandFilter` 的 `physics_dt=0.002s` 只属于 sim；真机 sender 的周期受线程调度与网络调用影响。limiter 必须用 `time.monotonic_ns()` 得到本次 sender tick 与上一次实际 send tick 的 elapsed dt。若实际周期**短于**假定周期，而仍按较大的固定 dt 计算允许步长，实际速度会被放大；因此短周期必须按真实 dt 收紧 `Δq`。反过来，如果 sender 卡顿造成异常长 dt，也**不能**借由 `qdot_max · dt` 一次性授权一个大跳步：当 `dt` 超过配置的 sender-period 容忍上限时，直接记为 timing fault 并进入停止序列，而不是扩大允许步长。

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

（v3.2 最终决策：best-effort fallback 按本节分类策略保留；continuity gate 只负责数值连续性/毛刺兜底。）

### A3.4 LatestTargetBuffer：latest-target 不是消息队列

遥操作只关心**当前最新目标**，不是“每个上游目标都必须被消费一次”。IK/mapper 约 60 Hz，而 real sender 是 100/125/200 Hz，所以一个上游 target 正常情况下会被多个 sender tick 重采样。

```python
@dataclass
class ArmTarget:
    q_rad: np.ndarray
    source_timestamp_ns: int   # 上游产出时刻；用于 freshness watchdog
    sequence: int              # 上游单调递增版本号；用于诊断/日志/检测是否更新
```

`LatestTargetBuffer` 是**线程安全单槽覆盖语义**：

```text
producer 写 seq=101
producer 很快又写 seq=102
sender 下一次读取时直接拿 seq=102
seq=101 不排队、不补发
```

sender 每个固定周期的正确逻辑：

```text
tick
  ↓
atomic snapshot(latest target)
  ↓
没有 target?
  → HOLD current / 等待，不发未经初始化的目标

target age > timeout?
  → SafetyMonitor → 停止序列

target fresh?
  → 把 latest q_rad 作为“当前追踪目标”
  → JointTrajectorySafetyLimiter 用本周期实际 dt
     从上一个 q_cmd_real 向 latest q_rad 连续推进
  → rm_movej_canfd(q_cmd_real)
```

**关键合同**：

- `seq` 没变只代表“60 Hz 上游本周期没有产生新目标”，**不代表 sender 本周期不发送**；
- 高跟随下 sender 必须维持 125/200 Hz 固定发送节奏，只要 target 仍 fresh，即使连续多个 sender tick 引用同一个 source `seq`；
- 如果 limiter 已经追到 target，也允许继续发送相同/近似相同的 `q_cmd_real` 以维持固定透传节奏；
- 真正阻止“无限重发陈旧目标”的机制是 `source_timestamp_ns` 的 **target-age watchdog**，不是 `seq` 去重；
- buffer 永远只保留最新值，不允许形成 FIFO backlog，否则遥操作延迟会累积。

### A3.5 停止序列：竞态、bounded drain 与正确顺序

v2 的核心竞态判断仍成立：如果先 slow-stop、后停 sender，100–200 Hz sender 可能在 stop 之后又送入一条 CANFD 指令。v3.2 保留“先停 sender”的原则，但补上 **drain 不能无限等待**。

```text
(1) 原子 inhibit sender
    —— sender 每 tick 检查的 enabled flag 与 watchdog 使用同一同步机制；
       一旦置 false，禁止开始新的 rm_movej_canfd 调用。

(2) bounded drain 当前 in-flight send
    —— 只等待已经进入 SDK 的那一次调用结束；
       最大等待 sender_drain_timeout_ms；
       禁止无限 join / 无限等 socket 返回。

(3) drain 正常结束:
      rm_set_arm_slow_stop()
      返回非 0 → 重试一次
      仍失败 → 升级 rm_set_arm_stop()

    drain 超时:
      记录 SENDER_DRAIN_TIMEOUT fault
      sender 保持 disabled
      尝试独立安全控制路径发送 stop
      若 SDK/连接状态不允许可靠软件 stop → 明确提示物理急停
      软件保持 DISARMED/FAULTED

(4) clear LatestTargetBuffer
    → 记录 stop reason / timestamps
    → DISARM
```

`sender_drain_timeout_ms` 必须可配置并有单元测试。其 production 数值在真机 Phase 3 根据 SDK 正常调用耗时分布标定；**安全路径不能依赖无限阻塞等待**。

上述停止序列用于：tracking stale、feedback stale、hard/persistent follow error、controller fault、电子围栏/碰撞安全事件、A 主动 DISARM、Ctrl-C。`C` 重标定和 `Space` pause 使用 §A4.4 的专用原子过渡，不直接等价于 watchdog fault。

**fault 后恢复一律手工**：重读 actual q → reset limiter / IK seed → clear target → 重新按 C → 重新 A。禁止网络恢复后自动回到 RUNNING。

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
    ↓ connect + 读 actual q 校验
CONNECTED_SAFE
    └─ twin / IK seed / real limiter 都由 actual q 初始化
    ↓ 按 C
CALIBRATED
    ↓ 按 A
ARMED
    ↓ 首个 post-arm fresh target 通过接近门
RUNNING
    ├─ 按 C → REANCHORING → RUNNING（§A4.4）
    ├─ 空格 → PAUSED → 空格 → ARMED/RUNNING（§A4.4）
    ├─ 按 A → 正常 DISARM
    └─ 任一 safety fault → §A3.5 → DISARMED/FAULTED
```

| 键 | 语义 | 合法状态 |
|---|---|---|
| `C` | 标定/重 anchor。RUNNING 中必须走 §A4.4 原子 re-anchor，不能在 sender 继续工作时直接修改 mapper anchor | CONNECTED_SAFE 之后 |
| `A` | ARM / 正常 DISARM；DISARM 仍先 inhibit sender + bounded drain，再 clear target | CALIBRATED 之后 |
| `空格` | pause / resume；pause 时 sender 不得继续追踪新目标，resume 必须以 actual q reset limiter/IK 并等待 fresh target | RUNNING / PAUSED |
| `Ctrl-C` | §A3.5 停止序列 → disconnect | 任意 |

首跑选项 `--arm-hold-to-run`：按住 A 才维持允许运动，松开立即执行正常 DISARM；建议第一次 AVP 联调强制启用。

实现上**扩展而非重写** `TeleopArmingController`：sim 的“按 C 即 TELEOP_ACTIVE”行为保持原样；real-only 增加 `ARM_PENDING/ARMED/PAUSED/FAULTED` 控制层。tracking timeout 后不可自动 re-arm 的现有语义继续保留。

### A4.3 ARMED→RUNNING 的首目标门

ARM 瞬间第一个 target 若远离 actual q，会造成起步跳变。首目标门明确采用 **per-joint max norm**：

```text
max_i |q_target[i] - q_actual[i]| < first_target_gate_rad
```

初值：

```text
first_target_gate_rad = 0.15 rad
```

不采用含糊的 6D L2 “距离 < 0.15 rad”。未通过时丢弃 target，不写入 LatestTargetBuffer，并提示用户把 AVP 手腕回到 anchor 附近；只有新的 fresh target 通过后才进入 RUNNING。

### A4.4 RUNNING 中按 C 与 pause/resume 的原子过渡

**RUNNING 中按 C（re-anchor）**：

```text
RUNNING
→ inhibit sender
→ bounded drain
→ clear LatestTargetBuffer
→ read actual q
→ actual q → software FK 得 canonical TCP
→ TcpTargetFilter.reset(actual TCP)
→ IK seed.reset(actual q)
→ JointTrajectorySafetyLimiter.reset(actual q, actual qdot if valid else 0)
→ wrist_mapper.calibrate(...)
→ calibration_epoch += 1
→ 只接受 re-anchor 完成后的 fresh tracking/target
→ 首目标接近门 PASS
→ re-enable sender
→ RUNNING
```

旧 anchor 下生成的 target 绝不允许跨 epoch 进入 sender。推荐 `ArmTarget` 增加 `calibration_epoch`；若不增加字段，至少要求 clear buffer 后 target 的 source timestamp 晚于 re-anchor 完成时刻。

**Pause**：

```text
RUNNING
→ inhibit sender
→ bounded drain
→ clear target
→ rm_set_arm_pause()
→ PAUSED

PAUSED
→ rm_set_arm_continue()
→ read actual q
→ reset limiter / IK seed / TcpTargetFilter
→ 等待 post-resume fresh target
→ 首目标接近门 PASS
→ re-enable sender
```

第一次真机联调可采用更保守策略：`Space` 直接进入 DISARMED，恢复必须重新 C + A；待 pause/resume 路径实测稳定后再开启无 DISARM 的 PAUSED 状态。

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

**SG100 装上之后（需要手部接触物理时）才拆双 MjData**：一个 kinematic（供可视/记录），一个 physics（arm 关节每步用 qpos+qvel 覆写后再 `mj_step` 积分手与物体）。分界写死：flange-only 不引入第二个 MjData。

**显示免责声明**：flange-only 阶段如果真实 RM65 法兰上尚未安装 SG100，但 AVP 为开发仍加载 RM65+SG100 USDZ，则画面中的 SG100 只是 **development visualization**，不是实体 digital twin。workspace guard、fence 与碰撞包络必须按 `flange_only` 的真实物理配置计算。

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

### A6.3 tool profile：TCP 变换必须成为代码合同

tool profile 不能只是一组 safety margin；必须明确软件 canonical TCP 相对法兰的固定变换：

```text
T_base_tcp = T_base_flange · T_flange_tcp(profile)
```

建议配置：

```yaml
flange_only:
  T_flange_tcp: identity
  workspace_margin: ...
  collision_envelope: ...
  fence_margin: ...

sg100_attached:
  T_flange_tcp: <实测/已验证的 flange→sg100_base 固定 SE(3)>
  workspace_margin: ...
  collision_envelope: ...
  fence_margin: ...
```

语义：

```text
flange_only:
  TCP = 法兰中心
  第一阶段真实硬件没有 SG100

sg100_attached:
  TCP = sg100_base / 项目 canonical hand TCP
  与当前 MuJoCo/IK 定义保持一致
```

FK/IK 必须通过同一个 profile 使用 `T_flange_tcp`，不能只在 anchor 或 fence 单独补偿。至少新增两个回归：

1. `flange_only`：software TCP 与 flange pose 一致；
2. `sg100_attached`：software TCP 与当前 MuJoCo `sg100_base_link` TCP 一致。

装手后不得沿用 flange_only 的安全参数，必须重新验证 workspace guard、fence、首目标门和方向/单位。

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

方法（示教法，约 15 分钟）：

1. 控制器上电，使能拖动示教；
2. 明确测量 tool/profile：
   - 直接用法兰中心触点：`T_flange_probe = identity`；
   - 使用尖笔/探针：先测量并记录固定 `T_flange_probe`；
3. 手动把 probe tip 依次点到真实桌面的**四个角 + 前缘中点**。每点静止时优先记录 actual joint q，而不是把 controller 当前 tool pose 当 canonical 真值：

```bash
conda activate mujoco-learn
python scripts/rm65_smoke.py --ip 192.168.1.18 state
# 每点至少保存 [STATE] joint (deg)
```

4. 离线用项目 software FK 得 `T_base_flange(q_actual)`，再计算：

```text
T_base_probe = T_base_flange(q_actual) · T_flange_probe
```

取 probe tip 的 base-frame position 作为测量点；
5. 再点桌面上表面任意 3 个分散点，用同样方法拟合/核对桌面平面；
6. 写入 `sg100_teleop/config/T_base_table.yaml`：四角/前缘、桌面平面、初始 safety margin、测量 profile 与 `T_flange_probe`；
7. YAML 供 software workspace guard 与 controller electronic fence 生成。

若临时直接使用 SDK 回报 TCP pose，必须在测量前明确 controller tool frame 与 probe 完全一致并记录该前提；production 推荐 actual q + software FK。

**退出条件**：ping 通；急停演练 PASS；`T_base_table.yaml` 生成；测量 tool/profile 可追溯；断电行为记录。

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

### B2.2 无硬件单元验证（必须先于真机）

```bash
# 1. deg bridge：rad→deg→mock SDK→deg feedback→rad；误差 <1e-9；
#    另测 UDP joint_speed 0.01°/s 缩放。
#
# 2. LatestTargetBuffer：
#    - seq=101/102 连续写，consumer 只读 latest=102，无 FIFO backlog；
#    - source seq 多个 sender tick 不变但仍 fresh，sender 仍每 tick limiter+send；
#    - stale age 触发 watchdog；clear 后旧 epoch/旧 timestamp 不得重现。
#
# 3. limiter/sender dt：
#    - 斜坡/阶跃/正弦/jittered dt 满足 q/qdot/qddot；
#    - short dt 会收紧 Δq；abnormal long dt 触发 timing fault，不允许大步。
#
# 4. IK continuity gate：
#    - +20°→-150° 单帧毛刺丢弃；0.05 rad 合法步 PASS；
#    - 连续 3 帧异常 → HOLD + actual-q reseed；
#    - best-effort fallback 不因“fallback”标签本身被拒绝，只按连续性数值判据。
#
# 5. stop coordinator：
#    - inhibit → bounded drain → slow_stop；
#    - 覆盖 drain normal / timeout / slow_stop retry / immediate-stop escalation；
#    - 失败路径不得重新 enable sender。
#
# 6. C re-anchor / pause：
#    - 旧 target 在 buffer 中触发 C，断言 clear + reset + epoch 更新；
#    - 旧 epoch target 不得在 re-enable 后发送；pause/resume 同理。
#
# 7. tool profile：
#    - flange_only FK == flange；
#    - sg100_attached FK == current MuJoCo sg100_base TCP。

python -m sg100_teleop.tests.test_robot_backend_units
```

### B2.3 runner 接线：必须在 `q_arm_goal` 后分叉

1. CLI 参数至少包括：`--arm-backend {sim,real}`（默认 sim）、`--arm-ip`、`--arm-port`、`--arm-follow {low,filter,high}`、`--arm-radio`、`--arm-send-hz`、`--arm-safety-profile {10,25,50}`、`--arm-tool-profile`、`--arm-target-timeout-ms`、`--arm-feedback-timeout-ms`、`--arm-sender-drain-timeout-ms`、`--arm-max-follow-error-deg`（兼容旧 CLI，内部拆 soft/hard）、`--arm-hold-to-run`、`--arm-verbose`。

2. 启动：real → 读 actual q → 校验 6DoF/单位 → twin、IK seed、continuity gate、real limiter 由 actual q 初始化；跳过 simulation bottle pre-grasp startup。

3. **核心分叉发生在 `q_arm_goal`**：

```text
q_arm_goal
   ├─ sim:
   │    existing ArmCommandFilter @500Hz
   │    → q_arm_cmd_sim
   │    → data.ctrl
   │
   └─ real:
        IKContinuityGate
        → LatestTargetBuffer
        → RM65RealBackend sender
            → JointTrajectorySafetyLimiter(actual sender dt)
            → q_arm_cmd_real
            → rm_movej_canfd
```

real path **不得**先经过现有 MuJoCo 500 Hz `ArmCommandFilter`。

4. C：real 按 §A4.4 原子 re-anchor；anchor = actual q → software FK。A 键只在 real 模式分离 calibration 与 motion authority。

5. pause/disarm：统一走 §A3.5/§A4.4；runner 键位不得直接绕过 backend sender 状态。

6. Digital twin：real arm 的可视化 q 来自 feedback，不来自 `q_arm_cmd_real`；flange-only 只 `mj_forward`。

**退出条件**：§B2.2 全部通过；`--arm-backend sim` 现有 regression 逐项零变化；real 无硬件时清晰报错且不留下半初始化 sender thread。

## B3. Phase 3 — 安全层配置教程

### B3.1 数值初值表（first-run provisional；真机实测后回填）

| 参数 | PROFILE_10 | PROFILE_25 | PROFILE_50 | 说明 |
|---|---:|---:|---:|---|
| qdot_max 临时标量(rad/s) | 0.5 | 1.25 | 2.5 | 仅 smoke 初值；production 改 6D per-joint vector |
| qddot_max 临时标量(rad/s²) | 2 | 5 | 10 | 同上 |
| TCP 线速度上限(m/s) | 0.05 | 0.125 | 0.25 | TcpTargetFilter 同步 profile 化 |
| joint soft margin(deg) | 2 | 2 | 2 | 最终可 per-joint |
| target age(ms) | 150 | 150 | 150 | freshness watchdog |
| feedback age(ms) | 200(TCP polling) | 200 | 100(UDP) | 随反馈通道切换 |
| follow-error soft(deg) | 5 | 5 | 5 | provisional；需 persistence 条件 |
| follow-error persistence(ms) | 100 | 100 | 100 | provisional，真机回填 |
| follow-error hard(deg) | 真机标定后回填 | 同左 | 同左 | hard 一旦越过立即 stop；未标定不得标 READY |
| sender drain timeout(ms) | 真机实测后回填 | 同左 | 同左 | bounded wait，禁止无限 join |
| sender timing fault | period 超配置上限 | 同左 | 同左 | 超长 dt fault，不用大 dt 放宽 Δq |
| σ_min 下限 | 0.05 | 0.05 | 0.05 | provisional |
| 碰撞等级 `rm_set_collision_state` | 6 | 6 | 6 | provisional；实体测试后确认 |
| gate per-solve(rad) | 0.2 | 0.2 | 0.2 | `max_i |Δq_i|` |
| 首目标接近门(rad) | 0.15 | 0.15 | 0.15 | `max_i |q_target-q_actual|` |

Production limiter 接口应使用：

```python
qdot_limit_rad_s: np.ndarray   # shape (6,)
qddot_limit_rad_s2: np.ndarray # shape (6,)
```

`PROFILE_10/25/50` 是对已验证 per-joint nominal safety limits 的缩放，不应长期用一个统一标量代替六关节限制。

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

每项安全机制都要主动验证，但**不把“人在 active CANFD 遥操作时手推关节”作为标准测试**。

1. target watchdog：fault injection 让 tracking producer 停止更新；
2. feedback watchdog：mock/受控测试停止 feedback 更新；
3. follow error：先 mock soft-persistent / hard 偏差；真机只在 PROFILE_10、清空工作区、持物理急停条件下用受控 mismatch 验证；
4. sender drain：覆盖正常、慢返回、超时；
5. workspace guard：先离线越界 target，再真机低速接近边界；
6. electronic fence / collision：按厂商安全流程、低速验证，不做高速碰撞；
7. re-anchor / pause：确认旧 target 不跨 epoch/resume 被发送。

观察：sender 是否先 inhibit、bounded drain 是否按时返回、stop/disarm reason 是否记录、恢复后是否只接受 fresh target。

**退出条件**：七层防御均有测试证据；provisional 参数回填实测值；任何尚未真机标定的 hard threshold 不得标 READY。

## B4. Phase 4 — 联调阶梯教程(逐级通过才进下一级)

| 级 | 内容 | 命令/方法 | 通过判据 |
|---|---|---|---|
| 1 | sim 回归 | README §12/§13 全量 | sim 的 `ArmCommandFilter @500Hz → data.ctrl` 行为逐项零变化 |
| 2 | SDK 冒烟 | §B1 | 同 §B1 退出条件 |
| 3 | backend 无 AVP | `--arm-backend real` + ±3° 单关节正弦 @100Hz | latest-target 不排队；同一 fresh source target 可跨多个 sender tick；sender timing / limiter / twin / bounded drain 全部达标；停目标源后显式 slow-stop |
| 4 | AVP + PROFILE_10 | `--wrist-roll-ik --arm-follow low --arm-safety-profile 10 --arm-hold-to-run` | 六轴方向/比例确认；不可达贴边界滑移无冻结卡顿；C re-anchor 与 pause 不发送 stale target；watchdog 可用 |
| 5 | 安全机制联试 | §B3.4 全项；UDP 验证后再切 `--arm-follow filter` | 每项 fault/stop/recovery 与状态机一致；filter 前 UDP 到达率/周期达标 |
| 6 | 提速 | PROFILE_10 → 25 → 50 | 每档重跑 q/qdot/qddot、follow-error、sender timing、watchdog 与 fence |
| 7 | 装手 | `--arm-tool-profile sg100_attached` | `T_flange_tcp`、software FK、workspace/fence/包络、首目标门全部重验证 |

### B4.4 sender timing 测量（第 3 级起持续记录）

sender 每 tick 记录：

```text
tick_monotonic_ns
send_start_ns
send_return_ns
source_seq
source_age_ms
q_arm_cmd_real
actual_dt
```

统计：

```text
mean / p95 / p99 / max tick period
mean / p95 / max SDK call duration
missed deadline count
sender timing fault count
source target age distribution
```

对 high-follow，`>10 ms` 是关键 deadline 指标；low/filter 仍按各自配置周期统计。任何异常长 dt 超 safety 配置时触发 timing fault，不允许 limiter 通过“大 dt”放宽一次位移。

### B4.5 单位审计（Go/No-Go 硬项）

第 3 级：发单关节 `+1°` 指令，feedback 应 `+1°±0.05°`；twin 同关节显示 `+1°`。命令、SDK 边界、反馈、twin 四处一致才算单位桥闭合。

## B5. Phase 5 — 验收测量

记录机制:**扩展** `mujoco_recording.npz` 与 sender 日志,不另造轮子:

```text
每 tracking 帧:
  AVP timestamp、source seq、calibration_epoch、q_arm_goal、gate 判定

每 sender tick:
  tick ts、send start/return ts、source seq、source age、
  q_arm_cmd_real、actual dt、limiter clipping flags

每 feedback 帧:
  ts、q_actual、qd(若有)、controller err、arm_status

事件流:
  target/feedback/follow/timing watchdog、drain timeout、fence/collision、
  C re-anchor、pause/resume、ARM/DISARM、IK fallback/reject/reseed
```

指标：端到端 latency、`q_cmd-q_actual`、source-age、sender timing、drain latency、watchdog 恢复延迟、IK fallback/reject、各 PROFILE 对比。

---

# 第三部分 参考

## C1. Go / No-Go(首次 AVP→真机连续遥操作,全项可溯源)

| # | 项 | 溯源 |
|---|---|---|
| 1 | state 冒烟 PASS | §B1.1 |
| 2 | wiggle ±3° PASS | §B1.2 |
| 3 | 六关节方向人工确认 | §B1.2 |
| 4 | T_base_table 按 actual q + canonical FK 测量 | §B0.4 |
| 5 | 断电行为确认 | §B0.3 |
| 6 | actual q 初始化 twin/IK/gate/real limiter | §B2.3 |
| 7 | sim/real 在 q_arm_goal 后正确分叉；real 未经过 500Hz sim filter | §A1.3/§B2.3 |
| 8 | software FK anchor PASS | §A6.2 |
| 9 | C/A 分离 + hold-to-run | §A4.2 |
| 10 | C re-anchor 清 stale target / reset epoch | §A4.4 |
| 11 | pause/resume 不发送 pre-pause stale target | §A4.4 |
| 12 | 首目标 per-joint max gate PASS | §A4.3 |
| 13 | LatestTargetBuffer latest-only，无 FIFO backlog | §A3.4 |
| 14 | fresh 同一 source seq 可跨多个 sender tick 正常发送 | §A3.4 |
| 15 | IK continuity gate + best-effort fallback 分类策略 PASS | §A3.3 |
| 16 | q soft limit PASS | §A3.2 |
| 17 | per-joint qdot limiter PASS | §A3.2/§B3.1 |
| 18 | per-joint qddot limiter PASS | §A3.2/§B3.1 |
| 19 | abnormal-long-dt timing fault PASS | §A3.2/§B4.4 |
| 20 | target watchdog PASS | §B3.4 |
| 21 | feedback watchdog PASS | §B3.4 |
| 22 | follow-error soft-persistent / hard 两级路径 PASS | §B3.1/§B3.4 |
| 23 | bounded drain + drain-timeout escalation PASS | §A3.5 |
| 24 | cmd-feedback monitor PASS | §B3.4 |
| 25 | +1° unit audit PASS | §B4.5 |
| 26 | tool profile `T_flange_tcp` 两种 FK 回归 PASS | §A6.3 |
| 27 | software workspace guard PASS | §A3.1 |
| 28 | controller whole-arm fence 在 CANFD 下实测 PASS | §B3.2/§B4 |
| 29 | 碰撞保护按安全流程验证 | §B3.2 |
| 30 | sender timing / SDK call duration / source-age 达标 | §B4.4 |
| 31 | twin 跟随 feedback，不含 arm actuator 积分 | §A5.3 |
| 32 | 物理急停已演练 | §B0.2 |
| 33 | PROFILE_10 已选；1m 区域清空 | §B4 |
| 34 | wrist-roll 方向/比例确认；不可达贴边界滑移无卡顿 | §A3.3/§B4 |

任一未过 = NO-GO;回退 `--arm-backend sim` 永远可用。

## C2. SDK 速查(robotic-arm 1.1.6)

| 用途 | API | 位置 |
|---|---|---|
| 连接/断开 | low + TCP polling 可用 `RM_SINGLE_MODE_E`；启用 UDP realtime push 时使用 `RM_TRIPLE_MODE_E`；均通过 `rm_create_robot_arm(ip,8080)` / `rm_delete_robot_arm` | :7115 / §A5.2 |
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

## C3. 与 v1/v2/v3.1 的关系

- v1(`REAL_RM65_MIGRATION.md`)：Phase 0-1 操作细节已并入本文，保留作历史；
- v2(`RM65_真机迁移方案_修订版_v2.md`)：架构/安全修订已并入本文，保留作历史；
- v3/v3.1(`RM65_real_v3.md`)：IK 分类策略、停止竞态、反馈分级等已并入 v3.2；v3.2 进一步修正 latest-target、sim/real filter 分叉、bounded drain、re-anchor/pause 与 tool profile；
- **后续代码改造只以本 v3.2 为实施依据。**

## C4. 风险与缓解

| 风险 | 缓解 | 溯源 |
|---|---|---|
| canfd 周期抖动 | 有线 NIC 独占命令路径;实测五指标;filter/high 用 125/200Hz | §A2.4/§A5.4 |
| slow-stop 前仍有 CANFD in-flight | 先 inhibit sender；bounded drain；再 slow-stop；drain timeout 有升级路径 | §A3.5/§B2.2 |
| stale target 被无限追踪 | sender 可重复使用 fresh source seq，但以 source-age watchdog 阻止陈旧 target；buffer 无 FIFO | §A3.4 |
| 反馈断流 | feedback watchdog → 停止序列 | §A3.5 |
| IK branch / 单帧毛刺 | best-effort fallback 保留；continuity gate 只按数值连续性拦毛刺，连续异常才 HOLD + actual-q reseed | §A3.3 |
| 命令跳变/冲击 | q/qdot/qddot 三层(实测 dt) | §A3.2 |
| 轴向标错 | PROFILE_10 + hold-to-run + 逐轴验证 | §B4-4 |
| 控制器/软件 TCP 定义不一致 | anchor=实际 q+软件 FK | §A6.2 |
| 真实桌面≠仿真桌面 | 实测 T_base_table,双层围栏 | §A6.1/§A3.1 |
| 装手后包络变化 | tool profile 强制重验证 | §A6.3 |
| 单位换算错误 | 单一转换点+审计 | §A6.4 |
| 软件整体失效 | whole-arm fence + 碰撞保护 + 物理急停 | §A3.1 |
| 断电行为未知 | §B0.3 事前确认记录 | §B0.3 |

## C5. README 同步事项（Phase 2 完成后执行）

1. 勘误 `README.md:253`：当前 sim 的 `ArmCommandFilter` 真实存在；同时明确它**只属于 sim path**，real 在 `q_arm_goal` 后独立分叉。
2. 能力状态表更新“实体 RM65 控制”行。
3. §16 维护约束至少新增：
   - real 模式任何 `q_arm_goal` 不得绕过 `IKContinuityGate → LatestTargetBuffer → JointTrajectorySafetyLimiter` 直达 `rm_movej_canfd`；
   - real path 不得复用 MuJoCo 500Hz `ArmCommandFilter` 作为硬件 limiter；
   - LatestTargetBuffer 是 latest-only 单槽；`seq` 不变不能作为 sender 停发条件，freshness 由 timestamp watchdog 判定；
   - safety stop 必须 `inhibit → bounded drain → stop → clear target`；
   - rad↔deg 与 UDP speed scale 只许存在于 `RM65RealBackend`；
   - `T_flange_tcp` 是 FK/IK canonical TCP 合同，不得在多个模块重复补偿；
   - fault/re-anchor/pause 后旧 epoch target 不得进入 sender。
4. §17 文件索引加入 `sg100_teleop/robot/` 与对应 tests。

## C6. VSCode Agent 代码改造合同（按此顺序实施）

> 本节给代码 Agent 使用。修改源码前必须读取本文件对应章节和当前 `sg100_teleop/README.md`；若文档旧行号与源码不一致，以当前源码为准并回报差异，不得按旧行号硬改。

### C6.1 Phase 2A：先建纯 Python 安全核心，不碰 runner

先新增/完成：

```text
sg100_teleop/robot/
├── arm_state.py
├── latest_target_buffer.py
├── ik_continuity_gate.py
├── joint_trajectory_safety.py
└── safety_monitor.py
```

要求：

- 不 import 真机 SDK 也能测试；
- LatestTargetBuffer 只保留 latest，不实现 queue；
- limiter 输入为 latest target + actual sender dt；
- abnormal-long-dt 进入 timing fault；
- stop coordinator 支持 bounded drain；
- unit tests 全部通过后才进入 backend。

### C6.2 Phase 2B：实现 backend，SDK 转换只在一个文件

新增：

```text
arm_backend.py
rm65_sim.py
rm65_real.py
```

`RM65RealBackend` 独占：

```text
RoboticArm SDK object
rad ↔ deg
UDP joint_speed 0.01°/s ↔ rad/s
sender thread
feedback path
stop/pause API
```

禁止 mapper / IK / runner 其它位置出现新的 degree conversion。

### C6.3 Phase 2C：runner 只在 q_arm_goal 后分叉

Agent 先画出当前源码实际路径并报告调用点，再修改：

```text
SIM:
q_arm_goal
→ existing ArmCommandFilter
→ data.ctrl

REAL:
q_arm_goal
→ gate
→ latest buffer
→ backend sender/limiter
→ CANFD
```

禁止 real 先走 sim filter。

### C6.4 Phase 2D：real-only 状态机

增加/映射：

```text
CONNECTED_SAFE
CALIBRATED
ARMED
RUNNING
PAUSED
DISARMED
FAULTED
```

必须测试：C re-anchor epoch、A arm/disarm、hold-to-run、pause/resume、tracking timeout 不自动 re-arm、fault 后手工恢复。sim 行为不能改变。

### C6.5 必须新增/维护的 tests

```text
test_latest_target_is_single_slot
test_sender_reuses_fresh_source_target
test_stale_target_triggers_watchdog
test_no_fifo_backlog
test_limiter_q_qdot_qddot
test_limiter_short_dt
test_long_dt_is_fault_not_large_step
test_gate_single_glitch
test_gate_three_glitches_reseed
test_best_effort_fallback_not_rejected_by_label
test_stop_order
test_stop_bounded_drain_timeout
test_reanchor_clears_old_epoch_target
test_pause_resume_requires_fresh_target
test_first_target_gate_is_per_joint_max
test_flange_only_tcp_contract
test_sg100_attached_tcp_contract
test_sim_backend_regression
test_real_sdk_deg_bridge
```

### C6.6 每个 Agent 阶段结束必须报告

```text
Files changed
Why each file changed
Tests added
Exact test commands
PASS/FAIL
Known hardware-only gaps
git diff --check
git status
```

无真机环境不得伪造 `UDP feedback / electronic fence / collision protection / CANFD timing` PASS；统一标成 `NOT TESTED — HARDWARE REQUIRED`。

### C6.7 硬性禁止项

- real mode 不得用 `data.ctrl` 作为真机 command source；
- real mode 不得绕过 limiter；
- LatestTargetBuffer 不得变成 FIFO；
- `seq` 不变不得自动停掉固定周期 sender；
- stop path 不得无限 join；
- C/pause 后旧 target 不得继续发送；
- 不得把 grasp-ready pose 重定义成 joint zero；
- 不得修改 sim 默认行为来“顺便兼容真机”；
- 不得在多个模块重复 rad/deg 或 tool-offset 变换；
- 无硬件时不得把 hardware tests 标 PASS。
