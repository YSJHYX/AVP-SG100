# RM65 真机迁移方案(sim → real)

本文档是从当前 MuJoCo 仿真链路迁移到实体 RM65-B 机械臂的完整实施方案,包含目标架构、分阶段计划、安全清单与验收标准。现状描述以 `sg100_teleop/README.md` 为准;本文档只覆盖"实体 RM65 控制接入"这一增量,实体 SG100 驱动仍不在范围内。

写作时间:2026-08-27。执行过程中若实际链路与本文冲突,以源码为准并回改本文档。

## 0. 一句话结论

上游链路(AVP tracking → frame gate → wrist mapper → TCP filter → 官方 IK → `q_arm_goal`)全部原样复用;迁移 = 在 `write_robot_controls()` 这一个分叉点接入真机后端,外加真机必备的安全层。新增代码集中在 `sg100_teleop/robot/` 一个子包内。

## 1. 现状与前提

### 1.1 当前链路(sim-only)

```text
AVP 右手 tracking (60 Hz)
  → TrackingFrameGate(去重 / 新鲜度 / 超时)
  → AVPWristRelativeMapper(按 C 建立 anchor,相对平移 + 相对旋转)
  → TcpTargetFilter(SE(3) 滤波,τ=0.06 s,0.25 m/s,1.5 rad/s)
  → RM65IKSolver(RealMan 官方闭式 IK)
  → q_arm_goal (60 Hz)
  → ArmCommandFilter @ 500 Hz 物理率
  → write_robot_controls() → data.ctrl(MuJoCo)
  → USDZ scene stream → AVP
```

`sg100_teleop/README.md` 已明确:"实体 RM65 / SG100 控制 | 不在当前实现范围 | 当前 runner 只写 MuJoCo control"。

### 1.2 已确认的前提事实

| 事实 | 依据 | 对迁移的意义 |
|---|---|---|
| IK 用的就是 RealMan 官方离线算法库(`Robotic_Arm` pip 包的 `Algo`) | `kinematics/rm65_ik.py` `_init_official_algorithm()` | IK 输出关节角与真机控制器语义一致,rad/deg、四元数次序桥接已有先例 |
| MuJoCo 世界系 = 官方 RM65 基座系(FK 一致性 ≤ 5 µm / 0.022°) | `rm65_ik.py` 内注释,曾由独立脚本验证 | 真机不需要任何 SE(3) 修正,控制器 work frame 设为基座系即可 |
| 真机控制 SDK 已在本地安装 | `mujoco-learn` conda env 内 `robotic-arm==1.1.6`,`Robotic_Arm.rm_robot_interface.RoboticArm` | 无需新装依赖;注意全机只有该 env 有此包,base(3.13)没有 |
| 命令链已有限速/限幅平滑 | `JointCommandFilter` @ 物理率 | 天然适合真机下发,但参数需保守化(见 §5 Phase 3) |

### 1.3 真机 SDK 关键接口速查

SDK 版本:`robotic-arm 1.1.6`(`Robotic_Arm` 包,基于 ctypes 包装控制器 TCP API)。

| 用途 | 接口 | 备注 |
|---|---|---|
| 连接 / 断开 | `RoboticArm(rm_thread_mode_e.RM_SINGLE_MODE_E)` + `rm_create_robot_arm(ip, port)` / `rm_delete_robot_arm()` | RM65-B V3 控制器默认端口 8080;返回码与 `handle.id == -1` 都要检查 |
| 关节透传(遥操作主通道) | `rm_movej_canfd(joint_deg, follow, expand, trajectory_mode, radio)` | 角度不经规划直接执行;`follow=True` 高跟随要求周期 ≤ 10 ms;`trajectory_mode`:0 完全透传 / 1 曲线拟合 / 2 滤波,`radio` 为平滑系数 |
| 规划式关节运动(冒烟/回 home) | `rm_movej(joint_deg, v_pct, r, connect, block)` | `v` 是速度百分比 1–100;单线程模式 `block` 值为超时秒数 |
| 停止 | `rm_set_arm_stop()`(立即)/ `rm_set_arm_slow_stop()`(减速)/ `rm_set_arm_pause()` + `rm_set_arm_continue()` | 网络急停用 slow stop |
| 状态 | `rm_get_current_arm_state()`(pose + joint + err)、`rm_get_joint_degree()` | 角度单位 °;pose 含 position + quaternion |
| 安全 | `rm_set_collision_detection(mode)`、`ElectronicFenceConfig` 电子围栏、`SelfCollision` | 电子围栏实现工作空间软限位 |
| 初始位姿 | `rm_set_init_pose(joint)` / `rm_get_init_pose()` | 定义真机 home |

## 2. 目标架构(迁移后)

```text
AVP 右手 tracking (60 Hz)
  → TrackingFrameGate
  → AVPWristRelativeMapper(按 C 建立 anchor)
  → TcpTargetFilter
  → RM65IKSolver(官方闭式 IK)
  → q_arm_goal
  → ArmCommandFilter
  → q_arm_cmd ──┬─ sim 后端: data.ctrl → MuJoCo physics(现状,保留)
                └─ real 后端: 发送线程 @ ~100 Hz,latest-target 语义
                     → rm_movej_canfd() → RM65 控制器(以太网 TCP)
                     ← rm_get_current_arm_state() 反馈 @ ~50 Hz
                          ├ 同步 MuJoCo twin(data.qpos + mj_forward)→ USDZ → AVP
                          ├ 按 C 时 anchor = 真机实际 TCP pose
                          └ cmd/反馈偏差监控,超差 → rm_set_arm_slow_stop()
```

设计要点:

1. **MuJoCo 保留为 digital twin**。real 模式下真机反馈关节角写入 `data.qpos` 并 `mj_forward`;SG100 手支路 physics 照跑,AVP 的 USDZ 可视化链路零改动。
2. **后端抽象**,插入点就是 runner 物理循环里的 `write_robot_controls()` 调用。sim 行为完全不变,所有真机逻辑收在 `sg100_teleop/robot/` 子包。
3. **发送线程 latest-target**:不排队,永远发最新目标,旧目标丢弃——跟踪遥操作只关心当前值,排队反而造成延迟累积。

## 3. 关键设计决策

### 3.1 指令通道:rm_movej_canfd,不用 rm_movej

- `rm_movej` 每次调用都做完整轨迹规划(带加减速),流式下发会产生顿挫,只用于冒烟测试和回 home;
- `rm_movej_canfd` 是角度透传,官方遥操作标准路径;
- **起步参数:`follow=False`(低跟随)+ `trajectory_mode=2`(滤波模式)+ 适中 `radio`**。AVP 目标本身有噪声,滤波模式最稳;链路验证稳定后再试 `follow=True`(高跟随,延迟最低,要求发送周期稳定 ≤ 10 ms);
- **注意:canfd 模式下速度上限完全由发送轨迹的斜率决定**,不受 `rm_movej` 的 `v` 百分比约束。因此 `ArmCommandFilter` 的关节速度上限就是真机实际安全限速,必须保守设置。

### 3.2 单位与坐标

- SDK 全角度制(°),项目内部 rad;转换集中在 real 后端一处(仿照 `rm65_ik.py` 的度/弧度桥接);
- MuJoCo 世界系 = 官方基座系,无额外变换;控制器 work frame 保持基座系;
- TCP 定义:仿真 TCP 是 `sg100_base_link`(挂在 link_6 上,不是法兰中心)。**第一阶段不装 SG100,真机 tool frame 先用法兰盘**;装手后按安装偏移重设 tool frame,或维持法兰 TCP + 在软件侧补偿固定 SE(3) 偏移。

### 3.3 启动与 anchor 语义(real 模式)

- 启动时读真机当前关节角 → 作为 IK seed + twin 初始 `qpos`。**跳过**现在基于 bottle 位置求解 pre-grasp 的 startup 逻辑——真机上机器人停在哪儿就是哪儿;
- 按 `C` 的 TCP anchor 现在读 MuJoCo 实际 pose;real 模式必须改为真机反馈 pose 的 FK 结果,否则 anchor 基准是假的,第一次按 C 就会跳。

## 4. 新增 / 修改文件清单

```text
sg100_teleop/robot/                  (新增子包)
├── __init__.py
├── arm_backend.py                   抽象接口:connect / disconnect /
│                                    send_joint_target(q_rad) /
│                                    joint_state() -> q_rad /
│                                    stop_immediate() / stop_smooth()
├── rm65_sim.py                      包一层现有 write_robot_controls,sim 语义不变
└── rm65_real.py                     RoboticArm 封装:deg 桥接、100 Hz 发送线程
                                     (latest-target)、50 Hz 状态轮询、慢停/急停、
                                     cmd-反馈偏差监控

scripts/rm65_smoke.py                (已就位)Phase 1 冒烟脚本:state / wiggle / stop

sg100_teleop/sim/run_avp_arm_hand_sim.py   (修改,四处)
├── 新参数:--arm-backend {sim,real}、--arm-ip / --arm-port、
│           --arm-follow {low,high,filter}、--arm-radio
├── 启动:real → 读真机关节,替换 startup pre-grasp 逻辑
├── 物理循环:q_arm_cmd 照旧计算,real 模式交 backend 发送,twin 用反馈同步
└── 按 C anchor:real 模式用真机反馈 pose

sg100_teleop/README.md               完成后同步能力状态表(实体 RM65 行)
```

## 5. 分阶段实施

### Phase 0 — 硬件 / 网络准备(不写代码)

- [ ] 控制器上电,网线接 PC;PC 以太网口配同网段(控制器默认静态 IP `192.168.1.18`);
- [ ] `ping 192.168.1.18` 通;
- [ ] 底座刚性固定(桌面夹持/打孔,迁移期间不允许晃动);
- [ ] 物理急停按钮放在操作者手边,演练一次按下-释放-复位;
- [ ] 暂不装 SG100:先用法兰盘做 TCP 跑通全程。

**退出条件**:ping 通,急停演练完成。

### Phase 1 — SDK 冒烟(第一步,脚本已就位)

```bash
conda activate mujoco-learn          # 全机唯一有 Robotic_Arm 的 env

# 只读检查——不动机械臂
python scripts/rm65_smoke.py --ip 192.168.1.18 state

# 通过后,小幅动一个关节(5°、5% 速度、动完自动回原位)
python scripts/rm65_smoke.py --ip 192.168.1.18 wiggle --joint 1

# 随时可用的网络端减速急停
python scripts/rm65_smoke.py --ip 192.168.1.18 stop
```

`state` 验收:`[CONN] arm_dof=6`;`[STATE] joint (deg)` 六个合理角度;TCP position/quat 为有限数字;无 `[FAIL]`。
`wiggle` 验收:两次 `[MOVE]` 返回码均为 0;`following error < 1°`;结束位姿 ≈ 起始位姿。

**退出条件**:state + wiggle 全部通过;关节顺序/方向与 `ARM_JOINT_NAMES` 人工核对一致(wiggle joint 1 时肉眼看是哪个关节动)。

### Phase 2 — ArmBackend 代码改造(真机未到也可完成)

- [ ] `sg100_teleop/robot/` 三件套:抽象 + sim 后端 + real 后端;
- [ ] runner 四处改动(§4),`--arm-backend sim` 默认,现网行为零变化;
- [ ] real 后端单元级验证(无真机):接口 mock、deg 桥接、latest-target 线程语义;
- [ ] twin 同步:real 模式下 `data.qpos` 写反馈 + `mj_forward`,手支路照跑。

**退出条件**:`--arm-backend sim` 全部现有 regression 通过;real 模式在无硬件时能优雅报错退出。

### Phase 3 — 安全层(上真机前必须全部完成)

| # | 项 | 做法 | 优先级 |
|---|---|---|---|
| 1 | **joint-step clamp 补全** | README 已知限制:主 IK 成功解未统一钳制(仅 fallback 有 ±0.1 rad)。真机上所有下发解 clamp 到 上帧 target ± Δmax(Δmax = 最大角速度 × 命令周期)。仿真里只是跳变,真机会伤机器 | **最高** |
| 2 | 限速保守化 | TcpTargetFilter `max_linear_velocity` 0.25 → 0.05~0.1 m/s;ArmCommandFilter 关节速度上限按真机规格 50% 设置 | 高 |
| 3 | 电子围栏 | `ElectronicFenceConfig`,基座系 box:桌面以上 + 可达包络内(sim 故意不加 Cartesian box,真机必须加) | 高 |
| 4 | 碰撞检测 | `rm_set_collision_detection()` 开启并调灵敏度 | 高 |
| 5 | 跟踪丢失/超时 | 现有逻辑仅 HOLD target;real 模式要主动 `rm_set_arm_slow_stop()` + 停发送线程,恢复后重按 C | 高 |
| 6 | cmd/反馈偏差监控 | 偏差 > 5° 自动慢停并 disarm | 中 |
| 7 | 键盘 | 空格 = 暂停(`rm_set_arm_pause`),Ctrl-C = 慢停;物理急停独立于一切软件 | 中 |
| 8 | 首跑围栏 | 真机 1 m 范围内无人、无易损物品;操作者持急停 | 流程 |

**退出条件**:表格逐项打勾;`--verbose` 能打印 cmd/反馈偏差与下发频率。

### Phase 4 — 联调阶梯(每级通过才进下一级)

1. sim 全链路过 README §13 checklist(回归基准);
2. 冒烟脚本 `state` → `wiggle`(Phase 1 已完成);
3. **real 后端 + 无 AVP**:发 ±3° 关节正弦 @ 100 Hz,验证 canfd 链路、twin 跟随、停止发送即慢停;
4. **AVP + 真机**:滤波模式、限速 10% 档,空载走工作空间,逐轴验证方向/比例——顺便完成 identity basis 的实机确认(`config/avp_to_tcp_identity.yaml` 自述 uncalibrated development default);
5. 固化电子围栏/碰撞参数,速度逐步提到 25% → 50%;
6. 装 SG100(真手驱动为独立工程,先仿真手 + 真机臂),再上抓取。

### Phase 5 — 验收测量

沿用 `mujoco_recording.npz` + 后端 sender 日志:

- 端到端延迟:AVP frame timestamp → cmd 下发 → 反馈到位;
- 跟踪误差(cmd vs 反馈)、静止抖动、IK fallback 率;
- 电子围栏/碰撞检测触发-恢复行为;
- 上述指标在速度 10% / 25% / 50% 三档下的对比。

## 6. 风险与回退

| 风险 | 缓解 |
|---|---|
| canfd 高跟随周期不稳(网络抖动) | 起步用滤波模式;PC 与控制器有线直连或同交换机,不跨 WLAN;AVP 走另一网卡 |
| IK 解跳变(奇异附近 branch switch) | Phase 3 #1 的 joint-step clamp;`--verbose` 监控 condition number |
| identity basis 轴向错误导致真机冲向桌面 | 第 4 级联调逐轴验证;电子围栏兜底;首跑限速 10% |
| twin 与真机不一致误导操作者 | AVP 显示以反馈驱动;cmd/反馈偏差超阈值告警 |
| 误发旧目标(latest-target 之外的场景) | 发送线程暂停/断开时丢弃目标;重连后必须重按 C |
| 任何一级联调失败 | 回退上一级;`--arm-backend sim` 永远可用作对照 |

## 7. 与 README 的关系

- 本文档实施期间,`sg100_teleop/README.md` 仍是主链路权威;完成 Phase 2 后在 README 能力状态表中把"实体 RM65 控制"从"不在当前实现范围"更新为对应状态,并在 §17 文件索引加入 `sg100_teleop/robot/`;
- README §16 维护约束新增:real 模式下所有下发目标必须经过 clamp + 限速,不得绕过 `ArmCommandFilter` 直发 canfd。

## 8. 当前状态(执行打勾用)

- [ ] Phase 0 硬件/网络
- [ ] Phase 1 冒烟(state / wiggle)
- [ ] Phase 2 ArmBackend + runner 接线
- [ ] Phase 3 安全层
- [ ] Phase 4 联调 1–6 级
- [ ] Phase 5 验收测量
