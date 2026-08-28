# RM65 真机接入:当前进度与测试指南

> 快照日期:2026-08-28。方案文档见 `RM65_real_v3.md`(架构与原理的权威来源,用户另有 v3.2 版);本文只回答三件事:**现在到哪了、每个文件干什么、怎么测**。与实际不符时以源码为准并回改本文。

---

## 1. 当前进度总览

| 能力 | 状态 | 依据(实测) |
|---|---|---|
| SDK 冒烟(连接/读状态/小幅运动) | ✅ 真机验证 | arm_dof=6,V3 控制器 192.168.1.18:8080 |
| 真机后端软件包(`sg100_teleop/robot/`) | ✅ 完成+单元验证 | 9 项无硬件测试 PASS |
| CANFD 流式下发(100 Hz) | ✅ 真机验证 | 正弦:p99 10.5 ms、零超期、mean ≈10 ms |
| 停止序列(停发→watchdog→慢停) | ✅ 真机验证 | target_stale 0.156 s 触发,停后漂移 0.018°/s |
| 速度式死臂 watchdog | ✅ 真机验证 | 正确捕获过两类真实异常(流损坏/速度饱和) |
| AVP→真机遥操(wrist-roll-ik,PROFILE_10) | ✅ 可用 | 慢动作跟手;tracking timeout→重 C+A 全链走通 |
| sim 仿真链路 | ✅ 零变化 | `--arm-backend` 默认 sim,回归一致 |
| kinematic twin(viewer/AVP 显示=真机实际) | ✅ 验证 | 位姿与实物一致 |
| 控制器电子围栏 / 碰撞保护 / T_base_table | ❌ 未做(Phase 3) | 提速/真实任务前必须补 |
| UDP push 反馈 / filter 模式 125 Hz | ❌ 未做(Backlog) | 现用 TCP 50 Hz 轮询 + follow=low |

**架构要点**(为什么现在是这个样子):SDK 双线程模式 + 互斥锁;反馈轮询独立线程(sender 每 tick 只发 canfd,保住 10 ms 周期);rad↔deg 转换只在 `rm65_real.py` 一处;watchdog 在 sender 线程内执行停止序列。

**已知现象**(均为实测确认的行为,非 bug):

1. **低跟随滞后**:follow=low 下臂以 ~0.2 s 时间常数跟随指令流,慢速(~<0.15 rad/s)滞后约 2°,手感良好;
2. **快动作 disarm**:快速大幅动作时命令速度可达 PROFILE_10 上限 0.5 rad/s,实际交付速度明显更低 → `arm_not_moving` watchdog 触发减速停;**按 A 重新 ARM 即可继续**(当前被接受的稳态用法;根因假设是低跟随模式的速度上限,量化与对策见 v3 文档 backlog);
3. **慢停沉降尾巴**:停机后臂会向最后一条指令角收敛最多 ~2° 再静止(~1 s),安全判据应为"沉降后静止";
4. 录制:`mujoco_recording.npz` 在 real 模式只录 1 帧(按仿真时间采样,kinematic twin 不推进仿真时间),待改墙钟采样;
5. AVP 帧偶发 malformed(~5%),不影响遥操,待排查。

---

## 2. 文件清单与作用

### 2.1 新增:`sg100_teleop/robot/`(真机后端子包,所有 SDK 交互唯一入口)

| 文件 | 作用 |
|---|---|
| `__init__.py` | 模块图说明 |
| `arm_state.py` | `ArmTarget`(q+时间戳+序号)/`ArmState`(q、qd 可 None、错误码)数据类 |
| `arm_backend.py` | 后端抽象接口(connect/send_joint_target/joint_state/stop...) |
| `rm65_sim.py` | sim 旁路后端(写 MuJoCo ctrl,接口对等);RM65 关节限位常量(与 MJCF 一致) |
| `rm65_real.py` | **真机后端**:SDK 创建(双线程模式)、CANFD sender 线程(100 Hz)、反馈轮询线程(50 Hz)、deg 桥唯一转换点、停止序列接线、sender 周期统计 |
| `latest_target_buffer.py` | 线程安全单槽目标 buffer(最新胜、旧序号拒收) |
| `joint_trajectory_safety.py` | q/qdot/qddot 限幅器,实测 dt,带制动项(√(2·qddot·误差),保证无过冲到达) |
| `ik_continuity_gate.py` | IK 毛刺帧门:单帧跳变丢弃,连续 3 帧才 HOLD+重播种 |
| `safety_monitor.py` | watchdog 集合(target/feedback 年龄、控制器错误、**速度式死臂检测**、0.35 rad 粗判)+ `StopSequence`(先停 sender→排空→慢停→失败升级→清 buffer→DISARM) |

### 2.2 新增:测试脚本

| 文件 | 作用 | 需要设备 |
|---|---|---|
| `sg100_teleop/tests/test_robot_backend_units.py` | 9 项无硬件单元测试(Fake 控制器注入) | 无 |
| `scripts/rm65_backend_sinusoid.py` | 真机集成测试:正弦流式(A 段)→停发实验(B 段)→重 ARM 归位(C 段) | 仅 RM65 |
| `scripts/rm65_smoke.py` | 冒烟:state(只读)/wiggle(±3° 单关节)/stop(网络急停) | 仅 RM65 |

### 2.3 修改:`sg100_teleop/sim/run_avp_arm_hand_sim.py`(主 runner,sim 路径零变化)

| 改动 | 说明 |
|---|---|
| `--arm-*` 参数组(13 个) | backend/ip/port/follow/radio/send-hz/safety-profile/tool-profile/两个超时/粗判上限/verbose |
| real 启动 | 连真机→读实际 q 作为 startup(跳过 bottle pre-grasp),下游全部对齐 |
| 物理循环分叉 | real:发布 q_arm_goal 给后端+twin 同步(arm qpos=反馈,hand qpos=滤波目标,mj_forward,不 mj_step);sim 分支原样 |
| 键位 | `A`=ARM/DISARM(disarm 先停 sender 再慢停);`C` 标定不变;tracking 超时/退出自动停机 |
| verbose | 每 0.5 s 打印 `real sender` 行(n/mean/p95/p99/max/misses) |

### 2.4 文档

| 文件 | 内容 |
|---|---|
| `RM65_real_v3.md`(及 v3_2) | 迁移方案权威文档:原理篇(纵深防御/透传原理/状态机/twin)、教程篇(Phase 0-5)、Go/No-Go 26 项、SDK 速查 |
| `REAL_RM65_MIGRATION.md` | v1 历史文档(已被 v3 取代,保留) |
| 本文 | 进度快照+文件指南+测试手册 |

---

## 3. 如何测试(按层级,每级有通过判据)

环境:**必须** `conda activate mujoco-learn`(全机唯一装有 `Robotic_Arm` 的环境)。所有真机操作前:1 m 清空、物理急停在手边。

### L0 单元测试(无设备,30 s)

```bash
python -m sg100_teleop.tests.test_robot_backend_units
```

通过判据:最后一行 `ROBOT BACKEND UNITS: PASS`(9 项:deg 桥/buffer 语义/限幅器速度加速度界与软限位/连续性门三段语义/停止顺序与升级/watchdog 四类触发/速度式死臂三场景/线程级流式-停发-慢停)。

### L1 冒烟(仅 RM65,2 min)

```bash
python scripts/rm65_smoke.py --ip 192.168.1.18 state
python scripts/rm65_smoke.py --ip 192.168.1.18 wiggle --joint 1 --amplitude 3 --speed 5
python scripts/rm65_smoke.py --ip 192.168.1.18 stop     # 网络急停,随时可用
```

| 命令 | 通过判据 |
|---|---|
| state | `arm_id=正数`、`arm_dof=6`、六个有限关节角、有限 TCP position/euler(注意 pose 是扁平列表 [x,y,z,rx,ry,rz],欧拉角单位度)、错误码 0、无 `[FAIL]` |
| wiggle | 两次 `[MOVE]` 返回 0、following error <1°、结束回到起始角度 |

### L2 集成:正弦脚本(仅 RM65,40 s)

```bash
python scripts/rm65_backend_sinusoid.py --ip 192.168.1.18
```

| 段 | 内容 | 通过判据(基准实测) |
|---|---|---|
| A | 关节 1 ±3°/0.5 Hz 流式 10 s | p95 ≈10.2 ms、p99 ≈10.5 ms、misses=0;follow_err ≈2°(低跟随固有滞后) |
| B | 停发实验 | 3 s 内 `disarmed by watchdog: PASS`(target age ~0.156 s);settle tail ~1°;沉降后 1 s 漂移 <0.2°;final vs last cmd ≈0 |
| C | 重 ARM 归位 | 残差 <0.2°(基准 0.003°);结尾 `SINUSOID TEST: PASS` |

参数:`--joint/--amplitude-deg/--frequency-hz/--duration/--send-hz/--follow/--safety-profile`。

### L3 sim 回归(AVP,零变化确认)

不带 `--arm-backend`(默认 sim),按原流程跑一遍;startup print、C 标定、遥操行为应与接入前一致。

### L4 真机遥操(AVP + RM65,当前主用法)

单行命令(整行复制,避免换行符问题):

```bash
PYTHONPATH=. python sg100_teleop/sim/run_avp_arm_hand_sim.py --ip 192.168.2.179 --calibration sg100_teleop/config/avp_right_calibration.yaml --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v8_compat.usdz --wrist-roll-ik --verbose --arm-backend real --arm-ip 192.168.1.18 --arm-verbose
```

(AVP IP 以当日实际为准,当前为 192.168.2.179。)

操作流程与预期:

| 步骤 | 操作 | 预期 |
|---|---|---|
| 1 | 启动 | `[REAL-ARM] connected ... q0=[...]`;`startup pose: real arm actual q`;`streaming is DISABLED until you press A`;viewer/AVP 中臂位姿=实物 |
| 2 | **鼠标点 viewer 窗口获得焦点**,按 `A` | `ARMED (A)`;臂不动(目标=当前位姿) |
| 3 | 右手入画、保持中性,按 `C` | `CALIBRATION_REQUESTED`→下一有效帧 `TELEOP_ACTIVE`,anchor 建立,无跳变 |
| 4 | 手移动 | 臂连续跟随(wrist-roll:位置+腕滚);慢动作跟手;`real sender` 行 mean≈10 ms |
| 5 | 快速大幅动作(可选) | 可能 `disarm: arm_not_moving/...` 减速停——按 `A` 重 ARM 继续(已知现象,见 §1) |
| 6 | 运动中按 `A` | 立即减速停,`DISARMED (A)` |
| 7 | 摘头显/遮右手 2 s | `DISARMED (tracking timeout); slow stop`;回画后重新 `C`+`A` |
| 8 | Ctrl-C | 停止序列+`backend disconnected` |

键位速查:`C`=标定(viewer 焦点下)、`A`=ARM/DISARM、`Ctrl-C`=停止+断开、物理急停=最高层。

---

## 4. real 模式参数速查

| 参数 | 默认 | 说明 |
|---|---|---|
| `--arm-backend` | sim | real 才连真机 |
| `--arm-ip / --arm-port` | 必填 / 8080 | 控制器 192.168.1.18:8080(V3 控制器) |
| `--arm-follow` | low | low/filter/high 三档;**当前验证过的只有 low** |
| `--arm-send-hz` | 100 | sender 频率 |
| `--arm-safety-profile` | 10 | 10/25/50 → qdot 0.5/1.25/2.5 rad/s、qddot 2/5/10 rad/s²(**仅 10 经真机验证**) |
| `--arm-target-timeout-ms` | 150 | 目标老化 watchdog |
| `--arm-feedback-timeout-ms` | 200 | 反馈老化 watchdog |
| `--arm-max-follow-error-deg` | 20 | 位置粗判上限(主检测是速度式死臂) |
| `--arm-verbose` | off | 后端事件+每 0.5 s sender 指标 |

网络拓扑:控制器走有线 NIC(192.168.1.x),AVP 走无线(192.168.2.x),互不干扰。

---

## 5. 待办(未做,按优先级)

1. **Phase 3 安全层**(提速/真实任务前必须):T_base_table 实测 → 控制器电子围栏(whole-arm)+ `rm_set_collision_state` → 围栏/碰撞/急停逐项主动触发验证 → 单位审计 +1°;
2. UDP push 反馈(`rm_set_realtime_push`,5 ms 周期,解锁 qd 与 filter 模式);
3. filter 模式 @125 Hz(更高跟手性;依赖上一条);
4. 快动作 `arm_not_moving` 的根治(实测通道速度上限并对齐 limiter,或切 filter 模式);
5. 人体工学:AVP 侧 ARM 触发(免键盘);
6. real 模式录制改墙钟采样;malformed frames 排查;装 SG100 后 tool profile 重验证。

对应的 Go/No-Go 完整清单见 `RM65_real_v3.md` §C1(当前约 18/26 项满足)。
