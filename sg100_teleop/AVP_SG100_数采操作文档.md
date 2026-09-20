# AVP-SG100 真机数采操作文档

本文面向使用 Apple Vision Pro 遥操作 RM65-B 机械臂与 SG100 灵巧手的现场数采人员。正式数采采用两个独立进程：

- 控制端：接收 AVP 跟踪，计算机械臂 IK 和手部重定向，驱动真机，并发布只读遥测；
- 数采端：采集 D455 头部相机、D405 腕部相机和机器人遥测，按 30 Hz 写入 RAW episode。

数采端不会连接或控制机器人。`On` 只开始录制，真正的真机运动授权仍由 MuJoCo 窗口中的 `A` 键控制。

## 1. 标准操作顺序

正式 episode 固定采用：

```text
C → 等待稳定 → On → A → 执行任务 → D → 等待稳定 → Off → Y
```

各步骤含义：

1. `C`：在 MuJoCo 窗口建立 AVP 相对锚点；
2. 等待稳定：保持手腕和手势不动 1～2 秒；
3. `On`：在 Recorder 终端开始采集；
4. `A`：在 MuJoCo 窗口授权真机运动；
5. 执行任务：完成一次完整、连续的操作；
6. `D`：撤销真机运动授权；
7. 等待稳定：保留任务结束后的静止状态 0.5～1 秒；
8. `Off`：在 Recorder 终端停止并验证本条数据；
9. `Y`：确认保存；验证异常时输入 `N` 丢弃。

正式录制期间不要再次按 `C`，避免把坐标重建过程写入任务轨迹。

## 2. 设备与网络

### 2.1 设备清单

- Apple Vision Pro，运行 Tracking Streamer；
- RM65-B 机械臂，当前控制器地址 `192.168.1.18:8080`；
- SG100 灵巧手，当前串口 `/dev/ttyUSB0`，波特率 1,000,000；
- Intel RealSense D455 头部相机；
- Intel RealSense D405 腕部相机；
- 可随时触达的物理急停；
- 足够的磁盘空间。

### 2.2 启动前检查

进入项目并激活环境：

```bash
cd /home/khan/AVP-SG100
conda activate avp_sg100
```

确认关键 Python 依赖：

```bash
python -c "import mujoco, numpy, pyrealsense2; print('environment OK')"
```

查询 RealSense 序列号：

```bash
rs-enumerate-devices -s
```

记录 D455 和 D405 的实际序列号。不要把示例序列号直接用于另一台工作站。

确认 SG100 串口：

```bash
ls -l /dev/ttyUSB0
ls -l /dev/serial/by-id/
```

确认网络：

```bash
ping -c 2 192.168.1.18
ping -c 2 192.168.2.190
```

现场还必须人工确认：

- 机械臂、手和相机安装牢固；
- SG100 已安装时使用 `sg100_attached` 工具配置；
- 工作空间无人，线缆不会被机械臂拉扯；
- 物理急停有效且操作员能够立即触达；
- 任务物体位于预定初始位置；
- AVP 电量、相机连接和磁盘空间足够完成本轮采集。

## 3. 启动正式数采

使用两个终端。建议先启动 Recorder，使相机和 UDP 接收端先进入等待状态，再启动控制程序。

### 3.1 终端 1：Recorder

将 `<D455_SERIAL>`、`<D405_SERIAL>` 和任务指令替换为本次实际值：

```bash
cd /home/khan/AVP-SG100
conda activate avp_sg100

python -m sg100_teleop.data_collection.run_vla_raw_recorder \
  --telemetry 127.0.0.1:5560 \
  --required-cameras head,right_wrist \
  --camera-modalities rgb \
  --head-serial <D455_SERIAL> \
  --wrist-serial <D405_SERIAL> \
  --instruction "Pick up the object and place it in the target area" \
  --output-dir recording/vla_raw \
  --hz 30
```

启动成功后应看到：

```text
HEAD D455: READY
WRIST D405: READY
Robot telemetry: WAITING
STATE: READY
READY (输入 On):
```

默认情况下 D455 和 D405 都只使用 RGB。若研发任务需要保留深度，显式增加：

```bash
--camera-modalities rgb,depth
```

深度数据是对齐到彩色图的原始 Z16 `uint16` PNG，会明显增加 USB、CPU、磁盘和写入压力。

### 3.2 终端 2：AVP 真机控制与遥测

```bash
cd /home/khan/AVP-SG100
conda activate avp_sg100

python -m sg100_teleop.sim.run_avp_arm_hand_sim \
  --ip 192.168.2.190 \
  --arm-backend real \
  --arm-ip 192.168.1.18 \
  --arm-safety-profile 10 \
  --arm-tool-profile sg100_attached \
  --arm-follow low \
  --hand-backend real \
  --hand-port /dev/ttyUSB0 \
  --calibration sg100_teleop/config/avp_right_calibration.yaml \
  --usdz sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_avp_grasp_v8_compat.usdz \
  --wrist-roll-ik \
  --telemetry-publish 127.0.0.1:5560 \
  --telemetry-publish-hz 100 \
  --arm-joint-compare \
  --sg100-command-debug
```

说明：

- 启动程序不会自动授权机械臂或手运动；
- `--telemetry-publish` 是正式 VLA 数采必需参数；
- `--arm-joint-compare` 只增加 2 Hz 只读诊断日志，可用于核对 MuJoCo、反馈和实际下发角度；
- `arm_not_moving` 自动停止默认关闭，但反馈超时、目标超时、控制器故障和大跟随误差保护仍然有效；
- 当前手部首条有效命令不执行单周期变化检查，但仍受 1 rad 跟踪突变保护和 4 rad/s 速度限制；
- 不要在正式数采时临时改变 `attach`、关节映射、滤波、follow 模式或安全参数。

## 4. 单条 Episode 操作

### 4.1 建立锚点

1. 戴上 Vision Pro 并进入场景；
2. 确认手部骨架和手腕跟踪连续；
3. 将机械臂、灵巧手和任务物体置于预定初始状态；
4. 手腕和手势保持稳定，在 MuJoCo 窗口按 `C`；
5. 等待终端出现锚点建立成功信息；
6. 继续保持稳定 1～2 秒。

`C` 只是相对遥操作锚点，不是机械零位，也不会让真机回零。

### 4.2 开始录制

在 Recorder 终端输入：

```text
On
```

确认终端显示：

```text
RECORDING
candidate episode index: ...
language (frozen): ...
```

`On` 会冻结本条 episode 的语言指令。一个 episode 只执行该指令描述的一次完整任务，不要中途改变任务定义。

### 4.3 授权并执行任务

人工再次确认工作区安全后，在 MuJoCo 窗口按 `A`。看到机械臂和手均进入 enabled 状态后，平缓执行任务。

执行期间：

- 保持头部和手部处于 Vision Pro 可跟踪范围；
- 避免突然的大幅手势或腕部跳变；
- 不要按 `C`；
- 不要移动相机、机器人底座或任务台；
- 发现跟踪丢失、异常噪声、碰撞趋势或姿态异常时，立即按 `D`，必要时使用物理急停。

### 4.4 正常结束

1. 任务完成后，在 MuJoCo 窗口按 `D`；
2. 确认机械臂和手已退出使能；
3. 保持最终状态 0.5～1 秒；
4. 在 Recorder 终端输入：

```text
Off
```

Recorder 会停止产生新帧、刷新写入队列并输出质量报告，然后进入：

```text
Keep this episode? [Y/N/Reset]:
```

检查通过后输入 `Y` 保存；出现任务失败、碰撞、跟踪长时间中断、图像异常、遥测异常或 writer drop/error 时输入 `N` 丢弃。`Reset` 除了丢弃 pending episode，还会清空相机/遥测环形缓存和计时状态，但保持相机连接。

不要在 `RECORDING` 状态直接按 `Ctrl-C`；程序中断会丢弃尚未确认的 pending episode。

## 5. 按键与命令速查

| 输入 | 操作位置 | 含义 |
|---|---|---|
| `C` | MuJoCo 窗口 | 建立或重建 AVP 相对锚点，不是机械回零 |
| `A` | MuJoCo 窗口 | 真机 ARM/DISARM 切换 |
| `D` | MuJoCo 窗口 | 只撤销真机授权，不会重新使能 |
| `E` | MuJoCo 窗口 | SG100 软件急停锁存；需要重启程序才能恢复 |
| `On` | Recorder 终端 | 开始 episode，不控制机器人 |
| `Off` | Recorder 终端 | 停止、flush 并验证 episode |
| `Y` | Recorder 终端 | 保存当前 pending episode |
| `N` | Recorder 终端 | 丢弃当前 episode，编号不增加 |
| `Reset` | Recorder 终端 | 丢弃并清空采集缓存，编号不增加 |
| `Ctrl-C` | 两个终端 | 退出程序；正常流程应先 `D`、`Off` 和 `Y/N` |

`E` 是 SG100 软件急停，不能代替机械臂的物理急停。

## 6. 数据目录与检查

### 6.1 正式 RAW 数据

确认保存后生成：

```text
recording/vla_raw/
  episode_000000.npz
  episode_000000_media/
    head/rgb/000000.jpg
    right_wrist/rgb/000000.jpg
```

RGB-D 模式还会在各相机目录中生成 `depth/*.png`。

RAW episode 使用 30 Hz 主时间线，但相机、机械臂、手和遥测保留各自时间戳、序号、数据年龄和有效性；不能仅凭数组非空判断数据有效。

### 6.2 控制程序诊断日志

控制程序还会生成独立会话目录：

```text
recording/YYYYMMDD_HHMMSS/
  metadata.json
  real_hardware_errors.jsonl
  mujoco_recording.npz
  arm_joint_comparison.jsonl
```

这些文件用于排查安全停止、跟踪异常、控制器错误以及关节偏差，不等同于正式 VLA RAW episode。

### 6.3 保存后检查

将编号替换为刚保存的 episode：

```bash
python -m sg100_teleop.data_collection.inspect_vla_episode \
  recording/vla_raw/episode_000000.npz
```

至少检查：

- episode 时长和 30 Hz tick 数合理；
- `head`、`right_wrist` RGB 文件数与数据帧索引一致；
- required camera valid 比例正常；
- telemetry、arm command/feedback、hand command/feedback 的 age 和 valid 状态；
- 无 writer error、queue drop、媒体缺失或 sequence 异常；
- 语言指令与实际任务一致；
- desired、effective、measured 字段没有被混淆。

SG100 11D canonical measured feedback 已完成真机验证与 replay 验证。Recorder 不会把
该项目级结论套到所有未来 episode：`training_ready` 仍按 episode 派生，要求
`arm_backend_kind=real`、`hand_backend_kind=real`、robot telemetry present、实际
`hand_feedback_mapping_status=F_FB_VALIDATED`，以及 head + right-wrist RGB production
contract。满足时 status 为 `RAW_REAL_VALIDATED`；camera-only、sim、mixed backend、
identity 缺失或 mapping 未 validated 时仍为 `RAW_CAPTURE_QUARANTINED`。

不要用 command 变化、finite feedback、图像内容或 class name 猜测
`hardware_execution`，也不要因单帧 invalid 改写 episode readiness；逐 transition
有效性由 D2/D3 causal cleaning 处理。

## 7. 常见问题

### 7.1 输入 On 被拒绝

检查：

- D455/D405 是否都显示 READY；
- `--required-cameras` 与实际连接设备一致；
- 控制端是否包含 `--telemetry-publish 127.0.0.1:5560`；
- 两个进程的 UDP 地址和端口是否完全一致；
- 控制程序是否仍在运行并持续发布遥测。

### 7.2 按 A 后手立即停止

查看最新 `real_hardware_errors.jsonl`。常见原因包括：

- `TRACKING_SPIKE`：目标变化超过 1 rad；
- `PER_CYCLE_DELTA_EXCEEDED`：使能后的后续相邻目标帧变化超过 0.20 rad；
- `STALE_AVP_TRACKING`：AVP 跟踪数据过期；
- `TRANSPORT_STALL` / `TRANSPORT_FEEDBACK_MISSING`：SG100 反馈停止；
- `UNEXPECTED_RX_MODE`：返回的手部关节模式异常；
- `CRC_STORM`：串口 CRC 错误集中出现。

恢复前先按 `D`，重新确认跟踪和现场安全，再按 `C` 建立新锚点，最后按 `A`。不要通过增大或删除安全阈值来掩盖跟踪或通信故障。

### 7.3 机械臂突然停止

查看控制终端和 `real_hardware_errors.jsonl`，重点检查：

- `target_stale`；
- `feedback_stale`；
- controller error，例如 `4109`；
- gross follow error；
- 网络断开或发送失败。

出现控制器错误后先按 `D`，按控制器说明解除故障；不要连续按 `A` 尝试强行恢复。

### 7.4 Vision Pro 跟踪帧被拒绝

常见日志包括：

```text
Cannot normalize near-zero vector
target_pose rotation must have determinant +1
target_pose rotation must be orthonormal
```

这通常表示手部骨架暂时无效或姿态矩阵异常。把手重新放回可见区域并保持稳定。如果正在录制，先执行 `D → Off → N` 丢弃异常 episode；恢复有效跟踪后，从 `C → 稳定 → On → A` 重新开始。

### 7.5 MuJoCo 手不动但真机手在动

这可能是 MuJoCo 真机手部显示没有使用有效 canonical 反馈刷新，不应据此判断真机命令没有发送。以控制终端的 `SG100 command debug`、`real_hardware_errors.jsonl` 和正式遥测字段为准，并先撤销真机授权再排查。

### 7.6 相机掉帧或写入失败

检查 USB 带宽、磁盘空间、writer queue drop 和相机 age。正式 RGB 基线优先使用 `--camera-modalities rgb`；不要通过复制上一帧或生成黑图来填补缺失数据。异常 episode 应丢弃后重新采集。

## 8. 一轮多条数据的建议流程

1. 先完成一次不保存的短流程，验证机器人、相机和遥测；
2. 固定任务指令、相机位置、机器人初始姿态和物体初始位置；
3. 每条 episode 只执行一次完整任务；
4. 每条结束后检查报告，再决定 `Y` 或 `N`；
5. 下一条开始前恢复相同初始状态；
6. 不要在同一批数据中临时改变滤波、安全参数、相机模态或坐标映射；
7. 批次结束时先 `D`，再退出 Recorder 和控制程序；
8. 备份整个 `recording/vla_raw` 目录以及相应的控制诊断会话目录。

## 9. 现场检查清单

开始本轮前：

- [ ] 已激活 `avp_sg100` 环境；
- [ ] AVP、RM65、SG100、D455、D405 均已连接；
- [ ] 相机序列号和 SG100 串口正确；
- [ ] RM65 与 AVP 网络可达；
- [ ] 物理急停有效且可触达；
- [ ] `arm-tool-profile=sg100_attached`；
- [ ] Recorder 显示两台相机 READY；
- [ ] 控制端遥测地址为 `127.0.0.1:5560`；
- [ ] 任务指令和初始场景已经确认。

每条 episode：

- [ ] `C` 后跟踪稳定；
- [ ] Recorder 输入 `On`；
- [ ] 确认安全后按 `A`；
- [ ] 完整执行一次任务；
- [ ] 按 `D` 撤销授权；
- [ ] 保留最终稳定状态；
- [ ] Recorder 输入 `Off`；
- [ ] 检查报告后选择 `Y` 或 `N`；
- [ ] 保存后运行 inspector；
- [ ] 恢复下一条数据的初始状态。
