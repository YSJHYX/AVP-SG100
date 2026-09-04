# 30 Hz Arm-Hand + Dual RealSense 原始 VLA 数据采集

## 系统架构

本系统把 robot control plane 与 learning/data plane 分成两个独立 Python
进程：

```text
Terminal 1 / teleop control plane
AVP ~60 Hz ──> IK / retarget / safety ──> RM65 + SG100
                       │
                       └── read-only, non-blocking UDP telemetry @ 100 Hz
                                      │ 127.0.0.1:5560
Terminal 2 / data plane              ▼
D455 capture thread ──┐      telemetry subscriber ring
D405 capture thread ──┼──> 30 Hz causal sampler ──> numeric buffer
                      │                         └──> bounded media writer
                      └── HOST CLOCK_MONOTONIC master timeline
```

Recorder 是 passive recorder：不打开 `/dev/ttyUSB0`，不创建第二个
`SG100Retargeter`，不执行 AVP mapping，不发送 `C`/`A`，不调用 RM65 或
SG100 控制 API。teleop runner 中只存在一个可选的 telemetry hook；不包含
camera、episode buffer、On/Off 或 NPZ writer。

## 两终端工作流

Terminal 1 保持原 teleop 参数，并显式打开 telemetry：

```bash
.venv/bin/python -m sg100_teleop.sim.run_avp_arm_hand_sim \
  --ip <AVP_IP> \
  ...existing real backend args... \
  --telemetry-publish 127.0.0.1:5560
```

默认 `--telemetry-publish ""`。关闭时不创建 socket，也不执行 telemetry
序列化；原 control path 保持不变。publisher 采用 localhost UDP，默认
100 Hz、non-blocking、best-effort；发送失败只计数，不向 control loop 抛异常。

Terminal 2：

```bash
.venv/bin/python -m sg100_teleop.data_collection.run_vla_raw_recorder \
  --telemetry 127.0.0.1:5560 \
  --head-serial <D455_SERIAL> \
  --wrist-serial <D405_SERIAL> \
  --instruction "Pick up the object and place it in the target area" \
  --output-dir recording/vla_raw \
  --hz 30
```

如果 CLI 未提供 instruction，则采用代码中的
`DEFAULT_LANGUAGE_INSTRUCTION`；两者都为空时启动时提示输入。`On` 时复制并
冻结 instruction，episode 中途修改不会进入当前 episode。

## 30 Hz master timeline

唯一 dataset clock 是 `CLOCK_MONOTONIC` 上的 30 Hz 绝对 schedule：

```text
tick[k] = episode_start_monotonic_ns + floor(k * 1e9 / 30)
```

实现使用整数有理数运算，不使用循环 `sleep(1/30)`，因此不累计 drift。
`timestamp_ns` 是 scheduled tick，`sampler_wakeup_timestamp_ns` 是线程实际醒来
时间，二者差值保存在 `sampler_jitter_ns`。

所有异步 source 在 tick `t_k` 选择 `source_timestamp <= t_k` 的最新 sample；
绝不按 stream index zip。每层保存 source timestamp、age、sequence 和 valid。
超过 NPZ 内记录的 freshness threshold 时只置 `valid=False`，不删除该 tick。

30 Hz 是 learning/data plane，不是 robot control plane。AVP、RM65 sender /
feedback、SG100 target / vendor scheduler / feedback 保持各自原有频率。

## Cameras

- head：Intel RealSense D455；
- right wrist：Intel RealSense D405；
- 两台均采 RGB8 + Z16 depth；默认严格要求 640×480 @ 30 Hz；
- 每台 pipeline 内以 color stream 对齐 depth；NPZ 的 depth intrinsics 描述对齐后
  实际落盘的 depth image，并记录 `depth_aligned_to=color`；
- profile 无法 resolve 或 active profile 不完全匹配时立即失败并列出设备支持的
  profiles，禁止 SDK silent fallback；
- 每台相机使用独立 capture thread 和 ring；sampler 不做压缩；
- RGB 由独立 bounded writer 以 JPEG quality 97 保存，depth 以原始 `uint16`
  PNG 保存；queue drop/error 明确计数并使 validation FAIL。

每帧同时保存 capture thread 收到 frameset 时的
`host_receive_monotonic_ns`、RealSense `device_timestamp_ms` 和
`frame_number`。D455 与 D405 的 device clock 不被视为共享 clock。当前同步
声明严格为：

```text
DUAL CAMERA SYNCHRONIZATION = HOST MONOTONIC SOFTWARE ALIGNMENT
```

相机到机器人 extrinsics 在没有标定时写
`UNAVAILABLE / NOT_CALIBRATED`，绝不编造。

## Episode 状态机

```text
READY --On--> RECORDING --Off--> PENDING_CONFIRMATION
  ^                                      |
  |--------------- Y / N / Reset --------|
```

- `On` 是唯一开始条件：检查双相机 ready 和 telemetry alive，冻结 language，
  清空临时 numeric buffer，创建 `.pending_<uuid>`，固定 episode start，tick 从
  0 开始。它不会发送 `C`/`A` 或任何 robot command。
- `Off` 是唯一正常停止条件：先记录 stop-request monotonic timestamp，再停止
  生成 tick，flush writer 和 numeric buffer，运行 lightweight validation。
- `Y`：验证过的 NPZ 临时文件 reopen 后才 rename；media 目录 rename；最后
  atomic rename NPZ。只有此时永久 episode index 加一。
- `N`：删除当前 pending，camera/telemetry 保持连接，index 不变。
- `Reset`：除删除 pending 外，还清 telemetry/camera rings、sampler timing 和
  validation state；camera pipeline 保持 warm，teleop 不受影响，index 不变。

`C`、`A`、tracking、robot movement 或 joint movement 都不会自动开始/停止
recording。PENDING 状态只接受 `Y`、`N`、`Reset`。

启动时按文件名扫描 `episode_*.npz`，next index 是已有最大值加一；不是文件
数量。因此已有 0、1、4 时 next=5，且任何历史 episode 都不会被覆盖。

## 文件布局

```text
recording/vla_raw/
  episode_000000.npz
  episode_000000_media/
    head/rgb/000000.jpg
    head/depth/000000.png
    right_wrist/rgb/000000.jpg
    right_wrist/depth/000000.png
```

所有 modality 共享 `N`。媒体 frame index 严格为 `0..N-1`，对应 NPZ 中的
`head_rgb_frame_index`、`head_depth_frame_index`、
`wrist_rgb_frame_index`、`wrist_depth_frame_index`。相机重复 sample 可以被
写到相邻 dataset tick，但其 frame number、age、duplicate 数均诚实保留。

## NPZ schema

Episode/time：`schema_name`、`schema_version`、`episode_index`、
`language_instruction`、`dataset_hz`、start/stop timestamps、
`timestamp_ns[N]`、relative/wakeup/jitter arrays。

RM65：

- `arm_q_goal_rad[N,6]`：AVP/IK 上游目标；
- `arm_qcmd_sent_rad[N,6]`：`rm_movej_canfd` 成功接受的 post-limiter 最终命令；
- qcmd source timestamp/age/sequence/valid；
- `arm_qpos_rad[N,6]`：RM65 backend 实际 feedback，不使用 MuJoCo qpos；
- qpos source timestamp/age/sequence/valid；
- controller error、arm status、feedback channel、sender Hz。

SG100：

- `hand_qcmd_retargeted_canonical_rad[N,11]`：post-retargeter + post-filter
  authoritative desired qcmd；
- `hand_qcmd_effective_canonical_rad[N,11]`：restricted safety/slew/allowlist
  后实际 SDK payload 对应的 canonical target；
- `hand_qcmd_sdk_deg[N,11]`：实际 `SetHandControl` user-domain payload；
- command source timestamp/age/sequence/valid；
- `hand_feedback_sdk_rad[N,11]`：callback 仅做 degree→rad；
- `hand_feedback_modes[N,11]`、feedback timestamp/age/sequence/valid；
- `hand_qpos_canonical_rad[N,11]` 和 canonical-valid。

17D alias 顺序由 `robot_joint_names[17]` 固定为 RM65 6 + SG100 canonical 11：

- `robot_qcmd_17d_rad` 使用实际 arm command + actual/effective hand command；
- `robot_qpos_17d_rad` 使用实际 arm feedback + canonical hand feedback。

若任一部分不能可靠转换，相关 17D valid 为 False，未知值保留 NaN，绝不把
desired command 或 MuJoCo qpos 伪装为 hardware truth。

Teleop status：每 tick 记录 tracking-valid、AVP anchored、motion-enabled、
estop-latched、telemetry-valid 和 telemetry-age；这些只用于记录，不控制 episode。

Camera metadata 包含 model、serial、width/height/fps、depth scale、color/depth
intrinsics、format、extrinsics 状态，并保存每 tick host/device timestamp、frame
number、age、valid。

## Validation 与数据隔离

Off 后打印 duration、N、effective Hz、jitter mean/p95/p99/max、RM65 与 SG100
valid ratio 和 age、双相机 valid ratio/duplicates/max age、UDP sequence gap、
writer backlog/drop/error。source future sample、modality N 不一致、媒体缺失、
writer drop/error 均明确 FAIL。

SG100 `F_fb` 当前仍 pending；禁止 `F_fb = inverse(F_cmd)`。因此所有 episode
强制保存：

```text
dataset_status = RAW_CAPTURE_QUARANTINED
training_ready = False
```

未验证时 hand canonical qpos 为 NaN，`hand_feedback_canonical_valid=False`，
`robot_qpos_17d_valid=False`。本 recorder 只负责 raw capture，不包含 OpenPI、
LeRobot、π0.5 normalization、action chunking、image resize、tokenization 或 prompt
encoding。后续必须另写 `RAW DATA -> Pi0.5 / OpenPI training dataset` adapter。
