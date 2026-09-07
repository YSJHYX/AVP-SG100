# 30 Hz Arm-Hand + RealSense 原始 VLA 数据采集

## 系统架构

本系统把 robot control plane 与 learning/data plane 分成两个独立 Python
进程：

```text
Terminal 1 / teleop control plane
AVP ~60 Hz ──> IK / retarget / safety ──> RM65 + SG100
                       │
                       └── read-only UDP telemetry (requested max 100 Hz)
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
序列化；原 control path 保持不变。publisher 采用 localhost UDP、non-blocking、
best-effort；发送失败只计数，不向 control loop 抛异常。默认 100 Hz 是 requested
maximum/throttle target，不是 guaranteed actual rate。

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

上述 production 默认等价于
`--required-cameras head,right_wrist`，仍同时要求 D455、D405 和 telemetry。

D405 尚未到位时，正式支持 D455 head-only camera bring-up：

```bash
.venv/bin/python -m sg100_teleop.data_collection.run_vla_raw_recorder \
  --required-cameras head \
  --camera-modalities rgb \
  --head-serial <D455_SERIAL> \
  --telemetry optional \
  --instruction "Validate D455 raw capture" \
  --output-dir recording/vla_raw \
  --hz 30
```

默认仍为 `--camera-modalities rgb,depth` 以保持旧 H1 命令兼容；上面的当前 H1
SOP 显式选择 RGB-only。

`--telemetry optional` 是显式 camera-only 模式：不创建 telemetry socket，`On`
不要求 publisher alive，所有 robot joint value arrays 为 NaN、robot valid 全为
False。它仍写入 `hardware_execution=False`、
`robot_telemetry_present=False`、`training_ready=False` 和
`dataset_status=RAW_CAPTURE_QUARANTINED`，不生成任何 robot state/action。

若要使用 D455 + AVP sim telemetry 验证 command/time synchronization，保持
head-only camera 配置并传入正常 endpoint。当前 H2 推荐关闭 depth，只验证
D455 RGB 与 command/time alignment：

```bash
.venv/bin/python -m sg100_teleop.data_collection.run_vla_raw_recorder \
  --required-cameras head \
  --camera-modalities rgb \
  --head-serial <D455_SERIAL> \
  --telemetry 127.0.0.1:5560 \
  --instruction "Validate command and D455 time alignment"
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
此外 valid 还要求 `source_timestamp_ns >= episode_start_monotonic_ns`（On 边界
floor）：pre-On 产生的 source sample 即使仍被 latest-before 选中，也只能以
invalid 出现在 RAW 中，绝不作为正式 sample 使用。

30 Hz 是 learning/data plane，不是 robot control plane。AVP、RM65 sender /
feedback、SG100 target / vendor scheduler / feedback 保持各自原有频率。

External telemetry publisher 没有独立 100 Hz thread；它由 teleop runner main loop
达到 throttle period 时调用。因此 `--telemetry-publish-hz 100` 只规定 requested
maximum，actual packet rate 会受 caller loop 限制。每 tick 保存
`telemetry_publish_sequence`、`telemetry_publish_timestamp_ns` 与 age；summary 用
sequence delta / publish monotonic elapsed time 估计 `telemetry_effective_hz`，而不是
用 30 Hz recorder tick 数冒充 publish rate。另报告 telemetry age 的
mean/p95/p99/max 和 sequence gap。v3 episode 保存 requested 与 derived effective
rate；旧 v1/v2 可从 tick-age 重建 publish timestamp。

## Cameras

- head：Intel RealSense D455；
- right wrist：Intel RealSense D405；
- production 默认启用 `head,right_wrist`；partial bring-up 只启用 `head`，且完全
  不实例化 D405 pipeline；
- `--camera-modalities` 正式支持 `rgb` 与 `rgb,depth`，默认 `rgb,depth` 以保持
  旧命令兼容；RGB 是 VLA recorder 必需模态，`depth`/depth-only 会被 CLI 拒绝；
- RGB-only 时 pipeline 只 enable RGB8，不 enable depth stream、不创建 align 或
  depth ndarray，不压缩/写入 depth PNG；这会实际降低 USB、CPU、IO 与容量占用；
- RGB-D 时采 RGB8 + raw Z16 `uint16`，禁止 uint8 conversion、normalization、
  colormap 或 threshold-to-binary；默认严格要求 640×480 @ 30 Hz；
- RGB-D pipeline 内以 color stream 对齐 depth；NPZ 的 depth intrinsics 描述对齐后
  实际落盘的 depth image，并记录 `depth_aligned_to=color`；
- profile 无法 resolve 或 active profile 不完全匹配时立即失败并列出设备支持的
  profiles，禁止 SDK silent fallback；
- 每台相机使用独立 capture thread 和 ring；sampler 不做压缩；
- RGB 由独立 bounded writer 以 JPEG quality 97 保存，depth 以原始 `uint16`
  PNG 保存；queue drop/error 明确计数并使 validation FAIL。

### Camera Modalities

当前 π0.5 / OpenPI Arm-Hand baseline 的视觉 observation 是 head RGB 与 right
wrist RGB。Depth 不直接送入当前 π0.5 image encoder，但作为可选的 raw geometric
provenance 保留，供研发、标定与质量诊断使用。关闭 depth 不代表删除 depth
capability。

研发 RGB-D：

```bash
--camera-modalities rgb,depth
```

正式 π0.5 RGB-only：

```bash
--required-cameras head,right_wrist --camera-modalities rgb
```

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

- `On` 是唯一开始条件：检查所有 required camera ready；非 camera-only 模式还
  检查 telemetry alive；然后冻结 language，
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

RGB-only episode 仅创建启用 role 的 RGB 目录：

```text
episode_000000_media/
  head/rgb/
  right_wrist/rgb/       # 仅 dual-camera 配置存在
```

不会创建 depth 目录或任何全零、黑图、空图、上一帧复制等占位文件。

所有 modality 共享 `N`。媒体 frame index 严格为 `0..N-1`，对应 NPZ 中的
`head_rgb_frame_index`、`head_depth_frame_index`、
`wrist_rgb_frame_index`、`wrist_depth_frame_index`。相机重复 sample 可以被
写到相邻 dataset tick，但其 frame number、age、duplicate 数均诚实保留。

head-only episode 只创建 `head/rgb` 与 `head/depth`。不会创建虚假
`right_wrist` 图片；wrist schema 字段继续保留，但每 tick 的 valid=False、frame
index/frame number/host timestamp/age=-1、device timestamp=NaN。validator 按
`required_cameras` 验证启用角色，因此缺少 disabled wrist 目录不是 corruption。
如果同时使用 RGB-only，head-only episode 只创建 `head/rgb`。

## NPZ schema

Episode/time：`schema_name`、`schema_version`、`episode_index`、
`language_instruction`、`dataset_hz`、start/stop timestamps、
`timestamp_ns[N]`、relative/wakeup/jitter arrays。

RM65：

- `arm_q_goal_rad[N,6]`：DESIRED；AVP wrist mapping + IK 上游 absolute target；
- `arm_qcmd_sent_rad[N,6]`：`rm_movej_canfd` 成功接受的 post-limiter 最终命令；
- qcmd source timestamp/age/sequence/valid；
- `arm_qpos_rad[N,6]`：RM65 backend 实际 feedback，不使用 MuJoCo qpos；
- qpos source timestamp/age/sequence/valid；
- controller error、arm status、feedback channel、sender Hz。

SG100：

- `hand_qcmd_retargeted_canonical_rad[N,11]`：DESIRED；AVP tracking → canonical
  retargeter → `JointCommandFilter` 后 authoritative target；
- `hand_qcmd_effective_canonical_rad[N,11]`：restricted safety/slew/allowlist
  后实际 SDK payload 对应的 canonical target；
- `hand_qcmd_sdk_deg[N,11]`：实际 `SetHandControl` user-domain payload；
- command source timestamp/age/sequence/valid；
- `hand_feedback_sdk_rad[N,11]`：callback 仅做 degree→rad；
- `hand_feedback_modes[N,11]`、feedback timestamp/age/sequence/valid；
- `hand_qpos_canonical_rad[N,11]` 和 canonical-valid。

17D 顺序由 `robot_joint_names[17]` 固定为 RM65 6 + SG100 canonical 11：

- `robot_desired_qcmd_17d_rad` = `arm_q_goal_rad` +
  `hand_qcmd_retargeted_canonical_rad`，是 DESIRED/INTENT；
- `robot_qcmd_17d_rad` = `arm_qcmd_sent_rad` +
  `hand_qcmd_effective_canonical_rad`，是 FINAL EFFECTIVE BACKEND COMMAND；
- `robot_qpos_17d_rad` = `arm_qpos_rad` + `hand_qpos_canonical_rad`，是
  MEASURED/FEEDBACK。

Desired17 只有两侧 desired 都来自 fresh、tracking-valid、anchored telemetry 且
finite 时 valid；否则整行 NaN。Effective17 缺失时绝不以 desired fallback。
Simulation qpos、MuJoCo qpos、desired qcmd 均不得冒充 real measured qpos。v3 用
episode metadata 冻结 `robot_desired_qcmd_semantics`、
`robot_effective_qcmd_semantics`、`robot_qpos_semantics`，不在每 tick 重复长字符串。

若任一部分不能可靠转换，相关 17D valid 为 False，未知值保留 NaN，绝不把
desired command 或 MuJoCo qpos 伪装为 hardware truth。

Teleop status：每 tick 记录 tracking-valid、AVP anchored、motion-enabled、
estop-latched、telemetry-valid 和 telemetry-age；这些只用于记录，不控制 episode。

Camera metadata 包含 model、serial、width/height/fps、depth scale、color/depth
intrinsics、format、extrinsics 状态，并保存每 tick host/device timestamp、frame
number、age、valid。`head_camera_present`/`wrist_camera_present` 明确区分 enabled
role；head-only 时仍保留与 dual-camera 相同的 NPZ keys 和数组形状。

schema v2 新增 episode metadata：`camera_modalities`、`record_depth`、
`head_depth_present`、`wrist_depth_present`。RGB-only 不删除任何 depth tick field；
每个 disabled depth 的 `depth_valid=False`，frame index/host timestamp/frame
number/age 均为 `-1`，device timestamp 为 `NaN`。RGB-D 时新增的独立 depth
device timestamp/frame number 来自 aligned depth frame；host timestamp 仍是同一
frameset 在 capture thread 的接收时刻。validator 会把缺少这些 v2 字段的 v1 H1
episode 按旧 RGB-D 语义读取。

schema v3 新增 `robot_desired_qcmd_17d_rad[N,17]`、对应 valid、显式 telemetry
publish timestamp、requested/effective rate metadata 和三层 semantics metadata。
Validator 按 version 处理：v1/v2 不要求 v3 字段；v3 必须检查 shape、canonical
ordering、valid/NaN contract、desired 与两侧 source fields 的对应关系。它不会检查
desired 等于 effective，因为 safety limiter/allowlist 本来允许两者不同；但会检查
effective17 与 effective component fields、measured17 与 measured component fields
严格一致。

关闭 depth 不改变 `training_ready=False` 或
`dataset_status=RAW_CAPTURE_QUARANTINED`；当前 blocker 是 SG100 `F_fb` 与真实
hardware chain readiness，而不是 depth。

当前 recorder 无可信的 backend-identity attestation，因此 `hardware_execution`
保守固定为 False；即使 telemetry 来自后续 real runner，也不能仅凭数据值推断为
True。正式发布 real dataset 前需另行建立可审计的 hardware execution provenance。

## D455 depth diagnostic probe

独立只读工具只连接 RealSense，不连接 robot，也不写正式 episode：

```bash
.venv/bin/python -m sg100_teleop.camera.diagnose_realsense_depth \
  --serial 333422300281 \
  --width 640 --height 480 --fps 30 --seconds 10
```

probe 在同一 pipeline 内比较 native raw depth 与 `align-to-color` depth，采集
100~300 帧。每帧统计全图 valid/zero ratio、非零 Z16 的 min/median/p5/p95/max、
按 `depth_scale` 换算的 meter，以及 x/y=25%~75% 中心 ROI valid ratio 与 median
meters；最后汇总 raw/aligned valid ratio 的 mean/p5/p50/p95/min/max 和
aligned-minus-raw delta。

它还只读打印 model、serial、firmware、USB type、depth scale，以及设备支持时的
visual preset、emitter、laser power、min/max distance；不修改任何 option。可选
`--preview-dir recording/depth_diagnostics` 会生成 colormap 与 valid mask，并明确
标记为 `NOT RAW DATA`，不会覆盖或冒充 Z16。

## Validation 与数据隔离

Off 后打印 duration、N、dataset effective Hz、jitter mean/p95/p99/max、三种 17D
valid ratio、RM65/SG100 valid ratio 和 age、相机 valid/duplicates/max age、telemetry
effective Hz 与 age mean/p95/p99/max、UDP sequence gap、writer backlog/drop/error。
source future sample、17D provenance mismatch、modality N 不一致、媒体缺失、writer
drop/error 均明确 FAIL。

只读检查已有 episode：

```bash
.venv/bin/python -m sg100_teleop.data_collection.inspect_vla_episode \
  recording/vla_raw/episode_000001.npz
```

SG100 `F_fb` 当前仍 pending；禁止 `F_fb = inverse(F_cmd)`。因此所有 episode
强制保存：

```text
dataset_status = RAW_CAPTURE_QUARANTINED
training_ready = False
```

未验证时 hand canonical qpos 为 NaN，`hand_feedback_canonical_valid=False`，
`robot_qpos_17d_valid=False`。本 recorder 只负责 raw capture，不包含 OpenPI、
LeRobot、π0.5 normalization、action chunking、image resize、tokenization 或 prompt
encoding。

## 下游管线：RAW → Curated → π0.5-AHFE

RAW 是不可变的完整原始证据：native timestamp/sequence/payload、telemetry /
writer / sampler diagnostics 与可选 depth 只保存在这一层，供 replay/audit
无限追溯。下游分两层，完整 contract 见 `sg100_teleop/README.md`
「π0.5-AHFE Training Data Contract」：

```text
recording/vla_raw/       RAW（本文档；schema v1–v3）
      │ build_curated_episode：validation + cleaning + causal transition
      ▼
recording/vla_curated/   CURATED v1 FINAL（30 Hz valid-only transition；
      │                    observation=RGB+qpos17，action=qcmd17）
      ▼
datasets/pi05_ahfe/      π0.5-AHFE model-ready（ADAPTER: NOT IMPLEMENTED YET；
                         dataset NOT GENERATED YET）
```

- 清洗入口：

```bash
.venv/bin/python -m sg100_teleop.data_collection.build_curated_episode \
  recording/vla_raw/episode_000001.npz \
  --output-dir recording/vla_curated
```

  默认只写 training-valid transition；0 accepted 时拒绝输出（`--allow-empty`
  可强制写空 episode 供联调）。被拒原因只进入 `cleaning_report.json`。
- **component-wise causality（v1 freeze）**：ARM 与 HAND 分别要求
  `qpos_ts[k] <= qcmd_ts[k] < qpos_ts[next_state_tick]`，两侧同时 PASS 才接受；
  next 侧 timestamp 经 `next_state_tick_index` 从 RAW 读取，不在 Curated 重复
  保存。unified `state/action/next_state_timestamp_ns` = 各侧组件 timestamp 的
  max，是展示层，不替代 component 验证。有效 action 不存在时 reject，绝不
  fallback 到 desired。
- **segment**：metadata `segment_offsets` 标记连续段；tick 不连续或 estop /
  tracking / telemetry validity loss 即切分，action horizon 禁止跨 segment。
  RAW 未记录 C re-anchor generation event（limitation）；正式 SOP 禁止 episode
  中按 `C`。
- **On/Off 精确边界**：On 前允许 warmup/tracking/C，但 0 row、0 media enqueue；
  正式 source sample 必须 `source_timestamp_ns >= episode_start_monotonic_ns`
  （pre-On ring 数据只能以 invalid 出现，不静默使用）。Off 接受后立即固定
  `episode_stop_request_monotonic_ns`，禁止新 tick / row / media enqueue，
  writer 只 flush Off 前已 enqueue 的内容。
- Curated 不保存 RAW 的任何派生量（relative timestamp、age、skew/dt 数组、
  desired/effective norm、native Hz 统计），inspector 从保留的 4 个 native
  source timestamp 运行时推导；native event rate 从 Curated NOT EXACTLY
  RECOVERABLE，只能以 RAW sequence/timestamp 证据估计。
- Curated 禁止重复表示：17D 是唯一 state/action 存储，AHFE arm/hand 分解由
  `arm_slice=[0,6]` / `hand_slice=[6,17]` slice metadata 确定性完成；
  SDK degree、MODE7 raw feedback、telemetry sequence、writer diagnostics、
  depth 不进入 Curated；RGB media 按 RAW frame identity 去重保存。
- 修改 raw/curated/training 目录、训练字段、17D ordering 或 camera modality
  时，必须同步更新 README 的 Training Data Contract 与本文档。
