# D455 初始姿态参考预览

独立脚本先用头部 D455 拍摄一张 RGB 照片，保存原图和反色图，然后在实时
预览窗口中叠加固定的反色背景。运行前将机器人摆到需要作为参考的姿态。

在项目根目录运行：

```bash
/home/khan/miniconda3/envs/avp_sg100/bin/python -m \
  sg100_teleop.camera.view_head_camera_reference
```

默认自动选择唯一的 D455；多台 D455 时用 `--serial <头部相机序列号>` 指定。
RGB 为 640×480、30 fps，使用项目现有的 D455 彩色相机曝光和增益配置，
预热 2 秒后拍照。原图、反色图 PNG 和相机信息 JSON 保存到
`recording/head_camera_reference/`，文件名包含时间，每次运行都会保留新照片。

```bash
python -m sg100_teleop.camera.view_head_camera_reference \
  --serial 333422300281 --alpha 0.5 \
  --output-dir recording/head_camera_reference
```

窗口默认 50% 实时画面 + 50% 反色背景。实时画面与初始照片一致时，叠加
结果接近均匀灰色；画面变化会留下轮廓差异。滑块 `Live weight %` 调整
实时画面的权重。

- `O`：叠加预览。
- `L`：实时原始画面。
- `B`：固定反色背景。
- `Q`、`Esc` 或关闭窗口：退出并释放相机。

使用 `--capture-only` 可只拍照保存，适用于没有桌面窗口的环境。

也可以跳过重新拍照，直接加载一张已经反色的参考图并实时预览：

```bash
python -m sg100_teleop.camera.view_head_camera_reference \
  --reference-image recording/head_camera_reference/head_20261020_120000_inverted.png \
  --alpha 0.5
```

已有反色图必须与实时画面的 `--width`、`--height` 完全一致；脚本不会缩放
参考图，以免改变像素对应关系。`--reference-image` 只读取该图，不会重新拍照
或在 `--output-dir` 中生成新文件。

该脚本独立占用 D455 的 RGB 流；使用 RAW recorder 或官方 RealSense Viewer
采集同一台相机前，应关闭此预览窗口。
