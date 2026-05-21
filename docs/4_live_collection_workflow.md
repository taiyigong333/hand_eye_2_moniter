# 实时采集与自动标定流程

## 1. 目标

本流程用于真机采集新的手眼标定数据：

1. PC 通过 UR Dashboard 加载并启动示教器中的 `autoHandEye.urp`。
2. PC 通过 RTDE 读取 UR7e 当前 TCP 位姿。
3. PC 同时读取两台 RealSense 的 RGB 图像，并可按配置预留深度图。
4. 采集期间打开两路 OpenCV 预览窗口，便于观察标定板是否同时在视野内。
5. 按固定时间间隔或手动按键保存一组 `TCP + 双相机图像`。
6. 采集结束后自动调用现有 `calib.py` 完成标定。

`temp/` 下的脚本只作为参考材料，本项目主流程不会导入或调用其中代码。

## 2. 入口文件

- `scripts/collect_and_calibrate.py`：实时采集和自动标定入口。
- `configs/live_collection.example.json`：示例配置，包含机器人 IP、相机序列号、采样方式、内参输出和 `calib.py` 参数。
- `handeye/robot.py`：RTDE 只读 TCP、Dashboard 加载/启动 URP。
- `handeye/realsense.py`：双 RealSense color/depth 流启动、RGB 采集、active profile 内参读取。
- `handeye/preview.py`：采集期间刷新双相机 OpenCV 预览窗口。
- `handeye/dataset.py`：保存为 `calib.py` 已支持的 `sample_xxx/pose.json` 数据格式。
- `handeye/workflow.py`：串联加载 URP、采样、写内参和自动标定。

## 3. 环境

使用本项目的 `hand_eye` conda 环境：

```powershell
conda env update -n hand_eye -f environment.yml
conda activate hand_eye
```

实时采集额外依赖：

- `pyrealsense2`：访问 RealSense。
- `ur-rtde`：提供 `rtde_receive.RTDEReceiveInterface`。

当前 Windows 环境中也可以直接调用：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py --dry_run
```

## 4. 配置重点

先复制或直接修改 `configs/live_collection.example.json`：

- `robot.host`：UR7e IP，当前示例为 `192.168.1.88`。
- `robot.program`：示教器中已存在的 URP，当前为 `autoHandEye.urp`。
- `cameras[].serial`：两台 RealSense 序列号。建议实机前确认，不建议长期依赖自动枚举。
- `cameras[].intrinsics_path`：启动相机后会把 active color profile 的内参写成 `calib.py` 可读格式。
- `cameras[].enable_depth`：设为 `true` 时同时保存深度图；当前标定仍只使用 RGB。
- `preview.enabled`：默认 `true`，采集时打开两路 RGB 预览窗口；命令行可用 `--no_preview` 关闭。
- `preview.scale`：预览缩放比例，默认 `0.5`，只影响显示尺寸，不影响保存图像。
- `sampling.mode`：`timed` 或 `manual`。
- `calibration`：采集结束后传给 `calib.py` 的参数。

当前约定：

- `camera_index=0` / `role=end` / `cam0_wrist.png` 是末端腕部相机。
- `camera_index=1` / `role=fixed` / `cam1_main.png` 是外部固定相机。

## 5. 运行方式

先 dry-run 检查配置和最终标定命令：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py --dry_run
```

定时采集：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json
```

手动采集：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --mode manual
```

手动模式默认按 `c` 保存一次，按 `q` 结束采集。预览开启时可在窗口中按键；Windows 终端也支持单键读取，不需要回车；关闭预览且非 Windows 时会回退到命令行输入。

默认会打开两个 OpenCV 预览窗口。手动模式下，预览窗口或终端聚焦时按 `c` 保存、按 `q` 结束；定时模式下也可以按 `q` 提前结束采集。若当前环境没有桌面显示，使用：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --no_preview
```

只采集不自动标定：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --skip_calibration
```

如果 URP 已经在示教器上手动启动，只想读取 RTDE 和相机：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --skip_robot_program
```

## 6. 输出

默认样本目录：

```text
data/live_capture/sample_000/
├── cam0_wrist.png
├── cam1_main.png
└── pose.json
```

`pose.json` 会包含：

- `tcp_pose`：RTDE 读取的 `[x, y, z, rx, ry, rz]`。
- `joint_angles`：若 RTDE 可读取关节角则保存。
- `images`：两台相机的图像文件、`camera_index`、`role`、序列号和内参文件。
- `depth_file` / `depth_scale_m`：仅在对应相机 `enable_depth=true` 且采到深度帧时出现，深度图按 RealSense 原始 `z16` 保存。

默认标定输出：

```text
outputs/live_calibration/
├── calibration_result.json
├── dynamic_end_camera_poses.json
└── debug_vis/
```

## 7. 实机注意事项

- UR 控制柜需要启用 Dashboard server 和 RTDE 访问，且 PC 能访问 `robot.host`。
- `autoHandEye.urp` 必须已保存在示教器中；Dashboard 的 `load autoHandEye.urp` 只加载示教器已有程序。
- 两台 RealSense 不要同时被 RealSense Viewer 或其他进程占用。
- 若换相机、分辨率或 FPS，必须重新读取并保存内参；当前流程默认会在相机启动后覆盖配置里的内参文件。
- 标定要求每个有效样本中两台相机都能看到标定板，建议采集 15 到 30 组以上姿态变化明显的样本。
