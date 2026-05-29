# UR7e RealSense 手眼标定
本代码基于Xiangyi Wang同学的代码改造，主要添加的实现是半自动化实现手眼标定。
这个仓库用于 UR7e 机械臂与双 RealSense 相机的手眼标定。当前数据中：

- `cam0_wrist.png` / `camera_index=0` 是末端腕部相机，用于眼在手上标定。
- `cam1_main.png` / `camera_index=1` 是外部固定相机，用于估计固定相机到机器人基座的外参。
- `calib.py` 是主标定入口，会同时输出 `T_ee_cam_end`、`T_base_board` 和 `T_base_cam_fixed`。

## 目录结构

```text
.
├── assets/boards/                 # 标定板图片、ID 映射和标注图
├── configs/                       # 相机内参
├── data/                          # 标定样本和 npz 检查输出
├── docs/                          # 使用说明、流程说明、交接文档
├── handeye/                       # RTDE、RealSense 和实时采集流程模块
├── outputs/                       # 已生成的标定结果
├── scripts/                       # 可直接运行的项目脚本
├── tools/                         # 数据转换、检测和标定板映射工具
├── calib.py                       # 主标定入口
├── environment.yml                # hand_eye 环境依赖
└── README.md
```

## 文档

- [docs/quick_use.md](docs/quick_use.md)：精简快速上手流程，包含最常用命令和默认输出位置。
- [docs/1_usage.md](docs/1_usage.md)：项目使用方式、数据格式、常用命令和输出解释。
- [docs/2_hand_eye_calibration_process.md](docs/2_hand_eye_calibration_process.md)：眼在手上与眼在手外的标定流程，以及当前项目支持状态。
- [docs/3_project_handoff.md](docs/3_project_handoff.md)：当前仓库状态、验证记录和后续维护注意事项。
- [docs/4_live_collection_workflow.md](docs/4_live_collection_workflow.md)：UR7e + 双 RealSense 实时采集和自动标定流程。
- [docs/5_live_calibration_output_reference.md](docs/5_live_calibration_output_reference.md)：`live_calibration` 输出文件和 JSON 字段说明。
- [docs/6_hand_eye_result_validation.md](docs/6_hand_eye_result_validation.md)：手眼标定结果的重投影、调试图和一致性检查方法。
- [docs/7_hand_eye_calculation_details.md](docs/7_hand_eye_calculation_details.md)：手眼标定计算细节、数据要求和常见误差来源。
- [docs/9_end_to_end_hand_eye_workflow.md](docs/9_end_to_end_hand_eye_workflow.md)：从建立标定板字典、拍摄样本到计算和检查结果的端到端操作流程。

## 使用方式总览

本项目有两种常用运行方式：

- 使用本地已有样本直接跑标定：适合验证环境、复现当前结果；`data/dataset_from_npz/` 不随 Git 保存。
- 连接 UR7e 和两台 RealSense 实时采集：适合重新采集数据并自动标定。

## 1. 环境准备

先创建并使用本项目的 conda 环境：

```powershell
conda env update -n hand_eye -f environment.yml
conda activate hand_eye
```

如果只想临时调用，也可以直接使用当前机器上的环境 Python：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe calib.py --help
```

## 2. 使用已有样本运行标定

`data/dataset_from_npz/` 是本地样本目录，已加入 `.gitignore`。若目录不存在，需要先由原始 `.npz` 导出，或从本机备份恢复。

在仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1
```

等价的完整命令是：

```powershell
python calib.py `
  --dataset_dir data/dataset_from_npz `
  --fixed_camera_index 1 `
  --end_camera_index 0 `
  --intr_fixed configs/intr_d435i_1920x1080.json `
  --intr_end configs/intr_d405_1280x720.json `
  --board_layout interleaved_checker `
  --grid_cols 20 `
  --grid_rows 15 `
  --tag_size 0.015 `
  --cell_size 0.019 `
  --aruco_dict DICT_6X6_250 `
  --top_left_is_tag true `
  --id_map_json assets/boards/aruco_id_map_new.json `
  --max_reproj_rmse 10.0 `
  --output_dir outputs/calib_output_correct_intr
```

如果本地保留已有输出，可查看 `outputs/calib_output_correct_intr/calibration_result.json`；`outputs/` 默认也不随 Git 保存。

### 输出位置

默认输出目录由 `scripts/run_current_calibration.ps1` 的 `OutputDir` 参数决定：

```text
outputs/calib_output_correct_intr/
```

也可以手动指定：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1 `
  -OutputDir outputs\my_calibration_run
```

输出文件：

```text
outputs/calib_output_correct_intr/
├── calibration_result.json          # 最终标定结果
├── dynamic_end_camera_poses.json    # 每个有效样本对应的动态末端相机位姿
└── debug_vis/                       # 每张图像的检测和重投影调试图
    ├── 000_end.jpg                  # 末端相机调试图
    ├── 000_fixed.jpg                # 固定相机调试图
    └── ...
```

`calibration_result.json` 主要包含：

- `num_total_samples`：读取到的样本总数。
- `num_valid_samples`：通过双相机检测和重投影误差筛选后的有效样本数。
- `handeye_method`：OpenCV 手眼标定方法。
- `board_config`：标定板布局、尺寸和 ID 映射相关配置。
- `T_ee_cam_end`：末端腕部相机到机械臂末端坐标系的外参。
- `T_base_board`：标定板到机器人基座坐标系的外参。
- `T_base_cam_fixed`：外部固定相机到机器人基座坐标系的外参。

`dynamic_end_camera_poses.json` 主要包含：

- `sample_json`：对应的样本 `pose.json`。
- `timestamp`：样本时间戳，若原始数据提供。
- `T_base_cam_end`：该样本时刻末端相机到机器人基座坐标系的位姿。
- `T_cam_fixed_cam_end`：该样本时刻末端相机到固定相机坐标系的相对位姿。

`debug_vis/*.jpg` 中绿色点是检测到的 ArUco 角点，红色点是 PnP 重投影点。两者越接近，单张图像的板位姿估计越可靠。

## 3. 实时采集并自动标定

真机采集入口：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py --dry_run
```

确认 `configs/live_collection.example.json` 中的机器人 IP、RealSense 序列号和采样参数后，运行：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json
```

该流程会加载配置文件中的 URP 程序、通过 RTDE 读取 TCP、打开两路实时预览窗口、保存双相机 RGB 样本，并在采集结束后自动调用 `calib.py`。详细说明见 [docs/4_live_collection_workflow.md](docs/4_live_collection_workflow.md)。

### 常用命令

只检查配置和最终标定命令，不连接机器人或相机：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py --dry_run
```

定时采集并自动标定：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json
```

手动采集并自动标定：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --mode manual
```

手动模式默认按 `c` 保存一次样本，按 `q` 结束采集。Windows 下是单键读取，不需要回车；预览窗口聚焦时也可以直接按 `c` / `q`。

只采集样本，不自动运行标定：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --skip_calibration
```

如果已经在示教器上手动加载并启动配置文件中的 URP 程序，可以跳过 Dashboard 控制，只读取 RTDE 和相机：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --skip_robot_program
```

采集默认打开双相机预览窗口。无桌面显示或不需要观察画面时可关闭：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --no_preview
```

### 实时采集输出位置

实时采集的默认位置由 `configs/live_collection.example.json` 决定：

```json
{
  "dataset_dir": "data/live_capture",
  "calibration": {
    "output_dir": "outputs/live_calibration"
  }
}
```

运行时如果标定输出目录名是 `live_calibration`，程序会自动追加本轮启动时间戳，避免覆盖上一轮结果。

采集数据默认写入：

```text
data/live_capture/
├── capture_session_config.json      # 本次采集使用的配置快照
├── sample_000/
│   ├── cam0_wrist.png               # 末端腕部相机 RGB 图像
│   ├── cam1_main.png                # 外部固定相机 RGB 图像
│   └── pose.json                    # RTDE TCP、关节角和图像元数据
├── sample_001/
│   └── ...
└── ...
```

如果在配置中把某台相机的 `enable_depth` 设为 `true`，样本目录中还会保存对应深度图，例如：

```text
sample_000/
├── depth0_wrist.png
└── depth1_main.png
```

深度图按 RealSense 原始 `z16` 保存，`pose.json` 中会记录 `depth_file` 和 `depth_scale_m`，后续需要真实深度值时用 `depth_m = raw_value * depth_scale_m` 转换。

实时采集结束后的自动标定结果默认写入：

```text
outputs/live_calibration_YYYYMMDD_HHMMSS/
├── calibration_result.json
├── dynamic_end_camera_poses.json
└── debug_vis/
```

这三个输出文件的含义与“使用已有样本运行标定”中的说明一致。

### `pose.json` 字段

每个实时采集样本都会生成一个 `pose.json`：

```json
{
  "sample_index": 0,
  "timestamp": "2026-05-21T00:00:00+00:00",
  "tcp_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "tcp_pose_fields": ["x", "y", "z", "rx", "ry", "rz"],
  "joint_angles": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "robot": {
    "host": "192.168.1.88",
    "tcp_source": "rtde_receive.getActualTCPPose"
  },
  "images": [
    {
      "file": "cam0_wrist.png",
      "camera_index": 0,
      "camera_name": "wrist_camera",
      "role": "end",
      "serial": "419122270341",
      "intrinsics_file": "configs/intr_d405_1280x720.json"
    },
    {
      "file": "cam1_main.png",
      "camera_index": 1,
      "camera_name": "main_camera",
      "role": "fixed",
      "serial": "254522072098",
      "intrinsics_file": "configs/intr_d435i_1920x1080.json"
    }
  ]
}
```

`calib.py` 会使用其中的 `tcp_pose` 和 `images` 字段。其他字段用于追踪采集环境、相机序列号和后续调试。

## 4. 关键配置说明

实时采集前重点检查 `configs/live_collection.example.json`：

- `robot.host`：UR7e 的 IP 地址。
- `robot.program`：示教器中已经保存的 URP 文件名，当前示例配置为 `autoHandEye2.urp`。
- `cameras[].serial`：RealSense 序列号，建议实机前确认。
- `cameras[].color_width` / `color_height` / `fps`：RGB 流参数。
- `cameras[].intrinsics_path`：相机启动后写入 active profile 内参的位置，也是后续标定使用的内参路径。
- `cameras[].enable_depth`：是否同时采集深度图；当前标定只使用 RGB，深度接口是为后续功能预留。
- `preview.enabled` / `preview.scale`：是否打开实时预览窗口，以及预览缩放比例；不影响保存图像分辨率。
- `sampling.mode`：`timed` 定时采集或 `manual` 手动采集。
- `sampling.interval_s`：定时模式下的采样间隔。
- `sampling.max_samples`：采样数量上限。
- `calibration`：采集结束后传给 `calib.py` 的标定参数。

默认相机约定：

- `camera_index=0` / `role=end` / `cam0_wrist.png`：末端腕部相机。
- `camera_index=1` / `role=fixed` / `cam1_main.png`：外部固定相机。

## 5. 实机前检查

- UR7e 和 PC 网络互通，PC 能访问 `robot.host`。
- UR 控制柜已启用 Dashboard server 和 RTDE。
- `configs/live_collection.example.json` 中的 `robot.program` 已经保存在示教器中。
- 两台 RealSense 没有被 RealSense Viewer 或其他程序占用。
- `configs/live_collection.example.json` 中的 RealSense 序列号、分辨率和 FPS 与实际设备一致。
- 标定板在每个有效采样姿态中都能同时被两台相机看到。
