# 项目交接文档

## 当前状态

- 仓库目录：`F:\research\guo\hand-eye\calibration`
- 当前分支：`main`
- 第一个提交已创建，提交信息为 `first code by xiangyi wang`。
- 当前项目包含代码、相机内参、标定板图片、标定板 ID 映射、已导出的样本数据和一份已生成标定结果。
- 文档入口在根目录 `README.md`，详细说明在 `docs/`。
- 用户已创建 conda 环境：`hand_eye`，当前已在该环境中补齐 `numpy`、`opencv-contrib-python`、`pyrealsense2` 和 `ur-rtde`。

## 关键结论

- 主入口是 `calib.py`。
- 当前数据集中 `camera_index=0` 是腕部末端相机，`camera_index=1` 是外部固定相机。
- 当前代码会先求腕部相机的 `T_ee_cam_end`，再基于标定板公共坐标系估计固定相机的 `T_base_cam_fixed`。
- 当前项目支持双相机联合流程下的眼在手上和眼在手外结果输出。
- 当前项目暂不支持只用固定相机独立求解眼在手外标定。
- 新增实时采集入口 `scripts/collect_and_calibrate.py`，可加载 `autoHandEye.urp`、通过 RTDE 读取 TCP、采集双 RealSense RGB 图像，并在采集结束后自动调用 `calib.py`。
- `temp/` 目录只作为本地参考材料，已加入 `.gitignore`，主流程不会导入或调用其中脚本。

## 常用命令

查看入口参数：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe calib.py --help
```

运行当前示例标定：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1
```

检查实时采集配置：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py --dry_run
```

定时采集并自动标定：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json
```

等价的完整命令：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe calib.py `
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

## 已知环境注意事项

conda 环境创建命令：

```powershell
conda env update -n hand_eye -f environment.yml
conda activate hand_eye
```

当前环境已验证：

- Python 位于 `F:\Anaconda\Anaconda3\envs\hand_eye\python.exe`。
- `numpy 2.2.6` 可导入。
- `opencv-contrib-python 4.13.0` 可导入。
- `cv2.aruco` 可用。
- `pyrealsense2 2.57.7.10387` 已安装。
- `ur-rtde 1.6.3` 已安装。

本次已完成的验证：

- `F:\Anaconda\Anaconda3\envs\hand_eye\python.exe -m compileall calib.py tools/detect.py tools/inspect_npz_layout.py tools/make_aruco_id_map.py tools/npz.py`
- `F:\Anaconda\Anaconda3\envs\hand_eye\python.exe calib.py --help`
- `powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1 -OutputDir _verify_calib_output`
- 使用 `data/dataset_from_npz`、`configs/` 和 `assets/boards/aruco_id_map_new.json` 完整运行主标定，读取 35 个样本，有效样本 34 个，并成功输出 `calibration_result.json`、`dynamic_end_camera_poses.json` 和 `debug_vis/*.jpg` 到临时验证目录。

新增实时采集相关验证：

- `F:\Anaconda\Anaconda3\envs\hand_eye\python.exe -m compileall calib.py handeye scripts\collect_and_calibrate.py tools`
- `F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py --dry_run`
- `F:\Anaconda\Anaconda3\envs\hand_eye\python.exe -m pip install pyrealsense2 ur-rtde`

尚未做实机验证：当前会话没有连接 UR7e 和 RealSense，所以未实际执行 Dashboard 加载 `autoHandEye.urp`、RTDE TCP 读取和双相机采图。实机前先确认 `configs/live_collection.example.json` 中的机器人 IP 和两台相机序列号。

这个目录在当前 Windows 环境下可能触发 Git 的 `dubious ownership` 检查。可以对本仓库使用 per-command 方式运行 Git：

```powershell
git -c safe.directory=F:/research/guo/hand-eye/calibration status --short --branch
```

如需长期处理，也可以由用户自行决定是否添加全局 safe.directory；自动化流程不建议擅自改全局 Git 配置。

## 后续建议

- 如果要新增单相机眼在手外支持，优先在 `calib.py` 中新增显式 `--mode`，不要复用当前双相机路径的隐式行为。
- 如果要复现实验结果，先固定 OpenCV 版本，因为 `cv2.aruco` 新旧 API 兼容分支可能影响检测细节。
- 如果替换标定板或重拍板图，先重新运行 `tools/make_aruco_id_map.py`，不要沿用旧的 `assets/boards/aruco_id_map_new.json`。
- 如果采集新的 `.npz`，先用 `tools/inspect_npz_layout.py` 确认 key 和 shape，再决定是否可以直接使用 `tools/npz.py`。
- 如果使用实时采集，优先先跑 `--dry_run`，再确认 UR Dashboard、RTDE、RealSense Viewer 占用状态和相机序列号。
