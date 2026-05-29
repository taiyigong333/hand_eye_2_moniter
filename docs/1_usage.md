# 项目使用说明

## 1. 项目目标

本项目用于基于 ArUco 标定板、UR7e TCP 位姿和双相机同步图像，完成以下外参估计：

- `T_ee_cam_end`：末端腕部相机到机械臂末端坐标系的变换，属于眼在手上结果。
- `T_base_board`：标定板到机器人基座坐标系的变换。
- `T_base_cam_fixed`：外部固定相机到机器人基座坐标系的变换，属于眼在手外结果。

代码约定 `T_A_B` 表示从 `B` 坐标系变换到 `A` 坐标系，即 `p_A = T_A_B @ p_B`。

## 2. 主要文件

- `calib.py`：主标定脚本，读取数据集、相机内参、标定板配置并输出标定结果。
- `tools/npz.py`：将原始 `.npz` 数据导出为 `data/dataset_from_npz/sample_xxx/` 格式。
- `tools/inspect_npz_layout.py`：检查 `.npz` 内部 key、shape 和图像样例，用于确认数据布局。
- `tools/detect.py`：对单张图片尝试多个 ArUco / AprilTag 字典，辅助确认标定板字典类型。
- `tools/make_aruco_id_map.py`：根据一张清晰标定板图片生成 marker ID 到棋盘格行列的映射。
- `scripts/run_current_calibration.ps1`：使用本地样例数据运行完整标定。
- `scripts/collect_and_calibrate.py`：通过 UR RTDE 和双 RealSense 实时采集样本，并在结束后自动调用标定。
- `configs/live_collection.example.json`：实时采集配置示例。
- `configs/intr_d405_1280x720.json`：末端 D405 相机内参示例。
- `configs/intr_d435i_1920x1080.json`：固定 D435i 相机内参示例。
- `assets/boards/aruco_id_map_new.json`：当前标定板的 ID 映射。
- `data/dataset_from_npz/`：本地已导出的标定样本，已加入 `.gitignore`，不随 Git 保存。
- `outputs/calib_output_correct_intr/`：本地已生成的一份标定结果和调试可视化，`outputs/` 默认不随 Git 保存。

## 3. 环境依赖

使用用户已创建的 `hand_eye` conda 环境。若环境已存在，使用下面命令补齐依赖；若环境不存在，可先用 `conda env create -f environment.yml` 创建。

```powershell
conda env update -n hand_eye -f environment.yml
conda activate hand_eye
```

`environment.yml` 当前包含：

- `python=3.10`
- `numpy`
- `opencv-contrib-python`
- `pyrealsense2`
- `ur-rtde`

说明：

- 必须使用 `opencv-contrib-python`，因为代码依赖 `cv2.aruco`。
- 实时采集依赖 `pyrealsense2` 和 `ur-rtde`；只跑已有样本标定时不需要连接相机或机器人。
- `pupil-apriltags` 在 `calib.py` 中只作为旧版 AprilTag 分支的可选依赖，当前 ArUco 主流程不需要它。
- 如果只想临时运行，也可以直接调用环境内 Python：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe calib.py --help
```

## 4. 数据格式

`calib.py` 期望 `--dataset_dir` 下递归存在若干 `pose.json`，每个样本目录同时包含两张图像。当前样本格式如下：

```json
{
  "tcp_pose": [x, y, z, rx, ry, rz],
  "tcp_pose_fields": ["x", "y", "z", "rx", "ry", "rz"],
  "images": [
    {
      "file": "cam0_wrist.png",
      "camera_index": 0,
      "camera_name": "wrist_camera"
    },
    {
      "file": "cam1_main.png",
      "camera_index": 1,
      "camera_name": "main_camera"
    }
  ]
}
```

关键约束：

- `tcp_pose` 的平移默认按米解释；如果原始数据是毫米，需要使用 `--translation_scale 0.001`。
- `rx, ry, rz` 按 Rodrigues 旋转向量解释。
- `camera_index=0` 当前代表腕部相机，命令中传给 `--end_camera_index`。
- `camera_index=1` 当前代表固定相机，命令中传给 `--fixed_camera_index`。
- 每个有效样本中，两台相机都必须能看到标定板，且检测到的 marker 数量不能少于 `--min_tags`。

如果原始数据是 `.npz`，先检查布局：

```powershell
python tools/inspect_npz_layout.py data.npz --index 0 --out_dir data/npz_inspect
```

如果 `.npz` 包含当前脚本期望的 `tcp_poses`、`joint_angles`、`images_main`、`images_wrist`，可以导出数据集：

```powershell
python tools/npz.py data.npz --out_dir data/dataset_from_npz
```

## 5. 生成标定板 ID 映射

当前项目使用 `interleaved_checker` 标定板，也就是 ArUco marker 与黑色方块交错排列。若换了标定板、重新拍了板图或 ID 位置发生变化，需要重新生成 `id_map_json`。

示例命令：

```powershell
python tools/make_aruco_id_map.py assets/boards/board.jpg `
  --aruco_dict DICT_6X6_250 `
  --grid_cols 20 `
  --grid_rows 15 `
  --top_left_is_tag true `
  --corners "589,77 3757,203 3849,2603 444,2592" `
  --tag_size 0.015 `
  --cell_size 0.019 `
  --max_cell_dist 0.55 `
  --out assets/boards/aruco_id_map_new.json `
  --vis assets/boards/aruco_id_map_new.jpg
```

参数说明：

- `--corners` 是标定板有效棋盘区域四角像素，顺序为左上、右上、右下、左下。
- `--grid_cols` / `--grid_rows` 包含 ArUco 格和黑色方块格的总列数、总行数。
- `--tag_size` 是 marker 外层黑色正方形边长，单位米。
- `--cell_size` 是单个棋盘格边长，单位米。
- `--top_left_is_tag true` 表示左上角第一个格子是 ArUco marker。

## 6. 运行主标定

本地 `data/dataset_from_npz/` 数据对应的完整命令：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1
```

等价的完整命令：

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

常用可调参数：

- `--max_reproj_rmse`：单样本重投影误差阈值，超过则跳过该样本。
- `--min_tags`：每张图至少需要使用的 marker 数量，默认 4。
- `--handeye_method`：OpenCV 手眼标定方法，可选 `tsai`、`park`、`horaud`、`andreff`、`daniilidis`。
- `--translation_scale`：TCP 平移单位缩放，默认 1.0。

## 7. 输出解释

主脚本输出目录包含：

- `calibration_result.json`：最终标定结果。
- `dynamic_end_camera_poses.json`：每个有效样本下动态末端相机在基座和固定相机坐标系中的位姿。
- `debug_vis/*.jpg`：每张图像的检测角点和 PnP 重投影调试图。

`calibration_result.json` 中主要字段：

- `num_total_samples`：读取到的样本数。
- `num_valid_samples`：通过双相机检测和 RMSE 过滤后的样本数。
- `T_ee_cam_end`：末端相机到末端执行器坐标系的变换。
- `T_base_board`：标定板到机器人基座坐标系的变换。
- `T_base_cam_fixed`：固定相机到机器人基座坐标系的变换。

当前仓库中的示例结果显示总样本数为 35，有效样本数为 34。

## 8. 最小自检

修改代码后建议至少运行：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe -m compileall calib.py tools/detect.py tools/inspect_npz_layout.py tools/make_aruco_id_map.py tools/npz.py
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe calib.py --help
```

如果改动涉及标定逻辑，再用当前数据跑一次第 6 节命令，并检查：

- 有效样本数是否合理。
- `debug_vis` 中绿色检测点和红色重投影点是否基本重合。
- `T_base_board`、`T_base_cam_fixed` 的一致性统计是否没有明显恶化。

## 9. 实时采集入口

配置文件：

```text
configs/live_collection.example.json
```

配置中包含：

- `robot.host`：UR7e IP。
- `robot.program`：示教器中的 URP 程序名，当前示例配置为 `autoHandEye2.urp`。
- `cameras`：两台 RealSense 的序列号、分辨率、图像文件名、内参保存路径和可选深度图开关。
- `preview`：采集时的双相机 OpenCV 预览窗口，默认开启。
- `sampling`：`timed` 定时采集或 `manual` 手动采集。
- `calibration`：采集结束后传给 `calib.py` 的参数。

先检查配置和最终标定命令：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py --dry_run
```

定时采集并自动标定：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json
```

手动模式：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --mode manual
```

采集默认会打开两个预览窗口。手动模式默认按 `c` 保存一次，按 `q` 结束；预览窗口聚焦时也能直接使用这两个按键。无桌面显示时可加 `--no_preview`。更多细节见 `docs/4_live_collection_workflow.md`。
