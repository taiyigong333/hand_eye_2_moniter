# UR7e RealSense 手眼标定

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
├── outputs/                       # 已生成的标定结果
├── scripts/                       # 可直接运行的项目脚本
├── tools/                         # 数据转换、检测和标定板映射工具
├── calib.py                       # 主标定入口
├── environment.yml                # hand_eye 环境依赖
└── README.md
```

## 文档

- [docs/1_usage.md](docs/1_usage.md)：项目使用方式、数据格式、常用命令和输出解释。
- [docs/2_hand_eye_calibration_process.md](docs/2_hand_eye_calibration_process.md)：眼在手上与眼在手外的标定流程，以及当前项目支持状态。
- [docs/3_project_handoff.md](docs/3_project_handoff.md)：当前仓库状态、验证记录和后续维护注意事项。

## 快速运行

先创建并使用本项目的 conda 环境：

```powershell
conda env update -n hand_eye -f environment.yml
conda activate hand_eye
```

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

当前仓库已经包含一份示例输出：`outputs/calib_output_correct_intr/calibration_result.json`。
