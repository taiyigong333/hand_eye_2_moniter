# 端到端手眼标定操作流程

本文档描述从标定板字典建立、双相机拍摄、拍摄后计算到结果检查的完整流程。它面向实际重新做一轮 UR7e + 双 RealSense 手眼标定的人，优先给出当前仓库真实使用的文件、命令和检查点。

## 1. 当前项目的标定目标

当前项目不是只用固定相机独立做眼在手外标定，而是使用双相机联合流程：

1. 腕部相机随 UR7e 末端运动，先求眼在手上外参 `T_ee_cam_end`。
2. 通过腕部相机结果，把同一块静止标定板统一转换到机器人基座下，得到 `T_base_board`。
3. 固定相机也观察同一块标定板，再由 `T_base_board` 和固定相机 PnP 结果反推出 `T_base_cam_fixed`。

最终最常用的结果是 `calibration_result.json` 中的：

- `T_ee_cam_end.matrix_4x4`：末端相机到机器人末端坐标系的外参。
- `T_base_board.matrix_4x4`：标定板到机器人基座坐标系的外参。
- `T_base_cam_fixed.matrix_4x4`：固定相机到机器人基座坐标系的外参。

本项目坐标约定为 `T_A_B` 表示“从坐标系 B 到坐标系 A”的变换，即 `p_A = T_A_B @ p_B`。

## 2. 准备环境

在仓库根目录 `F:\research\guo\hand-eye\calibration` 下执行：

```powershell
conda env update -n hand_eye -f environment.yml
conda activate hand_eye
```

当前机器也可以直接调用固定 Python：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe calib.py --help
```

如果要实时采集，需要确认：

- 两台 RealSense 能被 Windows 识别。
- UR7e 与本机网络可达。
- 机器人示教器中存在配置文件指定的 URP 程序；当前 `configs/live_collection.example.json` 里是 `autoHandEye2.urp`。
- `pyrealsense2` 和 `ur-rtde` 已安装在 `hand_eye` 环境。

## 3. 建立或更新标定板字典

### 3.1 什么时候需要重新建字典

当前标定板是 `interleaved_checker`：ArUco marker 和纯黑格交错排列。`calib.py` 需要知道每个真实 marker ID 对应棋盘格的行、列和旋转，否则 PnP 的 2D-3D 角点对应会错。

以下情况需要重新生成 `id_map_json`：

- 换了新的实体标定板。
- 重新打印、裁切、旋转或粘贴了标定板。
- marker ID 排布和旧文件不一致。
- `debug_vis` 中检测点和重投影点明显错位，并怀疑是 ID 到格子位置错误。

当前默认字典文件是：

```text
assets/boards/aruco_id_map_new.json
```

### 3.2 拍一张用于建字典的标定板图片

先拍一张能看清整块或大部分标定板的图片，建议保存到：

```text
assets/boards/board.jpg
```

拍摄要求：

- 标定板尽量铺满画面，但四角要完整可见。
- 图像清晰，不要严重反光、遮挡或运动模糊。
- 透视可以存在，但有效棋盘区域四角必须能准确点击或手工读出像素。

### 3.3 生成 `id_map_json`

有 GUI 时可以不传 `--corners`，脚本会弹窗要求依次点击有效棋盘区域四角：左上、右上、右下、左下。

无 GUI 或想复现实验时，建议显式传入 `--corners`：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe tools\make_aruco_id_map.py assets\boards\board.jpg `
  --aruco_dict DICT_6X6_250 `
  --grid_cols 20 `
  --grid_rows 15 `
  --top_left_is_tag true `
  --corners "589,77 3757,203 3849,2603 444,2592" `
  --tag_size 0.015 `
  --cell_size 0.019 `
  --max_cell_dist 0.55 `
  --out assets\boards\aruco_id_map_new.json `
  --vis assets\boards\aruco_id_map_new.jpg
```

关键参数含义：

- `--grid_cols 20` / `--grid_rows 15`：整块棋盘的总列数和总行数，包含 ArUco 格和黑色格。
- `--tag_size 0.015`：marker 外层黑色正方形边长，单位米。
- `--cell_size 0.019`：棋盘单元格边长，单位米。
- `--top_left_is_tag true`：左上角第一个格子是否为 marker。
- `--corners`：有效棋盘区域四角像素，顺序必须是左上、右上、右下、左下。
- `--out`：生成给 `calib.py` 使用的字典 JSON。
- `--vis`：生成可视化检查图。

生成后打开 `assets/boards/aruco_id_map_new.jpg` 检查：

- 每个识别出的 marker 都标有类似 `id23->r4c7rot1` 的文本。
- 蓝色检测中心和红色映射后格子中心距离应较近。
- 同一个格子不应出现明显错误分配。
- 如果很多 marker 被跳过，先检查四角点、`top_left_is_tag`、棋盘行列数和 `max_cell_dist`。

## 4. 配置相机、机器人和标定参数

实时采集配置文件：

```text
configs/live_collection.example.json
```

重点检查这些字段：

- `robot.host`：UR7e IP。
- `robot.program`：示教器中的 URP 程序名；当前示例是 `autoHandEye2.urp`。
- `cameras[0]`：末端腕部相机，当前 `camera_index=0`，保存 `cam0_wrist.png`。
- `cameras[1]`：外部固定相机，当前 `camera_index=1`，保存 `cam1_main.png`。
- `cameras[].serial`：RealSense 序列号，实机前应确认和真实设备一致。
- `cameras[].intrinsics_path`：采集启动后会写入 active color profile 内参。
- `sampling.mode`：`timed` 定时采集或 `manual` 手动采集。
- `sampling.max_samples`：计划采样数量，当前示例是 35。
- `calibration.id_map_json`：应指向第 3 节生成的字典。
- `calibration.tag_size`、`calibration.cell_size`、`calibration.grid_cols`、`calibration.grid_rows`：必须和实体标定板一致。

当前默认两台相机约定：

```text
camera_index=0 / role=end   / cam0_wrist.png  -> 末端腕部相机
camera_index=1 / role=fixed / cam1_main.png   -> 外部固定相机
```

## 5. 拍摄标定样本

### 5.1 采集前 dry-run

先只检查配置和最终会调用的 `calib.py` 命令：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --dry_run
```

确认 dry-run 输出里的 `--dataset_dir`、`--intr_fixed`、`--intr_end`、`--id_map_json`、`--tag_size`、`--cell_size` 和相机 index 都符合本轮真实配置。

### 5.2 推荐采样方式

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

手动模式默认按 `c` 保存一组样本，按 `q` 结束采集。预览窗口聚焦时也可直接按 `c` / `q`。

如果只想先采集，不想结束后立刻计算：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe scripts\collect_and_calibrate.py `
  --config configs\live_collection.example.json `
  --skip_calibration
```

### 5.3 拍摄时的动作要求

每个有效样本必须同时包含：

- UR7e 当前 TCP 位姿。
- 末端相机图像。
- 固定相机图像。

采样建议：

- 标定板在整轮采集中保持静止。
- 两台相机都能看到标定板，至少看到足够数量的 marker。
- 机器人每次采样时应静止，不要在运动中保存图像和 TCP。
- 姿态要有明显旋转变化，不能只做平移。
- 让末端相机从不同方向、不同距离观察标定板，覆盖图像中心和边缘区域。
- 建议采集 15 到 35 组有效样本；当前示例配置是 35 组。

## 6. 拍摄后的数据结构

实时采集默认写入：

```text
data/live_capture/
├── capture_session_config.json
├── sample_000/
│   ├── cam0_wrist.png
│   ├── cam1_main.png
│   └── pose.json
├── sample_001/
│   ├── cam0_wrist.png
│   ├── cam1_main.png
│   └── pose.json
└── ...
```

`pose.json` 中 `calib.py` 必需的字段是：

- `tcp_pose`：`[x, y, z, rx, ry, rz]`，用于生成 `T_base_ee`。
- `images`：两台相机图像路径和 `camera_index`，用于找到末端相机和固定相机图片。

其他字段如 `joint_angles`、相机序列号、时间戳和内参文件用于追踪和调试。

## 7. 拍摄完成后的计算

如果采集时没有加 `--skip_calibration`，脚本会在采集结束后自动调用 `calib.py`。

如果需要手动计算，使用当前示例数据和参数：

```powershell
F:\Anaconda\Anaconda3\envs\hand_eye\python.exe calib.py `
  --dataset_dir data/live_capture `
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
  --translation_scale 1.0 `
  --output_dir outputs/my_live_calibration
```

如果只是复现本机旧样本，可以直接运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1
```

这会使用 `data/dataset_from_npz`、当前内参和 `assets/boards/aruco_id_map_new.json`，输出到 `outputs/calib_output_correct_intr`。

## 8. `calib.py` 内部计算顺序

`calib.py` 的计算可以按五步理解：

1. 读取样本：从每个 `pose.json` 读取 `tcp_pose`，转换成 `T_base_ee`。
2. 估计标定板位姿：两台相机分别检测 ArUco 角点，结合 `id_map_json` 和相机内参，用 `solvePnP` 求 `T_camend_board` 和 `T_camfixed_board`。
3. 过滤坏样本：计算重投影 RMSE，超过 `--max_reproj_rmse` 的样本不参与标定。
4. 求腕部相机外参：调用 OpenCV `calibrateHandEye`，由多组 `T_base_ee` 和 `T_camend_board` 求 `T_ee_cam_end`。
5. 求固定相机外参：先计算每帧 `T_base_board_i = T_base_ee_i * T_ee_cam_end * T_camend_board_i`，平均得到 `T_base_board`；再计算 `T_base_cam_fixed_i = T_base_board * inv(T_camfixed_board_i)`，平均得到 `T_base_cam_fixed`。

## 9. 输出文件和结果检查

标定输出目录包含：

```text
outputs/my_live_calibration/
├── calibration_result.json
├── dynamic_end_camera_poses.json
└── debug_vis/
    ├── 000_end.jpg
    ├── 000_fixed.jpg
    └── ...
```

检查顺序：

1. 看终端输出的 `Found ... usable samples` 和 `Valid samples after detection`。有效样本过少时，结果不可靠。
2. 打开 `debug_vis/*_end.jpg` 和 `debug_vis/*_fixed.jpg`。绿色点是检测角点，红色点是 PnP 重投影点，两者应基本重合。
3. 看每帧打印的 `rmse`。若大量样本超过 `--max_reproj_rmse`，先排查内参、字典、标定板尺寸和图像清晰度。
4. 看 `T_base_board consistency` 和 `T_base_cam_fixed consistency`。平移和旋转离散越小，跨样本越一致。
5. 打开 `calibration_result.json`，确认 `num_valid_samples`、`board_config` 和三组核心矩阵存在。

常见结果字段：

- `calibration_result.json/T_ee_cam_end/matrix_4x4`：腕部相机安装外参。
- `calibration_result.json/T_base_cam_fixed/matrix_4x4`：固定相机外参，后续固定相机引导机器人抓取通常使用这一项。
- `dynamic_end_camera_poses.json/T_base_cam_end`：每个采样姿态下腕部相机在机器人基座中的动态位姿。

## 10. 常见问题排查

### 10.1 marker 检测数量少

优先检查：

- 图像是否清晰、曝光是否合适。
- 标定板是否太远、太斜或被遮挡。
- `--aruco_dict` 是否和实体 marker 字典一致。
- RealSense 保存的分辨率是否与预期一致。

### 10.2 重投影 RMSE 大

优先检查：

- `id_map_json` 是否对应当前实体板。
- `--corners` 建字典时四角顺序是否错误。
- `tag_size` 和 `cell_size` 是否按真实物理尺寸填写。
- `intr_fixed` / `intr_end` 是否来自本轮实际相机 profile。
- 图像是否有运动模糊或明显畸变。

### 10.3 有效样本数够，但一致性很差

优先检查：

- 采样时机器人是否静止。
- 标定板在采集过程中是否移动。
- 图像和 TCP 是否为同一时刻。
- 姿态是否缺少旋转变化。
- 末端相机和固定相机的 `camera_index` 是否写反。

### 10.4 固定相机外参方向用错

本项目输出的 `T_base_cam_fixed` 是“固定相机坐标系 -> 机器人基座坐标系”。如果下游需要把相机下的点 `p_cam` 转到基座：

```text
p_base = T_base_cam_fixed @ p_cam
```

不要把它当作 `T_cam_fixed_base` 使用；如果需要相反方向，应显式取逆。

## 11. 每次重新标定建议记录

建议在实验记录中保存：

- 使用的 Git 提交号。
- `configs/live_collection.example.json` 或本轮实际配置副本。
- 标定板照片和 `aruco_id_map_new.json`。
- 采集数据目录，例如 `data/live_capture` 的时间和样本数。
- 标定输出目录，例如 `outputs/live_calibration_YYYYMMDD_HHMMSS`。
- `num_valid_samples`、RMSE 范围、一致性统计和最终 `T_base_cam_fixed.matrix_4x4`。
