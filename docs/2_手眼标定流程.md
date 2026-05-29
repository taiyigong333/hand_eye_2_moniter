# 手眼标定流程说明

## 1. 坐标约定

本项目采用 `T_A_B` 记法，含义是从 `B` 坐标系变换到 `A` 坐标系：

```text
p_A = T_A_B @ p_B
```

当前代码中常见坐标系：

- `base`：机器人基座坐标系。
- `ee` / `gripper`：机械臂末端坐标系。
- `cam_end`：腕部末端相机坐标系。
- `cam_fixed`：外部固定相机坐标系。
- `board`：标定板坐标系。

## 2. 眼在手上标定流程

眼在手上表示相机安装在机械臂末端，相机随机器人末端一起运动。目标是求：

```text
T_ee_cam_end
```

也就是末端相机坐标系到机械臂末端坐标系的固定变换。

标准数据采集流程：

1. 固定标定板位置，标定过程中不要移动标定板。
2. 让机械臂移动到多个不同姿态，建议至少 15 到 30 个姿态，并包含足够的旋转变化。
3. 每个姿态记录机器人 TCP 位姿 `T_base_ee`。
4. 每个姿态用末端相机拍摄标定板图像。
5. 通过 ArUco 检测和 PnP 求出每张图像中的 `T_cam_end_board`。
6. 将多组 `T_base_ee` 与 `T_cam_end_board` 输入手眼标定算法，求解 `T_ee_cam_end`。

当前项目实现位置：

- `calib.py` 中 `load_samples()` 读取 `T_base_ee` 和末端相机图像。
- `ArucoBoardPoseEstimator.estimate_board_pose()` 检测标定板并用 `cv2.solvePnP()` 求 `T_cam_board`。
- `compute_handeye_eye_in_hand()` 调用 `cv2.calibrateHandEye()` 求 `T_ee_cam_end`。

当前命令中与眼在手上相关的参数：

```powershell
--end_camera_index 0 `
--intr_end configs/intr_d405_1280x720.json `
--handeye_method tsai
```

## 3. 眼在手外标定流程

眼在手外表示相机固定在机器人外部，不随末端运动。目标通常是求：

```text
T_base_cam_fixed
```

也就是固定相机坐标系到机器人基座坐标系的变换。

常见流程有两类。

### 3.1 独立眼在手外流程

这种流程只依赖固定相机和机器人运动，通常让标定板固定在末端或夹具上：

1. 将标定板刚性安装到机械臂末端，保证 `T_ee_board` 在采集过程中不变。
2. 固定外部相机，采集多个机器人姿态。
3. 每个姿态记录 `T_base_ee`。
4. 固定相机拍摄标定板，并通过 PnP 得到 `T_cam_fixed_board`。
5. 通过眼在手外或 robot-world-hand-eye 形式求解 `T_base_cam_fixed` 与 `T_ee_board`。

这种方式适合只有外部固定相机、没有末端相机的系统。

### 3.2 当前项目使用的双相机联合流程

当前代码走的是双相机联合流程：

1. 标定数据中每个姿态同时包含腕部相机图像和固定相机图像。
2. 先用腕部相机完成眼在手上标定，得到 `T_ee_cam_end`。
3. 对每个有效样本计算标定板在基座中的位姿：

```text
T_base_board_i = T_base_ee_i @ T_ee_cam_end @ T_cam_end_board_i
```

4. 对所有样本的 `T_base_board_i` 求平均，得到稳定的 `T_base_board`。
5. 固定相机也能通过 PnP 得到 `T_cam_fixed_board_i`。
6. 根据同一个标定板坐标系计算固定相机外参：

```text
T_base_cam_fixed_i = T_base_board @ inv(T_cam_fixed_board_i)
```

7. 对所有 `T_base_cam_fixed_i` 求平均，得到最终 `T_base_cam_fixed`。

当前项目实现位置：

- `estimate_T_base_board()` 根据 `T_base_ee`、`T_ee_cam_end` 和 `T_cam_end_board` 估计 `T_base_board`。
- `estimate_T_base_cam_fixed()` 根据 `T_base_board` 和 `T_cam_fixed_board` 估计 `T_base_cam_fixed`。
- `calibration_result.json` 保存 `T_base_cam_fixed`。

当前命令中与眼在手外相关的参数：

```powershell
--fixed_camera_index 1 `
--intr_fixed configs/intr_d435i_1920x1080.json
```

## 4. 当前项目是否同时支持眼在手上和眼在手外

结论：当前项目支持“双相机联合标定”场景下同时输出眼在手上和眼在手外结果，但不支持只用固定相机独立运行眼在手外标定。

已支持的部分：

- 支持末端相机眼在手上标定，输出 `T_ee_cam_end`。
- 支持在同一批双相机样本中估计固定外部相机外参，输出 `T_base_cam_fixed`。
- 支持当前 `cam0_wrist` 与 `cam1_main` 的数据组织方式。
- 支持 `interleaved_checker` 和 `regular_aprilgrid` 两类标定板模型，其中当前数据使用 `interleaved_checker`。

尚未直接支持的部分：

- 没有单独的 `eye_to_hand` 命令或模式。
- `calib.py` 当前必须同时提供 `--intr_end`、`--intr_fixed`、`--end_camera_index`、`--fixed_camera_index`，并要求两个相机图像都可用。
- 代码没有实现只用 `T_base_ee` 与固定相机 `T_cam_fixed_board` 直接求 `T_base_cam_fixed` 的独立 eye-to-hand / robot-world-hand-eye 分支。

如果后续要支持“只有固定相机”的眼在手外标定，建议新增一个显式参数，例如：

```powershell
--mode eye_to_hand
```

并单独实现以下能力：

- 支持样本中只有固定相机图像。
- 支持标定板固定在末端或固定在环境中的两种采集约束，并在文档中明确约束。
- 使用 `cv2.calibrateRobotWorldHandEye()` 或等价公式求解 `T_base_cam_fixed`。
- 在输出 JSON 中明确记录采集模式、求解模式和坐标系定义。

## 5. 采集质量建议

- 姿态数量至少 5 个，实际建议 15 到 30 个。
- 姿态要有明显旋转变化，只平移会导致手眼标定退化。
- 标定板要尽量覆盖图像不同区域，避免所有样本角度和距离过于相近。
- 两个相机使用的内参必须和图像分辨率一致。
- 采集时要保证机器人位姿和图像严格对应同一时刻或同一静止姿态。
- 如果 RMSE 过大，优先检查内参、标定板尺寸、`id_map_json`、角点顺序和图像模糊。
