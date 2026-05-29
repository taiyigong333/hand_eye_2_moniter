# 手眼标定结果正确性检查方法

本文档回答一个具体问题：当前 `calibration` 仓库中是否有检验手眼标定结果正确性的方法。

结论：有，但属于“质量检查和一致性检查”，不是带外部真值的绝对正确性证明。当前仓库主要通过以下方法判断结果是否可信：

- 单张图像的 PnP 重投影 RMSE。
- 单张图像的检测点和重投影点调试图。
- 有效样本数量统计。
- `T_base_board` 和 `T_base_cam_fixed` 的跨样本一致性统计。
- 下游人工检查输出矩阵、采集姿态和相机/标定板配置是否匹配。

## 1. 直接运行现有标定验证命令

使用当前本地样本和默认参数运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1
```

该脚本实际调用 `calib.py`，关键质量参数是：

```powershell
--max_reproj_rmse 10.0
```

也可以指定输出目录，避免覆盖已有结果：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1 `
  -OutputDir outputs\verify_handeye_result
```

如果想长期保留终端中的一致性统计，可把输出同时写入日志：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_current_calibration.ps1 `
  -OutputDir outputs\verify_handeye_result `
  | Tee-Object -FilePath outputs\verify_handeye_result.log
```

## 2. 单样本重投影 RMSE 筛选

`calib.py` 在每个样本、每个相机上先估计标定板位姿。估计完成后会计算 PnP 重投影误差：

```python
proj = project_points(T_cam_board, obj_points, K, dist)
reproj_err = np.linalg.norm(proj - img_points, axis=1)
info["reproj_rmse"] = float(np.sqrt(np.mean(reproj_err ** 2)))
```

运行时每个样本会打印类似信息：

```text
[000] pose.json | end used=.../... , rmse=... | fixed used=.../... , rmse=...
```

如果任一路相机的 `reproj_rmse` 为空，或超过 `--max_reproj_rmse`，该样本会被跳过：

```python
if info_end["reproj_rmse"] > args.max_reproj_rmse or info_fixed["reproj_rmse"] > args.max_reproj_rmse:
    skip
```

含义：

- RMSE 越小，说明当前图像中检测到的 marker 角点与 PnP 重投影角点越接近。
- RMSE 过大时，优先检查相机内参、图像分辨率、标定板尺寸、`id_map_json`、角点顺序、图像模糊和标定板是否被遮挡。
- 这个检查验证的是“单帧板位姿估计质量”，不是最终手眼外参的全部正确性。

## 3. `debug_vis` 调试图

每次运行 `calib.py` 都会在输出目录生成：

```text
debug_vis/
├── 000_end.jpg
├── 000_fixed.jpg
├── 001_end.jpg
├── 001_fixed.jpg
└── ...
```

图中颜色含义：

- 绿色点：实际检测到的 ArUco 角点。
- 红色点：使用 PnP 位姿重投影回图像的角点。

判断方式：

- 绿色点和红色点基本重合，说明该帧板位姿估计较可靠。
- 某些图明显错位，说明该样本即使没有被阈值过滤，也应重点排查。
- 若大量图像错位，通常不是手眼算法本身问题，而是输入配置问题。

建议抽查：

```text
outputs\verify_handeye_result\debug_vis\*_end.jpg
outputs\verify_handeye_result\debug_vis\*_fixed.jpg
```

## 4. 有效样本数量

输出文件：

```text
calibration_result.json
```

关键字段：

```json
{
  "num_total_samples": 35,
  "num_valid_samples": 34
}
```

含义：

- `num_total_samples`：成功读取到的样本数。
- `num_valid_samples`：双相机都成功检测标定板，并通过 RMSE 阈值筛选后的样本数。

判断方式：

- 有效样本太少时，结果不可信；代码中少于 5 个有效样本会直接报错。
- 实际建议 15 到 30 个以上姿态，并且姿态要有明显旋转变化。
- 只平移、姿态变化过小或标定板只出现在图像局部区域，都会让手眼标定退化。

## 5. 跨样本一致性统计

`calib.py` 计算完手眼结果后，会把每个有效样本推导出的候选变换与最终平均/参考变换比较：

```python
summarize_transform_list("T_base_board", T_base_board_all, T_base_board)
summarize_transform_list("T_base_cam_fixed", T_base_cam_fixed_all, T_base_cam_fixed)
```

终端会打印：

```text
T_base_board consistency:
  num_samples: ...
  translation err mean/std: ... / ... m
  rotation    err mean/std: ... / ... deg

T_base_cam_fixed consistency:
  num_samples: ...
  translation err mean/std: ... / ... m
  rotation    err mean/std: ... / ... deg
```

含义：

- `translation err mean/std`：不同样本推导出的平移结果相对最终结果的均值和标准差，单位米。
- `rotation err mean/std`：不同样本推导出的旋转结果相对最终结果的均值和标准差，单位度。

判断方式：

- 均值和标准差越小，说明不同样本之间越一致。
- 某次运行相比历史结果明显变大，说明采集、内参、标定板配置或机器人位姿同步可能有问题。
- 这些统计目前只打印到终端，不写入 `calibration_result.json`；需要留档时用 `Tee-Object` 保存运行日志。

## 6. 输出矩阵检查

标定结果主要写在：

```text
calibration_result.json
```

重点检查字段：

| 字段 | 含义 | 用途 |
| --- | --- | --- |
| `T_ee_cam_end` | 末端相机到末端执行器坐标系的变换 | 眼在手上外参 |
| `T_base_board` | 标定板到机器人 base 的变换 | 固定板位姿估计 |
| `T_base_cam_fixed` | 固定相机到机器人 base 的变换 | 固定相机外参 |

基础检查：

- `matrix_4x4` 最后一行应为 `[0, 0, 0, 1]`。
- 旋转矩阵应接近正交矩阵。
- 平移量级应符合现场物理距离。
- 固定相机模式下，下游抓取配置应使用 `T_base_cam_fixed.matrix_4x4`，不要误用 `T_ee_cam_end` 或 `T_base_board`。

## 7. 当前仓库没有的验证能力

当前仓库没有独立真值系统来证明手眼结果绝对正确，例如：

- 没有用外部高精度测量设备对 `T_ee_cam_end` 或 `T_base_cam_fixed` 做真值对比。
- 没有自动把重建点投到真实机器人目标上做闭环抓取验证。
- 没有把一致性统计写入 JSON 后自动做历史阈值回归。

因此目前应把结果判断分成两层：

1. 标定内部质量检查：看 RMSE、`debug_vis`、有效样本数、一致性统计。
2. 机器人闭环验证：把 `T_base_cam_fixed.matrix_4x4` 接到下游抓取流程后，在低速和安全距离下检查投影点、预抓取点和实际运动方向。

## 8. 建议检查顺序

每次重新标定后建议按这个顺序检查：

1. 确认命令中的相机内参、分辨率、标定板尺寸和 `id_map_json` 是本轮真实配置。
2. 运行 `scripts\run_current_calibration.ps1 -OutputDir outputs\verify_handeye_result`。
3. 查看终端每个样本的 `end rmse` 和 `fixed rmse` 是否大量偏大。
4. 查看 `calibration_result.json` 中 `num_valid_samples` 是否足够。
5. 抽查 `debug_vis`，确认绿色点和红色点基本重合。
6. 查看 `T_base_board consistency` 和 `T_base_cam_fixed consistency` 是否没有明显发散。
7. 检查 `T_base_cam_fixed.matrix_4x4` 的平移量级和现场固定相机位置是否合理。
8. 再把 `T_base_cam_fixed.matrix_4x4` 写入下游抓取配置，做低速闭环验证。
