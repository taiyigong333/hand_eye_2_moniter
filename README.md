# Calibration
Calibration for ur7e-realsense camera


相机标定运行指令：
python calib.py \
  --dataset_dir dataset_from_npz \
  --fixed_camera_index 1 \
  --end_camera_index 0 \
  --intr_fixed intr_d435i_1920x1080.json \
  --intr_end intr_d405_1280x720.json \
  --board_layout interleaved_checker \
  --grid_cols 20 \
  --grid_rows 15 \
  --tag_size 0.015 \
  --cell_size 0.019 \
  --aruco_dict DICT_6X6_250 \
  --top_left_is_tag true \
  --id_map_json aruco_id_map_new.json \
  --max_reproj_rmse 10.0 \
  --output_dir calib_output_correct_intr


1.数据源与相机索引
--dataset_dir dataset_from_npz

标定数据集的文件夹路径。该目录下包含机械臂各点位TCP位姿的JSON以及两个相机同步拍摄的标定板图像。

--fixed_camera_index 1

固定相机（眼在手外 Eye-to-Hand，通常负责全局视角）在数据集中的相机编号。

--end_camera_index 0

末端相机（眼在手上 Eye-in-Hand，安装在机械臂末端）在数据集中的相机编号。

2，相机内参配置文件

--intr_fixed intr_d435i_1920x1080.json

固定相机的内参文件路径。

--intr_end intr_d405_1280x720.json

末端相机的内参文件路径。

3.标定板规格配置

这部分参数决定了代码如何构建标定板的 3D 物理模型。当前配置对应的是一种 ArUco 标签与黑色方块交错排列（类似棋盘格） 的特殊标定板。

--board_layout interleaved_checker

标定板的布局模式。选择 interleaved_checker 表示采用 ArUco 标记与纯黑方块交替相间的棋盘格布局。

--grid_cols 20

标定板的总列数。需要把 ArUco 单元格和纯黑单元格全部数进去，总共 20 列。

--grid_rows 15

标定板的总行数。同样包含所有单元格，总共 15 行。

--tag_size 0.015

ArUco 标记（二维码本身）最外层黑色正方形的实际物理边长，单位为米。


--cell_size 0.019

标定板上单个方格单元（包含白色边缘留白）的实际物理边长，单位为米。

--aruco_dict DICT_6X6_250

标定板所使用的 ArUco 字典类型。这里代表使用的是 6x6 矩阵、最大容纳 250 个不同 ID 的标准字典。

--top_left_is_tag true

定义标定板左上角第一个格子（第 0 行，第 0 列）是否为 ArUco 标记。设置为 true 意味着第一个格子就是标记，如果第一个格子是黑方块则需设为 false。

--id_map_json aruco_id_map_new.json

预先测绘好的 ID 映射文件，该 JSON 文件明确指定了每个 ID 对应在标定板上的真实行列坐标以及旋转方向。

标定板标定运行指令：

python make_aruco_id_map.py board.jpg \
  --aruco_dict DICT_6X6_250 \
  --grid_cols 20 \
  --grid_rows 15 \
  --top_left_is_tag true \
  --corners "589,77 3757,203 3849,2603 444,2592" \
  --tag_size 0.015 \
  --cell_size 0.019 \
  --max_cell_dist 0.55 \
  --out aruco_id_map_new.json \
  --vis aruco_id_map_new.jpg
  
board.jpg
清晰的标定板图像

--corners "589,77 3757,203 3849,2603 444,2592" 

标定板四角像素（左上角为0,0）

--max_cell_dist 0.55 

检测点距离理想格子中心的最大允许相对偏差

--out aruco_id_map_new.json 

--vis aruco_id_map_new.jpg

输出文件路径

   输出mapped ids: 150，即所有aruco块都成功识别
