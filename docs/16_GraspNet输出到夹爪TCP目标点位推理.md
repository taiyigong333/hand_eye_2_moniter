# GraspNet 输出到夹爪 TCP 目标点位推理

本文说明如何使用 `outputs/eye_in_hand_calibration_夹爪TCP/` 中已经标定出的结果，结合 GraspNet-baseline 的 `best_grasp` 输出，推导“夹爪 TCP 应该移动到的抓取点位”。

这里的前提非常重要：`outputs/eye_in_hand_calibration_夹爪TCP/` 这一轮标定时，RTDE 读取的是夹爪 TCP 在机器人 `base` 坐标系下的位姿。因此该目录中的 `T_ee_cam_end` 在物理语义上应解释为：

```text
T_tcp_cam
```

也就是“腕部相机坐标系到夹爪 TCP 坐标系”的变换，而不是“相机到法兰盘中心”的变换。

## 1. 适用范围

本文推理适用于：

- GraspNet 输入来自末端腕部相机，也就是本仓库默认的 `cam0_wrist` / `camera_index=0` / `serial=419122270341`。
- 当前时刻可以通过 RTDE 读到夹爪 TCP 在 `base` 下的实际位姿。
- `outputs/eye_in_hand_calibration_夹爪TCP/calibration_result.json` 是本轮要使用的眼在手上结果。

不适用于：

- GraspNet 输入来自外部固定相机 `serial=254522072098`。固定相机应使用 `T_base_cam_fixed` 或抓取配置中的 `calibration.T_base_cam` 链路，而不是本文的 `T_tcp_cam`。
- 采集标定时读的是法兰盘中心，但推理时把结果当成夹爪 TCP 外参使用。

当前 `../reproduction/graspnet-baseline-in-ur7e/outputs/last_grasp.json` 中记录的 `camera_serial` 是 `254522072098`，即固定相机。它可以用来说明 GraspNet 输出字段格式，但不能直接与本文的 `T_tcp_cam` 混合生成真实腕部相机抓取目标。

## 2. 当前是否可以直接使用 GraspNet

结论：如果目标是使用本文这条“腕部相机 + 夹爪 TCP 眼在手上结果”的链路，现在还不能直接用现有 `run_windows_pick_cycle.py` 让机器人抓取。

原因不是 GraspNet 本身不能推理，而是现有抓取编排还没有接入这条动态手眼链：

| 项目 | 当前状态 | 影响 |
| --- | --- | --- |
| GraspNet HTTP 推理 | 已有 `capture_and_infer.py` / `run_windows_pick_cycle.py` 调用链 | 可以请求 GraspNet，但输出仍在输入相机坐标系 |
| 当前单相机抓取配置 | `configs/ur7e_graspnet.local.json` 的 `camera.serial=254522072098` | 当前指向外部固定相机，不是腕部相机 |
| 当前外参配置 | `calibration.T_base_cam` 是固定相机到 base 的静态矩阵 | 不会使用 `outputs/eye_in_hand_calibration_夹爪TCP` 的 `T_tcp_cam` |
| 当前目标计算代码 | `pick_controller.py` 直接读取静态 `calibration.T_base_cam` | 没有运行时计算 `T_base_cam_now = T_base_tcp_now @ T_tcp_cam` |
| 当前 RTDE 读数 | 代码能通过 `read_current_tcp()` 读取当前 TCP | 还没有把该读数用于腕部相机动态外参 |
| TCP 偏置 | 已有 `tcp_translation_offset_m` 字段 | 需要重新确认它是否适用于当前夹爪 TCP 和当前 GraspNet 抓取中心 |

因此当前可继续沿用或单独验证的是：

- 固定相机旧链路：继续用 `serial=254522072098` 和静态 `calibration.T_base_cam`，但仍要以固定相机外参、TCP 偏置、URP 和安全阈值的实机验证为准。
- 腕部相机预览/推理：可以用双相机预览或单独配置腕部相机请求 GraspNet，验证它能返回 `best_grasp`。

当前还不能直接使用的是：

- 把 `outputs/eye_in_hand_calibration_夹爪TCP/T_ee_cam_end` 直接填进 `calibration.T_base_cam` 后抓取。这个矩阵是 `T_tcp_cam`，不是 `T_base_cam`。
- 把固定相机的 `last_grasp.json` 结果乘以 `T_tcp_cam` 后生成腕部抓取目标。相机坐标系不一致。

## 3. 接入腕部相机 GraspNet 还差什么

最小缺口如下：

1. 新增或修改抓取配置，让 GraspNet 输入相机切到腕部相机：

```json
{
  "camera": {
    "serial": "419122270341"
  }
}
```

同时确认腕部相机能提供 GraspNet 需要的 RGB-D 输入。手眼标定只用了 RGB 图像，但 GraspNet 抓取推理需要深度图、深度和彩色对齐、相机内参和 `factor_depth`。

2. 在抓取代码中读取夹爪 TCP 标定结果：

```text
T_tcp_cam = outputs/eye_in_hand_calibration_夹爪TCP/calibration_result.json
  /T_ee_cam_end/matrix_4x4
```

注意这个字段名虽然叫 `T_ee_cam_end`，但本轮采集时 RTDE 保存的是夹爪 TCP，所以在线抓取时应按 `T_tcp_cam` 解释。

3. 在每次 GraspNet 拍照对应的机器人稳定姿态下读取当前 RTDE 夹爪 TCP，并把 6D 位姿转成矩阵：

```text
tcp_now = rtde_receive.getActualTCPPose()  # [x, y, z, rx, ry, rz]
T_base_tcp_now = pose_to_matrix(tcp_now)
```

该读数必须和 GraspNet 使用的 RGB-D 帧尽量同步。机器人如果仍在运动，动态外参会错。

4. 用当前 TCP 动态生成腕部相机到 base 的外参：

```text
T_base_cam_now = T_base_tcp_now @ T_tcp_cam
```

然后把现有代码中的静态：

```text
T_base_cam = config["calibration"]["T_base_cam"]
```

替换为本次动态计算的 `T_base_cam_now`。

5. 重新确认夹爪 TCP 目标偏置：

```text
tcp_translation_offset_m
target_base_offset_m
tcp_rotation_mode
fixed_tcp_rotvec 或 tcp_rotation_offset
```

`tcp_translation_offset_m` 表示从 GraspNet 抓取中心到 UR TCP 原点的向量，按最终 TCP 坐标系表达。即使 `T_tcp_cam` 已经是腕部相机到夹爪 TCP 的外参，这个偏置仍可能需要，因为 GraspNet 的 `translation` 是抓取中心，不一定是 UR TCP 原点。

6. 增加离线/半实机验证：

- 先只运行腕部相机 `capture_and_infer.py`，确认 `best_grasp` 的 `camera_serial` 是 `419122270341`。
- 打印 `T_base_tcp_now`、`T_base_cam_now`、`grasp_center_base`、`tcp_goal`。
- 检查 `tcp_goal` 相对当前 TCP 的 `Δpos` 和 SO(3) `Δrot`，不要直接相减 Rodrigues 向量。
- 首次实机只走预抓取点，低速、张开夹爪，不闭合。

满足以上 6 项后，才可以认为本文这条腕部相机 GraspNet 链路具备实机试抓条件。

## 4. 坐标系和矩阵定义

本文继续使用项目内统一约定：

```text
T_A_B 表示把 B 坐标系下的点变到 A 坐标系
p_A = T_A_B @ p_B
```

常用坐标系如下：

| 名称 | 含义 |
| --- | --- |
| `base` | UR7e 机器人基座坐标系 |
| `tcp` | 当前夹爪 TCP 坐标系 |
| `cam` | 腕部相机坐标系，即 GraspNet 输入相机坐标系 |
| `grasp` | GraspNet 输出的抓取姿态坐标系 |
| `tcp_goal` | 希望机器人最终移动到的夹爪 TCP 目标坐标系 |

关键矩阵如下：

| 矩阵 / 向量 | 来源 | 含义 |
| --- | --- | --- |
| `T_tcp_cam` | `outputs/eye_in_hand_calibration_夹爪TCP/calibration_result.json/T_ee_cam_end/matrix_4x4` | 腕部相机到夹爪 TCP 的固定安装外参 |
| `T_base_tcp_now` | 运行时 RTDE 当前夹爪 TCP 读数 | 当前夹爪 TCP 到机器人 base 的位姿 |
| `T_base_cam_now` | `T_base_tcp_now @ T_tcp_cam` | 当前腕部相机到机器人 base 的位姿 |
| `T_cam_grasp` | GraspNet-baseline `best_grasp` | GraspNet 推荐抓取姿态在相机坐标系下的位姿 |
| `T_base_grasp` | `T_base_cam_now @ T_cam_grasp` | GraspNet 抓取姿态换算到机器人 base 下 |
| `T_base_tcp_goal` | 最终推导结果 | 机器人应执行的夹爪 TCP 目标位姿 |

点使用齐次形式：

```text
p = [x, y, z, 1]^T
```

旋转矩阵和平移向量组合成齐次矩阵：

```text
T_A_B = [
  R_A_B  t_A_B
  0 0 0  1
]
```

## 5. 标定文件中已有的实际数值

`outputs/eye_in_hand_calibration_夹爪TCP/calibration_result.json` 的关键信息：

```text
mode = eye_in_hand
num_total_samples = 35
num_valid_samples = 34
handeye_method = tsai
```

因为本轮 RTDE 读取的是夹爪 TCP，所以 `T_ee_cam_end` 应按 `T_tcp_cam` 使用：

```text
T_tcp_cam =
[
  [ 0.999801817,  0.014451171, -0.013692698, -0.003711826 ],
  [-0.013790133,  0.998790287,  0.047199517, -0.060501883 ],
  [ 0.014358222, -0.047001339,  0.998791628,  0.041266688 ],
  [ 0.000000000,  0.000000000,  0.000000000,  1.000000000 ]
]
```

等价分解为：

```text
R_tcp_cam =
[
  [ 0.999801817,  0.014451171, -0.013692698 ],
  [-0.013790133,  0.998790287,  0.047199517 ],
  [ 0.014358222, -0.047001339,  0.998791628 ]
]

t_tcp_cam = [-0.003711826, -0.060501883, 0.041266688]^T  m
```

含义是：任意腕部相机坐标系中的点 `p_cam`，可以用下面公式变到当前夹爪 TCP 坐标系：

```text
p_tcp = R_tcp_cam @ p_cam + t_tcp_cam
```

`calibration_result.json` 里还有 `T_base_board`，它是标定板到机器人 base 的位姿，用于标定质量检查和当时的标定链路闭环；在线 GraspNet 抓取时不直接用它生成 TCP 目标。

## 6. GraspNet-baseline 输出怎么解释

`graspnet_ur7e/transforms.py` 里的 `parse_grasp_array()` 把 GraspNet 的 17 维数组解释为：

```text
[score,
 width,
 height,
 depth,
 R_cam_grasp(9 个数),
 t_cam_grasp(3 个数),
 object_id]
```

其中：

- `t_cam_grasp` 是 GraspNet 推荐抓取中心在输入相机坐标系下的位置，单位是米。
- `R_cam_grasp` 是 GraspNet 推荐抓取姿态在输入相机坐标系下的旋转矩阵。
- `translation` 不是 UR TCP 原点，它是 GraspNet 的抓取中心。

由此构造：

```text
T_cam_grasp =
[
  R_cam_grasp  t_cam_grasp
  0 0 0        1
]
```

当前仓库中的 `last_grasp.json` 字段示例：

```text
t_cam_grasp = [0.096815281, -0.058040243, 0.426999986]^T

R_cam_grasp =
[
  [ 0.498022914, -0.837295294,  0.225632161 ],
  [-0.385696948, -0.446922183, -0.807154536 ],
  [ 0.776666701,  0.314955831, -0.545519710 ]
]
```

再次强调：这组 `last_grasp.json` 的元数据里是固定相机序列号 `254522072098`，这里只用它展示字段含义。真实使用本文链路时，应把同样格式的 GraspNet 输出换成腕部相机输出。

## 7. 第一步：从相机坐标系推到夹爪 TCP 坐标系

如果 GraspNet 输出来自腕部相机，则先用标定结果把抓取中心从相机坐标系变到当前夹爪 TCP 坐标系：

```text
p_tcp_grasp = T_tcp_cam @ p_cam_grasp
```

展开为：

```text
p_tcp_grasp = R_tcp_cam @ t_cam_grasp + t_tcp_cam
```

抓取姿态同理：

```text
R_tcp_grasp = R_tcp_cam @ R_cam_grasp
```

如果把上一节的 GraspNet 字段示例当作腕部相机输出代入，则得到：

```text
p_tcp_grasp = [0.086398736, -0.099652816, 0.471868764]^T  m

R_tcp_grasp =
[
  [ 0.481715780, -0.847900500,  0.221392753 ],
  [-0.355439874, -0.419969359, -0.835037876 ],
  [ 0.801007195,  0.323559116, -0.503683498 ]
]
```

这一步的物理含义是：在当前机器人姿态下，GraspNet 推荐的抓取中心相对于夹爪 TCP 原点大约位于：

```text
tcp 坐标系下 x =  0.0864 m
tcp 坐标系下 y = -0.0997 m
tcp 坐标系下 z =  0.4719 m
```

注意这还不是机器人 base 下的目标点位，因为此时还没有乘上当前 RTDE 读取的 `T_base_tcp_now`。

## 8. 第二步：从当前夹爪 TCP 坐标系推到机器人 base

运行时 RTDE 会读取当前夹爪 TCP 在 `base` 下的 6D 位姿：

```text
tcp_now = [x, y, z, rx, ry, rz]
```

把它转成齐次矩阵：

```text
T_base_tcp_now =
[
  R_base_tcp_now  t_base_tcp_now
  0 0 0           1
]
```

当前腕部相机到 base 的位姿是：

```text
T_base_cam_now = T_base_tcp_now @ T_tcp_cam
```

抓取中心在 base 下的位置是：

```text
p_base_grasp = T_base_cam_now @ p_cam_grasp
```

也可以先算到 TCP 再算到 base：

```text
p_base_grasp = T_base_tcp_now @ p_tcp_grasp
```

展开为：

```text
p_base_grasp = R_base_tcp_now @ p_tcp_grasp + t_base_tcp_now
```

抓取姿态在 base 下为：

```text
R_base_grasp = R_base_tcp_now @ R_tcp_grasp
```

到这里得到的是“GraspNet 抓取中心在 base 下的位置”和“GraspNet 抓取坐标系在 base 下的姿态”。

## 9. 第三步：由抓取中心推到夹爪 TCP 目标

机器人最终执行的是夹爪 TCP 目标，不是 GraspNet 抓取中心。因此还要明确一个偏移：

```text
o_tcp = 从 GraspNet 抓取中心指向 UR TCP 原点的向量，按最终 TCP 坐标系表达
```

如果当前 UR TCP 原点就定义在希望对准的抓取中心上，则：

```text
o_tcp = [0, 0, 0]^T
```

如果 TCP 原点不在 GraspNet 抓取中心，例如 TCP 定义在夹爪安装点、手指后方或其它机械参考点，则必须配置真实偏移。这个偏移对应 GraspNet-baseline 代码里的：

```text
grasp.tcp_translation_offset_m
```

`graspnet_ur7e/transforms.py` 中 `apply_tcp_translation_offset()` 的实现语义是：

```text
t_base_tcp_goal = t_base_grasp + R_base_tcp_goal @ o_tcp
```

如果还需要现场整体补偿，例如整体抬高 2 cm，则再加：

```text
b_base = grasp.target_base_offset_m
```

最终位置为：

```text
t_base_tcp_goal = t_base_grasp + R_base_tcp_goal @ o_tcp + b_base
```

其中 `R_base_tcp_goal` 是最终希望 UR TCP 采用的姿态。常见选择有三种：

| 模式 | 公式 | 含义 |
| --- | --- | --- |
| 跟随 GraspNet | `R_base_tcp_goal = R_base_grasp @ R_grasp_tcp_offset` | TCP 姿态跟随 GraspNet 抓取姿态，再叠加固定安装修正 |
| 固定姿态 | `R_base_tcp_goal = R_fixed` | 用现场验证过的固定 TCP 姿态抓取 |
| 当前姿态 | `R_base_tcp_goal = R_base_tcp_now` | 只移动位置，尽量保持当前 TCP 姿态 |

这三种模式对应 `graspnet_ur7e/pick_controller.py` 中的 `grasp.tcp_rotation_mode`：

- `graspnet`
- `fixed`
- `current`

最终机器人应执行的夹爪 TCP 抓取点位就是：

```text
tcp_goal = [
  t_base_tcp_goal.x,
  t_base_tcp_goal.y,
  t_base_tcp_goal.z,
  rotvec(R_base_tcp_goal).x,
  rotvec(R_base_tcp_goal).y,
  rotvec(R_base_tcp_goal).z
]
```

这就是 UR 控制侧常用的 `[x, y, z, rx, ry, rz]`。

## 10. 完整推理链

把上述步骤合起来，腕部相机 GraspNet 输出到夹爪 TCP 目标的完整链路是：

```text
输入：
  T_tcp_cam          # 来自 outputs/eye_in_hand_calibration_夹爪TCP
  T_base_tcp_now     # 来自运行时 RTDE，夹爪 TCP 在 base 下的当前位姿
  T_cam_grasp        # 来自 GraspNet best_grasp
  o_tcp              # 从 GraspNet 抓取中心到 UR TCP 原点的 TCP 坐标系偏移
  b_base             # 可选 base 坐标系整体补偿

计算：
  T_base_cam_now = T_base_tcp_now @ T_tcp_cam

  t_base_grasp = (T_base_cam_now @ [t_cam_grasp, 1])[:3]
  R_base_grasp = R_base_cam_now @ R_cam_grasp

  R_base_tcp_goal = resolve_tcp_rotation(R_base_grasp)
  t_base_tcp_goal = t_base_grasp + R_base_tcp_goal @ o_tcp + b_base

输出：
  tcp_goal = [t_base_tcp_goal, rotvec(R_base_tcp_goal)]
```

如果 `o_tcp = 0` 且 `b_base = 0`，则位置部分简化为：

```text
t_base_tcp_goal = t_base_grasp
```

这表示“把夹爪 TCP 原点直接移动到 GraspNet 抓取中心”。只有当 UR TCP 原点确实定义为夹爪实际抓取中心时，这个简化才成立。

## 11. 预抓取点位

GraspNet-baseline 的抓取流程通常还会生成预抓取点 `pregrasp`。当前代码中 `make_pregrasp_translation()` 使用 GraspNet 抓取姿态的某一列作为接近方向：

```text
a_base = normalize(R_base_grasp[:, approach_axis_index])
```

默认配置通常是：

```text
approach_axis_index = 0
approach_sign = -1
pregrasp_offset_m = 0.08
```

则预抓取 TCP 位置为：

```text
t_base_tcp_pregrasp =
  t_base_tcp_goal + approach_sign * pregrasp_offset_m * a_base
```

预抓取姿态通常和抓取姿态一致：

```text
R_base_tcp_pregrasp = R_base_tcp_goal
```

最终预抓取点位：

```text
tcp_pregrasp = [t_base_tcp_pregrasp, rotvec(R_base_tcp_goal)]
```

注意：预抓取方向使用 `R_base_grasp`，而不是已经叠加 TCP 姿态修正后的 `R_base_tcp_goal`。这是当前 `pick_controller.py` 的实现约定，目的是保持 GraspNet 的接近方向不被 TCP 姿态补偿破坏。

## 12. 和当前 GraspNet-baseline 代码的对应关系

固定相机抓取代码原本使用：

```text
T_base_cam = config["calibration"]["T_base_cam"]
grasp_t_base, grasp_R_base = transform_grasp_to_base(grasp, T_base_cam)
```

对于本文的腕部相机 + 夹爪 TCP 标定链路，应把运行时的相机外参改成动态计算：

```text
T_base_cam_now = T_base_tcp_now @ T_tcp_cam
grasp_t_base, grasp_R_base = transform_grasp_to_base(grasp, T_base_cam_now)
```

后续仍然沿用现有代码语义：

```text
tcp_R_base = resolve_tcp_rotation(grasp_R_base)
grasp_tcp_t_base = apply_tcp_translation_offset(
    grasp_t_base,
    tcp_R_base,
    tcp_translation_offset_m,
) + target_base_offset_m

grasp_tcp = pose_from_translation_rotation(grasp_tcp_t_base, tcp_R_base)
```

因此，真正需要替换的是 `calibration.T_base_cam` 的来源：

- 固定相机场景：`T_base_cam` 是固定外参，来自 `T_base_cam_fixed`。
- 腕部相机场景：`T_base_cam_now = T_base_tcp_now @ T_tcp_cam`，每次抓取都要用当前 RTDE 重新算。

## 13. 为什么本轮不需要法兰盘到 TCP 的额外变换

因为 `outputs/eye_in_hand_calibration_夹爪TCP/` 是用“夹爪 TCP 在 base 下的 RTDE 读数”计算出来的，所以标定结果已经把腕部相机外参绑定到了夹爪 TCP 坐标系：

```text
p_tcp = T_tcp_cam @ p_cam
```

因此在线推理时不需要再插入：

```text
T_flange_tcp
```

如果再额外乘一次法兰盘到 TCP 的变换，会重复补偿工具偏置，导致目标点偏移。

需要保留的是另一个概念：

```text
o_tcp = GraspNet 抓取中心 -> UR TCP 原点
```

它不是法兰盘到 TCP 的变换，而是“视觉抓取中心”和“机器人控制 TCP 原点”之间的物理差异。只有当这两个点完全一致时，才可以设为零。

## 14. 最小实现伪代码

```python
import numpy as np
from scipy.spatial.transform import Rotation

def pose_to_T_base_tcp(tcp_pose):
    x, y, z, rx, ry, rz = tcp_pose
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T

def graspnet_to_tcp_goal(
    T_tcp_cam,
    current_tcp_pose,
    best_grasp,
    tcp_translation_offset_m=(0.0, 0.0, 0.0),
    target_base_offset_m=(0.0, 0.0, 0.0),
    tcp_rotation_offset=np.eye(3),
):
    T_base_tcp_now = pose_to_T_base_tcp(current_tcp_pose)
    T_base_cam_now = T_base_tcp_now @ T_tcp_cam

    R_cam_grasp = np.asarray(best_grasp["rotation_matrix"], dtype=float)
    t_cam_grasp = np.asarray(best_grasp["translation"], dtype=float)

    t_base_grasp = (T_base_cam_now @ np.r_[t_cam_grasp, 1.0])[:3]
    R_base_grasp = T_base_cam_now[:3, :3] @ R_cam_grasp

    R_base_tcp_goal = R_base_grasp @ np.asarray(tcp_rotation_offset, dtype=float)
    t_base_tcp_goal = (
        t_base_grasp
        + R_base_tcp_goal @ np.asarray(tcp_translation_offset_m, dtype=float)
        + np.asarray(target_base_offset_m, dtype=float)
    )

    rotvec = Rotation.from_matrix(R_base_tcp_goal).as_rotvec()
    return [float(v) for v in np.r_[t_base_tcp_goal, rotvec]]
```

这里 `current_tcp_pose` 必须来自 GraspNet 拍摄同一时刻或足够接近同一稳定姿态的 RTDE 读数。如果机器人在拍照和读 TCP 之间发生运动，`T_base_cam_now` 就会错，最终抓取点位也会错。

## 15. 实机检查顺序

1. 确认 GraspNet 输入相机是腕部相机 `serial=419122270341`。
2. 读取 `outputs/eye_in_hand_calibration_夹爪TCP/calibration_result.json/T_ee_cam_end/matrix_4x4`，按 `T_tcp_cam` 使用。
3. 在 GraspNet 拍照时同步记录当前 RTDE 夹爪 TCP，得到 `T_base_tcp_now`。
4. 用 `T_base_cam_now = T_base_tcp_now @ T_tcp_cam` 把 GraspNet 抓取中心变到 base。
5. 明确 UR TCP 原点是否等于 GraspNet 抓取中心：
   - 如果相同，`tcp_translation_offset_m = [0, 0, 0]`。
   - 如果不同，用实测值填写 `tcp_translation_offset_m`。
6. 先只输出 `tcp_goal`，不要立即执行；检查目标相对当前 TCP 的位置跳变量和旋转跳变量。
7. 低速、张开夹爪、空跑到预抓取点，确认运动方向正确后再执行闭合抓取。

## 16. 最容易出错的地方

| 错误 | 后果 |
| --- | --- |
| 把固定相机 `last_grasp.json` 直接乘 `T_tcp_cam` | 相机坐标系不匹配，目标点完全错误 |
| 忘记乘当前 `T_base_tcp_now` | 只能得到 TCP 局部坐标，得不到机器人 base 下的目标 |
| 把 GraspNet `translation` 当作 UR TCP 原点 | 夹爪可能差出一个工具中心偏置 |
| 标定时用夹爪 TCP，推理时又额外加 `T_flange_tcp` | 工具偏置被重复计算 |
| 拍照时机器人还在运动 | GraspNet 图像和 RTDE TCP 不同步，`T_base_cam_now` 错 |
| 腕部相机和固定相机序列号混用 | 使用了错误外参链路 |
