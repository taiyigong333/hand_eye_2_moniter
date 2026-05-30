#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人基座 + 外部固定相机 + 末端腕部相机的手眼标定入口。

本脚本支持两类标定板：
1) regular_aprilgrid   : 每个网格位置都是一个 AprilTag
2) interleaved_checker : ArUco/AprilTag 与纯黑格像棋盘一样交错排列

典型 interleaved_checker 用法：
  --board_layout interleaved_checker \
  --grid_cols 20 \
  --grid_rows 15 \
  --tag_size 0.015 \
  --top_left_is_tag false   # 如果左上角第一个格子是 marker，则改为 true

坐标约定是阅读本文件最重要的前提：
- T_A_B 表示“从坐标系 B 到坐标系 A”的齐次变换。
- p_A = T_A_B @ p_B。
- 例如 T_base_ee 把末端坐标系下的点变到机器人基座坐标系；
  T_cam_board 把标定板坐标系下的点变到相机坐标系。

本项目默认采用的双相机计算链路：
1. 每个样本读取机器人 TCP 位姿，得到 T_base_ee。
2. 两台相机分别通过 ArUco 角点 + PnP 得到 T_camend_board 和 T_camfixed_board。
3. 先用腕部相机样本求眼在手上结果 T_ee_cam_end。
4. 再由 T_base_ee * T_ee_cam_end * T_camend_board 估计固定标定板的 T_base_board。
5. 最后由 T_base_board * inv(T_camfixed_board) 反推出固定相机外参 T_base_cam_fixed。

如果只需要眼在手上标定，可以使用 --mode eye_in_hand。该模式只要求末端相机
图像、机器人 TCP 和末端相机内参，不读取固定相机图像，也不会输出 T_base_cam_fixed。
"""

import argparse
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
    HAS_PUPIL_APRILTAGS = True
except Exception:
    HAS_PUPIL_APRILTAGS = False


# ----------------------------
# Basic geometry utilities
# ----------------------------

def rodrigues_to_matrix(rvec: np.ndarray) -> np.ndarray:
    """把旋转向量转换成 3x3 旋转矩阵；UR TCP 和 OpenCV PnP 都常用 Rodrigues 表达。"""
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return R


def matrix_to_rodrigues(R: np.ndarray) -> np.ndarray:
    """把 3x3 旋转矩阵转换成旋转向量，主要用于把结果写入 JSON 方便人工读取。"""
    rvec, _ = cv2.Rodrigues(R)
    return rvec.reshape(3)


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """由旋转 R 和平移 t 组装 4x4 齐次变换矩阵。"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """求刚体变换的逆；对正交旋转矩阵有 R^-1 = R.T，比通用矩阵求逆更稳定。"""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def compose(*Ts: np.ndarray) -> np.ndarray:
    """按从左到右的坐标链相乘，例如 compose(T_base_ee, T_ee_cam) 得到 T_base_cam。"""
    out = np.eye(4, dtype=np.float64)
    for T in Ts:
        out = out @ T
    return out


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """把旋转矩阵转成单位四元数，用于多帧旋转平均。"""
    trace = np.trace(R)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """把单位四元数还原为旋转矩阵。"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def average_transforms(Ts: List[np.ndarray]) -> np.ndarray:
    """
    对多帧刚体变换做一个保守平均。

    平移直接取算术均值；旋转不能逐元素平均，否则会破坏正交性，因此先转成四元数，
    用 Markley 风格的特征向量平均得到最接近的一致旋转。这里用于把每个有效样本
    推出的 T_base_board 或 T_base_cam_fixed 汇总成一个最终外参。
    """
    if not Ts:
        raise ValueError("No transforms to average.")
    t_mean = np.mean(np.array([T[:3, 3] for T in Ts], dtype=np.float64), axis=0)
    quats = []
    for T in Ts:
        q = rotation_matrix_to_quaternion(T[:3, :3])
        if q[0] < 0:
            q = -q
        quats.append(q)
    A = np.zeros((4, 4), dtype=np.float64)
    for q in quats:
        A += np.outer(q, q)
    eigvals, eigvecs = np.linalg.eigh(A)
    q_mean = eigvecs[:, np.argmax(eigvals)]
    q_mean /= np.linalg.norm(q_mean)
    return make_transform(quaternion_to_rotation_matrix(q_mean), t_mean)


def se3_distance(T1: np.ndarray, T2: np.ndarray) -> Tuple[float, float]:
    """计算两个 SE(3) 变换的相对平移误差和旋转角误差，用于检查跨样本一致性。"""
    dT = invert_transform(T1) @ T2
    trans = float(np.linalg.norm(dT[:3, 3]))
    cos_theta = (np.trace(dT[:3, :3]) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    rot_deg = float(np.degrees(np.arccos(cos_theta)))
    return trans, rot_deg


def project_points(T_cam_obj: np.ndarray, object_points: np.ndarray, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
    """
    按相机内参把物体三维点投影到图像平面。

    T_cam_obj 是 PnP 求出的“物体/标定板 -> 相机”位姿。重投影点与检测角点之间的
    像素误差越小，说明这张图里的标定板位姿估计越可信。
    """
    rvec, _ = cv2.Rodrigues(T_cam_obj[:3, :3])
    tvec = T_cam_obj[:3, 3].reshape(3, 1)
    img_pts, _ = cv2.projectPoints(object_points.astype(np.float64), rvec, tvec, K.astype(np.float64), dist.astype(np.float64))
    return img_pts.reshape(-1, 2)


# ----------------------------
# Board model
# ----------------------------

@dataclass
class BoardConfig:
    board_layout: str                 # regular_aprilgrid | interleaved_checker
    tag_size: float                   # meters
    tag_family: str = "tag16h5"

    # regular_aprilgrid
    tag_cols: Optional[int] = None
    tag_rows: Optional[int] = None
    tag_spacing: float = 0.0

    # interleaved_checker
    grid_cols: Optional[int] = None
    grid_rows: Optional[int] = None
    top_left_is_tag: bool = False
    cell_size: Optional[float] = None

    # 真实 marker ID 到棋盘格 row/col/角点旋转 的映射；用于非连续、非 row-major 的实体标定板。
    id_map: Optional[Dict[int, Tuple[int, int, int]]] = None


class BoardModel:
    """把检测到的 marker ID 转成标定板坐标系中的三维角点。"""

    def __init__(self, cfg: BoardConfig):
        self.cfg = cfg
        self.layout = cfg.board_layout
        if self.layout not in {"regular_aprilgrid", "interleaved_checker"}:
            raise ValueError(f"Unsupported board_layout: {self.layout}")
        self._validate()

    def _validate(self) -> None:
        if self.layout == "regular_aprilgrid":
            if self.cfg.tag_cols is None or self.cfg.tag_rows is None:
                raise ValueError("regular_aprilgrid requires --tag_cols and --tag_rows")
            if self.cfg.tag_cols <= 0 or self.cfg.tag_rows <= 0:
                raise ValueError("tag_cols/tag_rows must be positive")
        else:
            if self.cfg.grid_cols is None or self.cfg.grid_rows is None:
                raise ValueError("interleaved_checker requires --grid_cols and --grid_rows")
            if self.cfg.grid_cols <= 0 or self.cfg.grid_rows <= 0:
                raise ValueError("grid_cols/grid_rows must be positive")

            if self.cfg.cell_size is not None and self.cfg.cell_size <= 0:
                raise ValueError("cell_size must be positive")
            if self.cfg.cell_size is not None and self.cfg.cell_size < self.cfg.tag_size:
                raise ValueError("cell_size should be >= tag_size for interleaved_checker")

    def num_tags(self) -> int:
        if self.layout == "regular_aprilgrid":
            return int(self.cfg.tag_cols * self.cfg.tag_rows)
        return int((self.cfg.grid_cols * self.cfg.grid_rows) // 2)

    def describe(self) -> Dict:
        if self.layout == "regular_aprilgrid":
            return {
                "board_layout": self.layout,
                "tag_cols": self.cfg.tag_cols,
                "tag_rows": self.cfg.tag_rows,
                "tag_size_m": self.cfg.tag_size,
                "tag_spacing_ratio": self.cfg.tag_spacing,
                "tag_family": self.cfg.tag_family,
                "num_tags": self.num_tags(),
            }

        cell_size = self.cfg.cell_size if self.cfg.cell_size is not None else self.cfg.tag_size
        margin = 0.5 * (cell_size - self.cfg.tag_size)

        return {
            "board_layout": self.layout,
            "grid_cols": self.cfg.grid_cols,
            "grid_rows": self.cfg.grid_rows,
            "tag_size_m": self.cfg.tag_size,
            "cell_pitch_m": cell_size,
            "marker_margin_in_cell_m": margin,
            "top_left_is_tag": self.cfg.top_left_is_tag,
            "tag_family": self.cfg.tag_family,
            "num_tags": self.num_tags(),
        }

    def tag_object_corners(self, tag_id: int) -> Optional[np.ndarray]:
        """返回某个 marker 四个角点在标定板坐标系下的 3D 坐标，单位为米。"""
        if self.layout == "regular_aprilgrid":
            return self._regular_tag_corners(tag_id)
        return self._interleaved_tag_corners(tag_id)

    def _regular_tag_corners(self, tag_id: int) -> Optional[np.ndarray]:
        """规则 AprilGrid 中，marker ID 默认按从左到右、从上到下排列。"""
        cols = self.cfg.tag_cols
        rows = self.cfg.tag_rows
        if not (0 <= tag_id < cols * rows):
            return None
        c = tag_id % cols
        r = tag_id // cols
        s = self.cfg.tag_size
        pitch = s * (1.0 + self.cfg.tag_spacing)
        x0 = c * pitch
        y0 = r * pitch
        return np.array([
            [x0,     y0,     0.0],
            [x0 + s, y0,     0.0],
            [x0 + s, y0 + s, 0.0],
            [x0,     y0 + s, 0.0],
        ], dtype=np.float64)

    def _interleaved_tag_cell(self, tag_id: int) -> Optional[Tuple[int, int]]:
        cols = int(self.cfg.grid_cols)
        rows = int(self.cfg.grid_rows)

        # 优先使用真实 ID -> 棋盘格位置映射。实体板上的 ID 往往不是连续 row-major，
        # 错用默认规则会让 PnP 的 2D-3D 对应关系整体错位，后续手眼结果也会随之错误。
        if self.cfg.id_map is not None:
            rc = self.cfg.id_map.get(int(tag_id), None)
            if rc is None:
                return None

            row, col = int(rc[0]), int(rc[1])

            if not (0 <= row < rows and 0 <= col < cols):
                return None

            return row, col

        # 没有 id_map 时，才使用默认 row-major 规则：每行一半格子是 marker，
        # 由 top_left_is_tag 决定第一行 marker 从第 0 列还是第 1 列开始。
        tags_per_row = cols // 2
        max_tags = tags_per_row * rows

        if cols % 2 != 0:
            raise ValueError(
                "For interleaved_checker, grid_cols should be even for row-wise 50/50 tag layout."
            )

        if not (0 <= tag_id < max_tags):
            return None

        row = tag_id // tags_per_row
        k = tag_id % tags_per_row

        if self.cfg.top_left_is_tag:
            start_col = row % 2
        else:
            start_col = 1 - (row % 2)

        col = start_col + 2 * k

        if not (0 <= row < rows and 0 <= col < cols):
            return None

        return row, col

    def _interleaved_tag_rotation(self, tag_id: int) -> int:
        """读取实体 marker 在格子里的 90 度旋转次数，保证角点顺序与检测结果一致。"""
        if self.cfg.id_map is not None:
            rc = self.cfg.id_map.get(int(tag_id), None)
            if rc is not None and len(rc) >= 3:
                return int(rc[2]) % 4
        return 0

    def _interleaved_tag_corners(self, tag_id: int) -> Optional[np.ndarray]:
        """交错棋盘格中，marker 居中放在棋盘单元格里，四角点仍落在 z=0 的板平面。"""
        rc = self._interleaved_tag_cell(tag_id)
        if rc is None:
            return None

        row, col = rc

        marker_size = float(self.cfg.tag_size)
        cell_size = float(self.cfg.cell_size) if self.cfg.cell_size is not None else marker_size

        # marker 在棋盘单元格中居中
        offset = 0.5 * (cell_size - marker_size)

        x0 = col * cell_size + offset
        y0 = row * cell_size + offset
        s = marker_size

        corners = np.array([
            [x0,     y0,     0.0],  # TL
            [x0 + s, y0,     0.0],  # TR
            [x0 + s, y0 + s, 0.0],  # BR
            [x0,     y0 + s, 0.0],  # BL
        ], dtype=np.float64)

        # OpenCV 返回 marker 四角点时带有朝向；实体板如果旋转贴放，需要同步旋转 3D
        # 角点顺序，否则 solvePnP 会把同一个方块的角点对应错。
        rot = self._interleaved_tag_rotation(tag_id)
        corners = np.roll(corners, -rot, axis=0)

        return corners

    def collect_correspondences(self, detections: List[dict]) -> Tuple[np.ndarray, np.ndarray, List[int]]:
        """
        从检测结果构造 PnP 所需的 3D-2D 对应点。

        obj_points 是标定板坐标系中的已知三维角点，img_points 是同一批角点在图像中的
        像素坐标；这组对应关系直接决定 T_cam_board 的求解质量。
        """
        obj_points = []
        img_points = []
        used_tag_ids = []
        for det in detections:
            tag_id = int(det["tag_id"])
            obj = self.tag_object_corners(tag_id)
            if obj is None:
                continue
            img = np.asarray(det["corners"], dtype=np.float64).reshape(4, 2)
            obj_points.append(obj)
            img_points.append(img)
            used_tag_ids.append(tag_id)
        if not obj_points:
            return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 2), dtype=np.float64), []
        return np.concatenate(obj_points, axis=0), np.concatenate(img_points, axis=0), used_tag_ids


# ----------------------------
# Camera intrinsics and dataset
# ----------------------------


def load_intrinsics(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """加载相机内参 K 和畸变参数 dist；PnP 和重投影检查都依赖同一套内参。"""
    if path.lower().endswith(".json"):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return np.array(data["K"], dtype=np.float64), np.array(data["dist"], dtype=np.float64).reshape(-1, 1)
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"Cannot open intrinsics file: {path}")
    K = fs.getNode("K").mat()
    dist = fs.getNode("dist").mat()
    fs.release()
    if K is None or dist is None:
        raise ValueError(f"Invalid intrinsics file: {path}")
    return K.astype(np.float64), dist.astype(np.float64)


@dataclass
class Sample:
    json_path: Path
    T_base_ee: np.ndarray
    image_fixed: Optional[Path]
    image_end: Path
    raw: dict


def tcp_pose_to_T_base_ee(tcp_pose: List[float], translation_scale: float = 1.0) -> np.ndarray:
    """
    把机器人控制器记录的 TCP 位姿转为 T_base_ee。

    tcp_pose = [x, y, z, rx, ry, rz]，其中 xyz 是 TCP 原点在机器人基座下的位置，
    rxyz 是末端坐标系相对基座的 Rodrigues 旋转向量。translation_scale 用来兼容
    毫米/米等不同数据来源，但本项目正常应保持米制。
    """
    if len(tcp_pose) != 6:
        raise ValueError(f"tcp_pose must have length 6, got {len(tcp_pose)}")
    x, y, z, rx, ry, rz = [float(v) for v in tcp_pose]
    t = np.array([x, y, z], dtype=np.float64) * float(translation_scale)
    R = rodrigues_to_matrix(np.array([rx, ry, rz], dtype=np.float64))
    return make_transform(R, t)


def load_samples(
    dataset_dir: str,
    end_camera_index: int,
    translation_scale: float,
    fixed_camera_index: Optional[int] = None,
    require_fixed: bool = True,
) -> List[Sample]:
    """
    读取一批样本。

    每个有效样本必须同时包含：
    - 机器人 TCP 位姿，用于得到 T_base_ee；
    - 末端相机图像，用于估计 T_camend_board。
    双相机联合模式还要求固定相机图像，用于估计 T_camfixed_board；眼在手上
    独立模式不需要固定相机图像。
    """
    json_files = sorted(Path(dataset_dir).rglob("*.json"))
    samples = []
    for jp in json_files:
        with open(jp, "r", encoding="utf-8") as f:
            data = json.load(f)
        if "tcp_pose" not in data or "images" not in data:
            continue

        fixed_rel, end_rel = None, None
        for item in data["images"]:
            if fixed_camera_index is not None and int(item["camera_index"]) == int(fixed_camera_index):
                fixed_rel = item["file"]
            if int(item["camera_index"]) == int(end_camera_index):
                end_rel = item["file"]
        if end_rel is None:
            continue
        if require_fixed and fixed_rel is None:
            continue

        fixed_path = jp.parent / fixed_rel if fixed_rel is not None else None
        end_path = jp.parent / end_rel
        missing_fixed = require_fixed and (fixed_path is None or not fixed_path.exists())
        if missing_fixed or not end_path.exists():
            print(f"[WARN] Missing required image for {jp.name}, skip.")
            continue

        samples.append(Sample(
            json_path=jp,
            T_base_ee=tcp_pose_to_T_base_ee(data["tcp_pose"], translation_scale),
            image_fixed=fixed_path,
            image_end=end_path,
            raw=data,
        ))
    return samples


# ----------------------------
# AprilTag detection + PnP
# ----------------------------

# class AprilTagBoardPoseEstimator:
#     def __init__(self, board: BoardModel, tag_family: str = "tag16h5"):
#         if not HAS_PUPIL_APRILTAGS:
#             raise ImportError("Please install pupil-apriltags: pip install pupil-apriltags")
#         self.board = board
#         self.detector = Detector(
#             families=tag_family,
#             nthreads=4,
#             quad_decimate=1.0,
#             quad_sigma=0.0,
#             refine_edges=1,
#             decode_sharpening=0.25,
#             debug=0,
#         )

#     def detect(self, image_gray: np.ndarray) -> List[dict]:
#         dets = self.detector.detect(image_gray, estimate_tag_pose=False)
#         out = []
#         for d in dets:
#             out.append({
#                 "tag_id": int(d.tag_id),
#                 "corners": np.asarray(d.corners, dtype=np.float64),
#                 "center": np.asarray(d.center, dtype=np.float64),
#             })
#         return out

#     def estimate_board_pose(self,
#                             image_bgr: np.ndarray,
#                             K: np.ndarray,
#                             dist: np.ndarray,
#                             min_tags: int = 4,
#                             debug_vis_path: Optional[str] = None) -> Tuple[Optional[np.ndarray], dict]:
#         gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
#         detections = self.detect(gray)
#         obj_points, img_points, used_tag_ids = self.board.collect_correspondences(detections)
#         info = {
#             "num_detected_tags": len(detections),
#             "num_used_tags": len(used_tag_ids),
#             "used_tag_ids": used_tag_ids,
#             "num_points": int(len(obj_points)),
#             "reproj_rmse": None,
#         }
#         if len(used_tag_ids) < min_tags or len(obj_points) < 4 * min_tags:
#             return None, info

#         ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
#         if not ok:
#             return None, info

#         R, _ = cv2.Rodrigues(rvec)
#         T_cam_board = make_transform(R, tvec.reshape(3))
#         proj = project_points(T_cam_board, obj_points, K, dist)
#         reproj_err = np.linalg.norm(proj - img_points, axis=1)
#         info["reproj_rmse"] = float(np.sqrt(np.mean(reproj_err ** 2)))

#         if debug_vis_path is not None:
#             vis = image_bgr.copy()
#             for p in img_points.astype(int):
#                 cv2.circle(vis, tuple(p), 3, (0, 255, 0), -1)
#             for p in proj.astype(int):
#                 cv2.circle(vis, tuple(p), 2, (0, 0, 255), -1)
#             cv2.imwrite(debug_vis_path, vis)

#         return T_cam_board, info

class ArucoBoardPoseEstimator:
    """检测 ArUco marker，并用 PnP 估计标定板相对相机的位姿。"""

    def __init__(self, board: BoardModel, aruco_dict_name: str = "DICT_6X6_250"):
        self.board = board

        if not hasattr(cv2, "aruco"):
            raise ImportError(
                "当前 OpenCV 没有 aruco 模块，请安装: pip install opencv-contrib-python"
            )

        if not hasattr(cv2.aruco, aruco_dict_name):
            raise ValueError(f"Unsupported ArUco dictionary: {aruco_dict_name}")

        dict_id = getattr(cv2.aruco, aruco_dict_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self.params = cv2.aruco.DetectorParameters()

        # 兼容 OpenCV 新旧 API
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        else:
            self.detector = None

    def detect(self, image_gray: np.ndarray) -> List[dict]:
        """返回每个 marker 的 ID、四角点像素坐标和中心点。"""
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(image_gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                image_gray,
                self.dictionary,
                parameters=self.params,
            )

        out = []
        if ids is None:
            return out

        for marker_id, c in zip(ids.flatten(), corners):
            pts = np.asarray(c, dtype=np.float64).reshape(4, 2)
            out.append({
                "tag_id": int(marker_id),       # 为了复用 BoardModel，这里仍叫 tag_id
                "corners": pts,
                "center": pts.mean(axis=0),
            })

        return out

    def estimate_board_pose(
        self,
        image_bgr: np.ndarray,
        K: np.ndarray,
        dist: np.ndarray,
        min_tags: int = 4,
        debug_vis_path: Optional[str] = None,
    ) -> Tuple[Optional[np.ndarray], dict]:
        """
        从单张图像估计 T_cam_board。

        计算原理：
        - BoardModel 提供 marker 角点在标定板坐标系下的三维位置 obj_points。
        - ArUco 检测提供这些角点在图像上的二维像素位置 img_points。
        - solvePnP 在已知相机内参 K、畸变 dist 的条件下，求解一个刚体变换
          T_cam_board，使得 obj_points 经过该变换和相机投影后尽量落在 img_points 上。
        - 得到 T_cam_board 后再做一次重投影，RMSE 用来过滤角点对应错误、内参不匹配
          或图像质量过差的样本。
        """

        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        detections = self.detect(gray)

        # PnP 的核心输入：标定板平面上的 3D 角点 + 图像中的 2D 角点。
        obj_points, img_points, used_tag_ids = self.board.collect_correspondences(detections)

        info = {
            "num_detected_tags": len(detections),
            "num_used_tags": len(used_tag_ids),
            "used_tag_ids": used_tag_ids,
            "num_points": int(len(obj_points)),
            "reproj_rmse": None,
        }

        if len(used_tag_ids) < min_tags or len(obj_points) < 4 * min_tags:
            return None, info

        # solvePnP 返回的是 object/board -> camera 的 rvec/tvec，即本项目记作 T_cam_board。
        # 注意这里不是 camera -> board；如果后续需要相反方向，必须显式 invert_transform。
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            K,
            dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not ok:
            return None, info

        R, _ = cv2.Rodrigues(rvec)
        T_cam_board = make_transform(R, tvec.reshape(3))

        # 用同一个 T_cam_board 把 3D 角点投回图像。检测点和投影点的距离是单帧
        # 标定板位姿质量的直接证据，后面会用 --max_reproj_rmse 丢弃坏样本。
        proj = project_points(T_cam_board, obj_points, K, dist)
        reproj_err = np.linalg.norm(proj - img_points, axis=1)
        info["reproj_rmse"] = float(np.sqrt(np.mean(reproj_err ** 2)))
        if debug_vis_path is not None:
            vis = image_bgr.copy()

            def draw_point_safe(img, pt, radius, color):
                pt = np.asarray(pt, dtype=np.float64).reshape(-1)
                if pt.size < 2:
                    return
                x, y = pt[0], pt[1]
                if not np.isfinite(x) or not np.isfinite(y):
                    return
                x = int(round(float(x)))
                y = int(round(float(y)))

                h, w = img.shape[:2]
                if 0 <= x < w and 0 <= y < h:
                    cv2.circle(img, (x, y), radius, color, -1)

            # 绿色：检测到的真实角点
            for p in img_points:
                draw_point_safe(vis, p, 3, (0, 255, 0))

            # 红色：PnP 重投影角点
            for p in proj:
                draw_point_safe(vis, p, 2, (0, 0, 255))

            cv2.imwrite(debug_vis_path, vis)

        # if debug_vis_path is not None:
        #     vis = image_bgr.copy()

        #     # 绿色：检测角点
        #     for p in img_points.astype(int):
        #         cv2.circle(vis, tuple(p), 3, (0, 255, 0), -1)

        #     # 红色：PnP 重投影角点
        #     for p in proj.astype(int):
        #         cv2.circle(vis, tuple(p), 2, (0, 0, 255), -1)

        #     cv2.imwrite(debug_vis_path, vis)

        return T_cam_board, info


# ----------------------------
# Calibration core
# ----------------------------


# def compute_handeye_eye_in_hand(T_base_ee_list: List[np.ndarray],
#                                 T_cam_board_list: List[np.ndarray],
#                                 method: int = cv2.CALIB_HAND_EYE_TSAI) -> np.ndarray:
#     R_gripper2base, t_gripper2base = [], []
#     R_target2cam, t_target2cam = [], []
#     for T_base_ee, T_cam_board in zip(T_base_ee_list, T_cam_board_list):
#         T_ee_base = invert_transform(T_base_ee)
#         R_gripper2base.append(T_ee_base[:3, :3])
#         t_gripper2base.append(T_ee_base[:3, 3].reshape(3, 1))
#         R_target2cam.append(T_cam_board[:3, :3])
#         t_target2cam.append(T_cam_board[:3, 3].reshape(3, 1))

#     R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
#         R_gripper2base=R_gripper2base,
#         t_gripper2base=t_gripper2base,
#         R_target2cam=R_target2cam,
#         t_target2cam=t_target2cam,
#         method=method,
#     )
#     return make_transform(R_cam2gripper, t_cam2gripper.reshape(3))

def compute_handeye_eye_in_hand(
    T_base_ee_list: List[np.ndarray],
    T_cam_board_list: List[np.ndarray],
    method: int = cv2.CALIB_HAND_EYE_TSAI,
) -> np.ndarray:
    """
    求腕部相机的眼在手上外参 T_ee_cam_end。

    已知量：
    - T_base_ee_i：第 i 个样本中，机器人末端到基座的位姿，由 TCP 读数得到。
    - T_cam_board_i：第 i 个样本中，固定标定板到腕部相机的位姿，由 PnP 得到。

    未知量：
    - T_ee_cam_end：腕部相机到机器人末端坐标系的固定安装关系。

    核心约束：
    标定板在机器人基座中不动，所以每一帧都应满足：
        T_base_board = T_base_ee_i * T_ee_cam_end * T_cam_board_i
    对任意两帧 i、j 消去固定的 T_base_board，就形成经典手眼方程 A X = X B：
    - A 来自机器人末端在基座下的相对运动；
    - B 来自相机观测到的标定板相对运动；
    - X 就是要求的 T_ee_cam_end。

    OpenCV 的 calibrateHandEye 会内部构造并求解这个方程。这里传入的 gripper2base
    对应 T_base_ee，target2cam 对应 T_cam_board，返回的 cam2gripper 正好是
    T_ee_cam_end。
    """
    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []

    for T_base_ee, T_cam_board in zip(T_base_ee_list, T_cam_board_list):
        # OpenCV 参数名 gripper2base 的含义是“gripper/末端 -> base”，
        # 与本项目的 T_base_ee 坐标约定一致，不需要取逆。
        R_gripper2base.append(T_base_ee[:3, :3])
        t_gripper2base.append(T_base_ee[:3, 3].reshape(3, 1))

        # target2cam 的含义是“标定目标/标定板 -> 相机”，与 solvePnP 得到的
        # T_cam_board 一致，也不需要取逆。
        R_target2cam.append(T_cam_board[:3, :3])
        t_target2cam.append(T_cam_board[:3, 3].reshape(3, 1))

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base=R_gripper2base,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
        method=method,
    )

    # OpenCV 返回 cam2gripper，即“相机 -> 末端”，按本项目记法就是 T_ee_cam_end。
    return make_transform(R_cam2gripper, t_cam2gripper.reshape(3))


def estimate_T_base_board(
    T_base_ee_list: List[np.ndarray],
    T_ee_cam_end: np.ndarray,
    T_cam_board_list: List[np.ndarray],
) -> Tuple[np.ndarray, List[np.ndarray]]:
    """
    用腕部相机手眼结果反推出标定板在机器人基座下的固定位置 T_base_board。

    对每个样本：
        T_base_board_i = T_base_ee_i * T_ee_cam_end * T_cam_board_i
    如果手眼结果、PnP 和同步都可靠，这些 T_base_board_i 应该彼此接近。
    最终返回它们的平均值以及逐样本结果，后者用于一致性统计。
    """
    Ts = [
        compose(T_base_ee, T_ee_cam_end, T_cam_board)
        for T_base_ee, T_cam_board in zip(T_base_ee_list, T_cam_board_list)
    ]
    return average_transforms(Ts), Ts


def estimate_T_base_cam_fixed(
    T_base_board: np.ndarray,
    T_camfixed_board_list: List[np.ndarray],
) -> Tuple[np.ndarray, List[np.ndarray]]:
    """
    由固定标定板位姿和固定相机观测反推出固定相机外参 T_base_cam_fixed。

    固定相机的 PnP 给出：
        T_camfixed_board = 固定相机坐标系 <- 标定板坐标系
    因此它的逆变换 inv(T_camfixed_board) 是：
        T_board_camfixed = 标定板坐标系 <- 固定相机坐标系
    再接上已知的 T_base_board：
        T_base_cam_fixed_i = T_base_board * inv(T_camfixed_board_i)

    多帧结果理论上应相同；这里同样做平均并保留逐样本结果用于一致性检查。
    """
    Ts = [
        compose(T_base_board, invert_transform(T_cam_board))
        for T_cam_board in T_camfixed_board_list
    ]
    return average_transforms(Ts), Ts


# ----------------------------
# Saving / reporting
# ----------------------------


def transform_to_dict(T: np.ndarray) -> Dict:
    """把齐次变换保存成矩阵、平移、旋转矩阵和 Rodrigues 四种形式，便于后续程序和人工检查。"""
    return {
        "matrix_4x4": np.asarray(T, dtype=float).tolist(),
        "translation_xyz_m": np.asarray(T[:3, 3], dtype=float).tolist(),
        "rotation_matrix": np.asarray(T[:3, :3], dtype=float).tolist(),
        "rotation_rodrigues": matrix_to_rodrigues(T[:3, :3]).astype(float).tolist(),
    }


def save_json(path: str, data) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def print_transform(name: str, T: np.ndarray) -> None:
    print(f"\n{name}\n{'-' * len(name)}")
    print(np.array2string(T, precision=6, suppress_small=True))


def summarize_transform_list(name: str, Ts: List[np.ndarray], T_ref: np.ndarray) -> None:
    """打印逐样本变换相对最终平均结果的离散程度；离散越大，说明标定链路越不稳定。"""
    trans_errs, rot_errs = [], []
    for T in Ts:
        dt, dr = se3_distance(T_ref, T)
        trans_errs.append(dt)
        rot_errs.append(dr)
    print(f"\n{name} consistency:")
    print(f"  num_samples: {len(Ts)}")
    print(f"  translation err mean/std: {np.mean(trans_errs):.6f} / {np.std(trans_errs):.6f} m")
    print(f"  rotation    err mean/std: {np.mean(rot_errs):.6f} / {np.std(rot_errs):.6f} deg")


def str2bool(v: str) -> bool:
    if isinstance(v, bool):
        return v
    s = v.strip().lower()
    if s in {"1", "true", "t", "yes", "y"}:
        return True
    if s in {"0", "false", "f", "no", "n"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid bool value: {v}")


# ----------------------------
# Main
# ----------------------------


def _load_id_map(path: str | None) -> Optional[Dict[int, Tuple[int, int, int]]]:
    """读取 marker ID 到物理棋盘格位置的映射。"""
    if path is None:
        return None

    with open(path, "r", encoding="utf-8") as f:
        raw_map = json.load(f)

    id_map: Dict[int, Tuple[int, int, int]] = {}
    for k, v in raw_map.items():
        marker_id = int(k)
        row = int(v[0])
        col = int(v[1])
        rot = int(v[2]) if len(v) >= 3 else 0
        id_map[marker_id] = (row, col, rot)
    print(f"[INFO] Loaded id_map from {path}, num ids = {len(id_map)}")
    return id_map


def _build_board_from_args(args: argparse.Namespace) -> BoardModel:
    id_map = _load_id_map(args.id_map_json)
    board_cfg = BoardConfig(
        board_layout=args.board_layout,
        tag_size=args.tag_size,
        tag_family=args.tag_family,
        tag_cols=args.tag_cols,
        tag_rows=args.tag_rows,
        tag_spacing=args.tag_spacing,
        grid_cols=args.grid_cols,
        grid_rows=args.grid_rows,
        top_left_is_tag=args.top_left_is_tag,
        cell_size=args.cell_size,
        id_map=id_map,
    )
    board = BoardModel(board_cfg)
    print(f"[INFO] Board config: {json.dumps(board.describe(), ensure_ascii=False)}")
    print("[DEBUG] args.tag_size =", args.tag_size)
    print("[DEBUG] args.cell_size =", args.cell_size)
    print("[DEBUG] board_cfg.tag_size =", board_cfg.tag_size)
    print("[DEBUG] board_cfg.cell_size =", board_cfg.cell_size)

    if args.board_layout == "interleaved_checker":
        assert board_cfg.cell_size is not None, "cell_size 没有传进 BoardConfig"
        assert board_cfg.cell_size >= board_cfg.tag_size, "cell_size should be >= tag_size"
    return board


def _detect_valid_samples(
    samples: List[Sample],
    estimator: "ArucoBoardPoseEstimator",
    K_end: np.ndarray,
    dist_end: np.ndarray,
    debug_dir: Path,
    min_tags: int,
    max_reproj_rmse: float,
    K_fixed: Optional[np.ndarray] = None,
    dist_fixed: Optional[np.ndarray] = None,
    require_fixed: bool = True,
) -> Tuple[List[Sample], List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    """
    对样本逐帧估计标定板位姿。

    require_fixed=True 时执行原来的双相机过滤；False 时只过滤末端相机结果，
    用于眼在手上独立模式。
    """
    valid_samples: List[Sample] = []
    T_base_ee_list: List[np.ndarray] = []
    T_camend_board_list: List[np.ndarray] = []
    T_camfixed_board_list: List[np.ndarray] = []

    for idx, s in enumerate(samples):
        img_end = cv2.imread(str(s.image_end))
        if img_end is None:
            print(f"[WARN] Failed to read end image for {s.json_path.name}, skip.")
            continue

        T_camend_board, info_end = estimator.estimate_board_pose(
            img_end, K_end, dist_end, min_tags=min_tags,
            debug_vis_path=str(debug_dir / f"{idx:03d}_end.jpg"),
        )

        T_camfixed_board = None
        info_fixed = None
        if require_fixed:
            if s.image_fixed is None:
                print(f"[WARN] Missing fixed image for {s.json_path.name}, skip.")
                continue
            if K_fixed is None or dist_fixed is None:
                raise ValueError("dual_camera 模式必须提供固定相机内参。")
            img_fixed = cv2.imread(str(s.image_fixed))
            if img_fixed is None:
                print(f"[WARN] Failed to read fixed image for {s.json_path.name}, skip.")
                continue
            T_camfixed_board, info_fixed = estimator.estimate_board_pose(
                img_fixed, K_fixed, dist_fixed, min_tags=min_tags,
                debug_vis_path=str(debug_dir / f"{idx:03d}_fixed.jpg"),
            )

        if require_fixed and info_fixed is not None:
            print(
                f"[{idx:03d}] {s.json_path.name} | "
                f"end used={info_end['num_used_tags']}/{info_end['num_detected_tags']}, rmse={info_end['reproj_rmse']} | "
                f"fixed used={info_fixed['num_used_tags']}/{info_fixed['num_detected_tags']}, rmse={info_fixed['reproj_rmse']}"
            )
        else:
            print(
                f"[{idx:03d}] {s.json_path.name} | "
                f"end used={info_end['num_used_tags']}/{info_end['num_detected_tags']}, rmse={info_end['reproj_rmse']}"
            )

        if T_camend_board is None:
            print("      -> skip (end camera board pose failed)")
            continue
        if info_end["reproj_rmse"] is None:
            print("      -> skip (end camera rmse is None)")
            continue
        if info_end["reproj_rmse"] > max_reproj_rmse:
            print(f"      -> skip (end rmse too large: {info_end['reproj_rmse']:.3f})")
            continue

        if require_fixed:
            if T_camfixed_board is None:
                print("      -> skip (fixed camera board pose failed)")
                continue
            if info_fixed is None or info_fixed["reproj_rmse"] is None:
                print("      -> skip (fixed camera rmse is None)")
                continue
            if info_fixed["reproj_rmse"] > max_reproj_rmse:
                print(
                    f"      -> skip (fixed rmse too large: "
                    f"{info_fixed['reproj_rmse']:.3f})"
                )
                continue
            T_camfixed_board_list.append(T_camfixed_board)

        valid_samples.append(s)
        T_base_ee_list.append(s.T_base_ee)
        T_camend_board_list.append(T_camend_board)

    return valid_samples, T_base_ee_list, T_camend_board_list, T_camfixed_board_list


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, default="dual_camera",
                        choices=["dual_camera", "eye_in_hand"],
                        help="dual_camera 保持原双相机联合流程；eye_in_hand 只跑腕部相机眼在手上标定。")
    parser.add_argument("--dataset_dir", type=str, required=True)
    parser.add_argument("--fixed_camera_index", type=int, default=1)
    parser.add_argument("--end_camera_index", type=int, default=0)
    parser.add_argument("--intr_fixed", type=str, default=None)
    parser.add_argument("--intr_end", type=str, required=True)

    parser.add_argument("--board_layout", type=str, default="interleaved_checker",
                        choices=["regular_aprilgrid", "interleaved_checker"])
    parser.add_argument("--tag_size", type=float, required=True,
                        help="ArUco marker outer square side length in meters.")

    parser.add_argument("--cell_size", type=float, default=None,
                        help="For interleaved_checker: physical checkerboard cell side length in meters.")
    parser.add_argument("--tag_family", type=str, default="tag16h5")
    parser.add_argument("--aruco_dict", type=str, default="DICT_6X6_250")
    parser.add_argument("--tag_cols", type=int, default=None,
                        help="For regular_aprilgrid only.")
    parser.add_argument("--tag_rows", type=int, default=None,
                        help="For regular_aprilgrid only.")
    parser.add_argument("--tag_spacing", type=float, default=0.0,
                        help="For regular_aprilgrid only. gap/tag_size.")

    parser.add_argument("--grid_cols", type=int, default=None,
                        help="For interleaved_checker only: total cols including black cells.")
    parser.add_argument("--grid_rows", type=int, default=None,
                        help="For interleaved_checker only: total rows including black cells.")
    parser.add_argument("--top_left_is_tag", type=str2bool, default=False,
                        help="For interleaved_checker only. Use true/false.")
    parser.add_argument("--id_map_json", type=str, default=None,
                    help="JSON file mapping marker id to [row, col].")
    parser.add_argument("--max_reproj_rmse", type=float, default=5.0,
                    help="Skip sample if required camera reprojection RMSE is larger than this value.")
    parser.add_argument("--translation_scale", type=float, default=1.0)
    parser.add_argument("--min_tags", type=int, default=4)
    parser.add_argument("--output_dir", type=str, default="outputs/calib_output")
    parser.add_argument("--handeye_method", type=str, default="tsai",
                        choices=["tsai", "park", "horaud", "andreff", "daniilidis"])
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    debug_dir = Path(args.output_dir) / "debug_vis"
    debug_dir.mkdir(parents=True, exist_ok=True)

    method_map = {
        "tsai": cv2.CALIB_HAND_EYE_TSAI,
        "park": cv2.CALIB_HAND_EYE_PARK,
        "horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    if args.mode == "dual_camera" and not args.intr_fixed:
        raise ValueError("dual_camera 模式必须提供 --intr_fixed。")

    K_fixed, dist_fixed = (
        load_intrinsics(args.intr_fixed)
        if args.mode == "dual_camera" and args.intr_fixed
        else (None, None)
    )
    K_end, dist_end = load_intrinsics(args.intr_end)

    require_fixed = args.mode == "dual_camera"
    samples = load_samples(
        args.dataset_dir,
        end_camera_index=args.end_camera_index,
        translation_scale=args.translation_scale,
        fixed_camera_index=args.fixed_camera_index,
        require_fixed=require_fixed,
    )
    print(f"[INFO] Found {len(samples)} usable samples.")
    if len(samples) < 5:
        raise RuntimeError("Too few samples. At least 5; recommend 15-30.")

    board = _build_board_from_args(args)
    estimator = ArucoBoardPoseEstimator(board, args.aruco_dict)

    valid_samples, T_base_ee_list, T_camend_board_list, T_camfixed_board_list = _detect_valid_samples(
        samples=samples,
        estimator=estimator,
        K_end=K_end,
        dist_end=dist_end,
        debug_dir=debug_dir,
        min_tags=args.min_tags,
        max_reproj_rmse=args.max_reproj_rmse,
        K_fixed=K_fixed,
        dist_fixed=dist_fixed,
        require_fixed=require_fixed,
    )

    print(f"[INFO] Valid samples after detection: {len(valid_samples)}")
    if len(valid_samples) < 5:
        raise RuntimeError("Too few valid samples after AprilTag detection.")

    # 第一阶段：用末端相机 + 机器人运动求眼在手上外参。
    T_ee_cam_end = compute_handeye_eye_in_hand(T_base_ee_list, T_camend_board_list, method_map[args.handeye_method])
    print_transform("T_ee_cam_end", T_ee_cam_end)

    # 第二阶段：把末端相机看到的同一块固定标定板统一转换到机器人基座坐标系。
    T_base_board, T_base_board_all = estimate_T_base_board(T_base_ee_list, T_ee_cam_end, T_camend_board_list)
    print_transform("T_base_board", T_base_board)
    summarize_transform_list("T_base_board", T_base_board_all, T_base_board)

    result = {
        "mode": args.mode,
        "num_total_samples": len(samples),
        "num_valid_samples": len(valid_samples),
        "handeye_method": args.handeye_method,
        "board_config": board.describe(),
        "T_ee_cam_end": transform_to_dict(T_ee_cam_end),
        "T_base_board": transform_to_dict(T_base_board),
    }

    if args.mode == "dual_camera":
        # 第三阶段：固定相机也看到了同一块标定板，因此可由 T_base_board 反推出固定相机外参。
        T_base_cam_fixed, T_base_cam_fixed_all = estimate_T_base_cam_fixed(T_base_board, T_camfixed_board_list)
        print_transform("T_base_cam_fixed", T_base_cam_fixed)
        summarize_transform_list("T_base_cam_fixed", T_base_cam_fixed_all, T_base_cam_fixed)
        result["T_base_cam_fixed"] = transform_to_dict(T_base_cam_fixed)

    save_json(str(Path(args.output_dir) / "calibration_result.json"), result)

    dynamic = []
    for s, T_base_ee in zip(valid_samples, T_base_ee_list):
        # 腕部相机随机器人末端运动，所以它的基座外参不是常量，需要每帧由 T_base_ee 组合得到。
        T_base_cam_end_i = compose(T_base_ee, T_ee_cam_end)
        dynamic.append({
            "sample_json": str(s.json_path),
            "timestamp": s.raw.get("timestamp", None),
            "T_base_cam_end": transform_to_dict(T_base_cam_end_i),
        })
        if args.mode == "dual_camera":
            # 表示末端相机坐标系中的点如何转换到固定相机坐标系，便于双相机相对位姿分析。
            dynamic[-1]["T_cam_fixed_cam_end"] = transform_to_dict(compose(invert_transform(T_base_cam_fixed), T_base_cam_end_i))
    save_json(str(Path(args.output_dir) / "dynamic_end_camera_poses.json"), dynamic)

    print(f"\n[OK] Results saved to: {args.output_dir}")
    print("  - calibration_result.json")
    print("  - dynamic_end_camera_poses.json")
    print("  - debug_vis/*.jpg")


if __name__ == "__main__":
    main()
