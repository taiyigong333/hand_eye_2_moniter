#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot + fixed camera + eye-in-hand dual-camera calibration.

This version supports two board layouts:
1) regular_aprilgrid   : every grid location is an AprilTag
2) interleaved_checker : AprilTag / pure-black squares alternate like a checkerboard

For your board, use:
  --board_layout interleaved_checker \
  --grid_cols 20 \
  --grid_rows 15 \
  --tag_size 0.015 \
  --top_left_is_tag false   # change to true if your first cell is a tag

Coordinate convention:
- T_A_B means transform from frame B to frame A.
- p_A = T_A_B @ p_B
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
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return R


def matrix_to_rodrigues(R: np.ndarray) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(R)
    return rvec.reshape(3)


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def compose(*Ts: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    for T in Ts:
        out = out @ T
    return out


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
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
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def average_transforms(Ts: List[np.ndarray]) -> np.ndarray:
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
    dT = invert_transform(T1) @ T2
    trans = float(np.linalg.norm(dT[:3, 3]))
    cos_theta = (np.trace(dT[:3, :3]) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    rot_deg = float(np.degrees(np.arccos(cos_theta)))
    return trans, rot_deg


def project_points(T_cam_obj: np.ndarray, object_points: np.ndarray, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
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

    # 新增：真实 marker ID 到棋盘格 row/col 的映射
    id_map: Optional[Dict[int, Tuple[int, int, int]]] = None


class BoardModel:
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
        if self.layout == "regular_aprilgrid":
            return self._regular_tag_corners(tag_id)
        return self._interleaved_tag_corners(tag_id)

    def _regular_tag_corners(self, tag_id: int) -> Optional[np.ndarray]:
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

        # 优先使用真实 ID -> 棋盘格位置映射
        if self.cfg.id_map is not None:
            rc = self.cfg.id_map.get(int(tag_id), None)
            if rc is None:
                return None

            row, col = int(rc[0]), int(rc[1])

            if not (0 <= row < rows and 0 <= col < cols):
                return None

            return row, col

        # 没有 id_map 时，才使用默认 row-major 规则
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
        if self.cfg.id_map is not None:
            rc = self.cfg.id_map.get(int(tag_id), None)
            if rc is not None and len(rc) >= 3:
                return int(rc[2]) % 4
        return 0
    def _interleaved_tag_corners(self, tag_id: int) -> Optional[np.ndarray]:
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

        rot = self._interleaved_tag_rotation(tag_id)
        corners = np.roll(corners, -rot, axis=0)

        return corners

    def collect_correspondences(self, detections: List[dict]) -> Tuple[np.ndarray, np.ndarray, List[int]]:
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
    image_fixed: Path
    image_end: Path
    raw: dict


def tcp_pose_to_T_base_ee(tcp_pose: List[float], translation_scale: float = 1.0) -> np.ndarray:
    if len(tcp_pose) != 6:
        raise ValueError(f"tcp_pose must have length 6, got {len(tcp_pose)}")
    x, y, z, rx, ry, rz = [float(v) for v in tcp_pose]
    t = np.array([x, y, z], dtype=np.float64) * float(translation_scale)
    R = rodrigues_to_matrix(np.array([rx, ry, rz], dtype=np.float64))
    return make_transform(R, t)


def load_samples(dataset_dir: str, fixed_camera_index: int, end_camera_index: int, translation_scale: float) -> List[Sample]:
    json_files = sorted(Path(dataset_dir).rglob("*.json"))
    samples = []
    for jp in json_files:
        with open(jp, "r", encoding="utf-8") as f:
            data = json.load(f)
        if "tcp_pose" not in data or "images" not in data:
            continue

        fixed_rel, end_rel = None, None
        for item in data["images"]:
            if int(item["camera_index"]) == int(fixed_camera_index):
                fixed_rel = item["file"]
            if int(item["camera_index"]) == int(end_camera_index):
                end_rel = item["file"]
        if fixed_rel is None or end_rel is None:
            continue

        fixed_path = jp.parent / fixed_rel
        end_path = jp.parent / end_rel
        if not fixed_path.exists() or not end_path.exists():
            print(f"[WARN] Missing image for {jp.name}, skip.")
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

        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        detections = self.detect(gray)

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
    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []

    for T_base_ee, T_cam_board in zip(T_base_ee_list, T_cam_board_list):
        # OpenCV wants gripper -> base, i.e. T_base_ee
        R_gripper2base.append(T_base_ee[:3, :3])
        t_gripper2base.append(T_base_ee[:3, 3].reshape(3, 1))

        # target(board) -> camera, i.e. T_cam_board
        R_target2cam.append(T_cam_board[:3, :3])
        t_target2cam.append(T_cam_board[:3, 3].reshape(3, 1))

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base=R_gripper2base,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
        method=method,
    )

    # This is T_gripper_cam, same as T_ee_cam_end
    return make_transform(R_cam2gripper, t_cam2gripper.reshape(3))
def estimate_T_base_board(T_base_ee_list: List[np.ndarray], T_ee_cam_end: np.ndarray, T_cam_board_list: List[np.ndarray]) -> Tuple[np.ndarray, List[np.ndarray]]:
    Ts = [compose(T_base_ee, T_ee_cam_end, T_cam_board) for T_base_ee, T_cam_board in zip(T_base_ee_list, T_cam_board_list)]
    return average_transforms(Ts), Ts


def estimate_T_base_cam_fixed(T_base_board: np.ndarray, T_camfixed_board_list: List[np.ndarray]) -> Tuple[np.ndarray, List[np.ndarray]]:
    Ts = [compose(T_base_board, invert_transform(T_cam_board)) for T_cam_board in T_camfixed_board_list]
    return average_transforms(Ts), Ts


# ----------------------------
# Saving / reporting
# ----------------------------


def transform_to_dict(T: np.ndarray) -> Dict:
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset_dir", type=str, required=True)
    parser.add_argument("--fixed_camera_index", type=int, default=1)
    parser.add_argument("--end_camera_index", type=int, default=0)
    parser.add_argument("--intr_fixed", type=str, required=True)
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
                    help="Skip sample if end/fixed reprojection RMSE is larger than this value.")
    parser.add_argument("--translation_scale", type=float, default=1.0)
    parser.add_argument("--min_tags", type=int, default=4)
    parser.add_argument("--output_dir", type=str, default="calib_output")
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

    K_fixed, dist_fixed = load_intrinsics(args.intr_fixed)
    K_end, dist_end = load_intrinsics(args.intr_end)

    samples = load_samples(
        args.dataset_dir,
        fixed_camera_index=args.fixed_camera_index,
        end_camera_index=args.end_camera_index,
        translation_scale=args.translation_scale,
    )
    print(f"[INFO] Found {len(samples)} usable samples.")
    if len(samples) < 5:
        raise RuntimeError("Too few samples. At least 5; recommend 15-30.")
    id_map = None
    if args.id_map_json is not None:
        with open(args.id_map_json, "r", encoding="utf-8") as f:
            raw_map = json.load(f)

        id_map = {}
        for k, v in raw_map.items():
            marker_id = int(k)
            row = int(v[0])
            col = int(v[1])
            rot = int(v[2]) if len(v) >= 3 else 0
            id_map[marker_id] = (row, col, rot)
        print(f"[INFO] Loaded id_map from {args.id_map_json}, num ids = {len(id_map)}")

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
    #estimator = AprilTagBoardPoseEstimator(board, board_cfg.tag_family)
    estimator = ArucoBoardPoseEstimator(board, args.aruco_dict)

    valid_samples = []
    T_base_ee_list = []
    T_camend_board_list = []
    T_camfixed_board_list = []

    for idx, s in enumerate(samples):
        img_end = cv2.imread(str(s.image_end))
        img_fixed = cv2.imread(str(s.image_fixed))
        if img_end is None or img_fixed is None:
            print(f"[WARN] Failed to read images for {s.json_path.name}, skip.")
            continue

        T_camend_board, info_end = estimator.estimate_board_pose(
            img_end, K_end, dist_end, min_tags=args.min_tags,
            debug_vis_path=str(debug_dir / f"{idx:03d}_end.jpg"),
        )
        T_camfixed_board, info_fixed = estimator.estimate_board_pose(
            img_fixed, K_fixed, dist_fixed, min_tags=args.min_tags,
            debug_vis_path=str(debug_dir / f"{idx:03d}_fixed.jpg"),
        )

        print(
            f"[{idx:03d}] {s.json_path.name} | "
            f"end used={info_end['num_used_tags']}/{info_end['num_detected_tags']}, rmse={info_end['reproj_rmse']} | "
            f"fixed used={info_fixed['num_used_tags']}/{info_fixed['num_detected_tags']}, rmse={info_fixed['reproj_rmse']}"
        )

        if T_camend_board is None or T_camfixed_board is None:
            print("      -> skip (board pose failed in one camera)")
            continue
        if info_end["reproj_rmse"] is None or info_fixed["reproj_rmse"] is None:
            print("      -> skip (rmse is None)")
            continue

        if info_end["reproj_rmse"] > args.max_reproj_rmse or info_fixed["reproj_rmse"] > args.max_reproj_rmse:
            print(
                f"      -> skip (rmse too large: "
                f"end={info_end['reproj_rmse']:.3f}, "
                f"fixed={info_fixed['reproj_rmse']:.3f})"
            )
            continue

        valid_samples.append(s)
        T_base_ee_list.append(s.T_base_ee)
        T_camend_board_list.append(T_camend_board)
        T_camfixed_board_list.append(T_camfixed_board)

    print(f"[INFO] Valid samples after detection: {len(valid_samples)}")
    if len(valid_samples) < 5:
        raise RuntimeError("Too few valid samples after AprilTag detection.")

    T_ee_cam_end = compute_handeye_eye_in_hand(T_base_ee_list, T_camend_board_list, method_map[args.handeye_method])
    print_transform("T_ee_cam_end", T_ee_cam_end)

    T_base_board, T_base_board_all = estimate_T_base_board(T_base_ee_list, T_ee_cam_end, T_camend_board_list)
    print_transform("T_base_board", T_base_board)
    summarize_transform_list("T_base_board", T_base_board_all, T_base_board)

    T_base_cam_fixed, T_base_cam_fixed_all = estimate_T_base_cam_fixed(T_base_board, T_camfixed_board_list)
    print_transform("T_base_cam_fixed", T_base_cam_fixed)
    summarize_transform_list("T_base_cam_fixed", T_base_cam_fixed_all, T_base_cam_fixed)

    result = {
        "num_total_samples": len(samples),
        "num_valid_samples": len(valid_samples),
        "handeye_method": args.handeye_method,
        "board_config": board.describe(),
        "T_ee_cam_end": transform_to_dict(T_ee_cam_end),
        "T_base_board": transform_to_dict(T_base_board),
        "T_base_cam_fixed": transform_to_dict(T_base_cam_fixed),
    }
    save_json(str(Path(args.output_dir) / "calibration_result.json"), result)

    dynamic = []
    for s, T_base_ee in zip(valid_samples, T_base_ee_list):
        T_base_cam_end_i = compose(T_base_ee, T_ee_cam_end)
        dynamic.append({
            "sample_json": str(s.json_path),
            "timestamp": s.raw.get("timestamp", None),
            "T_base_cam_end": transform_to_dict(T_base_cam_end_i),
            "T_cam_fixed_cam_end": transform_to_dict(compose(invert_transform(T_base_cam_fixed), T_base_cam_end_i)),
        })
    save_json(str(Path(args.output_dir) / "dynamic_end_camera_poses.json"), dynamic)

    print(f"\n[OK] Results saved to: {args.output_dir}")
    print("  - calibration_result.json")
    print("  - dynamic_end_camera_poses.json")
    print("  - debug_vis/*.jpg")


if __name__ == "__main__":
    main()
