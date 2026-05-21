from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2

from .realsense import CapturedFrame


class CalibrationDatasetWriter:
    """把实时采集样本保存成 calib.py 已支持的 sample_xxx/pose.json 格式。"""

    def __init__(self, root: str | Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self._next_index = self._scan_next_index()

    def write_session_config(self, config: dict[str, Any]) -> Path:
        path = self.root / "capture_session_config.json"
        path.write_text(
            json.dumps(config, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return path

    def write_sample(
        self,
        *,
        tcp_pose: list[float],
        frames: dict[str, CapturedFrame],
        joint_angles: list[float] | None,
        robot_host: str,
    ) -> Path:
        sample_index = self._next_index
        self._next_index += 1

        sample_dir = self.root / f"sample_{sample_index:03d}"
        sample_dir.mkdir(parents=False, exist_ok=False)

        image_entries = []
        for role, frame in frames.items():
            spec = frame.spec
            image_file = spec.image_file
            image_path = sample_dir / image_file
            if not cv2.imwrite(str(image_path), frame.color_bgr):
                raise RuntimeError(f"写入图像失败: {image_path}")

            entry: dict[str, Any] = {
                "file": image_file,
                "camera_index": int(spec.camera_index),
                "camera_name": spec.camera_name,
                "role": role,
                "serial": spec.serial,
                "timestamp_ms": frame.timestamp_ms,
                "intrinsics_file": spec.intrinsics_path,
            }

            if spec.enable_depth and frame.depth_mm is not None:
                depth_file = spec.depth_file or f"depth_cam{spec.camera_index}_{role}.png"
                depth_path = sample_dir / depth_file
                if not cv2.imwrite(str(depth_path), frame.depth_mm):
                    raise RuntimeError(f"写入深度图失败: {depth_path}")
                entry["depth_file"] = depth_file
                entry["depth_unit"] = "raw_z16"
                entry["depth_scale_m"] = frame.depth_scale_m

            image_entries.append(entry)

        payload: dict[str, Any] = {
            "sample_index": sample_index,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "tcp_pose": [float(value) for value in tcp_pose],
            "tcp_pose_fields": ["x", "y", "z", "rx", "ry", "rz"],
            "robot": {
                "host": robot_host,
                "tcp_source": "rtde_receive.getActualTCPPose",
            },
            "images": sorted(image_entries, key=lambda item: int(item["camera_index"])),
        }

        if joint_angles is not None:
            payload["joint_angles"] = [float(value) for value in joint_angles]
            payload["joint_angle_fields"] = [f"q{i}" for i in range(len(joint_angles))]

        pose_path = sample_dir / "pose.json"
        pose_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return sample_dir

    def _scan_next_index(self) -> int:
        pattern = re.compile(r"^sample_(\d+)$")
        max_index = -1
        for path in self.root.iterdir():
            if not path.is_dir():
                continue
            match = pattern.match(path.name)
            if match:
                max_index = max(max_index, int(match.group(1)))
        return max_index + 1


def count_pose_json(dataset_dir: str | Path) -> int:
    root = Path(dataset_dir)
    if not root.exists():
        return 0
    return sum(1 for _ in root.rglob("pose.json"))
