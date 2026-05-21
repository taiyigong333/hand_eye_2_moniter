from __future__ import annotations

import time
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from .intrinsics import intrinsics_to_dict


@dataclass(frozen=True)
class CameraSpec:
    role: str
    camera_index: int
    camera_name: str
    serial: str | None
    color_width: int
    color_height: int
    fps: int
    image_file: str
    intrinsics_path: str | None = None
    enable_depth: bool = False
    depth_width: int | None = None
    depth_height: int | None = None
    depth_fps: int | None = None
    depth_file: str | None = None
    align_depth_to_color: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CameraSpec":
        role = str(data["role"])
        camera_index = int(data["camera_index"])
        image_file = str(data.get("image_file") or f"cam{camera_index}_{role}.png")
        return cls(
            role=role,
            camera_index=camera_index,
            camera_name=str(data.get("camera_name") or f"{role}_camera"),
            serial=str(data["serial"]) if data.get("serial") else None,
            color_width=int(data["color_width"]),
            color_height=int(data["color_height"]),
            fps=int(data.get("fps", 30)),
            image_file=image_file,
            intrinsics_path=str(data["intrinsics_path"]) if data.get("intrinsics_path") else None,
            enable_depth=bool(data.get("enable_depth", False)),
            depth_width=int(data["depth_width"]) if data.get("depth_width") else None,
            depth_height=int(data["depth_height"]) if data.get("depth_height") else None,
            depth_fps=int(data["depth_fps"]) if data.get("depth_fps") else None,
            depth_file=str(data["depth_file"]) if data.get("depth_file") else None,
            align_depth_to_color=bool(data.get("align_depth_to_color", True)),
        )


@dataclass
class CapturedFrame:
    spec: CameraSpec
    color_bgr: np.ndarray
    depth_mm: np.ndarray | None
    depth_scale_m: float | None
    timestamp_ms: float
    color_intrinsics: dict[str, Any]
    depth_intrinsics: dict[str, Any] | None


class RealSenseCaptureSystem:
    """双 RealSense 同步采集封装；深度流可按配置打开但标定默认只用 RGB。"""

    def __init__(self, specs: list[CameraSpec], warmup_frames: int = 8):
        self.specs = specs
        self.warmup_frames = int(warmup_frames)
        self._rs = None
        self._items: list[dict[str, Any]] = []

    def start(self) -> None:
        try:
            import pyrealsense2 as rs
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(
                "未找到 pyrealsense2，请在 hand_eye 环境安装 pyrealsense2。"
            ) from exc

        self._rs = rs
        resolved_specs = self._resolve_serials(rs, self.specs)

        for spec in resolved_specs:
            pipeline = rs.pipeline()
            config = rs.config()
            if spec.serial:
                config.enable_device(spec.serial)
            config.enable_stream(
                rs.stream.color,
                spec.color_width,
                spec.color_height,
                rs.format.bgr8,
                spec.fps,
            )
            if spec.enable_depth:
                config.enable_stream(
                    rs.stream.depth,
                    spec.depth_width or spec.color_width,
                    spec.depth_height or spec.color_height,
                    rs.format.z16,
                    spec.depth_fps or spec.fps,
                )

            profile = pipeline.start(config)
            align = rs.align(rs.stream.color) if spec.enable_depth and spec.align_depth_to_color else None

            for _ in range(self.warmup_frames):
                try:
                    pipeline.wait_for_frames(1000)
                except RuntimeError:
                    pass

            color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            color_intr = color_profile.get_intrinsics()
            color_fov = rs.rs2_fov(color_intr)

            depth_intr = None
            depth_scale_m = None
            if spec.enable_depth:
                try:
                    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
                    raw_depth_intr = depth_profile.get_intrinsics()
                    depth_intr = intrinsics_to_dict(raw_depth_intr, rs.rs2_fov(raw_depth_intr))
                    depth_sensor = profile.get_device().first_depth_sensor()
                    depth_scale_m = float(depth_sensor.get_depth_scale())
                except Exception:
                    depth_intr = None
                    depth_scale_m = None

            self._items.append(
                {
                    "spec": spec,
                    "pipeline": pipeline,
                    "align": align,
                    "color_intrinsics": intrinsics_to_dict(color_intr, color_fov),
                    "depth_intrinsics": depth_intr,
                    "depth_scale_m": depth_scale_m,
                }
            )
            serial_text = spec.serial or "auto"
            print(
                f"[camera] 已启动 {spec.role}: {spec.camera_name}, "
                f"S/N={serial_text}, color={spec.color_width}x{spec.color_height}@{spec.fps}"
            )

    def capture_all(self, timeout_ms: int = 2000) -> dict[str, CapturedFrame]:
        if not self._items:
            raise RuntimeError("RealSense 尚未启动。")

        frames: dict[str, CapturedFrame] = {}
        for item in self._items:
            spec: CameraSpec = item["spec"]
            frame_set = item["pipeline"].wait_for_frames(int(timeout_ms))
            if item["align"] is not None:
                frame_set = item["align"].process(frame_set)

            color_frame = frame_set.get_color_frame()
            if not color_frame:
                raise RuntimeError(f"相机 {spec.role} 未获取到 color frame。")

            depth_mm = None
            if spec.enable_depth:
                depth_frame = frame_set.get_depth_frame()
                if depth_frame:
                    depth_mm = np.asanyarray(depth_frame.get_data()).copy()

            frames[spec.role] = CapturedFrame(
                spec=spec,
                color_bgr=np.asanyarray(color_frame.get_data()).copy(),
                depth_mm=depth_mm,
                depth_scale_m=item["depth_scale_m"],
                timestamp_ms=float(getattr(color_frame, "get_timestamp", lambda: time.time() * 1000.0)()),
                color_intrinsics=item["color_intrinsics"],
                depth_intrinsics=item["depth_intrinsics"],
            )

        return frames

    def color_intrinsics(self) -> dict[str, dict[str, Any]]:
        return {
            item["spec"].role: item["color_intrinsics"]
            for item in self._items
        }

    def stop(self) -> None:
        for item in self._items:
            try:
                item["pipeline"].stop()
            except Exception:
                pass
        self._items.clear()

    def __enter__(self) -> "RealSenseCaptureSystem":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def _resolve_serials(self, rs: Any, specs: list[CameraSpec]) -> list[CameraSpec]:
        if all(spec.serial for spec in specs):
            return specs

        ctx = rs.context()
        devices = []
        for dev in ctx.query_devices():
            devices.append(
                {
                    "serial": str(dev.get_info(rs.camera_info.serial_number)),
                    "name": str(dev.get_info(rs.camera_info.name)),
                }
            )
        if len(devices) < len(specs):
            raise RuntimeError(f"RealSense 数量不足：需要 {len(specs)} 个，只找到 {len(devices)} 个。")

        used = {spec.serial for spec in specs if spec.serial}
        available = [dev for dev in sorted(devices, key=_device_sort_key) if dev["serial"] not in used]
        resolved: list[CameraSpec] = []
        for spec in specs:
            if spec.serial:
                resolved.append(spec)
                continue
            if not available:
                raise RuntimeError("没有可分配的 RealSense 序列号。")
            dev = _pick_device_for_role(spec.role, available)
            available.remove(dev)
            print(f"[camera] {spec.role} 未配置 serial，自动分配 {dev['name']} ({dev['serial']})")
            resolved.append(replace(spec, serial=dev["serial"]))
        return resolved


def _device_sort_key(device: dict[str, str]) -> tuple[int, str]:
    name = device["name"].upper()
    if "D405" in name:
        return (0, device["serial"])
    if "D435" in name:
        return (1, device["serial"])
    return (9, device["serial"])


def _pick_device_for_role(role: str, devices: list[dict[str, str]]) -> dict[str, str]:
    role_l = role.lower()
    if role_l in {"end", "wrist"}:
        for dev in devices:
            if "D405" in dev["name"].upper():
                return dev
    if role_l in {"fixed", "main"}:
        for dev in devices:
            if "D435" in dev["name"].upper():
                return dev
    return devices[0]
