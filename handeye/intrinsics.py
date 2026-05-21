from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def _field(source: Any, name: str, default: Any = None) -> Any:
    if isinstance(source, dict):
        return source.get(name, default)
    return getattr(source, name, default)


def enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str) and name:
        return name
    text = str(value)
    return text.split(".")[-1]


def intrinsics_to_dict(intrinsics: Any, fov_deg: tuple[float, float] | None = None) -> dict[str, Any]:
    """把 pyrealsense2 intrinsics 或同结构 dict 转成可序列化字段。"""
    coeffs = _field(intrinsics, "coeffs", [])
    if coeffs is None:
        coeffs = []

    payload: dict[str, Any] = {
        "width": int(_field(intrinsics, "width")),
        "height": int(_field(intrinsics, "height")),
        "fx": float(_field(intrinsics, "fx")),
        "fy": float(_field(intrinsics, "fy")),
        "ppx": float(_field(intrinsics, "ppx")),
        "ppy": float(_field(intrinsics, "ppy")),
        "distortion_model": enum_name(_field(intrinsics, "model", _field(intrinsics, "distortion_model", "unknown"))),
        "coeffs": [float(value) for value in coeffs],
    }

    if fov_deg is not None:
        payload["fov_deg"] = {"x": float(fov_deg[0]), "y": float(fov_deg[1])}
    elif isinstance(intrinsics, dict) and "fov_deg" in intrinsics:
        payload["fov_deg"] = intrinsics["fov_deg"]

    return payload


def to_calib_intrinsics(intrinsics: Any, *, source: dict[str, Any] | None = None) -> dict[str, Any]:
    """转成 calib.py 可直接读取的 K/dist JSON 格式。"""
    item = intrinsics_to_dict(intrinsics)
    dist = list(item["coeffs"])
    while len(dist) < 5:
        dist.append(0.0)

    payload: dict[str, Any] = {
        "K": [
            [float(item["fx"]), 0.0, float(item["ppx"])],
            [0.0, float(item["fy"]), float(item["ppy"])],
            [0.0, 0.0, 1.0],
        ],
        "dist": [float(value) for value in dist[:8]],
        "width": int(item["width"]),
        "height": int(item["height"]),
        "distortion_model": item.get("distortion_model", "unknown"),
    }

    if source:
        payload["source"] = source
    return payload


def save_calib_intrinsics(path: str | Path, intrinsics: Any, *, source: dict[str, Any] | None = None) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = to_calib_intrinsics(intrinsics, source=source)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def load_realsense_intrinsics_bundle(path: str | Path) -> list[dict[str, Any]]:
    """读取 RealSense 内参采集脚本输出的 cameras 列表。"""
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    cameras = payload.get("cameras")
    if not isinstance(cameras, list):
        raise ValueError(f"内参文件缺少 cameras 列表: {path}")
    return cameras


def find_camera_intrinsics(cameras: list[dict[str, Any]], *, serial: str | None = None, name_contains: str | None = None) -> dict[str, Any]:
    for camera in cameras:
        if serial and str(camera.get("serial")) == str(serial):
            return camera
        if name_contains and name_contains.lower() in str(camera.get("name", "")).lower():
            return camera
    raise KeyError(f"没有找到匹配的相机内参: serial={serial}, name_contains={name_contains}")
