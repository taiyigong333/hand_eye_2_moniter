from __future__ import annotations

from typing import Any

import cv2

from .realsense import CapturedFrame


class OpenCVCameraPreview:
    """采集期间的 OpenCV 实时预览窗口。"""

    def __init__(
        self,
        *,
        enabled: bool = True,
        scale: float = 0.5,
        wait_ms: int = 1,
        window_prefix: str = "hand-eye",
    ) -> None:
        self.enabled = bool(enabled)
        self.scale = _valid_scale(scale)
        self.wait_ms = max(1, int(wait_ms))
        self.window_prefix = str(window_prefix)
        self._window_names: set[str] = set()

    def show(self, frames: dict[str, CapturedFrame]) -> str | None:
        """刷新全部相机窗口，并返回 OpenCV 捕获到的单字符按键。"""
        if not self.enabled:
            return None

        try:
            for role, frame in sorted(frames.items(), key=lambda item: item[1].spec.camera_index):
                window_name = self._window_name(role, frame)
                image = frame.color_bgr
                if self.scale != 1.0:
                    width = max(1, int(image.shape[1] * self.scale))
                    height = max(1, int(image.shape[0] * self.scale))
                    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

                if window_name not in self._window_names:
                    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(window_name, image.shape[1], image.shape[0])
                    self._window_names.add(window_name)
                cv2.imshow(window_name, image)

            return _decode_wait_key(cv2.waitKey(self.wait_ms))
        except cv2.error as exc:
            # 无桌面会话或 HighGUI 后端异常时，不能让预览影响采集本身。
            self.enabled = False
            print(f"[preview] OpenCV 预览窗口不可用，后续继续采集但不显示画面：{exc}")
            return None

    def close(self) -> None:
        for window_name in list(self._window_names):
            try:
                cv2.destroyWindow(window_name)
            except cv2.error:
                pass
        self._window_names.clear()
        try:
            cv2.waitKey(1)
        except cv2.error:
            pass

    def _window_name(self, role: str, frame: CapturedFrame) -> str:
        return f"{self.window_prefix} cam{frame.spec.camera_index} {role}"


def preview_config_from_workflow_config(
    config: dict[str, Any],
    *,
    no_preview: bool = False,
    preview_scale: float | None = None,
) -> dict[str, Any]:
    preview_cfg = dict(config.get("preview", {}))
    preview_cfg.setdefault("enabled", True)
    preview_cfg.setdefault("scale", 0.5)
    preview_cfg.setdefault("wait_ms", 1)
    preview_cfg.setdefault("window_prefix", "hand-eye calibration")

    if no_preview:
        preview_cfg["enabled"] = False
    if preview_scale is not None:
        preview_cfg["scale"] = float(preview_scale)
    return preview_cfg


def _valid_scale(value: float) -> float:
    scale = float(value)
    if scale <= 0:
        return 1.0
    return scale


def _decode_wait_key(raw_key: int) -> str | None:
    if raw_key < 0:
        return None
    key = raw_key & 0xFF
    if key == 255:
        return None
    try:
        return chr(key).lower()
    except ValueError:
        return None
