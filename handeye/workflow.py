from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from .dataset import CalibrationDatasetWriter, count_pose_json
from .intrinsics import save_calib_intrinsics
from .preview import OpenCVCameraPreview, preview_config_from_workflow_config
from .realsense import CameraSpec, RealSenseCaptureSystem
from .robot import RTDERobotClient, RobotConfig, prepare_robot_program


def load_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    data = json.loads(config_path.read_text(encoding="utf-8"))
    if "robot" not in data:
        raise ValueError("配置缺少 robot 字段。")
    if "cameras" not in data or not isinstance(data["cameras"], list):
        raise ValueError("配置缺少 cameras 列表。")
    if "calibration" not in data:
        raise ValueError("配置缺少 calibration 字段。")
    return data


def run_collection_workflow(args: argparse.Namespace) -> int:
    project_root = Path(args.project_root).resolve()
    config_path = Path(args.config).resolve()
    config = load_config(config_path)

    robot_cfg = RobotConfig.from_dict(config["robot"])
    camera_specs = [CameraSpec.from_dict(item) for item in config["cameras"]]
    sampling_cfg = dict(config.get("sampling", {}))
    preview_cfg = preview_config_from_workflow_config(
        config,
        no_preview=bool(getattr(args, "no_preview", False)),
        preview_scale=getattr(args, "preview_scale", None),
    )
    mode = args.mode or str(sampling_cfg.get("mode", "timed"))
    dataset_dir = _resolve_path(project_root, args.dataset_dir or config.get("dataset_dir", "data/live_capture"))
    output_dir = _resolve_path(project_root, args.output_dir or config["calibration"].get("output_dir", "outputs/live_calibration"))
    output_dir = _timestamp_live_calibration_dir(output_dir)

    config["dataset_dir"] = str(_relative_or_absolute(project_root, dataset_dir))
    config["calibration"]["output_dir"] = str(_relative_or_absolute(project_root, output_dir))
    config["preview"] = preview_cfg

    if args.dry_run:
        _print_dry_run(project_root, config_path, robot_cfg, camera_specs, dataset_dir, output_dir, mode, config)
        return 0

    writer = CalibrationDatasetWriter(dataset_dir)
    writer.write_session_config(config)

    if not args.skip_robot_program:
        prepare_robot_program(robot_cfg)
    else:
        print("[robot] 已跳过 Dashboard 加载/启动 URP，仅使用 RTDE 读取 TCP。")

    preview = OpenCVCameraPreview(
        enabled=bool(preview_cfg.get("enabled", True)),
        scale=float(preview_cfg.get("scale", 0.5)),
        wait_ms=int(preview_cfg.get("wait_ms", 1)),
        window_prefix=str(preview_cfg.get("window_prefix", "hand-eye calibration")),
    )
    if preview.enabled:
        print(
            "[preview] 已启用双相机实时预览；聚焦预览窗口或终端后，"
            "手动模式按 c 保存，按 q 结束。"
        )

    captured = 0
    try:
        with RTDERobotClient(robot_cfg.host) as robot, RealSenseCaptureSystem(camera_specs) as cameras:
            if bool(config.get("refresh_intrinsics_from_device", True)):
                _save_live_intrinsics(project_root, camera_specs, cameras)

            if mode == "manual":
                captured = _manual_capture_loop(writer, robot, cameras, robot_cfg, sampling_cfg, preview)
            elif mode == "timed":
                captured = _timed_capture_loop(writer, robot, cameras, robot_cfg, sampling_cfg, preview)
            else:
                raise ValueError(f"不支持的采样模式: {mode}")
    except KeyboardInterrupt:
        print("\n[collect] 采集被用户中断，已保存的样本会保留。")
    finally:
        preview.close()

    total_samples = count_pose_json(dataset_dir)
    print(f"[collect] 本次新增 {captured} 个样本，数据集当前共有 {total_samples} 个 pose.json。")

    if args.skip_calibration:
        print("[calib] 已按参数跳过自动标定。")
        return 0

    min_samples = int(config["calibration"].get("min_samples", 5))
    if total_samples < min_samples:
        print(f"[calib] 样本数 {total_samples} 少于 {min_samples}，跳过自动标定。")
        return 0

    command = build_calibration_command(project_root, config, dataset_dir, output_dir)
    print("[calib] 开始自动标定:")
    print(" ".join(str(part) for part in command))
    subprocess.run(command, cwd=str(project_root), check=True)
    return 0


def build_calibration_command(
    project_root: Path,
    config: dict[str, Any],
    dataset_dir: Path,
    output_dir: Path,
) -> list[str]:
    calib_cfg = dict(config["calibration"])
    calib_cfg["dataset_dir"] = str(_relative_or_absolute(project_root, dataset_dir))
    calib_cfg["output_dir"] = str(_relative_or_absolute(project_root, output_dir))

    command = [sys.executable, str(project_root / "calib.py")]
    passthrough = {
        key: value
        for key, value in calib_cfg.items()
        if key not in {"min_samples"}
    }
    for key, value in passthrough.items():
        if value is None:
            continue
        command.append(f"--{key}")
        if isinstance(value, bool):
            command.append("true" if value else "false")
        else:
            command.append(str(value))
    return command


def _manual_capture_loop(
    writer: CalibrationDatasetWriter,
    robot: RTDERobotClient,
    cameras: RealSenseCaptureSystem,
    robot_cfg: RobotConfig,
    sampling_cfg: dict[str, Any],
    preview: OpenCVCameraPreview,
) -> int:
    capture_key = str(sampling_cfg.get("manual_capture_key", "c")).lower()
    stop_key = str(sampling_cfg.get("manual_stop_key", "q")).lower()
    max_samples = sampling_cfg.get("max_samples")
    max_samples = int(max_samples) if max_samples is not None else None

    print(f"[collect] 手动模式：按 {capture_key} 采样，按 {stop_key} 结束。")
    captured = 0
    while max_samples is None or captured < max_samples:
        if preview.enabled:
            frames = cameras.capture_all()
            command = _command_from_key(preview.show(frames), capture_key, stop_key)
            command = command or _poll_manual_command(capture_key, stop_key)
            if command is None:
                continue
        else:
            command = _wait_manual_command(capture_key, stop_key)
        if command == "stop":
            break
        sample_dir = _capture_once(writer, robot, cameras, robot_cfg, preview)
        captured += 1
        print(f"[collect] 已保存样本 {captured}: {sample_dir}")
    return captured


def _timed_capture_loop(
    writer: CalibrationDatasetWriter,
    robot: RTDERobotClient,
    cameras: RealSenseCaptureSystem,
    robot_cfg: RobotConfig,
    sampling_cfg: dict[str, Any],
    preview: OpenCVCameraPreview,
) -> int:
    interval_s = float(sampling_cfg.get("interval_s", 2.0))
    max_samples = sampling_cfg.get("max_samples", 35)
    max_samples = int(max_samples) if max_samples is not None else None
    stop_key = str(sampling_cfg.get("manual_stop_key", "q")).lower()

    print(f"[collect] 定时模式：间隔 {interval_s:.3f}s，目标样本数 {max_samples or '不限'}。")
    captured = 0
    while max_samples is None or captured < max_samples:
        started = time.monotonic()
        sample_dir = _capture_once(writer, robot, cameras, robot_cfg, preview)
        captured += 1
        print(f"[collect] 已保存样本 {captured}: {sample_dir}")
        if max_samples is not None and captured >= max_samples:
            break
        sleep_s = max(0.0, interval_s - (time.monotonic() - started))
        if _wait_interval(sleep_s, cameras, preview, stop_key):
            break
    return captured


def _capture_once(
    writer: CalibrationDatasetWriter,
    robot: RTDERobotClient,
    cameras: RealSenseCaptureSystem,
    robot_cfg: RobotConfig,
    preview: OpenCVCameraPreview,
) -> Path:
    tcp_pose = robot.get_tcp_pose()
    joint_angles = robot.get_joint_angles()
    frames = cameras.capture_all()
    preview.show(frames)
    return writer.write_sample(
        tcp_pose=tcp_pose,
        frames=frames,
        joint_angles=joint_angles,
        robot_host=robot_cfg.host,
    )


def _wait_manual_command(capture_key: str, stop_key: str) -> str:
    try:
        import msvcrt

        while True:
            key = msvcrt.getwch().lower()
            command = _command_from_key(key, capture_key, stop_key)
            if command is not None:
                return command
    except ImportError:
        while True:
            value = input(f"[collect] 输入 {capture_key} 采样，输入 {stop_key} 结束: ").strip().lower()
            command = _command_from_key(value, capture_key, stop_key)
            if command is not None:
                return command


def _wait_interval(
    duration_s: float,
    cameras: RealSenseCaptureSystem,
    preview: OpenCVCameraPreview,
    stop_key: str,
) -> bool:
    if duration_s <= 0:
        return False
    if not preview.enabled:
        time.sleep(duration_s)
        return False

    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        frames = cameras.capture_all()
        key_command = _command_from_key(preview.show(frames), None, stop_key)
        terminal_command = _poll_manual_command(None, stop_key)
        if key_command == "stop" or terminal_command == "stop":
            return True
    return False


def _poll_manual_command(capture_key: str | None, stop_key: str) -> str | None:
    try:
        import msvcrt
    except ImportError:
        return None

    while msvcrt.kbhit():
        key = msvcrt.getwch().lower()
        command = _command_from_key(key, capture_key, stop_key)
        if command is not None:
            return command
    return None


def _command_from_key(key: str | None, capture_key: str | None, stop_key: str) -> str | None:
    if not key:
        return None
    key = key.lower()
    if capture_key is not None and key == capture_key:
        return "capture"
    if key == stop_key:
        return "stop"
    return None


def _save_live_intrinsics(
    project_root: Path,
    camera_specs: list[CameraSpec],
    cameras: RealSenseCaptureSystem,
) -> None:
    live_intrinsics = cameras.color_intrinsics()
    for spec in camera_specs:
        if not spec.intrinsics_path:
            continue
        intrinsics = live_intrinsics.get(spec.role)
        if intrinsics is None:
            continue
        path = _resolve_path(project_root, spec.intrinsics_path)
        save_calib_intrinsics(
            path,
            intrinsics,
            source={
                "type": "realsense_active_profile",
                "role": spec.role,
                "camera_index": spec.camera_index,
                "camera_name": spec.camera_name,
                "serial": spec.serial,
            },
        )
        print(f"[camera] 已写入实时内参: {path}")


def _print_dry_run(
    project_root: Path,
    config_path: Path,
    robot_cfg: RobotConfig,
    camera_specs: list[CameraSpec],
    dataset_dir: Path,
    output_dir: Path,
    mode: str,
    config: dict[str, Any],
) -> None:
    print(f"[dry-run] project_root: {project_root}")
    print(f"[dry-run] config: {config_path}")
    print(f"[dry-run] robot: {robot_cfg.host}, program={robot_cfg.program}")
    print(f"[dry-run] dataset_dir: {dataset_dir}")
    print(f"[dry-run] output_dir: {output_dir}")
    print(f"[dry-run] mode: {mode}")
    preview_cfg = dict(config.get("preview", {}))
    print(
        "[dry-run] preview: "
        f"enabled={preview_cfg.get('enabled')}, "
        f"scale={preview_cfg.get('scale')}, "
        f"wait_ms={preview_cfg.get('wait_ms')}"
    )
    for spec in camera_specs:
        print(
            f"[dry-run] camera {spec.role}: index={spec.camera_index}, "
            f"name={spec.camera_name}, serial={spec.serial}, "
            f"color={spec.color_width}x{spec.color_height}@{spec.fps}, "
            f"depth={spec.enable_depth}"
        )
    command = build_calibration_command(project_root, config, dataset_dir, output_dir)
    print("[dry-run] calibration command:")
    print(" ".join(str(part) for part in command))


def _resolve_path(project_root: Path, value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    return project_root / path


def _timestamp_live_calibration_dir(path: Path, timestamp: str | None = None) -> Path:
    # 默认实时标定输出需要按采集轮次隔离，避免覆盖上一轮结果。
    if path.name.lower() != "live_calibration":
        return path
    stamp = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S")
    return path.with_name(f"{path.name}_{stamp}")


def _relative_or_absolute(project_root: Path, path: Path) -> str:
    try:
        return str(path.resolve().relative_to(project_root.resolve()))
    except ValueError:
        return str(path)
