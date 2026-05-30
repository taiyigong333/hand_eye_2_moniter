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
from .robot import (
    RTDERobotClient,
    RobotConfig,
    URProgramCaptureSync,
    prepare_robot_program,
    stop_robot_program_after_collection,
)


def load_config(path: str | Path) -> dict[str, Any]:
    """
    读取实时采集配置，并做最小结构校验。

    这里不做过重的参数合法性检查，原因是 robot/camera/calibration 的细节会分别由
    RobotConfig、CameraSpec 和 calib.py 继续校验；本函数只保证后续流程需要的顶层字段存在。
    """
    config_path = Path(path)
    data = json.loads(config_path.read_text(encoding="utf-8"))
    if "robot" not in data:
        raise ValueError("配置缺少 robot 字段。")
    if "cameras" not in data or not isinstance(data["cameras"], list):
        raise ValueError("配置缺少 cameras 列表。")
    if "calibration" not in data:
        raise ValueError("配置缺少 calibration 字段。")
    return data


def _resolve_calibration_mode(config: dict[str, Any], override: str | None) -> str:
    """把采样模式和标定模式拆开，避免 `--mode timed` 被误认为 calib.py 的模式。"""
    calibration_cfg = config["calibration"]
    mode = override or str(calibration_cfg.get("mode", "dual_camera"))
    if mode not in {"dual_camera", "eye_in_hand"}:
        raise ValueError(f"不支持的标定模式: {mode}")
    calibration_cfg["mode"] = mode
    return mode


def _select_camera_items_for_calibration(
    camera_items: list[dict[str, Any]],
    calibration_cfg: dict[str, Any],
) -> list[dict[str, Any]]:
    """
    根据标定模式选择本轮需要启动的相机。

    eye_in_hand 不依赖固定相机，因此只启动 end_camera_index 对应相机，避免固定相机
    未连接时阻塞定时采集和自动标定。
    """
    if str(calibration_cfg.get("mode", "dual_camera")) != "eye_in_hand":
        return camera_items

    end_camera_index = int(calibration_cfg.get("end_camera_index", 0))
    selected = [
        item
        for item in camera_items
        if int(item.get("camera_index", -1)) == end_camera_index
    ]
    if not selected:
        raise ValueError(
            f"eye_in_hand 模式找不到 end_camera_index={end_camera_index} 对应的相机配置。"
        )
    return selected


def run_collection_workflow(args: argparse.Namespace) -> int:
    """
    实时采集 + 自动标定的主编排函数。

    整体流程是：
    1. 从 JSON 配置和命令行覆盖项解析机器人、相机、采样和标定参数。
    2. dry-run 时只打印将要连接的设备和最终 calib.py 命令，不碰机器人/相机。
    3. 真正运行时先按配置准备 URP，再同时打开 RTDE 和当前模式所需 RealSense。
    4. 每次拍照前可按配置暂停 URP、等待机械臂稳定，再保存 `TCP + 相机图像`。
    5. 样本数量满足要求后，调用 calib.py 计算手眼标定结果。
    """
    project_root = Path(args.project_root).resolve()
    config_path = Path(args.config).resolve()
    config = load_config(config_path)
    calibration_mode = _resolve_calibration_mode(
        config,
        getattr(args, "calibration_mode", None),
    )

    # 配置文件保持面向用户的 JSON 格式；这里转换成后续模块使用的 dataclass，
    # 使机器人连接、相机启动和路径处理都集中在各自模块里。
    robot_cfg = RobotConfig.from_dict(config["robot"])
    camera_items = _select_camera_items_for_calibration(
        list(config["cameras"]),
        config["calibration"],
    )
    config["cameras"] = camera_items
    camera_specs = [CameraSpec.from_dict(item) for item in camera_items]
    sampling_cfg = dict(config.get("sampling", {}))
    preview_cfg = preview_config_from_workflow_config(
        config,
        no_preview=bool(getattr(args, "no_preview", False)),
        preview_scale=getattr(args, "preview_scale", None),
    )
    sampling_mode = args.mode or str(sampling_cfg.get("mode", "timed"))

    # dataset_dir 保存原始采集样本；output_dir 保存 calib.py 的结果。
    # 命令行参数优先级高于配置文件，便于同一份配置临时跑不同输出目录。
    dataset_dir = _resolve_path(project_root, args.dataset_dir or config.get("dataset_dir", "data/live_capture"))
    output_dir = _resolve_path(project_root, args.output_dir or config["calibration"].get("output_dir", "outputs/live_calibration"))

    # 默认 live_calibration 目录会自动追加时间戳，避免重复采集时覆盖上一轮标定结果。
    output_dir = _timestamp_live_calibration_dir(output_dir)

    # 把最终采用的路径和预览配置写回 config。后续既会保存 capture_session_config.json，
    # 也会用这份 config 构造 calib.py 命令，所以这里是“本轮真实配置”的统一入口。
    config["dataset_dir"] = str(_relative_or_absolute(project_root, dataset_dir))
    config["calibration"]["output_dir"] = str(_relative_or_absolute(project_root, output_dir))
    config["preview"] = preview_cfg

    if args.dry_run:
        # dry-run 必须在连接硬件前返回，用于实机前检查 IP、序列号、输出路径和标定命令。
        _print_dry_run(
            project_root,
            config_path,
            robot_cfg,
            camera_specs,
            dataset_dir,
            output_dir,
            sampling_mode,
            calibration_mode,
            config,
        )
        return 0

    # writer 负责把每组样本落盘为 calib.py 已支持的 sample_xxx/pose.json 结构。
    writer = CalibrationDatasetWriter(dataset_dir)
    writer.write_session_config(config)

    if not args.skip_robot_program:
        # 只负责 Dashboard 加载/启动 URP；真正的 TCP 读取仍由 RTDE 完成。
        prepare_robot_program(robot_cfg)
    else:
        print("[robot] 已跳过 Dashboard 加载/启动 URP。")

    capture_sync = URProgramCaptureSync(robot_cfg) if robot_cfg.pause_before_capture else None
    if capture_sync is not None:
        print(
            "[robot] 已启用采样前暂停同步：每次拍照前发送 Dashboard pause，"
            f"等待 {robot_cfg.capture_settle_s:.3f}s，拍照后发送 play。"
        )

    # 预览只影响人眼观察和按键输入，不改变保存图像的原始分辨率。
    preview = OpenCVCameraPreview(
        enabled=bool(preview_cfg.get("enabled", True)),
        scale=float(preview_cfg.get("scale", 0.5)),
        wait_ms=int(preview_cfg.get("wait_ms", 1)),
        window_prefix=str(preview_cfg.get("window_prefix", "hand-eye calibration")),
    )
    if preview.enabled:
        print(
            "[preview] 已启用实时预览；聚焦预览窗口或终端后，"
            "手动模式按 c 保存，按 q 结束。"
        )

    captured = 0
    try:
        # 用一个 with 同时管理 RTDE 和 RealSense 生命周期，确保异常或 Ctrl+C 后能释放资源。
        with RTDERobotClient(robot_cfg.host) as robot, RealSenseCaptureSystem(camera_specs) as cameras:
            if bool(config.get("refresh_intrinsics_from_device", True)):
                # 使用 RealSense 当前 active profile 写内参，避免配置里的内参与实际分辨率/FPS 不一致。
                _save_live_intrinsics(project_root, camera_specs, cameras)

            if sampling_mode == "manual":
                captured = _manual_capture_loop(writer, robot, cameras, robot_cfg, sampling_cfg, preview, capture_sync)
            elif sampling_mode == "timed":
                captured = _timed_capture_loop(writer, robot, cameras, robot_cfg, sampling_cfg, preview, capture_sync)
            else:
                raise ValueError(f"不支持的采样模式: {sampling_mode}")
    except KeyboardInterrupt:
        print("\n[collect] 采集被用户中断，已保存的样本会保留。")
    finally:
        preview.close()

    stop_robot_program_after_collection(robot_cfg)

    total_samples = count_pose_json(dataset_dir)
    print(f"[collect] 本次新增 {captured} 个样本，数据集当前共有 {total_samples} 个 pose.json。")

    if args.skip_calibration:
        # 只采集不计算，常用于先人工检查图像质量或后续手动调参重跑 calib.py。
        print("[calib] 已按参数跳过自动标定。")
        return 0

    min_samples = int(config["calibration"].get("min_samples", 5))
    if total_samples < min_samples:
        # OpenCV 手眼标定至少需要多组姿态约束；样本太少时直接跳过，避免输出误导性结果。
        print(f"[calib] 样本数 {total_samples} 少于 {min_samples}，跳过自动标定。")
        return 0

    command = build_calibration_command(project_root, config, dataset_dir, output_dir)
    print("[calib] 开始自动标定:")
    print(" ".join(str(part) for part in command))
    # check=True 让 calib.py 失败时把错误传回采集入口，避免用户误以为整轮流程成功。
    subprocess.run(command, cwd=str(project_root), check=True)
    return 0


def build_calibration_command(
    project_root: Path,
    config: dict[str, Any],
    dataset_dir: Path,
    output_dir: Path,
) -> list[str]:
    """
    把配置文件中的 calibration 字段转换成 calib.py 命令行。

    设计上让实时采集和手动运行 calib.py 共用同一个入口参数集合；新增标定参数时，
    只要写入配置的 calibration 字段，一般就能自动透传给 calib.py。
    """
    calib_cfg = dict(config["calibration"])

    # dataset/output 以运行时解析后的路径为准，覆盖配置文件原值。
    calib_cfg["dataset_dir"] = str(_relative_or_absolute(project_root, dataset_dir))
    calib_cfg["output_dir"] = str(_relative_or_absolute(project_root, output_dir))
    if str(calib_cfg.get("mode", "dual_camera")) == "eye_in_hand":
        # 只跑眼在手上时不要把固定相机参数带进命令，dry-run 也能清楚反映真实依赖。
        calib_cfg.pop("fixed_camera_index", None)
        calib_cfg.pop("intr_fixed", None)

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
            # calib.py 使用 str2bool 解析 true/false；不要把 Python True/False 直接传给命令行。
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
    capture_sync: URProgramCaptureSync | None,
) -> int:
    """
    手动采集循环。

    有预览时，每轮先刷新一次相机画面，再读取窗口或终端按键；无预览时只等待终端按键。
    只有收到 capture_key 才保存样本，收到 stop_key 则结束本轮采集。
    """
    capture_key = str(sampling_cfg.get("manual_capture_key", "c")).lower()
    stop_key = str(sampling_cfg.get("manual_stop_key", "q")).lower()
    max_samples = sampling_cfg.get("max_samples")
    max_samples = int(max_samples) if max_samples is not None else None

    print(f"[collect] 手动模式：按 {capture_key} 采样，按 {stop_key} 结束。")
    captured = 0
    while max_samples is None or captured < max_samples:
        if preview.enabled:
            # 手动模式也持续拉取相机帧用于预览，便于确认当前模式所需相机能看到标定板。
            frames = cameras.capture_all()
            command = _command_from_key(preview.show(frames), capture_key, stop_key)
            command = command or _poll_manual_command(capture_key, stop_key)
            if command is None:
                continue
        else:
            command = _wait_manual_command(capture_key, stop_key)
        if command == "stop":
            break
        # 真正保存时会重新读取 TCP 和相机图像，保证落盘样本对应按键触发时刻。
        sample_dir = _capture_once(writer, robot, cameras, robot_cfg, preview, capture_sync)
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
    capture_sync: URProgramCaptureSync | None,
) -> int:
    """
    定时采集循环。

    每轮保存一次完整样本，然后在剩余间隔内继续刷新预览；这样既能按固定频率采样，
    又能允许用户在等待期间按 stop_key 提前结束。
    """
    interval_s = float(sampling_cfg.get("interval_s", 2.0))
    max_samples = sampling_cfg.get("max_samples", 35)
    max_samples = int(max_samples) if max_samples is not None else None
    stop_key = str(sampling_cfg.get("manual_stop_key", "q")).lower()

    print(f"[collect] 定时模式：间隔 {interval_s:.3f}s，目标样本数 {max_samples or '不限'}。")
    captured = 0
    while max_samples is None or captured < max_samples:
        started = time.monotonic()
        sample_dir = _capture_once(writer, robot, cameras, robot_cfg, preview, capture_sync)
        captured += 1
        print(f"[collect] 已保存样本 {captured}: {sample_dir}")
        if max_samples is not None and captured >= max_samples:
            break
        # 采样本身会消耗时间，只等待剩余时间，尽量保持实际采样周期接近 interval_s。
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
    capture_sync: URProgramCaptureSync | None,
) -> Path:
    """
    保存一组用于标定的同步样本。

    当前同步策略是“暂停后近似同步”：如果配置启用，先让示教器程序暂停并等待稳定，
    再读取机器人 TCP、关节角和双相机当前帧。拍照完成后先继续 URP，再把样本写入
    sample_xxx 目录，尽量缩短机器人停留时间。
    """
    if capture_sync is not None:
        capture_sync.pause_before_capture()
    try:
        tcp_pose = robot.get_tcp_pose()
        joint_angles = robot.get_joint_angles()
        frames = cameras.capture_all()
        preview.show(frames)
    finally:
        if capture_sync is not None:
            capture_sync.resume_after_capture()
    return writer.write_sample(
        tcp_pose=tcp_pose,
        frames=frames,
        joint_angles=joint_angles,
        robot_host=robot_cfg.host,
    )


def _wait_manual_command(capture_key: str, stop_key: str) -> str:
    """无预览或终端输入场景下等待手动按键。Windows 用 msvcrt 单键读取，其他平台退回 input。"""
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
    """
    定时模式下等待下一次采样，并在等待期间处理预览和停止按键。

    返回 True 表示用户请求提前结束；False 表示正常等到下一次采样时间。
    """
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
    """非阻塞读取 Windows 终端按键；没有按键时立即返回 None，避免卡住预览刷新。"""
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
    """把原始按键统一翻译成 capture/stop 命令，便于预览窗口和终端共用同一套逻辑。"""
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
    """
    保存当前 RealSense active color profile 的内参。

    标定用的 K/dist 必须和实际采集图像的分辨率、流配置一致。启动相机后从设备读取
    active profile 并覆盖配置中的内参文件，可以降低“内参文件和本轮采集不匹配”的风险。
    """
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
    sampling_mode: str,
    calibration_mode: str,
    config: dict[str, Any],
) -> None:
    """打印本轮真实会使用的硬件配置、输出路径和 calib.py 命令，用于实机前核对。"""
    print(f"[dry-run] project_root: {project_root}")
    print(f"[dry-run] config: {config_path}")
    print(f"[dry-run] robot: {robot_cfg.host}, program={robot_cfg.program}")
    print(
        "[dry-run] capture_sync: "
        f"pause_before_capture={robot_cfg.pause_before_capture}, "
        f"capture_settle_s={robot_cfg.capture_settle_s}, "
        f"resume_after_capture={robot_cfg.resume_after_capture}, "
        f"stop_after_collection={robot_cfg.stop_after_collection}"
    )
    print(f"[dry-run] dataset_dir: {dataset_dir}")
    print(f"[dry-run] output_dir: {output_dir}")
    print(f"[dry-run] sampling_mode: {sampling_mode}")
    print(f"[dry-run] calibration_mode: {calibration_mode}")
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
    """把配置中的相对路径解释为相对项目根目录，绝对路径则保持不变。"""
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
    """能表示为项目内相对路径时就用相对路径，方便保存到配置和命令中跨机器阅读。"""
    try:
        return str(path.resolve().relative_to(project_root.resolve()))
    except ValueError:
        return str(path)
