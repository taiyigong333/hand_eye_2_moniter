from __future__ import annotations

import socket
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RobotConfig:
    host: str
    dashboard_port: int = 29999
    program: str = "autoHandEye.urp"
    load_program: bool = True
    play_after_load: bool = True
    stop_before_load: bool = False
    pause_before_capture: bool = False
    capture_settle_s: float = 0.5
    resume_after_capture: bool = True
    stop_after_collection: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotConfig":
        return cls(
            host=str(data.get("host", "192.168.1.88")),
            dashboard_port=int(data.get("dashboard_port", 29999)),
            program=str(data.get("program", "autoHandEye.urp")),
            load_program=bool(data.get("load_program", True)),
            play_after_load=bool(data.get("play_after_load", True)),
            stop_before_load=bool(data.get("stop_before_load", False)),
            pause_before_capture=bool(data.get("pause_before_capture", False)),
            capture_settle_s=float(data.get("capture_settle_s", 0.5)),
            resume_after_capture=bool(data.get("resume_after_capture", True)),
            stop_after_collection=bool(data.get("stop_after_collection", True)),
        )


class RTDERobotClient:
    """RTDE 只读客户端，当前只依赖 TCP 位姿和可选关节角。"""

    def __init__(self, host: str):
        self.host = host
        self._receive = None

    def connect(self) -> None:
        try:
            from rtde_receive import RTDEReceiveInterface
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(
                "未找到 rtde_receive，请在 hand_eye 环境安装 ur-rtde。"
            ) from exc

        self._receive = RTDEReceiveInterface(self.host)
        tcp = self.get_tcp_pose()
        print(f"[robot] RTDE 已连接: {self.host}, TCP={_format_pose(tcp)}")

    def get_tcp_pose(self) -> list[float]:
        if self._receive is None:
            raise RuntimeError("RTDE 尚未连接。")
        return [float(value) for value in self._receive.getActualTCPPose()]

    def get_joint_angles(self) -> list[float] | None:
        if self._receive is None:
            raise RuntimeError("RTDE 尚未连接。")
        try:
            return [float(value) for value in self._receive.getActualQ()]
        except Exception:
            return None

    def close(self) -> None:
        if self._receive is None:
            return
        for method_name in ("disconnect", "stopScript"):
            method = getattr(self._receive, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
        self._receive = None

    def __enter__(self) -> "RTDERobotClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


class URDashboardClient:
    """UR Dashboard 端口客户端，用于加载/启动示教器中的 URP 程序。"""

    def __init__(self, host: str, port: int = 29999, timeout_s: float = 3.0):
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)

    def command(self, command: str) -> str:
        message = command if command.endswith("\n") else command + "\n"
        with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as sock:
            sock.settimeout(self.timeout_s)
            try:
                sock.recv(1024)
            except socket.timeout:
                pass
            sock.sendall(message.encode("utf-8"))
            response = sock.recv(4096).decode("utf-8", errors="replace").strip()
        print(f"[dashboard] {command} -> {response}")
        return response

    def stop(self) -> str:
        return self.command("stop")

    def pause(self) -> str:
        return self.command("pause")

    def load_program(self, program: str) -> str:
        return self.command(f"load {program}")

    def play(self) -> str:
        return self.command("play")


class URProgramCaptureSync:
    """拍照前后通过 Dashboard 暂停/继续 URP，避免运动中采样。"""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.dashboard = URDashboardClient(config.host, config.dashboard_port)

    @property
    def enabled(self) -> bool:
        return bool(self.config.pause_before_capture)

    def pause_before_capture(self) -> None:
        if not self.enabled:
            return
        # 使用 pause/play 而不是 stop/play，避免每次采样后从头重启示教器程序。
        self.dashboard.pause()
        settle_s = max(0.0, float(self.config.capture_settle_s))
        print(f"[robot] 已发送采样暂停信号，等待 {settle_s:.3f}s 后拍照。")
        if settle_s > 0:
            time.sleep(settle_s)

    def resume_after_capture(self) -> None:
        if not self.enabled or not self.config.resume_after_capture:
            return
        self.dashboard.play()
        print("[robot] 拍照完成，已发送继续运行信号。")


def prepare_robot_program(config: RobotConfig) -> None:
    if not config.load_program and not config.play_after_load and not config.stop_before_load:
        return

    dashboard = URDashboardClient(config.host, config.dashboard_port)
    if config.stop_before_load:
        dashboard.stop()
    if config.load_program:
        dashboard.load_program(config.program)
    if config.play_after_load:
        dashboard.play()


def stop_robot_program_after_collection(config: RobotConfig) -> None:
    """采集阶段结束后停止示教器程序，避免自动标定时机器人继续运动。"""
    if not config.stop_after_collection:
        return
    dashboard = URDashboardClient(config.host, config.dashboard_port)
    dashboard.stop()
    print("[robot] 采集结束，已发送 Dashboard stop 停止示教器程序。")


def _format_pose(values: list[float]) -> str:
    return "[" + ", ".join(f"{value:.4f}" for value in values) + "]"
