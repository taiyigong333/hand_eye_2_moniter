#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "live_collection.example.json"


def load_robot_host(config_path: Path) -> str:
    """从实时采集配置读取 robot.host，避免在工具脚本里重复写机器人 IP。"""
    with config_path.open("r", encoding="utf-8") as f:
        data: dict[str, Any] = json.load(f)
    host = data.get("robot", {}).get("host")
    if not host:
        raise ValueError(f"配置文件缺少 robot.host: {config_path}")
    return str(host)


def read_current_ee6d(host: str) -> list[float]:
    """通过 RTDE 只读接口读取当前末端 6D 位姿 [x, y, z, rx, ry, rz]。"""
    try:
        from rtde_receive import RTDEReceiveInterface
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "未找到 rtde_receive，请在 hand_eye 环境安装 ur-rtde。"
        ) from exc

    rtde = RTDEReceiveInterface(host)
    try:
        ee6d = rtde.getActualTCPPose()
        return [float(value) for value in ee6d]
    finally:
        # ur-rtde 不同版本的释放接口不完全一致，这里做兼容式清理。
        for method_name in ("disconnect", "stopScript"):
            method = getattr(rtde, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass


def format_ee6d(values: list[float]) -> str:
    if len(values) != 6:
        raise ValueError(f"RTDE 返回的 EE 6D 位姿长度应为 6，实际为 {len(values)}")
    return (
        "["
        f"{values[0]:.9f}, {values[1]:.9f}, {values[2]:.9f}, "
        f"{values[3]:.9f}, {values[4]:.9f}, {values[5]:.9f}"
        "]"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="通过 RTDE 读取并打印当前机械臂末端 EE 6D 位姿。")
    parser.add_argument("--host", help="机器人 IP；不提供时从 --config 的 robot.host 读取。")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"包含 robot.host 的配置文件，默认 {DEFAULT_CONFIG}",
    )
    parser.add_argument("--repeat", action="store_true", help="持续打印；默认只读取一次。")
    parser.add_argument("--interval-s", type=float, default=1.0, help="持续打印时的间隔秒数，默认 1.0。")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    host = str(args.host) if args.host else load_robot_host(args.config)

    while True:
        ee6d = read_current_ee6d(host)
        print(f"[rtde] host={host} ee6d={format_ee6d(ee6d)}  # [x,y,z,rx,ry,rz], m/rad")
        if not args.repeat:
            return 0
        time.sleep(max(0.0, float(args.interval_s)))


if __name__ == "__main__":
    raise SystemExit(main())
