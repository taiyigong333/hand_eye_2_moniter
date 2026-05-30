#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from handeye.workflow import run_collection_workflow


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="通过 UR RTDE + RealSense 采集手眼标定样本，并在结束后自动调用 calib.py。"
    )
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "configs" / "live_collection.example.json"),
        help="采集配置 JSON。默认使用 configs/live_collection.example.json",
    )
    parser.add_argument(
        "--project_root",
        default=str(PROJECT_ROOT),
        help="项目根目录。通常不需要修改。",
    )
    parser.add_argument(
        "--mode",
        choices=["timed", "manual"],
        default=None,
        help="覆盖配置中的采样模式。",
    )
    parser.add_argument(
        "--calibration_mode",
        choices=["dual_camera", "eye_in_hand"],
        default=None,
        help="覆盖 calibration.mode；eye_in_hand 只采集末端相机并只运行眼在手上标定。",
    )
    parser.add_argument(
        "--dataset_dir",
        default=None,
        help="覆盖配置中的样本输出目录。",
    )
    parser.add_argument(
        "--output_dir",
        default=None,
        help="覆盖配置中的标定输出目录。",
    )
    parser.add_argument(
        "--skip_robot_program",
        action="store_true",
        help="不通过 Dashboard 加载/启动 URP；采样前暂停同步仍按配置执行。",
    )
    parser.add_argument(
        "--skip_calibration",
        action="store_true",
        help="只采集样本，不在结束后自动运行 calib.py。",
    )
    parser.add_argument(
        "--no_preview",
        action="store_true",
        help="采集时不打开实时预览窗口。",
    )
    parser.add_argument(
        "--preview_scale",
        type=float,
        default=None,
        help="覆盖预览窗口缩放比例，例如 0.5 表示按半尺寸显示。",
    )
    parser.add_argument(
        "--dry_run",
        action="store_true",
        help="只检查配置并打印将要执行的标定命令，不连接机器人或相机。",
    )
    return parser.parse_args()


def main() -> int:
    return run_collection_workflow(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
