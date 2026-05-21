#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import cv2
import numpy as np


def str2bool(v):
    if isinstance(v, bool):
        return v
    s = v.lower().strip()
    if s in {"1", "true", "yes", "y"}:
        return True
    if s in {"0", "false", "no", "n"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid bool value: {v}")


def parse_corners(s):
    """
    Format:
      "x1,y1 x2,y2 x3,y3 x4,y4"
    Order:
      top-left, top-right, bottom-right, bottom-left
    """
    pts = []
    for item in s.strip().split():
        x, y = item.split(",")
        pts.append([float(x), float(y)])
    if len(pts) != 4:
        raise ValueError("Need exactly 4 corner points.")
    return np.array(pts, dtype=np.float32)


def pick_corners_gui(img):
    points = []
    vis = img.copy()

    def on_mouse(event, x, y, flags, param):
        nonlocal vis
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append([x, y])
            cv2.circle(vis, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(
                vis,
                str(len(points)),
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("click corners", vis)

    print("[INFO] 请依次点击棋盘格有效区域四角：左上、右上、右下、左下")
    print("[INFO] 注意：点棋盘 20x15 格子的外边界，不要点纸张外边缘")
    cv2.imshow("click corners", vis)
    cv2.setMouseCallback("click corners", on_mouse)

    while True:
        key = cv2.waitKey(20) & 0xFF
        if len(points) == 4:
            break
        if key == 27:
            raise RuntimeError("User cancelled.")

    cv2.destroyWindow("click corners")
    return np.array(points, dtype=np.float32)


def detect_aruco(img, aruco_dict_name):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if not hasattr(cv2, "aruco"):
        raise ImportError("请安装 opencv-contrib-python")

    dict_id = getattr(cv2.aruco, aruco_dict_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    params = cv2.aruco.DetectorParameters()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=params
        )

    if ids is None:
        return []

    dets = []
    for marker_id, c in zip(ids.flatten(), corners):
        pts = np.asarray(c, dtype=np.float32).reshape(4, 2)
        center = pts.mean(axis=0)
        dets.append({
            "id": int(marker_id),
            "corners": pts,
            "center": center,
        })

    return dets


def make_valid_marker_cells(grid_cols, grid_rows, top_left_is_tag):
    cells = []

    for row in range(grid_rows):
        for col in range(grid_cols):
            if top_left_is_tag:
                is_tag_cell = ((row + col) % 2 == 0)
            else:
                is_tag_cell = ((row + col) % 2 == 1)

            if is_tag_cell:
                cells.append((row, col, np.array([col + 0.5, row + 0.5], dtype=np.float32)))

    return cells


def transform_points(H, pts):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    out = cv2.perspectiveTransform(pts, H).reshape(-1, 2)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", help="一张能看清整块或大部分标定板的图片")
    parser.add_argument("--out", default="aruco_id_map.json")
    parser.add_argument("--vis", default="aruco_id_map_vis.jpg")
    parser.add_argument("--aruco_dict", default="DICT_6X6_250")
    parser.add_argument("--grid_cols", type=int, default=20)
    parser.add_argument("--grid_rows", type=int, default=15)
    parser.add_argument("--tag_size", type=float, default=0.016)
    parser.add_argument("--cell_size", type=float, default=0.019)
    parser.add_argument("--top_left_is_tag", type=str2bool, default=False)
    parser.add_argument(
        "--corners",
        type=str,
        default=None,
        help='无 GUI 时使用。格式: "x_tl,y_tl x_tr,y_tr x_br,y_br x_bl,y_bl"',
    )
    parser.add_argument(
        "--max_cell_dist",
        type=float,
        default=0.45,
        help="marker 中心到最近合法格子中心的最大距离，单位是 cell",
    )
    parser.add_argument(
        "--merge",
        type=str,
        default=None,
        help="可选：合并已有 id_map json",
    )
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise FileNotFoundError(args.image)

    dets = detect_aruco(img, args.aruco_dict)
    print(f"[INFO] detected markers: {len(dets)}")

    if args.corners is not None:
        src = parse_corners(args.corners)
    else:
        src = pick_corners_gui(img)

    # 目标坐标用棋盘 cell 单位
    # 四角顺序：左上、右上、右下、左下
    dst = np.array([
        [0, 0],
        [args.grid_cols, 0],
        [args.grid_cols, args.grid_rows],
        [0, args.grid_rows],
    ], dtype=np.float32)

    H_img_to_board = cv2.getPerspectiveTransform(src, dst)
    H_board_to_img = np.linalg.inv(H_img_to_board)

    valid_cells = make_valid_marker_cells(
        args.grid_cols,
        args.grid_rows,
        args.top_left_is_tag,
    )

    id_map = {}

    if args.merge is not None:
        with open(args.merge, "r", encoding="utf-8") as f:
            old = json.load(f)
        id_map = {int(k): [int(v[0]), int(v[1])] for k, v in old.items()}
        print(f"[INFO] loaded existing map: {len(id_map)} ids")

    cell_occupied = {}

    vis = img.copy()
    cv2.aruco.drawDetectedMarkers(
        vis,
        [d["corners"].reshape(1, 4, 2) for d in dets],
        np.array([[d["id"]] for d in dets], dtype=np.int32),
    )

    skipped = []

    for d in dets:
        marker_id = d["id"]
        board_center = transform_points(H_img_to_board, d["center"])[0]

        # 找最近的合法 marker 单元格
        best = None
        best_dist = 1e9
        for row, col, cc in valid_cells:
            dist = float(np.linalg.norm(board_center - cc))
            if dist < best_dist:
                best_dist = dist
                best = (row, col, cc)

        if best is None or best_dist > args.max_cell_dist:
            skipped.append((marker_id, board_center.tolist(), best_dist))
            continue

        row, col, cc = best

        # 如果两个 ID 被分到同一个 cell，保留距离更近的
        key = (row, col)
        if key in cell_occupied:
            old_id, old_dist = cell_occupied[key]
            if best_dist >= old_dist:
                skipped.append((marker_id, board_center.tolist(), best_dist))
                continue
            else:
                if old_id in id_map:
                    del id_map[old_id]

        if marker_id in id_map:
            old_rc = id_map[marker_id]
            if old_rc != [row, col]:
                print(f"[WARN] ID {marker_id} conflict: old={old_rc}, new={[row, col]}")

        #id_map[marker_id] = [row, col]
        # ---------- 估计 marker 在格子里的旋转 ----------
        # detected marker corners -> board cell coordinate
        det_board_corners = transform_points(H_img_to_board, d["corners"])

        marker_ratio = float(args.tag_size) / float(args.cell_size)
        margin = 0.5 * (1.0 - marker_ratio)

        x0 = col + margin
        y0 = row + margin
        x1 = x0 + marker_ratio
        y1 = y0 + marker_ratio

        # 棋盘坐标系里的理想 marker 四角：
        # TL, TR, BR, BL
        ideal_corners = np.array([
            [x0, y0],
            [x1, y0],
            [x1, y1],
            [x0, y1],
        ], dtype=np.float32)

        best_rot = 0
        best_corner_err = 1e9

        for rot in range(4):
            # rot 表示检测角点顺序相对于棋盘理想角点的循环偏移
            cand = np.roll(ideal_corners, -rot, axis=0)
            err = float(np.mean(np.linalg.norm(det_board_corners - cand, axis=1)))
            if err < best_corner_err:
                best_corner_err = err
                best_rot = rot

        id_map[marker_id] = [row, col, best_rot]
        cell_occupied[key] = (marker_id, best_dist)

        # 可视化：蓝色为检测中心，红色为映射后格子中心投回图像
        detected_center = d["center"]
        proj_center = transform_points(H_board_to_img, cc)[0]

        p1 = tuple(np.round(detected_center).astype(int))
        p2 = tuple(np.round(proj_center).astype(int))

        cv2.circle(vis, p1, 4, (255, 0, 0), -1)
        cv2.circle(vis, p2, 4, (0, 0, 255), -1)
        cv2.line(vis, p1, p2, (0, 255, 255), 1)

        label = f"id{marker_id}->r{row}c{col}rot{best_rot}"
        cv2.putText(
            vis,
            label,
            (p1[0] + 5, p1[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    # JSON key 用字符串，排序保存
    out_map = {
        str(k): id_map[k]
        for k in sorted(id_map.keys())
    }

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out_map, f, indent=2, ensure_ascii=False)

    cv2.imwrite(args.vis, vis)

    print(f"[OK] saved id map: {args.out}")
    print(f"[OK] saved visualization: {args.vis}")
    print(f"[INFO] mapped ids: {len(out_map)}")
    print(f"[INFO] skipped ids: {len(skipped)}")

    if skipped:
        print("[WARN] skipped examples:")
        for item in skipped[:10]:
            print("  ", item)


if __name__ == "__main__":
    main()