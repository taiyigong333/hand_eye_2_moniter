import cv2
import argparse
import numpy as np


DICT_CANDIDATES = {
    # ArUco dictionaries
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,

    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,

    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,

    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,

    # AprilTag dictionaries supported by OpenCV aruco module
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def detect_with_dict(gray, dict_id):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

    params = cv2.aruco.DetectorParameters()

    # 兼容 OpenCV 新旧 API
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=params
        )

    return corners, ids, rejected


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", help="输入图片路径")
    parser.add_argument("--out", default="detected_markers.jpg", help="输出标注图路径")
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise FileNotFoundError(f"无法读取图片: {args.image}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    best = {
        "name": None,
        "corners": [],
        "ids": None,
        "count": 0,
    }

    print("开始尝试不同字典...\n")

    for name, dict_id in DICT_CANDIDATES.items():
        try:
            corners, ids, _ = detect_with_dict(gray, dict_id)
            count = 0 if ids is None else len(ids)

            print(f"{name}: {count} 个 marker")

            if count > best["count"]:
                best = {
                    "name": name,
                    "corners": corners,
                    "ids": ids,
                    "count": count,
                }

        except Exception as e:
            print(f"{name}: 检测失败，原因: {e}")

    print("\n==============================")
    print(f"最佳匹配字典: {best['name']}")
    print(f"检测到 marker 数量: {best['count']}")

    output = img.copy()

    if best["ids"] is not None and best["count"] > 0:
        cv2.aruco.drawDetectedMarkers(output, best["corners"], best["ids"])

        ids_flat = best["ids"].flatten().tolist()
        print(f"检测到的 ID:")
        print(ids_flat)

        # 额外画中心点
        for corners, marker_id in zip(best["corners"], ids_flat):
            pts = corners.reshape(4, 2)
            center = pts.mean(axis=0).astype(int)
            cv2.circle(output, tuple(center), 5, (0, 0, 255), -1)
            cv2.putText(
                output,
                str(marker_id),
                tuple(center + np.array([6, -6])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
    else:
        print("没有检测到 marker。可以尝试拍正一点、减少反光、提高对比度。")

    cv2.imwrite(args.out, output)
    print(f"\n标注结果已保存到: {args.out}")


if __name__ == "__main__":
    main()