import os
import json
import argparse
import numpy as np
import cv2


def ensure_uint8_bgr(img):
    arr = np.asarray(img)
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    if arr.ndim == 2:
        return arr
    if arr.ndim == 3 and arr.shape[2] == 3:
        return arr
    raise ValueError(f"Unsupported image shape: {arr.shape}, dtype={arr.dtype}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("npz_path", type=str)
    parser.add_argument("--out_dir", type=str, default="./dataset_from_npz")
    args = parser.parse_args()

    data = np.load(args.npz_path)

    tcp_poses = data["tcp_poses"]         # (N,6)
    joint_angles = data["joint_angles"]   # (N,6)
    images_main = data["images_main"]     # (N,H,W,3) 固定相机
    images_wrist = data["images_wrist"]   # (N,H,W,3) 末端相机

    n = len(tcp_poses)
    assert len(joint_angles) == n
    assert len(images_main) == n
    assert len(images_wrist) == n

    os.makedirs(args.out_dir, exist_ok=True)

    for i in range(n):
        sample_dir = os.path.join(args.out_dir, f"sample_{i:03d}")
        os.makedirs(sample_dir, exist_ok=True)

        cam0_name = "cam0_wrist.png"   # end camera
        cam1_name = "cam1_main.png"    # fixed camera

        cam0 = ensure_uint8_bgr(images_wrist[i])
        cam1 = ensure_uint8_bgr(images_main[i])

        cv2.imwrite(os.path.join(sample_dir, cam0_name), cam0)
        cv2.imwrite(os.path.join(sample_dir, cam1_name), cam1)

        pose_json = {
            "sample_index": i,
            "tcp_pose": tcp_poses[i].tolist(),
            "tcp_pose_fields": ["x", "y", "z", "rx", "ry", "rz"],
            "joint_angles": joint_angles[i].tolist(),
            "joint_angle_fields": ["q0", "q1", "q2", "q3", "q4", "q5"],
            "images": [
                {
                    "file": cam0_name,
                    "camera_index": 0,
                    "camera_name": "wrist_camera"
                },
                {
                    "file": cam1_name,
                    "camera_index": 1,
                    "camera_name": "main_camera"
                }
            ]
        }

        with open(os.path.join(sample_dir, "pose.json"), "w", encoding="utf-8") as f:
            json.dump(pose_json, f, ensure_ascii=False, indent=2)

    print(f"Done. Exported {n} samples to: {args.out_dir}")


if __name__ == "__main__":
    main()