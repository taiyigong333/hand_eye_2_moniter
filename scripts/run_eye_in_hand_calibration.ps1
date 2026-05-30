param(
    [string]$Python = "F:\Anaconda\Anaconda3\envs\hand_eye\python.exe",
    [string]$DatasetDir = "data/dataset_from_npz",
    [string]$OutputDir = "outputs/eye_in_hand_calibration"
)

$ErrorActionPreference = "Stop"

& $Python calib.py `
  --mode eye_in_hand `
  --dataset_dir $DatasetDir `
  --end_camera_index 0 `
  --intr_end configs/intr_d405_1280x720.json `
  --board_layout interleaved_checker `
  --grid_cols 20 `
  --grid_rows 15 `
  --tag_size 0.015 `
  --cell_size 0.019 `
  --aruco_dict DICT_6X6_250 `
  --top_left_is_tag true `
  --id_map_json assets/boards/aruco_id_map_new.json `
  --max_reproj_rmse 10.0 `
  --output_dir $OutputDir
