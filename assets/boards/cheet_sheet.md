python.exe tools\make_aruco_id_map.py assets\boards\board.jpg `
  --aruco_dict DICT_6X6_250 `
  --grid_cols 20 `
  --grid_rows 15 `
  --top_left_is_tag false `
  --tag_size 0.01486 `
  --cell_size 0.01885375 `
  --max_cell_dist 0.55 `
  --out assets\boards\aruco_id_map_new.json `
  --vis assets\boards\aruco_id_map_new.jpg