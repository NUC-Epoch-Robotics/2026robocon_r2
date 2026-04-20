# r2_lightboard_vision

3x4 lightboard detector node for ROS 2.

## Features

- Ultra-light processing pipeline for Jetson.
- No deep learning model.
- Grid fixed at runtime via params (`rows=3`, `cols=4`).
- Publishes only compact map data (`std_msgs/UInt8MultiArray`) and stable flag.

## Cell Encoding

- `0`: EMPTY
- `1`: R1
- `2`: R2
- `3`: FAKE

## Topics

- `lightboard/map_raw` (`std_msgs/UInt8MultiArray`): per-frame map result.
- `lightboard/map` (`std_msgs/UInt8MultiArray`): stable map result.
- `lightboard/stable` (`std_msgs/Bool`): whether current map is stable.

## Run

```bash
colcon build --packages-select r2_lightboard_vision r2_bringup
source install/setup.bash
ros2 launch r2_bringup r2_system.launch.py
```

Or launch detector only:

```bash
ros2 launch r2_lightboard_vision lightboard_detector.launch.py
```

## Key Params

- `camera_index` (int): camera device index.
- `rows` (int): grid rows, default `3`.
- `cols` (int): grid cols, default `4`.
- `roi_x`, `roi_y`, `roi_w`, `roi_h` (int): ROI crop for lightboard.
- `sample_ratio` (float): center sampling ratio in each cell.
- `min_v_lit` (int): brightness threshold for lit LEDs.
- `min_s_color` (int): color saturation threshold.
- `history_size` (int): map history window.
- `stable_frames_required` (int): stable vote threshold.

## Notes

- First step in field calibration: tune ROI (`roi_x/y/w/h`) to include only the board.
- Then tune color thresholds for your exact LED board and ambient light.
