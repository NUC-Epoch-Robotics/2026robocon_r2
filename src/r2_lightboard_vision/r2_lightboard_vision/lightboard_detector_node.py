from collections import Counter, deque
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray, MultiArrayDimension


class LightboardDetectorNode(Node):
    """Ultra-light 3x4 lightboard detector.

    Output encoding for each cell:
    - 0: EMPTY
    - 1: R1
    - 2: R2
    - 3: FAKE

    RGBW board assumption (default):
    - R1 -> Red
    - R2 -> Green
    - FAKE -> Blue
    - EMPTY -> White / Off
    """

    EMPTY = 0
    R1 = 1
    R2 = 2
    FAKE = 3

    def __init__(self) -> None:
        super().__init__("lightboard_detector")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("rows", 3)
        self.declare_parameter("cols", 4)
        self.declare_parameter("roi_x", 0)
        self.declare_parameter("roi_y", 0)
        self.declare_parameter("roi_w", 0)
        self.declare_parameter("roi_h", 0)
        self.declare_parameter("sample_ratio", 0.55)
        self.declare_parameter("min_v_lit", 120)
        self.declare_parameter("min_s_color", 45)
        self.declare_parameter("max_s_white", 35)
        self.declare_parameter("min_color_ratio", 0.22)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("stable_frames_required", 3)
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("start_enabled", True)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.fps = float(self.get_parameter("fps").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.rows = int(self.get_parameter("rows").value)
        self.cols = int(self.get_parameter("cols").value)
        self.roi_x = int(self.get_parameter("roi_x").value)
        self.roi_y = int(self.get_parameter("roi_y").value)
        self.roi_w = int(self.get_parameter("roi_w").value)
        self.roi_h = int(self.get_parameter("roi_h").value)
        self.sample_ratio = float(self.get_parameter("sample_ratio").value)
        self.min_v_lit = int(self.get_parameter("min_v_lit").value)
        self.min_s_color = int(self.get_parameter("min_s_color").value)
        self.max_s_white = int(self.get_parameter("max_s_white").value)
        self.min_color_ratio = float(self.get_parameter("min_color_ratio").value)
        self.history_size = int(self.get_parameter("history_size").value)
        self.stable_frames_required = int(self.get_parameter("stable_frames_required").value)
        self.flip_horizontal = bool(self.get_parameter("flip_horizontal").value)
        self.enabled = bool(self.get_parameter("start_enabled").value)

        self.rows = max(1, self.rows)
        self.cols = max(1, self.cols)
        self.history_size = max(1, self.history_size)
        self.stable_frames_required = max(1, self.stable_frames_required)
        self.sample_ratio = min(max(self.sample_ratio, 0.1), 1.0)

        self.map_raw_pub = self.create_publisher(UInt8MultiArray, "lightboard/map_raw", 10)
        self.map_pub = self.create_publisher(UInt8MultiArray, "lightboard/map", 10)
        self.stable_pub = self.create_publisher(Bool, "lightboard/stable", 10)

        self.enable_sub = self.create_subscription(
            Bool, "lightboard/enable", self._on_enable, 10
        )

        self.history: Deque[Tuple[int, ...]] = deque(maxlen=self.history_size)
        self.last_stable_map: Optional[Tuple[int, ...]] = None
        self.cap: Optional[cv2.VideoCapture] = None
        self.last_open_failed: bool = False

        if self.enabled:
            self._ensure_camera_open()

        interval = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(interval, self.process_frame)
        self.get_logger().info(
            f"lightboard_detector started: grid={self.rows}x{self.cols}, fps={self.fps:.1f}"
        )

    def _on_enable(self, msg: Bool) -> None:
        if self.enabled == msg.data:
            return
        self.enabled = msg.data
        self.history.clear()
        self.last_stable_map = None

        if self.enabled:
            self.get_logger().info("lightboard_detector ENABLED")
            self._ensure_camera_open()
        else:
            self.get_logger().info("lightboard_detector DISABLED")
            self._release_camera()
            self.stable_pub.publish(Bool(data=False))

    def process_frame(self) -> None:
        if not self.enabled:
            return
        if not self._ensure_camera_open():
            return

        assert self.cap is not None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        crop = self._crop_roi(frame)
        if crop is None:
            return

        current_map = self._classify_grid(crop)
        self._publish_raw_map(current_map)

        key = tuple(current_map)
        self.history.append(key)

        stable_map, stable = self._get_stable_map()
        self.stable_pub.publish(Bool(data=stable))
        if stable:
            if self.last_stable_map != stable_map:
                self.last_stable_map = stable_map
                self.map_pub.publish(self._build_map_msg(list(stable_map)))

    def _crop_roi(self, frame: np.ndarray) -> Optional[np.ndarray]:
        h, w = frame.shape[:2]

        x = max(0, self.roi_x)
        y = max(0, self.roi_y)
        rw = self.roi_w if self.roi_w > 0 else (w - x)
        rh = self.roi_h if self.roi_h > 0 else (h - y)

        rw = min(rw, w - x)
        rh = min(rh, h - y)

        if rw <= 0 or rh <= 0:
            return None

        return frame[y : y + rh, x : x + rw]

    def _classify_grid(self, crop: np.ndarray) -> List[int]:
        ch, cw = crop.shape[:2]
        output: List[int] = []

        for r in range(self.rows):
            y0 = int(r * ch / self.rows)
            y1 = int((r + 1) * ch / self.rows)
            for c in range(self.cols):
                x0 = int(c * cw / self.cols)
                x1 = int((c + 1) * cw / self.cols)

                sx0, sy0, sx1, sy1 = self._sample_box(x0, y0, x1, y1)
                cell = crop[sy0:sy1, sx0:sx1]
                if cell.size == 0:
                    output.append(self.EMPTY)
                    continue

                state = self._classify_cell(cell)
                output.append(state)

        return output

    def _sample_box(self, x0: int, y0: int, x1: int, y1: int) -> Tuple[int, int, int, int]:
        w = x1 - x0
        h = y1 - y0

        sw = int(w * self.sample_ratio)
        sh = int(h * self.sample_ratio)
        sw = max(1, sw)
        sh = max(1, sh)

        cx = (x0 + x1) // 2
        cy = (y0 + y1) // 2

        sx0 = max(x0, cx - sw // 2)
        sy0 = max(y0, cy - sh // 2)
        sx1 = min(x1, sx0 + sw)
        sy1 = min(y1, sy0 + sh)

        return sx0, sy0, sx1, sy1

    def _classify_cell(self, cell_bgr: np.ndarray) -> int:
        hsv = cv2.cvtColor(cell_bgr, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        lit = v >= self.min_v_lit
        lit_count = int(np.count_nonzero(lit))
        if lit_count == 0:
            return self.EMPTY

        # RGBW board: white LEDs are bright and low-saturation.
        white_mask = lit & (s <= self.max_s_white)
        white_count = int(np.count_nonzero(white_mask))
        if white_count / float(lit_count) >= self.min_color_ratio:
            return self.EMPTY

        valid_color = lit & (s >= self.min_s_color)
        valid_count = int(np.count_nonzero(valid_color))
        if valid_count == 0:
            # Bright but low-saturation is treated as EMPTY.
            return self.EMPTY

        # Hue-based class masks (OpenCV hue: 0..179)
        r1_mask = valid_color & ((h <= 15) | (h >= 160))
        r2_mask = valid_color & (h >= 35) & (h <= 95)
        fake_mask = valid_color & (h >= 96) & (h <= 140)

        r1_count = int(np.count_nonzero(r1_mask))
        r2_count = int(np.count_nonzero(r2_mask))
        fake_count = int(np.count_nonzero(fake_mask))

        counts = {
            self.R1: r1_count,
            self.R2: r2_count,
            self.FAKE: fake_count,
        }
        best_state = max(counts, key=counts.get)
        best_count = counts[best_state]

        if best_count / float(valid_count) < self.min_color_ratio:
            return self.EMPTY

        return best_state

    def _publish_raw_map(self, values: List[int]) -> None:
        self.map_raw_pub.publish(self._build_map_msg(values))

    def _build_map_msg(self, values: List[int]) -> UInt8MultiArray:
        msg = UInt8MultiArray()

        dim_rows = MultiArrayDimension()
        dim_rows.label = "rows"
        dim_rows.size = self.rows
        dim_rows.stride = self.rows * self.cols

        dim_cols = MultiArrayDimension()
        dim_cols.label = "cols"
        dim_cols.size = self.cols
        dim_cols.stride = self.cols

        msg.layout.dim = [dim_rows, dim_cols]
        msg.layout.data_offset = 0
        msg.data = [int(v) for v in values]
        return msg

    def _get_stable_map(self) -> Tuple[Tuple[int, ...], bool]:
        if not self.history:
            return tuple(), False

        counter = Counter(self.history)
        best_map, count = counter.most_common(1)[0]
        stable = (
            len(self.history) >= self.stable_frames_required
            and count >= self.stable_frames_required
        )
        return best_map, stable

    def _ensure_camera_open(self) -> bool:
        if self.cap is not None and self.cap.isOpened():
            return True
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            if not self.last_open_failed:
                self.get_logger().error(
                    f"Failed to open camera index {self.camera_index}"
                )
            self.last_open_failed = True
            return False
        self.last_open_failed = False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        return True

    def _release_camera(self) -> None:
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        self.cap = None

    def destroy_node(self) -> bool:
        self._release_camera()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LightboardDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
