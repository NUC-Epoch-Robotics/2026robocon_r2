from collections import deque
from typing import Deque, List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SpearheadDetectorNode(Node):
    """Lightweight spearhead existence detector using OpenCV only."""

    def __init__(self) -> None:
        super().__init__("spearhead_detector")

        self.declare_parameter("camera_index", 1)
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("roi_x", 0)
        self.declare_parameter("roi_y", 0)
        self.declare_parameter("roi_w", 0)
        self.declare_parameter("roi_h", 0)
        self.declare_parameter("min_contour_area", 1200.0)
        self.declare_parameter("min_fill_ratio", 0.015)
        self.declare_parameter("invert_binary", False)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("stable_frames_required", 3)
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("start_enabled", False)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.fps = float(self.get_parameter("fps").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.roi_x = int(self.get_parameter("roi_x").value)
        self.roi_y = int(self.get_parameter("roi_y").value)
        self.roi_w = int(self.get_parameter("roi_w").value)
        self.roi_h = int(self.get_parameter("roi_h").value)
        self.min_contour_area = float(self.get_parameter("min_contour_area").value)
        self.min_fill_ratio = float(self.get_parameter("min_fill_ratio").value)
        self.invert_binary = bool(self.get_parameter("invert_binary").value)
        self.history_size = max(1, int(self.get_parameter("history_size").value))
        self.stable_frames_required = max(
            1, int(self.get_parameter("stable_frames_required").value)
        )
        self.flip_horizontal = bool(self.get_parameter("flip_horizontal").value)
        self.enabled = bool(self.get_parameter("start_enabled").value)

        self.exists_pub = self.create_publisher(Bool, "spearhead/exists", 10)
        self.enable_sub = self.create_subscription(
            Bool, "spearhead/enable", self.on_enable, 10
        )

        self.cap: Optional[cv2.VideoCapture] = None
        self.history: Deque[bool] = deque(maxlen=self.history_size)
        self.last_published_exists = False
        self.last_open_failed = False

        interval = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(interval, self.process_frame)

        if self.enabled:
            self._ensure_camera_open()

        self.get_logger().info(
            f"spearhead_detector started: camera={self.camera_index}, enabled={self.enabled}"
        )

    def on_enable(self, msg: Bool) -> None:
        if self.enabled == msg.data:
            return

        self.enabled = msg.data
        self.history.clear()

        if self.enabled:
            self.get_logger().info("spearhead detector enabled")
            self._ensure_camera_open()
            return

        self.get_logger().info("spearhead detector disabled")
        self._release_camera()
        self._publish_exists(False)

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

        exists_raw = self._detect_exists(crop)
        self.history.append(exists_raw)

        stable_exists = self._get_stable_exists()
        self._publish_exists(stable_exists)

    def _detect_exists(self, crop: np.ndarray) -> bool:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        _, binary = cv2.threshold(
            blur,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU,
        )

        if self.invert_binary:
            binary = cv2.bitwise_not(binary)

        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area

        fill_ratio = float(np.count_nonzero(binary)) / float(binary.size)
        return max_area >= self.min_contour_area and fill_ratio >= self.min_fill_ratio

    def _get_stable_exists(self) -> bool:
        if not self.history:
            return False

        true_count = sum(1 for value in self.history if value)
        false_count = len(self.history) - true_count

        if len(self.history) < self.stable_frames_required:
            return self.last_published_exists

        if true_count >= self.stable_frames_required:
            return True

        if false_count >= self.stable_frames_required:
            return False

        return self.last_published_exists

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

    def _ensure_camera_open(self) -> bool:
        if self.cap is not None and self.cap.isOpened():
            return True

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            if not self.last_open_failed:
                self.get_logger().error(
                    f"Failed to open spearhead camera index {self.camera_index}"
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

    def _publish_exists(self, exists: bool) -> None:
        self.last_published_exists = exists
        self.exists_pub.publish(Bool(data=exists))

    def destroy_node(self) -> bool:
        self._release_camera()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SpearheadDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
