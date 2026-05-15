"""Grab-scene confirmation detector for R2 Zone 2 block picking.

Three grab scenarios (scenes):
  1 - LOW_GRAB_L1_HIGH : robot low, block on layer-1 high side
      -> large colour block on LEFT half of sideways frame
  2 - HIGH_GRAB_L1_LOW : robot high, block on layer-1 low side
      -> large colour block on RIGHT half of sideways frame
  3 - LOW_GRAB_L2_HIGH : robot low, block on layer-2 high side
      -> small colour block (far away)

Red / blue side is selected via the ``is_red_side`` parameter.
Blue is simply the complementary hue range in HSV.

This node shares the same physical camera as lightboard_detector
(camera_index 0).  It opens / releases the camera on enable / disable
so the two nodes never fight over the device.
"""

from collections import deque
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8


class GrabSceneDetectorNode(Node):
    """Confirm that the camera view matches the expected grab scene."""

    # Scene IDs  ── keep in sync with decision node
    SCENE_NONE = 0
    SCENE_LOW_GRAB_L1_HIGH = 1  # large block, LEFT side
    SCENE_HIGH_GRAB_L1_LOW = 2  # large block, RIGHT side
    SCENE_LOW_GRAB_L2_HIGH = 3  # small block (far)

    def __init__(self) -> None:
        super().__init__("grab_scene_detector")

        # ── camera ──────────────────────────────────────────────
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("flip_horizontal", False)

        # ── ROI (optional crop before analysis) ─────────────────
        self.declare_parameter("roi_x", 0)
        self.declare_parameter("roi_y", 0)
        self.declare_parameter("roi_w", 0)  # 0 = full width
        self.declare_parameter("roi_h", 0)  # 0 = full height

        # ── colour thresholds (HSV, OpenCV: H 0-179) ────────────
        self.declare_parameter("is_red_side", True)

        # Red hue ranges (two intervals because red wraps around 0)
        self.declare_parameter("red_h_low1", 0)
        self.declare_parameter("red_h_high1", 15)
        self.declare_parameter("red_h_low2", 160)
        self.declare_parameter("red_h_high2", 179)

        # Blue hue range
        self.declare_parameter("blue_h_low", 100)
        self.declare_parameter("blue_h_high", 130)

        # Saturation / value minimums for colour detection
        self.declare_parameter("min_s", 70)
        self.declare_parameter("min_v", 50)

        # ── scene classification thresholds ─────────────────────
        # colour_ratio = colour_pixels / total_pixels
        self.declare_parameter("large_block_min_ratio", 0.25)
        # cx threshold: block centroid x < this → LEFT (scene 1)
        self.declare_parameter("left_right_cx_threshold", 0.5)
        # Small-block minimum ratio (below this → SCENE_NONE)
        self.declare_parameter("small_block_min_ratio", 0.03)

        # ── morphology kernel size ──────────────────────────────
        self.declare_parameter("morph_kernel_size", 5)

        # ── stability ──────────────────────────────────────────
        self.declare_parameter("history_size", 8)
        self.declare_parameter("stable_frames_required", 5)

        # ── read all params ─────────────────────────────────────
        self.camera_index: int = int(self.get_parameter("camera_index").value)
        self.fps: float = float(self.get_parameter("fps").value)
        self.frame_width: int = int(self.get_parameter("frame_width").value)
        self.frame_height: int = int(self.get_parameter("frame_height").value)
        self.flip_horizontal: bool = bool(self.get_parameter("flip_horizontal").value)

        self.roi_x: int = int(self.get_parameter("roi_x").value)
        self.roi_y: int = int(self.get_parameter("roi_y").value)
        self.roi_w: int = int(self.get_parameter("roi_w").value)
        self.roi_h: int = int(self.get_parameter("roi_h").value)

        self.is_red_side: bool = bool(self.get_parameter("is_red_side").value)

        self.red_h_low1: int = int(self.get_parameter("red_h_low1").value)
        self.red_h_high1: int = int(self.get_parameter("red_h_high1").value)
        self.red_h_low2: int = int(self.get_parameter("red_h_low2").value)
        self.red_h_high2: int = int(self.get_parameter("red_h_high2").value)

        self.blue_h_low: int = int(self.get_parameter("blue_h_low").value)
        self.blue_h_high: int = int(self.get_parameter("blue_h_high").value)

        self.min_s: int = int(self.get_parameter("min_s").value)
        self.min_v: int = int(self.get_parameter("min_v").value)

        self.large_block_min_ratio: float = float(
            self.get_parameter("large_block_min_ratio").value
        )
        self.left_right_cx_threshold: float = float(
            self.get_parameter("left_right_cx_threshold").value
        )
        self.small_block_min_ratio: float = float(
            self.get_parameter("small_block_min_ratio").value
        )

        self.morph_kernel_size: int = int(self.get_parameter("morph_kernel_size").value)

        self.history_size: int = max(1, int(self.get_parameter("history_size").value))
        self.stable_frames_required: int = max(
            1, int(self.get_parameter("stable_frames_required").value)
        )

        # ── publishers / subscribers ────────────────────────────
        # The decision node publishes expected_scene and enable.
        # We publish detected scene and ready flag.
        self.detected_pub = self.create_publisher(
            UInt8, "grab_scene/detected", 10
        )
        self.ready_pub = self.create_publisher(
            Bool, "grab_scene/ready", 10
        )
        self.debug_ratio_pub = self.create_publisher(
            UInt8, "grab_scene/debug_ratio_pct", 10
        )

        self.enable_sub = self.create_subscription(
            Bool, "grab_scene/enable", self._on_enable, 10
        )
        self.expected_scene_sub = self.create_subscription(
            UInt8, "grab_scene/expected_scene", self._on_expected_scene, 10
        )

        # ── internal state ──────────────────────────────────────
        self.enabled: bool = False
        self.expected_scene: int = self.SCENE_NONE
        self.cap: Optional[cv2.VideoCapture] = None
        self.history: Deque[int] = deque(maxlen=self.history_size)
        self.last_published_ready: bool = False
        self.last_open_failed: bool = False

        # ── timer ───────────────────────────────────────────────
        interval = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(interval, self._tick)

        self.get_logger().info(
            f"grab_scene_detector started: camera={self.camera_index}, "
            f"is_red_side={self.is_red_side}"
        )

    # ─── topic callbacks ────────────────────────────────────────

    def _on_enable(self, msg: Bool) -> None:
        if self.enabled == msg.data:
            return
        self.enabled = msg.data
        self.history.clear()
        self.last_published_ready = False

        if self.enabled:
            self.get_logger().info("grab_scene_detector ENABLED")
            self._ensure_camera_open()
        else:
            self.get_logger().info("grab_scene_detector DISABLED")
            self._release_camera()
            self._publish_ready(False)

    def _on_expected_scene(self, msg: UInt8) -> None:
        if self.expected_scene == msg.data:
            return
        self.expected_scene = int(msg.data)
        self.history.clear()
        self.last_published_ready = False
        self.get_logger().info(f"expected scene set to {self.expected_scene}")

    # ─── main loop ──────────────────────────────────────────────

    def _tick(self) -> None:
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

        detected = self._classify_scene(crop)

        # publish raw detection every frame
        det_msg = UInt8()
        det_msg.data = detected
        self.detected_pub.publish(det_msg)

        # accumulate history
        self.history.append(detected)

        # check stability against expected scene
        if self.expected_scene == self.SCENE_NONE:
            self._publish_ready(False)
            return

        match_count = sum(
            1 for s in self.history if s == self.expected_scene
        )
        stable = (
            len(self.history) >= self.stable_frames_required
            and match_count >= self.stable_frames_required
        )

        if stable and not self.last_published_ready:
            self.get_logger().info(
                f"Scene CONFIRMED: detected={detected}, "
                f"expected={self.expected_scene}, "
                f"matches={match_count}/{len(self.history)}"
            )

        self._publish_ready(stable)

    # ─── scene classification ───────────────────────────────────

    def _classify_scene(self, crop: np.ndarray) -> int:
        """Classify the current camera crop into one of 3 scenes."""
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h, w = crop.shape[:2]
        total_pixels = h * w

        # 1. build colour mask
        colour_mask = self._build_colour_mask(hsv)

        # 2. morphological clean-up
        ks = self.morph_kernel_size
        kernel = np.ones((ks, ks), np.uint8)
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_OPEN, kernel)
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_CLOSE, kernel)

        # 3. colour ratio
        colour_pixels = int(cv2.countNonZero(colour_mask))
        colour_ratio = colour_pixels / max(total_pixels, 1)

        # debug: publish ratio as 0-100 percentage
        dbg = UInt8()
        dbg.data = min(int(colour_ratio * 100), 255)
        self.debug_ratio_pub.publish(dbg)

        if colour_ratio < self.small_block_min_ratio:
            return self.SCENE_NONE

        # 4. find largest contour → centroid
        contours, _ = cv2.findContours(
            colour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return self.SCENE_NONE

        largest = max(contours, key=cv2.contourArea)
        moments = cv2.moments(largest)
        if moments["m00"] < 1.0:
            return self.SCENE_NONE

        cx = moments["m10"] / moments["m00"] / w  # normalised 0..1
        # cy = moments["m01"] / moments["m00"] / h  # not used currently

        # 5. decision tree
        if colour_ratio >= self.large_block_min_ratio:
            # Big block → scene 1 or 2
            if cx < self.left_right_cx_threshold:
                return self.SCENE_LOW_GRAB_L1_HIGH  # LEFT → scene 1
            else:
                return self.SCENE_HIGH_GRAB_L1_LOW  # RIGHT → scene 2
        else:
            # Small block → scene 3
            return self.SCENE_LOW_GRAB_L2_HIGH

    def _build_colour_mask(self, hsv: np.ndarray) -> np.ndarray:
        """Build a binary mask of the target colour (red or blue)."""
        if self.is_red_side:
            mask1 = cv2.inRange(
                hsv,
                (self.red_h_low1, self.min_s, self.min_v),
                (self.red_h_high1, 255, 255),
            )
            mask2 = cv2.inRange(
                hsv,
                (self.red_h_low2, self.min_s, self.min_v),
                (self.red_h_high2, 255, 255),
            )
            return mask1 | mask2
        else:
            return cv2.inRange(
                hsv,
                (self.blue_h_low, self.min_s, self.min_v),
                (self.blue_h_high, 255, 255),
            )

    # ─── camera helpers ─────────────────────────────────────────

    def _crop_roi(self, frame: np.ndarray) -> Optional[np.ndarray]:
        fh, fw = frame.shape[:2]
        x = max(0, self.roi_x)
        y = max(0, self.roi_y)
        rw = self.roi_w if self.roi_w > 0 else (fw - x)
        rh = self.roi_h if self.roi_h > 0 else (fh - y)
        rw = min(rw, fw - x)
        rh = min(rh, fh - y)
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

    def _publish_ready(self, ready: bool) -> None:
        self.last_published_ready = ready
        self.ready_pub.publish(Bool(data=ready))

    # ─── cleanup ────────────────────────────────────────────────

    def destroy_node(self) -> bool:
        self._release_camera()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GrabSceneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
