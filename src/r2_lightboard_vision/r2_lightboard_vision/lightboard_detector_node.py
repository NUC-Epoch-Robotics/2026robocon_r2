from collections import Counter, deque
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
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

        self.declare_parameter("camera_index", 0)  # 兼容保留: D435i 无 V4L 节点, 实际用 pyrealsense2
        self.declare_parameter("serial_number", "")  # 可选: 指定 D435i 序列号, 空=第一台
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
        self.declare_parameter("min_board_area", 5000)   # 灯板矩形最小像素面积, 过滤噪声
        self.declare_parameter("board_rect_pad", 8)      # 灯板外接矩形外扩像素, 包住边缘灯
        self.declare_parameter("min_s_color", 45)
        self.declare_parameter("max_s_white", 35)
        self.declare_parameter("min_color_ratio", 0.22)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("stable_frames_required", 3)
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("show_debug", False)  # 显示调试窗口: 网格+采样框+分类颜色
        # ── D435i 曝光 / 白平衡 (color sensor) ──
        # auto_*: True=自动. 设了手动值时建议把对应 auto 关掉.
        # -1 = 不设 (用驱动默认), exposure_us 范围 1~10000 (越大越亮), gain 范围 0~128
        self.declare_parameter("auto_exposure", True)
        self.declare_parameter("exposure_us", -1)        # 手动曝光 (us), 仅 auto_exposure=False 时生效
        self.declare_parameter("gain", -1)               # 手动增益
        self.declare_parameter("auto_white_balance", False)  # 默认关: 自动会乱改色温影响红绿蓝判定
        self.declare_parameter("white_balance", -1)      # 手动白平衡, -1=不设

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.serial_number = str(self.get_parameter("serial_number").value).strip()
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
        self.min_board_area = int(self.get_parameter("min_board_area").value)
        self.board_rect_pad = int(self.get_parameter("board_rect_pad").value)
        self.min_s_color = int(self.get_parameter("min_s_color").value)
        self.max_s_white = int(self.get_parameter("max_s_white").value)
        self.min_color_ratio = float(self.get_parameter("min_color_ratio").value)
        self.history_size = int(self.get_parameter("history_size").value)
        self.stable_frames_required = int(self.get_parameter("stable_frames_required").value)
        self.flip_horizontal = bool(self.get_parameter("flip_horizontal").value)
        self.enabled = bool(self.get_parameter("start_enabled").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)
        self.auto_exposure = bool(self.get_parameter("auto_exposure").value)
        self.exposure_us = int(self.get_parameter("exposure_us").value)
        self.gain = int(self.get_parameter("gain").value)
        self.auto_white_balance = bool(self.get_parameter("auto_white_balance").value)
        self.white_balance = int(self.get_parameter("white_balance").value)

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
        self.pipeline: Optional[rs.pipeline] = None
        self.pipeline_started: bool = False
        self.last_open_failed: bool = False
        self._last_open_retry_ns: int = 0  # 限速重试 (失败后每 3s 再试)

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

        assert self.pipeline is not None
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=200)
        except Exception:
            return
        if not frames:
            return

        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        frame = np.asanyarray(color_frame.get_data())
        if frame is None or frame.size == 0:
            return

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        # 整帧里定位灯板矩形; 找不到就跳过本帧(不发布, 不污染稳定历史)
        search, off_x, off_y = self._crop_roi(frame)  # 可选搜索区 (roi_* 设了就限范围, 不设=整帧)
        if search is None:
            return
        board_rect_rel = self._find_board_rect(search)
        if board_rect_rel is None:
            if self.show_debug:
                self._show_debug(frame, None, None)
            return

        # 灯板矩形坐标转回整帧系 (叠加搜索区偏移)
        bx, by, bw, bh = board_rect_rel
        board_rect = (bx + off_x, by + off_y, bw, bh)

        current_map = self._classify_grid(frame, board_rect)
        self._publish_raw_map(current_map)

        if self.show_debug:
            self._show_debug(frame, board_rect, current_map)

        key = tuple(current_map)
        self.history.append(key)

        stable_map, stable = self._get_stable_map()
        self.stable_pub.publish(Bool(data=stable))
        if stable:
            if self.last_stable_map != stable_map:
                self.last_stable_map = stable_map
                self.map_pub.publish(self._build_map_msg(list(stable_map)))

    def _find_board_rect(self, search: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """整帧(或搜索区)里定位灯板: 高亮(V)二值化 → 最大轮廓外接矩形.

        返回灯板在 search 内的 (x, y, w, h). 找不到或太小返回 None.
        """
        hsv = cv2.cvtColor(search, cv2.COLOR_BGR2HSV)
        v = hsv[:, :, 2]
        lit = (v >= self.min_v_lit).astype(np.uint8) * 255

        # 闭运算填灯点间空隙, 让整块灯板连成一个连通域
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        closed = cv2.morphologyEx(lit, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # 取最大轮廓的外接矩形 = 灯板
        biggest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest) < self.min_board_area:
            return None

        bx, by, bw, bh = cv2.boundingRect(biggest)
        # 外扩 padding, 包住边缘灯点
        pad = self.board_rect_pad
        sh, sw = search.shape[:2]
        bx = max(0, bx - pad)
        by = max(0, by - pad)
        bw = min(sw - bx, bw + 2 * pad)
        bh = min(sh - by, bh + 2 * pad)
        return (bx, by, bw, bh)

    def _crop_roi(self, frame: np.ndarray) -> Tuple[Optional[np.ndarray], int, int]:
        """可选搜索区裁剪. 返回 (裁出图像, 偏移x, 偏移y). roi_* 不设 = 整帧 (偏移0,0)."""
        h, w = frame.shape[:2]

        x = max(0, self.roi_x)
        y = max(0, self.roi_y)
        rw = self.roi_w if self.roi_w > 0 else (w - x)
        rh = self.roi_h if self.roi_h > 0 else (h - y)

        rw = min(rw, w - x)
        rh = min(rh, h - y)

        if rw <= 0 or rh <= 0:
            return None, 0, 0

        return frame[y : y + rh, x : x + rw], x, y

    def _classify_grid(self, frame: np.ndarray,
                       board_rect: Tuple[int, int, int, int]) -> List[int]:
        """灯板矩形内按 rows×cols 均分, 每格中心采样判色. 行优先输出."""
        bx, by, bw, bh = board_rect
        output: List[int] = []

        for r in range(self.rows):
            y0 = by + int(r * bh / self.rows)
            y1 = by + int((r + 1) * bh / self.rows)
            for c in range(self.cols):
                x0 = bx + int(c * bw / self.cols)
                x1 = bx + int((c + 1) * bw / self.cols)

                sx0, sy0, sx1, sy1 = self._sample_box(x0, y0, x1, y1)
                cell = frame[sy0:sy1, sx0:sx1]
                if cell.size == 0:
                    output.append(self.EMPTY)
                    continue

                state = self._classify_cell(cell)
                output.append(state)

        return output

    def _show_debug(self, frame: np.ndarray,
                    board_rect: Optional[Tuple[int, int, int, int]],
                    current_map: Optional[List[int]]) -> None:
        """整帧上画灯板矩形 + 灯板内 4x3 网格 + 标签.

        board_rect=None 表示没找到灯板, 整帧提示.
        """
        # 配色: R1=红, R2=绿, FAKE=蓝, EMPTY=白 (RGBW 白灯=EMPTY, 没有灰)
        label_color = {
            self.EMPTY: (255, 255, 255),  # BGR white
            self.R1: (0, 0, 255),         # BGR red
            self.R2: (0, 255, 0),         # BGR green
            self.FAKE: (255, 0, 0),       # BGR blue
        }
        label_text = {self.EMPTY: "E", self.R1: "R1", self.R2: "R2", self.FAKE: "F"}

        h, w = frame.shape[:2]
        vis = frame.copy()

        _, stable = self._get_stable_map()

        if board_rect is None or current_map is None:
            cv2.rectangle(vis, (0, 0), (w, 26), (0, 0, 0), -1)
            cv2.putText(vis, f"board NOT found (V<threshold={self.min_v_lit}?) stable={stable}",
                        (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            try:
                cv2.imshow("lightboard_debug", vis)
                cv2.waitKey(1)
            except Exception:
                pass
            return

        bx, by, bw, bh = board_rect
        # 灯板外接矩形 (绿框)
        cv2.rectangle(vis, (bx, by), (bx + bw - 1, by + bh - 1), (0, 255, 0), 2)

        # 灯板内 4x3 网格 + 采样框 + 标签
        for r in range(self.rows):
            gy0 = by + int(r * bh / self.rows)
            gy1 = by + int((r + 1) * bh / self.rows)
            for c in range(self.cols):
                gx0 = bx + int(c * bw / self.cols)
                gx1 = bx + int((c + 1) * bw / self.cols)

                state = current_map[r * self.cols + c]
                color = label_color[state]

                cv2.rectangle(vis, (gx0, gy0), (gx1 - 1, gy1 - 1), color, 2)
                sx0, sy0, sx1, sy1 = self._sample_box(gx0, gy0, gx1, gy1)
                cv2.rectangle(vis, (sx0, sy0), (sx1 - 1, sy1 - 1), (255, 255, 255), 1)
                # 白色标签加黑描边, 防白底看不见
                cv2.putText(vis, label_text[state], (gx0 + 4, gy1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(vis, label_text[state], (gx0 + 4, gy1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

        # 信息条
        info = f"board ({bx},{by}) {bw}x{bh} grid={self.rows}x{self.cols} stable={stable} V_th={self.min_v_lit}"
        cv2.rectangle(vis, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.putText(vis, info, (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)

        try:
            cv2.imshow("lightboard_debug", vis)
            cv2.waitKey(1)
        except Exception:
            pass

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
        if self.pipeline_started and self.pipeline is not None:
            return True

        # 失败后限速重试: 每 3s 才再试一次, 避免每个 tick 反复 new pipeline 占死设备
        now = self.get_clock().now().nanoseconds
        if self.last_open_failed and (now - self._last_open_retry_ns) < 3_000_000_000:
            return False
        self._last_open_retry_ns = now

        cfg = rs.config()
        if self.serial_number:
            cfg.enable_device(self.serial_number)
        # D435i color 流: bgr8 与 cv2 兼容. 支持分辨率 640x480 / 1280x720 等
        cfg.enable_stream(
            rs.stream.color, self.frame_width, self.frame_height,
            rs.format.bgr8, int(self.fps),
        )
        pipeline = rs.pipeline()
        try:
            pipeline.start(cfg)
        except Exception as e:
            # start 失败要显式 stop 释放这个半构造的对象, 否则它会占住设备导致后续一直 No device
            try:
                pipeline.stop()
            except Exception:
                pass
            if not self.last_open_failed:
                self.get_logger().error(
                    f"Failed to start realsense pipeline {self.frame_width}x{self.frame_height}@{int(self.fps)}: {e}. "
                    f"D435i supports 640x480/1280x720@6/15/30; try changing frame_width/height/fps. "
                    f"Also check: another process holding the camera? (ps aux | grep lightboard)"
                )
            self.last_open_failed = True
            self.pipeline = None
            self.pipeline_started = False
            return False

        self.pipeline = pipeline
        self.pipeline_started = True
        self.last_open_failed = False
        self.get_logger().info(
            f"realsense color stream {self.frame_width}x{self.frame_height}@{int(self.fps)} started"
        )
        self._apply_color_options()
        return True

    def _apply_color_options(self) -> None:
        """设 D435i color sensor 的曝光/白平衡 (调用前 pipeline 已 start)."""
        if self.pipeline is None:
            return
        try:
            dev = self.pipeline.get_active_profile().get_device()
            color_sensor = dev.first_color_sensor()

            # 白平衡 (建议手动, 避免自动乱改色温影响红绿蓝 hue 判定)
            color_sensor.set_option(rs.option.enable_auto_white_balance,
                                    1 if self.auto_white_balance else 0)
            if not self.auto_white_balance and self.white_balance >= 0:
                color_sensor.set_option(rs.option.white_balance, float(self.white_balance))

            # 曝光
            color_sensor.set_option(rs.option.enable_auto_exposure,
                                    1 if self.auto_exposure else 0)
            if not self.auto_exposure and self.exposure_us >= 0:
                color_sensor.set_option(rs.option.exposure, float(self.exposure_us))
            if self.gain >= 0:
                color_sensor.set_option(rs.option.gain, float(self.gain))

            self.get_logger().info(
                f"color options: AE={self.auto_exposure} exp={self.exposure_us} "
                f"gain={self.gain} AWB={self.auto_white_balance} wb={self.white_balance}"
            )
        except Exception as e:
            self.get_logger().warn(f"set color sensor options failed: {e}")

    def _release_camera(self) -> None:
        if self.pipeline_started and self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None
        self.pipeline_started = False

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
