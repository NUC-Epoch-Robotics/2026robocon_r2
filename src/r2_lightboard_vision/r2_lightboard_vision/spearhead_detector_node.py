from collections import deque
from typing import Deque, List, Optional

import ctypes as ct
import ctypes.util
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


# ── libuvc ctypes wrapper (V4L2 broken on Jetson) ────────────
class _UvcFrame(ct.Structure):
    """Mirrors uvc_frame from libuvc."""
    _fields_ = [
        ("data", ct.c_void_p),
        ("data_bytes", ct.c_size_t),
        ("width", ct.c_uint32),
        ("height", ct.c_uint32),
        ("frame_format", ct.c_int),  # uvc_frame_format enum
        ("step", ct.c_size_t),
        ("sequence", ct.c_uint32),
        ("capture_time", ct.c_int64 * 2),  # timeval-ish
        ("source", ct.c_void_p),
        ("library_owns_data", ct.c_uint8),
    ]


class _UvcStreamCtrl(ct.Structure):
    """Mirrors uvc_stream_ctrl_t."""
    _fields_ = [
        ("bmHint", ct.c_uint16),
        ("bFormatIndex", ct.c_uint8),
        ("bFrameIndex", ct.c_uint8),
        ("dwFrameInterval", ct.c_uint32),
        ("wKeyFrameRate", ct.c_uint16),
        ("wPFrameRate", ct.c_uint16),
        ("wCompQuality", ct.c_uint16),
        ("wCompWindowSize", ct.c_uint16),
        ("wDelay", ct.c_uint16),
        ("dwMaxVideoFrameSize", ct.c_uint32),
        ("dwMaxPayloadTransferSize", ct.c_uint32),
        ("dwClockFrequency", ct.c_uint32),
        ("bmFramingInfo", ct.c_uint8),
        ("bPreferredVersion", ct.c_uint8),
        ("bMinVersion", ct.c_uint8),
        ("bMaxVersion", ct.c_uint8),
        ("bInterfaceNumber", ct.c_uint8),
    ]


# uvc_frame_format enum value for MJPEG
_UVC_FRAME_FORMAT_MJPEG = 7


class LibuvcCamera:
    """Minimal ctypes wrapper around libuvc for MJPEG capture."""

    def __init__(self, vid: int, pid: int, width: int, height: int, fps: int,
                 lib_path: str = ""):
        self._ctx = ct.c_void_p()
        self._dev = ct.c_void_p()
        self._devh = ct.c_void_p()
        self._strmh = ct.c_void_p()  # stream handle
        self._streaming = False
        self._ctrl = _UvcStreamCtrl()

        # find libuvc.so
        so_path = lib_path or self._find_so()
        self._lib = ct.CDLL(so_path)

        self._setup_prototypes()

        # uvc_init
        ret = self._lib.uvc_init(ct.byref(self._ctx), None)
        if ret != 0:
            raise RuntimeError(f"uvc_init failed: {ret}")

        # uvc_find_device
        ret = self._lib.uvc_find_device(
            self._ctx, ct.byref(self._dev), vid, pid, None
        )
        if ret != 0:
            self._lib.uvc_exit(self._ctx)
            raise RuntimeError(f"uvc_find_device({hex(vid)}:{hex(pid)}) failed: {ret}")

        # uvc_open
        ret = self._lib.uvc_open(self._dev, ct.byref(self._devh))
        if ret != 0:
            self._lib.uvc_unref_device(self._dev)
            self._lib.uvc_exit(self._ctx)
            raise RuntimeError(f"uvc_open failed: {ret}")

        # Manually fill stream ctrl (bypass broken uvc_get_stream_ctrl_format_size)
        # Values from C example: bFormatIndex=1 (MJPEG), bFrameIndex=1 (1920x1080)
        self._ctrl.bmHint = 1
        self._ctrl.bFormatIndex = 1  # MJPEG format index
        self._ctrl.bFrameIndex = 1   # 1920x1080 frame index
        self._ctrl.dwFrameInterval = int(10_000_000 / fps)  # 30fps → 333333
        self._ctrl.wKeyFrameRate = 0
        self._ctrl.wPFrameRate = 0
        self._ctrl.wCompQuality = 0
        self._ctrl.wCompWindowSize = 0
        self._ctrl.wDelay = 0
        self._ctrl.dwMaxVideoFrameSize = width * height * 2  # rough estimate
        self._ctrl.dwMaxPayloadTransferSize = 3072
        self._ctrl.bInterfaceNumber = 1

    @staticmethod
    def _find_so() -> str:
        """Try common locations for libuvc.so."""
        candidates = [
            "/usr/local/lib/libuvc.so",
            "/usr/lib/libuvc.so",
            str((ct.util.find_library("uvc") or "")),
        ]
        import os
        home = os.path.expanduser("~")
        candidates.insert(0, os.path.join(home, "Desktop/libuvc/build/libuvc.so"))
        for p in candidates:
            if p and os.path.isfile(p):
                return p
        raise FileNotFoundError("libuvc.so not found. Set 'uvc_lib_path' parameter.")

    def _setup_prototypes(self):
        lib = self._lib
        # uvc_init(ctx, usb_ctx) -> int
        lib.uvc_init.argtypes = [ct.POINTER(ct.c_void_p), ct.c_void_p]
        lib.uvc_init.restype = ct.c_int
        # uvc_find_device(ctx, dev, vid, pid, sn) -> int
        lib.uvc_find_device.argtypes = [
            ct.c_void_p, ct.POINTER(ct.c_void_p),
            ct.c_int, ct.c_int, ct.c_char_p,
        ]
        lib.uvc_find_device.restype = ct.c_int
        # uvc_open(dev, devh) -> int
        lib.uvc_open.argtypes = [ct.c_void_p, ct.POINTER(ct.c_void_p)]
        lib.uvc_open.restype = ct.c_int
        # uvc_get_stream_ctrl_format_size(devh, ctrl, format, w, h, fps) -> int
        lib.uvc_get_stream_ctrl_format_size.argtypes = [
            ct.c_void_p, ct.POINTER(_UvcStreamCtrl),
            ct.c_int, ct.c_int, ct.c_int, ct.c_int,
        ]
        lib.uvc_get_stream_ctrl_format_size.restype = ct.c_int
        # uvc_print_stream_ctrl(ctrl, stream) — debug
        if hasattr(lib, 'uvc_print_stream_ctrl'):
            lib.uvc_print_stream_ctrl.argtypes = [ct.POINTER(_UvcStreamCtrl), ct.c_void_p]
            lib.uvc_print_stream_ctrl.restype = None
        # uvc_stream_open_ctrl(devh, strmh, ctrl) -> int
        lib.uvc_stream_open_ctrl.argtypes = [
            ct.c_void_p, ct.POINTER(ct.c_void_p), ct.POINTER(_UvcStreamCtrl),
        ]
        lib.uvc_stream_open_ctrl.restype = ct.c_int
        # uvc_stream_start(strmh, cb, user_ptr, flags) -> int
        lib.uvc_stream_start.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_void_p, ct.c_uint8]
        lib.uvc_stream_start.restype = ct.c_int
        # uvc_stream_get_frame(strmh, frame, timeout_us) -> int
        lib.uvc_stream_get_frame.argtypes = [
            ct.c_void_p, ct.POINTER(ct.POINTER(_UvcFrame)), ct.c_int32,
        ]
        lib.uvc_stream_get_frame.restype = ct.c_int
        # uvc_stream_stop(strmh)
        lib.uvc_stream_stop.argtypes = [ct.c_void_p]
        lib.uvc_stream_stop.restype = None
        # uvc_stream_close(strmh)
        lib.uvc_stream_close.argtypes = [ct.c_void_p]
        lib.uvc_stream_close.restype = None
        # uvc_close(devh)
        lib.uvc_close.argtypes = [ct.c_void_p]
        lib.uvc_close.restype = None
        # uvc_unref_device(dev)
        lib.uvc_unref_device.argtypes = [ct.c_void_p]
        lib.uvc_unref_device.restype = None
        # uvc_exit(ctx)
        lib.uvc_exit.argtypes = [ct.c_void_p]
        lib.uvc_exit.restype = None

    def start(self):
        if self._streaming:
            return
        # uvc_stream_open_ctrl: open stream and get stream handle
        ret = self._lib.uvc_stream_open_ctrl(
            self._devh, ct.byref(self._strmh), ct.byref(self._ctrl)
        )
        if ret != 0:
            raise RuntimeError(f"uvc_stream_open_ctrl failed: {ret}")
        # uvc_stream_start: start USB transfers (required before get_frame)
        ret = self._lib.uvc_stream_start(self._strmh, None, None, 0)
        if ret != 0:
            self._lib.uvc_stream_close(self._strmh)
            self._strmh = ct.c_void_p()
            raise RuntimeError(f"uvc_stream_start failed: {ret}")
        self._streaming = True

    def read_frame(self, timeout_us: int = 2_000_000) -> Optional[np.ndarray]:
        """Read one MJPEG frame, return BGR numpy array or None."""
        if not self._streaming or not self._strmh:
            return None
        frame_ptr = ct.POINTER(_UvcFrame)()
        ret = self._lib.uvc_stream_get_frame(
            self._strmh, ct.byref(frame_ptr), timeout_us
        )
        if ret != 0:
            # uvc_stream_get_frame failed
            return None
        if not frame_ptr:
            return None
        f = frame_ptr.contents
        if not f.data or f.data_bytes == 0:
            return None
        # MJPEG → numpy
        try:
            buf = ct.cast(f.data, ct.POINTER(ct.c_uint8 * f.data_bytes)).contents
            arr = np.frombuffer(buf, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return img
        except Exception:
            return None

    def stop(self):
        if self._streaming and self._strmh:
            self._lib.uvc_stream_stop(self._strmh)
            self._lib.uvc_stream_close(self._strmh)
            self._strmh = ct.c_void_p()
            self._streaming = False

    def close(self):
        self.stop()
        if self._devh:
            self._lib.uvc_close(self._devh)
            self._devh = ct.c_void_p()
        if self._dev:
            self._lib.uvc_unref_device(self._dev)
            self._dev = ct.c_void_p()
        if self._ctx:
            self._lib.uvc_exit(self._ctx)
            self._ctx = ct.c_void_p()


class SpearheadDetectorNode(Node):
    """Spearhead existence detector + gray cylinder alignment detector."""

    def __init__(self) -> None:
        super().__init__("spearhead_detector")

        # ── existing parameters ──────────────────────────────────
        self.declare_parameter("camera_index", 1)
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        # libuvc backend (when V4L2 is broken)
        self.declare_parameter("use_libuvc", True)
        self.declare_parameter("uvc_vid", 0x0c45)
        self.declare_parameter("uvc_pid", 0x6368)
        self.declare_parameter("uvc_lib_path", "")
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

        # ── gray cylinder detection parameters ───────────────────
        # outer ROI for cylinder detection (on full frame)
        self.declare_parameter("cyl_roi_x", 100)
        self.declare_parameter("cyl_roi_y", 0)
        self.declare_parameter("cyl_roi_w", 1720)
        self.declare_parameter("cyl_roi_h", 1080)
        # inner target band width (centered in outer ROI), height = roi_h
        self.declare_parameter("cyl_band_width", 110)
        # HSV thresholds for gray: low saturation, mid-high value
        self.declare_parameter("cyl_s_max", 60)
        self.declare_parameter("cyl_v_min", 80)
        self.declare_parameter("cyl_v_max", 220)
        # minimum contour area to accept as cylinder
        self.declare_parameter("cyl_min_area", 3000.0)
        # shape filters: cylinder is ~110x100, roughly square
        self.declare_parameter("cyl_min_aspect", 0.5)       # height/width >= this
        self.declare_parameter("cyl_min_vert_fill", 0.05)   # bbox height / roi height >= this
        # expected cylinder width in pixels at correct distance
        self.declare_parameter("cyl_expected_width", 120.0)

        # ── read existing params ─────────────────────────────────
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

        # ── libuvc params ────────────────────────────────────────
        self.use_libuvc = bool(self.get_parameter("use_libuvc").value)
        self.uvc_vid = int(self.get_parameter("uvc_vid").value)
        self.uvc_pid = int(self.get_parameter("uvc_pid").value)
        self.uvc_lib_path = str(self.get_parameter("uvc_lib_path").value)

        # ── read cylinder params ─────────────────────────────────
        self.cyl_roi_x = int(self.get_parameter("cyl_roi_x").value)
        self.cyl_roi_y = int(self.get_parameter("cyl_roi_y").value)
        self.cyl_roi_w = int(self.get_parameter("cyl_roi_w").value)
        self.cyl_roi_h = int(self.get_parameter("cyl_roi_h").value)
        self.cyl_band_width = int(self.get_parameter("cyl_band_width").value)
        self.cyl_s_max = int(self.get_parameter("cyl_s_max").value)
        self.cyl_v_min = int(self.get_parameter("cyl_v_min").value)
        self.cyl_v_max = int(self.get_parameter("cyl_v_max").value)
        self.cyl_min_area = float(self.get_parameter("cyl_min_area").value)
        self.cyl_min_aspect = float(self.get_parameter("cyl_min_aspect").value)
        self.cyl_min_vert_fill = float(self.get_parameter("cyl_min_vert_fill").value)
        self.cyl_expected_width = float(self.get_parameter("cyl_expected_width").value)

        # ── publishers ───────────────────────────────────────────
        self.exists_pub = self.create_publisher(Bool, "spearhead/exists", 10)
        self.offset_pub = self.create_publisher(Float32, "spearhead/offset", 10)
        self.valid_pub = self.create_publisher(Bool, "spearhead/cyl_valid", 10)
        self.overlap_pub = self.create_publisher(Float32, "spearhead/overlap", 10)
        self.width_pub = self.create_publisher(Float32, "spearhead/cyl_width", 10)

        self.enable_sub = self.create_subscription(
            Bool, "spearhead/enable", self.on_enable, 10
        )

        self.cap: Optional[cv2.VideoCapture] = None
        self.uvc_cam: Optional[LibuvcCamera] = None
        self.history: Deque[bool] = deque(maxlen=self.history_size)
        self.last_published_exists = False
        self.last_open_failed = False
        self._frame_count = 0  # for debug logging throttle

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

        frame = self._read_frame()
        if frame is None:
            self.get_logger().warn_throttle(2.0, "read_frame returned None")
            return

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        # ── spearhead existence (original logic) ─────────────────
        crop = self._crop_roi(frame)
        if crop is not None:
            exists_raw = self._detect_exists(crop)
            self.history.append(exists_raw)
            stable_exists = self._get_stable_exists()
            self._publish_exists(stable_exists)

        # ── gray cylinder alignment detection ────────────────────
        self._detect_cylinder(frame)

        # ── debug: show annotated frame ───────────────────────────
        try:
            cv2.imshow("spearhead_debug", frame)
            cv2.waitKey(1)
        except Exception:
            pass

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
        # ── libuvc backend (V4L2 broken) ─────────────────────
        if self.use_libuvc:
            if self.uvc_cam is not None:
                return True
            try:
                self.uvc_cam = LibuvcCamera(
                    vid=self.uvc_vid, pid=self.uvc_pid,
                    width=self.frame_width, height=self.frame_height,
                    fps=int(self.fps), lib_path=self.uvc_lib_path,
                )
                self.uvc_cam.start()
                self.last_open_failed = False
                self.get_logger().info(
                    f"UVC camera opened: {hex(self.uvc_vid)}:{hex(self.uvc_pid)} "
                    f"{self.frame_width}x{self.frame_height}@{int(self.fps)}fps MJPEG"
                )
                return True
            except Exception as e:
                if not self.last_open_failed:
                    self.get_logger().error(f"Failed to open UVC camera: {e}")
                self.last_open_failed = True
                self.uvc_cam = None
                return False

        # ── V4L2 / cv2 backend ───────────────────────────────
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
        if self.use_libuvc:
            if self.uvc_cam is not None:
                try:
                    self.uvc_cam.close()
                except Exception:
                    pass
                self.uvc_cam = None
            return
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        self.cap = None

    def _read_frame(self) -> Optional[np.ndarray]:
        """Read one frame from the active backend, return BGR numpy array or None."""
        if self.use_libuvc:
            if self.uvc_cam is None:
                return None
            try:
                img = self.uvc_cam.read_frame(timeout_us=2_000_000)
                if img is None:
                    self.get_logger().warn_throttle(2.0, "uvc read_frame: None (decode failed or no data)")
                return img
            except Exception as e:
                self.get_logger().error_throttle(2.0, f"uvc read_frame exception: {e}")
                return None
        # V4L2 / cv2
        if self.cap is None or not self.cap.isOpened():
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def _publish_exists(self, exists: bool) -> None:
        self.last_published_exists = exists
        self.exists_pub.publish(Bool(data=exists))

    # ── gray cylinder detection ──────────────────────────────────

    def _detect_cylinder(self, frame: np.ndarray) -> None:
        """Detect gray cylinder in outer ROI, publish offset / valid / overlap / width."""
        self._frame_count += 1
        fh, fw = frame.shape[:2]

        # clamp ROI to frame
        rx = max(0, self.cyl_roi_x)
        ry = max(0, self.cyl_roi_y)
        rw = min(self.cyl_roi_w, fw - rx)
        rh = min(self.cyl_roi_h, fh - ry)

        # ── debug overlay helpers ────────────────────────────────
        band_half_draw = self.cyl_band_width / 2.0
        roi_cx_full = rx + rw / 2.0

        def draw_roi():
            """Draw ROI rect + center band lines + center line on frame."""
            if rw <= 0 or rh <= 0:
                return
            # outer ROI (green)
            cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
            # center band (red)
            bl = int(roi_cx_full - band_half_draw)
            br = int(roi_cx_full + band_half_draw)
            cv2.line(frame, (bl, ry), (bl, ry + rh), (0, 0, 255), 2)
            cv2.line(frame, (br, ry), (br, ry + rh), (0, 0, 255), 2)
            # center line (blue)
            cx = int(roi_cx_full)
            cv2.line(frame, (cx, ry), (cx, ry + rh), (255, 0, 0), 1)
            # label
            cv2.putText(frame, f"ROI {rw}x{rh} band={self.cyl_band_width}",
                        (rx + 5, ry + 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

        draw_roi()

        if rw <= 0 or rh <= 0:
            self._publish_cylinder(False, 0.0, 0.0, 0.0)
            return

        roi = frame[ry : ry + rh, rx : rx + rw]
        roi_cx = rw / 2.0  # center x of outer ROI

        # HSV threshold for gray: low saturation, mid-high value
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([0, 0, self.cyl_v_min], dtype=np.uint8),
            np.array([180, self.cyl_s_max, self.cyl_v_max], dtype=np.uint8),
        )

        # morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ── debug: draw all contours (faint cyan) ──
        if contours:
            shifted_all = [c + (rx, ry) for c in contours]
            cv2.drawContours(frame, shifted_all, -1, (255, 255, 0), 1)

        if not contours:
            if self._frame_count % 60 == 0:
                self.get_logger().warn(
                    f"cyl: no contours | ROI {rx},{ry} {rw}x{rh} | "
                    f"S<{self.cyl_s_max} V={self.cyl_v_min}-{self.cyl_v_max}")
            self._publish_cylinder(False, 0.0, 0.0, 0.0)
            return

        # find largest contour
        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)

        # ── debug: draw best contour + bbox (yellow) ──
        best_shifted = best + (rx, ry)
        bx_full, by_full, bw_full, bh_full = cv2.boundingRect(best_shifted)
        cv2.drawContours(frame, [best_shifted], -1, (0, 255, 255), 2)
        cv2.rectangle(frame, (bx_full, by_full),
                      (bx_full + bw_full, by_full + bh_full), (255, 0, 255), 2)

        if area < self.cyl_min_area:
            if self._frame_count % 60 == 0:
                self.get_logger().warn(
                    f"cyl: area={area:.0f} < min={self.cyl_min_area}")
            self._publish_cylinder(False, 0.0, 0.0, 0.0)
            return

        # bounding box
        bx, by, bw, bh = cv2.boundingRect(best)

        # ── shape filter: reject non-cylinder blobs ──────────────
        aspect = bh / bw if bw > 0 else 0.0
        vert_fill = bh / rh if rh > 0 else 0.0
        if aspect < self.cyl_min_aspect or vert_fill < self.cyl_min_vert_fill:
            if self._frame_count % 30 == 0:
                self.get_logger().warn(
                    f"cyl: shape reject aspect={aspect:.2f}(need>{self.cyl_min_aspect}) "
                    f"vfill={vert_fill:.2f}(need>{self.cyl_min_vert_fill}) "
                    f"area={area:.0f}")
            self._publish_cylinder(False, 0.0, 0.0, 0.0)
            return

        cylinder_cx = bx + bw / 2.0

        # normalized offset: [-1, 1], positive = right in image
        norm_offset = (cylinder_cx - roi_cx) / (roi_cx) if roi_cx > 0 else 0.0
        norm_offset = max(-1.0, min(1.0, norm_offset))

        # overlap: intersection of cylinder bbox with center band / bbox width
        band_half = self.cyl_band_width / 2.0
        band_left = roi_cx - band_half
        band_right = roi_cx + band_half
        inter_left = max(bx, band_left)
        inter_right = min(bx + bw, band_right)
        overlap = max(0.0, inter_right - inter_left) / bw if bw > 0 else 0.0

        # ── debug: status text ──
        cv2.putText(frame,
                    f"bw={bw} area={area:.0f} offset={norm_offset:+.3f} overlap={overlap:.2f}",
                    (rx + 5, ry + 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 1)

        if self._frame_count % 60 == 0:
            self.get_logger().info(
                f"cyl: OK area={area:.0f} bw={bw} offset={norm_offset:.3f} overlap={overlap:.2f}")
        self._publish_cylinder(True, norm_offset, overlap, float(bw))

    def _publish_cylinder(self, valid: bool, offset: float, overlap: float, width: float) -> None:
        self.valid_pub.publish(Bool(data=valid))
        self.offset_pub.publish(Float32(data=offset))
        self.overlap_pub.publish(Float32(data=overlap))
        self.width_pub.publish(Float32(data=width))

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
