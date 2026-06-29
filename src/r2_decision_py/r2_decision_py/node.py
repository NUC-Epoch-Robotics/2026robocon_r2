"""
ROS2 节点 — 把 Python 决策逻辑接入 ROS2

职责:
  1. 创建 ActionDispatcher (硬件 I/O + 可靠性)
  2. 创建 FSM (async 原语)
  3. 订阅 ROS2 topic, 转发给 ActionDispatcher 回调
  4. 运行 asyncio 事件循环, 驱动决策协程
"""

import asyncio
import threading
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(message)s')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, UInt8, UInt8MultiArray
from robot_serial.msg import Juece, Ack, Location

from .fsm import FSM, Event
from .actions import ActionDispatcher
from .decision import Config, State, Zone2BlockInfo, Point, run_mission

log = logging.getLogger("node")


class R2DecisionNode(Node):
    """
    Python 版决策节点.

    对应 C++ R2DecisionNode, 但决策逻辑全在 async 协程里.
    """

    def __init__(self):
        super().__init__('r2_decision_py')

        # ── 核心组件 ──
        self.act = ActionDispatcher(self)
        self.fsm = FSM()
        self.fsm.act = self.act
        self.state = State()
        self.config = Config()

        # ── ActionDispatcher 事件回调 → FSM ──
        self.act.post_event = self.fsm.post_event

        # ── 加载参数 ──
        self._load_params()

        # ── 订阅 (和 C++ 完全一致的消息类型) ──
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=5)

        self.create_subscription(Ack, '/juece_ack',
                                 self.act.on_upper_ack, qos)
        self.create_subscription(Juece, '/juece_done',
                                 self.act.on_upper_done, qos)
        self.create_subscription(Bool, 'spearhead/exists',
                                 self.act.on_spear_exists, 10)
        self.create_subscription(UInt8MultiArray, 'lightboard/map',
                                 self._on_lightboard_map, 10)
        self.create_subscription(Bool, 'grab_scene/ready',
                                 self.act.on_grab_scene_ready, 10)
        self.create_subscription(UInt8, 'r2/control/button_state',
                                 self.act.on_button_state, 10)
        self.create_subscription(Location, '/dt35/location',
                                 self._on_dt35_location, qos)

        # ── asyncio 事件循环 ──
        self._loop = asyncio.new_event_loop()
        self.fsm.set_loop(self._loop)
        self._loop_thread: threading.Thread = None

        self.get_logger().info('R2 Decision Python Node Started')

    def _load_params(self):
        """从 ROS 参数加载配置."""
        cfg = self.config

        # 红蓝区
        cfg.is_red_side = self.declare_parameter('is_red_side', False).value

        # Zone1
        cfg.zone1_route = list(self.declare_parameter('zone1_route', [4, 5]).value)
        cfg.zone1_max_time_s = self.declare_parameter('zone1_max_time_s', 120.0).value

        # Zone1 point table
        for pid in [5, 7, 8]:
            px = self.declare_parameter(f'zone1_point_{pid}_x', 0.0).value
            py = self.declare_parameter(f'zone1_point_{pid}_y', 0.0).value
            pz = self.declare_parameter(f'zone1_point_{pid}_z', 0.0).value
            cfg.point_table[pid] = Point(id=pid, x=px, y=py, z=pz)
        cfg.point_table[5].docking_cmd = 2

        # DT35
        cfg.fine_tune_target_x = self.declare_parameter('fine_tune_target_x', 0.0).value
        cfg.fine_tune_target_y = self.declare_parameter('fine_tune_target_y', 0.0).value
        cfg.fine_tune_xy_threshold = self.declare_parameter('fine_tune_xy_threshold', 0.01).value
        cfg.fine_tune_speed_x = self.declare_parameter('fine_tune_speed_x', 0.05).value
        cfg.fine_tune_speed_y = self.declare_parameter('fine_tune_speed_y', 0.05).value
        cfg.fine_tune_stable_required = self.declare_parameter('fine_tune_stable_required', 5).value
        cfg.fine_tune_timeout_s = self.declare_parameter('fine_tune_timeout_s', 15.0).value

        # Zone2
        cfg.use_fixed_route = self.declare_parameter('use_fixed_route', True).value
        cfg.zone2_fixed_backoff = self.declare_parameter('zone2_fixed_backoff', 0.1).value
        cfg.scene_confirm_timeout_s = self.declare_parameter('scene_confirm_timeout_s', 5.0).value

        # 入口抓取 — 默认值已在 GrabPoint 数据类中定义, 红区覆盖
        cfg.grab_qz = self.declare_parameter('grab_qz', 0.707).value
        cfg.grab_qw = self.declare_parameter('grab_qw', 0.707).value
        cfg.grab_forward_speed = self.declare_parameter('grab_forward_speed', 0.1).value

        # 红区: 不同的 grab_points
        if cfg.is_red_side:
            from .decision import GrabPoint
            cfg.grab_points = [
                GrabPoint(approach_x=-0.8, approach_y=-2.5,
                          dt35_x=0.400, dt35_y=3.924, is_finsh=2),
                GrabPoint(approach_x=-2.0, approach_y=-2.5,
                          dt35_x=-0.800, dt35_y=3.924, is_finsh=1),
                GrabPoint(approach_x=-3.2, approach_y=-2.5,
                          dt35_x=-2.000, dt35_y=3.924, is_finsh=2),
            ]
            cfg.stairs_start_x = -2.0
            cfg.stairs_start_y = -2.5

        # 台阶起始点 (蓝区默认值, 红区已覆盖)
        cfg.stairs_start_x = self.declare_parameter('stairs_start_x', cfg.stairs_start_x).value
        cfg.stairs_start_y = self.declare_parameter('stairs_start_y', cfg.stairs_start_y).value

        # 出口
        cfg.mf_exit_x = self.declare_parameter('mf_exit_x', 3.2).value
        cfg.mf_exit_y = self.declare_parameter('mf_exit_y', 0.0).value

        # ── 红区镜像 (一次性转换) ──
        cfg.mirror_for_red_side()

    def start(self):
        """启动决策协程."""
        log.info("Starting FSM coroutine thread...")
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        log.info("Asyncio loop started, running mission...")
        self._loop.run_until_complete(
            self.fsm.run(run_mission(self.fsm, self.act, self.config, self.state))
        )
        log.info("Mission coroutine finished")

    # ── 特殊回调 (需要更新 state 的) ──

    def _on_lightboard_map(self, msg):
        self.state.lightboard_map = list(msg.data)
        self.state.lightboard_received = True

    def _on_dt35_location(self, msg):
        self.state.dt35_x = float(msg.x)
        self.state.dt35_y = float(msg.y)


def main(args=None):
    rclpy.init(args=args)
    node = R2DecisionNode()
    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
