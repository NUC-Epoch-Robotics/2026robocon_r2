"""
ActionDispatcher — 所有 ROS I/O + 可靠性逻辑

C++ 对应:
  - r2_decision_actions.cpp (476 行) — 指令发送、ACK/DONE 重发、心跳
  - r2_decision_callbacks.cpp (97 行) — ROS 回调 → 事件路由

Python: ~250 行

可靠性机制 (和 C++ 一样):
  1. 指令发出后等 ACK, ACK 超时 → 快速重发 (每 100ms)
  2. 收到 ACK 后等 DONE, DONE 超时 → 重发 (每 3s), 最多 3 次
  3. 重试耗尽 → 当作失败, post ARM_DONE(success=False) 让 FSM 跳过
  4. 心跳: 空闲时每 500ms 重发当前指令 (hold_cmd), 防止下位机丢失状态
"""

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.action import ActionClient

from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from robot_serial.msg import Juece, Ack, Location

from .fsm import Event

log = logging.getLogger("actions")

# ── 常量 (和 C++ 一样) ──
UPPER_CMD_RESEND_MS = 100       # ACK 阶段: 每 100ms 重发
UPPER_CMD_TIMEOUT_MS = 1200     # ACK 总超时
SPEARHEAD_DONE_TIMEOUT_MS = 3000  # DONE 阶段: 3s 超时
SPEARHEAD_MAX_RETRY = 3         # 最多重发 3 次
IDLE_HEARTBEAT_MS = 500         # 空闲心跳周期
BUTTON_DEBOUNCE_MS = 120        # 按钮消抖


# ==========================================================================
# ActionDispatcher
# ==========================================================================

class ActionDispatcher:
    """
    硬件指令发送 + 可靠性保障.

    决策层调用: send_spearhead(1), send_grab(2), start_stair(1), ...
    这个类负责: ACK 重发、DONE 超时、心跳维持.
    收到 DONE/ACK 后通过 post_event 通知 FSM.
    """

    def __init__(self, node: Node):
        self.node = node
        self.post_event: Callable[[Event], None] = lambda e: None

        # ── publishers (和 C++ 完全一致的消息类型) ──
        self.upper_cmd_pub = node.create_publisher(
            Juece, '/juece', QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=5))
        self.spear_enable_pub = node.create_publisher(Bool, 'spearhead/enable', 10)
        self.lightboard_enable_pub = node.create_publisher(Bool, 'lightboard/enable', 10)
        self.grab_scene_enable_pub = node.create_publisher(Bool, 'grab_scene/enable', 10)
        self.grab_scene_expected_pub = node.create_publisher(UInt8, 'grab_scene/expected_scene', 10)
        self.cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)

        # ── Nav2 action client ──
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # ── 区号 ──
        self.area = 0

        # ── arm 指令状态 ──
        self.pending_upper_cmd = 0
        self.waiting_upper_ack = False
        self.upper_cmd_start_time = 0.0
        self.last_upper_send_time = 0.0

        # ── spearhead 指令状态 ──
        self.pending_spearhead_cmd = 0
        self.spearhead_active = False
        self.waiting_spearhead_ack = False
        self.spearhead_acked = False
        self.spearhead_retry_count = 0
        self.spearhead_cmd_start_time = 0.0
        self.last_spearhead_send_time = 0.0
        self.spearhead_done_pending = False

        # ── hold / heartbeat ──
        self.hold_cmd = 0
        self.suppress_heartbeat = False
        self.last_idle_heartbeat_time = 0.0

        # ── 台阶 ──
        self.stair_active = False
        self.pending_stair_cmd = 0

        # ── 相机状态 ──
        self.spear_camera_enabled = False
        self.lightboard_enabled = False
        self.grab_scene_enabled = False

        # ── 按钮消抖 ──
        self.last_button_state = 0
        self.last_button_time = 0.0

        # ── 定时 tick (20ms) ──
        self._tick_timer = node.create_timer(0.020, self._tick)

    # ==================================================================
    # 底层发送
    # ==================================================================

    def _publish_juece(self, status_bit: int, is_finsh: int = 0,
                       zhuangtai: int = 0, area: int = 0):
        """发送底层指令到 /juece topic (robot_serial/msg/Juece)."""
        msg = Juece()
        msg.status_bit = status_bit
        msg.is_finsh = is_finsh
        msg.zhuangtai = zhuangtai
        msg.area = area
        self.upper_cmd_pub.publish(msg)

    def _publish_with_area(self, status_bit: int, is_finsh: int = 0, zhuangtai: int = 0):
        """发送指令 (自动带当前区号)."""
        self._publish_juece(status_bit, is_finsh, zhuangtai, self.area)

    # ==================================================================
    # 导航
    # ==================================================================

    def send_navigate(self, x: float, y: float, z: float = 0.0,
                      qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                      nav_frame_id: str = 'odom'):
        """发送 Nav2 导航目标. 完成后 post NAV_DONE."""
        log.info("NAV to (%.2f, %.2f, %.2f) q=(%.3f, %.3f, %.3f, %.3f)",
                 x, y, z, qx, qy, qz, qw)

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            log.warning("Nav2 action server not available")
            self.post_event(Event("NAV_DONE", success=False))
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = nav_frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = float(z)
        goal.pose.pose.orientation.x = float(qx)
        goal.pose.pose.orientation.y = float(qy)
        goal.pose.pose.orientation.z = float(qz)
        goal.pose.pose.orientation.w = float(qw)

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            log.warning("Nav goal REJECTED")
            self.post_event(Event("NAV_DONE", success=False))
            return
        log.info("Nav goal ACCEPTED")
        goal_handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        result = future.result()
        ok = (result.status == 4)  # SUCCEEDED
        log.info("NAV done: %s", "OK" if ok else "FAIL")
        self.post_event(Event("NAV_DONE", success=ok))

    # ==================================================================
    # 机械臂指令 (is_finsh 字段)
    # ==================================================================

    def send_arm_command(self, cmd: int):
        """
        发送机械臂指令 (is_finsh 字段).
        完成后 post ARM_DONE.
        """
        if cmd != 0:
            self._publish_with_area(0)  # 先发一条空指令

        self.pending_upper_cmd = cmd
        self.waiting_upper_ack = True
        self.hold_cmd = cmd  # ACK→DONE 期间心跳维持此指令

        now = time.monotonic()
        self.upper_cmd_start_time = now
        self.last_upper_send_time = now

        self._publish_with_area(cmd)
        log.info("ARM cmd=%d (waiting ACK...)", cmd)

    # ==================================================================
    # 矛头指令 (zhuangtai 字段)
    # ==================================================================

    def send_spearhead_command(self, cmd: int):
        """
        发送矛头指令 (zhuangtai 字段).
        完成后 post ARM_DONE.
        带 ACK 重发 + DONE 超时重发.
        """
        # 清理旧状态
        self.spearhead_active = False
        self.waiting_spearhead_ack = False

        self.pending_spearhead_cmd = cmd
        self.waiting_spearhead_ack = True
        self.spearhead_active = True
        self.spearhead_acked = False
        self.spearhead_retry_count = 1

        now = time.monotonic()
        self.spearhead_cmd_start_time = now
        self.last_spearhead_send_time = now

        self._publish_with_area(0, 0, cmd)
        log.info("SPEARHEAD cmd zhuangtai=%d (waiting ACK...)", cmd)

    def set_hold_cmd(self, cmd: int):
        """等待期间心跳维持这个命令."""
        self.hold_cmd = cmd

    def suppress_heartbeat_flag(self, suppress: bool):
        """抑制空闲心跳 (DOCKING_DONE 收尾阶段)."""
        self.suppress_heartbeat = suppress

    # ==================================================================
    # 台阶
    # ==================================================================

    def start_stair(self, cmd: int):
        """
        发台阶指令, 等下位机回调后才发 0.
        cmd=1 上台阶, cmd=2 下台阶.
        """
        self.stop_stair()
        self.stair_active = True
        self.pending_stair_cmd = cmd
        self._publish_with_area(cmd)
        log.info("STAIR cmd=%d (waiting callback)", cmd)

    def stop_stair(self):
        """清台阶状态, 发 0."""
        if self.stair_active:
            self._publish_with_area(0)
        self.stair_active = False
        self.pending_stair_cmd = 0

    # ==================================================================
    # 抓取
    # ==================================================================

    def start_zone2_grab(self, is_finsh: int):
        """发抓取指令 (is_finsh 字段). 单次发送."""
        self.stop_zone2_grab()
        self._publish_with_area(0, is_finsh)
        log.info("GRAB is_finsh=%d", is_finsh)

    def stop_zone2_grab(self):
        """停止抓取 (is_finsh=0)."""
        self._publish_with_area(0, 0)

    # ==================================================================
    # 传感器开关
    # ==================================================================

    def enable_spear(self, enable: bool):
        if self.spear_camera_enabled == enable:
            return
        msg = Bool()
        msg.data = enable
        self.spear_enable_pub.publish(msg)
        self.spear_camera_enabled = enable
        log.info("spearhead camera %s", "ON" if enable else "OFF")

    def enable_lightboard(self, enable: bool):
        if self.lightboard_enabled == enable:
            return
        msg = Bool()
        msg.data = enable
        self.lightboard_enable_pub.publish(msg)
        self.lightboard_enabled = enable
        log.info("lightboard camera %s", "ON" if enable else "OFF")

    def enable_grab_scene(self, enable: bool, expected_scene: int = 0):
        if self.grab_scene_enabled == enable:
            return
        msg = Bool()
        msg.data = enable
        self.grab_scene_enable_pub.publish(msg)
        self.grab_scene_enabled = enable
        if enable:
            smsg = UInt8()
            smsg.data = expected_scene
            self.grab_scene_expected_pub.publish(smsg)
        log.info("grab_scene %s (scene=%d)", "ON" if enable else "OFF", expected_scene)

    # ==================================================================
    # cmd_vel
    # ==================================================================

    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def stop_cmd_vel(self):
        self.cmd_vel_pub.publish(Twist())

    # ==================================================================
    # 底层 publish (供 decision.py 直接调用)
    # ==================================================================

    def publish_cmd(self, status_bit: int, is_finsh: int = 0,
                    zhuangtai: int = 0, area: int = 0):
        self._publish_juece(status_bit, is_finsh, zhuangtai, area)

    def publish_cmd_with_area(self, status_bit: int, is_finsh: int = 0, zhuangtai: int = 0):
        self._publish_with_area(status_bit, is_finsh, zhuangtai)

    # ==================================================================
    # ROS 回调 (在 node 的订阅中调用)
    # ==================================================================

    def on_upper_ack(self, msg):
        """
        /juece_ack 回调 (robot_serial/msg/Ack: xipan_status, taijie_status).
        C++ 对应: R2DecisionNode::onUpperAck

        串口驱动收到下位机 0xAA 0x55 包后发布到这里:
          xipan_status → 吸盘状态 (1=吸到块)
          taijie_status → 台阶状态 (1=上台阶完成, 2=下台阶完成)
        """
        # xipan_status=1 → 吸到块
        if msg.xipan_status == 1:
            self.post_event(Event("UP_JUECE_DONE"))
            return

        # taijie_status=1/2 → 台阶完成: 先发 0, 再 post 事件
        if msg.taijie_status in (1, 2):
            log.info("STAIR callback: taijie_status=%d, sending 0", msg.taijie_status)
            self.stop_stair()   # 发 status_bit=0
            self.post_event(Event("DOWN_JUECE_DONE"))
            return

    def on_upper_done(self, msg):
        """
        /juece_done 回调 (robot_serial/msg/Juece).
        C++ 对应: R2DecisionNode::onUpperDone
        """
        if msg.zhuangtai != 1:
            return

        # 台阶完成 → DOWN_JUECE_DONE (最高优先级)
        if self.stair_active:
            self.stop_stair()
            self.post_event(Event("DOWN_JUECE_DONE"))
            return

        # spearhead 指令完成
        if self.spearhead_active:
            if self._handle_spearhead_done(msg.status_bit, msg.is_finsh != 0):
                self.post_event(Event("ARM_DONE", success=(msg.is_finsh != 0)))
            return

        # 普通机械臂指令完成
        self._handle_arm_done(msg.status_bit, msg.is_finsh != 0)
        self.post_event(Event("ARM_DONE", success=(msg.is_finsh != 0)))

    def on_spear_exists(self, msg):
        pass  # 更新状态用, 不产生事件

    def on_lightboard_map(self, msg):
        pass  # 在 node.py 中直接更新 state

    def on_grab_scene_ready(self, msg):
        """抓取场景确认."""
        if msg.data:
            log.info("Scene CONFIRMED!")
            self.post_event(Event("SCENE_READY"))

    def on_button_state(self, msg):
        """按钮状态 (带消抖)."""
        now_ms = time.monotonic() * 1000
        val = msg.data

        if val == self.last_button_state and (now_ms - self.last_button_time) < BUTTON_DEBOUNCE_MS:
            return

        self.last_button_state = val
        self.last_button_time = now_ms

        if val == 1:
            log.info("Button: START pressed")
            self.post_event(Event("START_PRESSED"))
        elif val == 2:
            log.info("Button: ZONE1_RETRY")
            self.post_event(Event("ZONE1_RETRY"))
        elif val == 3:
            log.info("Button: ZONE2_RETRY")
            self.post_event(Event("ZONE2_RETRY"))
        elif val == 4:
            log.info("Button: ZONE3_RETRY")
            self.post_event(Event("ZONE3_RETRY"))

    def on_dt35_location(self, msg):
        """DT35 位置更新. 在 node.py 中直接更新 state."""
        pass

    # ==================================================================
    # 可靠性: ACK/DONE 处理
    # ==================================================================

    def _handle_arm_done(self, command: int, success: bool):
        """普通机械臂指令 DONE."""
        self.waiting_upper_ack = False
        self.hold_cmd = 0
        log.info("ARM DONE: cmd=%d success=%d", command, success)

    def _handle_spearhead_done(self, command: int, success: bool) -> bool:
        """
        矛头指令 DONE. 严格校验 cmd 匹配.
        返回 True 表示接受了这个 DONE.
        """
        if not self.spearhead_active or command != self.pending_spearhead_cmd:
            log.debug("Drop stale SPEARHEAD DONE: cmd=%d (pending=%d)",
                      command, self.pending_spearhead_cmd)
            return False

        self.waiting_spearhead_ack = False
        self.spearhead_active = False
        self.pending_spearhead_cmd = 0
        self.spearhead_acked = False
        self.spearhead_retry_count = 0
        self.spearhead_done_pending = True

        log.info("SPEARHEAD DONE: cmd=%d success=%d", command, success)
        return True

    # ==================================================================
    # 定时 tick (每 20ms 调用)
    # ==================================================================

    def _tick(self):
        """
        可靠性定时器. 对应 C++ ActionDispatcher::tickReliability.

        逻辑:
          1. spearhead 等 ACK → 每 100ms 重发, 3 次后放弃
          2. spearhead 等 DONE → 3s 超时重发, 3 次后放弃
          3. arm 等 ACK → 每 100ms 重发
        """
        now = time.monotonic()

        # ── spearhead 可靠性 ──
        if self.spearhead_active and self.pending_spearhead_cmd != 0:
            # 阶段 1: 等 ACK
            if self.waiting_spearhead_ack:
                if (now - self.last_spearhead_send_time) * 1000 >= UPPER_CMD_RESEND_MS:
                    if self.spearhead_retry_count >= SPEARHEAD_MAX_RETRY:
                        # 重试耗尽
                        log.warning("SPEARHEAD cmd %d no ACK after %d attempts, give up",
                                    self.pending_spearhead_cmd, SPEARHEAD_MAX_RETRY)
                        giveup_cmd = self.pending_spearhead_cmd
                        self.spearhead_active = False
                        self.waiting_spearhead_ack = False
                        self.pending_spearhead_cmd = 0
                        self.spearhead_acked = False
                        self.spearhead_retry_count = 0
                        self.spearhead_done_pending = True
                        self.post_event(Event("ARM_DONE", success=False))
                        return

                    self.spearhead_retry_count += 1
                    self._publish_with_area(0, 0, self.pending_spearhead_cmd)
                    self.last_spearhead_send_time = now
                    self.spearhead_cmd_start_time = now
                return

            # 阶段 2: 收到 ACK, 等 DONE
            if (self.spearhead_acked and
                    (now - self.spearhead_cmd_start_time) * 1000 >= SPEARHEAD_DONE_TIMEOUT_MS):
                if self.spearhead_retry_count < SPEARHEAD_MAX_RETRY:
                    self.spearhead_retry_count += 1
                    log.warning("SPEARHEAD cmd %d DONE timeout, resend (%d/%d)",
                                self.pending_spearhead_cmd, self.spearhead_retry_count, SPEARHEAD_MAX_RETRY)
                    self._publish_with_area(0, 0, self.pending_spearhead_cmd)
                    self.waiting_spearhead_ack = True
                    self.spearhead_acked = False
                    self.last_spearhead_send_time = now
                    self.spearhead_cmd_start_time = now
                else:
                    log.warning("SPEARHEAD cmd %d DONE timeout after %d retries, give up",
                                self.pending_spearhead_cmd, SPEARHEAD_MAX_RETRY)
                    giveup_cmd = self.pending_spearhead_cmd
                    self.spearhead_active = False
                    self.waiting_spearhead_ack = False
                    self.pending_spearhead_cmd = 0
                    self.spearhead_acked = False
                    self.spearhead_retry_count = 0
                    self.spearhead_done_pending = True
                    self.post_event(Event("ARM_DONE", success=False))
            return

        # ── arm 可靠性 ──
        if self.waiting_upper_ack:
            if (now - self.last_upper_send_time) * 1000 >= UPPER_CMD_RESEND_MS:
                self._publish_with_area(self.pending_upper_cmd)
                self.last_upper_send_time = now
            if (now - self.upper_cmd_start_time) * 1000 >= UPPER_CMD_TIMEOUT_MS:
                log.warning("ARM cmd %d ACK timeout, resending...", self.pending_upper_cmd)
                self.upper_cmd_start_time = now
            return
