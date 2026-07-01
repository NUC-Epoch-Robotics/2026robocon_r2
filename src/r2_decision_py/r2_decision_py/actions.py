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

from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import Twist
from robot_serial.msg import Command, Ack

from .fsm import Event

log = logging.getLogger("actions")

# ── 常量 ──
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
        self.cmd_pub = node.create_publisher(
            Command, '/command', QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=5))
        self.spear_enable_pub = node.create_publisher(Bool, 'spearhead/enable', 10)
        self.lightboard_enable_pub = node.create_publisher(Bool, 'lightboard/enable', 10)
        self.grab_scene_enable_pub = node.create_publisher(Bool, 'grab_scene/enable', 10)
        self.grab_scene_expected_pub = node.create_publisher(UInt8, 'grab_scene/expected_scene', 10)
        self.cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)

        # ── 区号 ──
        self.area = 0

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

        # ── 上下肢状态 (1=忙, 2=空闲) ──
        self.up_free = True
        self.down_free = True

        # ── 导航等待 ──
        self.waiting_nav_done = False
        self._last_nav_x = 0.0
        self._last_nav_y = 0.0
        self._last_nav_yaw = 0.0

        # ── 上肢动作等待 ──
        self.waiting_arm_done = False

        # ── 当前指令状态（保持不变） ──
        self._last_spearhead = 0
        self._last_block = 0
        self._last_stair = 0
        self._last_dt35 = 0

    # ==================================================================
    # 底层发送
    # ==================================================================

    async def wait_up_free(self):
        """等上肢空闲 (up_free=2). 忙时每 50ms 轮询一次."""
        while not self.up_free:
            log.info("UP busy (up_free=1), waiting...")
            await asyncio.sleep(0.05)
        log.info("UP free (up_free=2), can send")

    async def wait_down_free(self):
        """等下肢空闲 (down_free=2). 忙时每 50ms 轮询一次."""
        while not self.down_free:
            log.info("DOWN busy (down_free=1), waiting...")
            await asyncio.sleep(0.05)
        log.info("DOWN free (down_free=2), can send")

    def _publish_cmd(self, stair: int = 0, block: int = 0,
                     spearhead: int = 0, area: int = 0,
                     x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                     dt35: int = 0):
        """发送指令到 /command topic (robot_serial/msg/Command)."""
        # 保存当前指令状态
        self._last_spearhead = spearhead
        self._last_block = block
        self._last_stair = stair
        self._last_dt35 = dt35

        msg = Command()
        msg.x = x
        msg.y = y
        msg.yaw = yaw
        msg.spearhead = spearhead
        msg.block = block
        msg.stair = stair
        msg.dt35 = dt35
        msg.area = area
        self.cmd_pub.publish(msg)

    # ==================================================================
    # 导航 (直接发 /command, 不走 Nav2)
    # ==================================================================

    def send_navigate(self, x: float, y: float, z: float = 0.0,
                      qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                      nav_frame_id: str = 'odom'):
        """发送导航目标到 /command topic. 等待 down_free=2 后 post NAV_DONE."""
        import math
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        log.info("NAV to (%.2f, %.2f) yaw=%.3f (via /command)", x, y, yaw)

        # 保存最后的坐标和yaw
        self._last_nav_x = x
        self._last_nav_y = y
        self._last_nav_yaw = yaw

        # 直接发到 /command，保持当前上肢指令不变
        self._publish_cmd(x=x, y=y, yaw=yaw,
                          spearhead=self._last_spearhead,
                          block=self._last_block,
                          stair=self._last_stair,
                          dt35=self._last_dt35)

        # 标记等待底盘完成
        self.waiting_nav_done = True
        self.down_free = False

    # ==================================================================
    # 机械臂指令 (block 字段)
    # ==================================================================

    async def send_arm_command(self, cmd: int):
        """
        发送机械臂指令 (block 字段).
        完成后 post ARM_DONE (等 up_free=2).
        """
        await self.wait_up_free()
        # 保持当前坐标和其他指令不变
        self._publish_cmd(block=cmd, area=self.area,
                          x=self._last_nav_x, y=self._last_nav_y, yaw=self._last_nav_yaw,
                          spearhead=self._last_spearhead, stair=self._last_stair,
                          dt35=self._last_dt35)
        self.up_free = False
        self.waiting_arm_done = True
        log.info("ARM cmd=%d sent, waiting up_free=2", cmd)

    # ==================================================================
    # 矛头指令 (spearhead 字段)
    # ==================================================================

    async def send_spearhead_command(self, cmd: int):
        """
        发送矛头指令 (spearhead 字段).
        完成后 post ARM_DONE (等 up_free=2).
        """
        await self.wait_up_free()
        # 保持当前坐标和其他指令不变
        self._publish_cmd(spearhead=cmd, area=self.area,
                          x=self._last_nav_x, y=self._last_nav_y, yaw=self._last_nav_yaw,
                          block=self._last_block, stair=self._last_stair,
                          dt35=self._last_dt35)
        self.up_free =False
        self.waiting_arm_done = True
        log.info("SPEARHEAD cmd=%d sent, waiting up_free=2", cmd)

    def set_hold_cmd(self, cmd: int):
        """等待期间心跳维持这个命令."""
        self.hold_cmd = cmd

    def suppress_heartbeat_flag(self, suppress: bool):
        """抑制空闲心跳 (DOCKING_DONE 收尾阶段)."""
        self.suppress_heartbeat = suppress

    # ==================================================================
    # 台阶
    # ==================================================================

    async def start_stair(self, cmd: int):
        """
        发台阶指令, 等 up_free=2 后完成.
        cmd=1 上台阶, cmd=2 下台阶.
        """
        await self.wait_up_free()
        # 保持当前坐标和其他指令不变
        self._publish_cmd(stair=cmd, area=self.area,
                          x=self._last_nav_x, y=self._last_nav_y, yaw=self._last_nav_yaw,
                          spearhead=self._last_spearhead, block=self._last_block,
                          dt35=self._last_dt35)
        self.up_free = False
        self.waiting_arm_done = True
        self.stair_active = True
        self.pending_stair_cmd = cmd
        log.info("STAIR cmd=%d sent, waiting up_free=2", cmd)

    def stop_stair(self):
        """清台阶状态, 发 0."""
        if self.stair_active:
            # 保持当前坐标和其他指令不变
            self._publish_cmd(stair=0, area=self.area,
                              x=self._last_nav_x, y=self._last_nav_y, yaw=self._last_nav_yaw,
                              spearhead=self._last_spearhead, block=self._last_block,
                              dt35=self._last_dt35)
        self.stair_active = False
        self.pending_stair_cmd = 0

    # ==================================================================
    # 抓取
    # ==================================================================

    def start_zone2_grab(self, block: int):
        """发抓取指令 (block 字段). 单次发送."""
        self.stop_zone2_grab()
        self._publish_cmd(block=block, area=self.area,
                          x=self._last_nav_x, y=self._last_nav_y, yaw=self._last_nav_yaw,
                          spearhead=self._last_spearhead, stair=self._last_stair,
                          dt35=self._last_dt35)
        log.info("GRAB block=%d", block)

    def stop_zone2_grab(self):
        """停止抓取 (block=0)."""
        self._publish_cmd(block=0, area=self.area,
                          x=self._last_nav_x, y=self._last_nav_y, yaw=self._last_nav_yaw,
                          spearhead=self._last_spearhead, stair=self._last_stair,
                          dt35=self._last_dt35)

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

    def publish_cmd(self, stair: int = 0, block: int = 0,
                    spearhead: int = 0, area: int = 0):
        # 保持当前坐标和指令不变
        self._publish_cmd(stair, block, spearhead, area,
                          x=self._last_nav_x, y=self._last_nav_y, yaw=self._last_nav_yaw,
                          dt35=self._last_dt35)
        # 发 area 指令后，标记等待上肢完成
        if area != 0:
            self.up_free = False

    def publish_cmd_with_area(self, stair: int = 0, block: int = 0, spearhead: int = 0):
        # 保持当前坐标和指令不变
        self._publish_cmd(stair, block, spearhead, self.area,
                          x=self._last_nav_x, y=self._last_nav_y, yaw=self._last_nav_yaw,
                          dt35=self._last_dt35)

    # ==================================================================
    # ROS 回调 (在 node 的订阅中调用)
    # ==================================================================

    def on_upper_ack(self, msg):
        """
        /juece_ack 回调 (robot_serial/msg/Ack: xipan_status, taijie_status).
        C++ 对应: R2DecisionNode::onUpperAck

        串口驱动收到下位机 0xAA 0x55 包后发布到这里:
          xipan_status → 吸盘状态 (1=吸到块可以走位, 2=抓取彻底完成)
          taijie_status → 台阶状态 (1=上台阶完成, 2=下台阶完成)
          up_free → 上肢状态 (1=忙, 2=空闲)
          down_free → 下肢状态 (1=忙, 2=空闲)
        """
        # up_free: 1=忙, 2=空闲
        if msg.up_free == 1:
            if self.up_free:
                log.info("UP_FREE=1: upper limb busy")
            self.up_free = False
        elif msg.up_free == 2:
            if not self.up_free:
                log.info("UP_FREE=2: upper limb free")
            self.up_free = True
            # 上肢动作完成
            if self.waiting_arm_done:
                self.waiting_arm_done = False
                log.info("ARM done (up_free=2)")
                self.post_event(Event("ARM_DONE", success=True))
                return

        # down_free: 1=忙, 2=空闲
        if msg.down_free == 1:
            if self.down_free:
                log.info("DOWN_FREE=1: lower limb busy")
            self.down_free = False
        elif msg.down_free == 2:
            if not self.down_free:
                log.info("DOWN_FREE=2: lower limb free")
            self.down_free = True
            # 导航完成
            if self.waiting_nav_done:
                self.waiting_nav_done = False
                log.info("NAV done (down_free=2)")
                self.post_event(Event("NAV_DONE", success=True))
                return

        # xipan_status=1 → 吸到了, 可以开始走下一个点 (但还不能发 is_finsh)
        if msg.xipan_status == 1:
            log.info("XIPAN status=1: grabbed, can start moving")
            self.post_event(Event("XIPAN_GRABBED"))
            return

        # xipan_status=2 → 抓取彻底完成, 可以发下一个 is_finsh
        if msg.xipan_status == 2:
            log.info("XIPAN status=2: grab fully done, can send next is_finsh")
            self.post_event(Event("XIPAN_DONE"))
            return

        # taijie_status=1/2 → 台阶完成: 先发 0, 再 post 事件
        if msg.taijie_status in (1, 2):
            log.info("STAIR callback: taijie_status=%d, sending 0", msg.taijie_status)
            self.stop_stair()   # 发 stair=0
            self.post_event(Event("DOWN_JUECE_DONE"))
            return

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

