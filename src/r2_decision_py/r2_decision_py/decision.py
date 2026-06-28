"""
R2 决策逻辑 — 全部用 async/await 写的线性流程

对比 C++ 版本:
  - C++: 1600 行, 11+9 个子状态, step 编号 0~16, onTick/handleSubEvent/enterSub 三处切换
  - Python: ~350 行, 每个任务流是一个 async 函数, await = 状态切换

所有函数接收两个参数:
  fsm — async 原语 (nav_to, wait, wait_event, ...)
  act — 硬件指令 (send_grab, start_stair, publish_cmd, ...)
"""

import asyncio
import logging
import math
from dataclasses import dataclass, field
from typing import Optional

from .fsm import FSM, Event

log = logging.getLogger("decision")



# ==========================================================================
# 配置 & 任务数据
# ==========================================================================

@dataclass
class Point:
    id: int
    x: float
    y: float
    z: float = 0.0
    docking_cmd: int = 0


@dataclass
class Zone2Task:
    id: int
    x: float
    y: float
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    use_rotate: bool = False
    rqx: float = 0.0
    rqy: float = 0.0
    rqz: float = 0.0
    rqw: float = 1.0
    approach_x: float = 0.0
    approach_y: float = 0.0
    stair_cmd: int = 0
    grab_is_finsh: int = 0
    arm_command: int = 0
    grab_scene: int = 0
    rotate_x: float = 0.0
    rotate_y: float = 0.0
    grab_adjacent_block: int = -1


@dataclass
class Zone2BlockInfo:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    grab_scene: int = 0


@dataclass
class Config:
    """从 ROS 参数加载的配置."""
    # 红蓝区
    is_red_side: bool = False
    turn_sign: float = 1.0  # 蓝区=+1, 红区=-1. 用于硬编码旋转

    def mirror_for_red_side(self):
        """
        红区镜像: 只反转 Y 坐标.

        坐标系不变, 四元数不变.
        蓝区左转 → 红区右转: 由运行时坐标自然推导, 不需要在这里处理.
        """
        if not self.is_red_side:
            return

        log.info("Applying red-side mirror (Y inversion, turn_sign=-1)")
        self.turn_sign = -1.0

        # ── Zone1 点表 ──
        for pt in self.point_table.values():
            pt.y = -pt.y

        # ── DT35 目标 ──
        self.fine_tune_target_y = -self.fine_tune_target_y

        # ── Zone2 方块坐标 ──
        for b in self.zone2_blocks:
            b.y = -b.y

        # ── Zone2 任务 (只改 Y, 四元数不变) ──
        for t in self.zone2_tasks:
            t.y = -t.y
            t.approach_y = -t.approach_y
            t.rotate_y = -t.rotate_y

        # ── 入口抓取坐标 ──
        self.entry_block0_y = -self.entry_block0_y
        self.entry_block2_y = -self.entry_block2_y
        self.entry_stair1_y = -self.entry_stair1_y

        # ── 出口 ──
        self.mf_exit_y = -self.mf_exit_y

    # Zone1
    zone1_route: list[int] = field(default_factory=lambda: [4, 5])
    point_table: dict[int, Point] = field(default_factory=dict)
    zone1_max_time_s: float = 120.0

    # DT35 微调
    fine_tune_target_x: float = 0.0
    fine_tune_target_y: float = 0.0
    fine_tune_xy_threshold: float = 0.01
    fine_tune_speed_x: float = 0.05
    fine_tune_speed_y: float = 0.05
    fine_tune_stable_required: int = 5
    fine_tune_timeout_s: float = 15.0

    # Zone2
    zone2_blocks: list[Zone2BlockInfo] = field(default_factory=lambda: [Zone2BlockInfo() for _ in range(12)])
    zone2_tasks: list[Zone2Task] = field(default_factory=list)
    use_fixed_route: bool = True
    zone2_fixed_backoff: float = 0.1
    scene_confirm_timeout_s: float = 5.0

    # 入口抓取
    entry_approach_x: float = 1.6
    entry_block0_x: float = 2.0
    entry_block0_y: float = 0.289
    entry_block0_is_finsh: int = 2
    entry_block2_x: float = 3.0
    entry_block2_y: float = 1.41
    entry_block2_is_finsh: int = 1
    entry_stair1_x: float = 1.8
    entry_stair1_y: float = 1.41
    entry_rotate_x: float = 3.0
    entry_rotate_y: float = 1.41

    # 出口
    mf_exit_x: float = 3.2
    mf_exit_y: float = 0.0
    mf_exit_z: float = 0.0


# ==========================================================================
# 运行时状态
# ==========================================================================

@dataclass
class State:
    """运行时状态. 比 C++ 的 Context 小得多."""
    area: int = 0
    current_yaw: float = 0.0
    dt35_x: float = 0.0
    dt35_y: float = 0.0
    lightboard_map: list[int] = field(default_factory=list)
    lightboard_received: bool = False
    zone2_blocks: list[Zone2BlockInfo] = field(default_factory=lambda: [Zone2BlockInfo() for _ in range(12)])


# ==========================================================================
# Zone1 — 矛头抓取 + 撤离
# ==========================================================================

async def zone1(fsm: FSM, act, cfg: Config, state: State):
    """
    一区完整流程.

    新流程 (坐标系已旋转90°):
      走X → 走Y → DT35微调 → 抓矛头 → 转180° → 对接 → 收尾
    不再需要中间转90°.
    """
    state.area = 1
    act.publish_cmd(0, 0, 0, 1)
    log.info("Zone1: start, area=1 sent")

    for idx, point_id in enumerate(cfg.zone1_route):
        pt = cfg.point_table.get(point_id)
        if pt is None:
            log.warning("Zone1: missing point id=%d, skip", point_id)
            continue

        log.info("Zone1: point %d (%.2f, %.2f)", pt.id, pt.x, pt.y)

        # ── 第一段: 走 Y (X 保持当前值) ──
        await fsm.nav_to(fsm._last_nav_x, pt.y, pt.z)

        # ── 第二段: 走 X (Y 已到位) ──
        await fsm.nav_to(pt.x, pt.y, pt.z)

        # ── DT35 微调 ──
        await fsm.fine_tune(
            cfg.fine_tune_target_x, cfg.fine_tune_target_y,
            cfg.fine_tune_xy_threshold,
            cfg.fine_tune_speed_x, cfg.fine_tune_speed_y,
            cfg.fine_tune_stable_required, cfg.fine_tune_timeout_s,
            lambda: (state.dt35_x, state.dt35_y),
        )

        # ── 抓矛头 ──
        act.set_hold_cmd(1)
        result = await fsm.spearhead_and_wait(1)
        if not result.success:
            log.warning("Zone1 grab failed, retry once")
            result = await fsm.spearhead_and_wait(1)
            if not result.success:
                log.warning("Zone1 grab failed after retry, continue")

        # ── 转 180° ──
        state.current_yaw += math.pi
        await fsm.rotate_to(fsm._last_nav_x, fsm._last_nav_y, state.current_yaw)

        # ── 对接 ──
        if pt.docking_cmd:
            act.set_hold_cmd(pt.docking_cmd)
            await fsm.spearhead_and_wait(pt.docking_cmd)

        # ── 等 5 秒 ──
        await fsm.wait(5.0)

        # ── DOCKING_DONE: 发 zhuangtai=4, 等 DONE, 再等 5s, 再发 area 切换 ──
        act.set_hold_cmd(4)
        act.suppress_heartbeat_flag(True)
        await fsm.spearhead_and_wait(4)
        log.info("Zone1: zhuangtai=4 done, wait 5s")
        await fsm.wait(5.0)

        act.set_hold_cmd(0)
        act.publish_cmd_with_area(0, 0, 0)
        await fsm.wait(5.0)

        state.area = 2
        act.publish_cmd(0, 0, 0, 2)
        await fsm.wait(5.0)

        act.enable_lightboard(False)
        log.info("Zone1: entry %d done", pt.id)

    act.suppress_heartbeat_flag(False)
    log.info("Zone1: FINISH → Zone2")


# ==========================================================================
# Zone2 入口抓取
# ==========================================================================

async def entry_grab(fsm: FSM, act, cfg: Config, state: State):
    """
    入口抓取: 抓两个块 + 上三段台阶 + 转向

    C++ 对应: tickEntryGrab 16 步 (约 200 行) → 这里约 60 行
    """

    # ── 块 1: 导航到 approach ──
    log.info("EntryGrab: nav→approach (%.2f, %.2f)", cfg.entry_approach_x, cfg.entry_block0_y)
    await fsm.nav_to(cfg.entry_approach_x, cfg.entry_block0_y)

    # ── 块 1: 抓取 + 导航并行 ──
    log.info("EntryGrab: 块1 grab(is_finsh=%d) + nav 并行", cfg.entry_block0_is_finsh)
    act.start_zone2_grab(cfg.entry_block0_is_finsh)
    nav_r, grab_r = await asyncio.gather(
        fsm.nav_to(cfg.entry_block0_x, cfg.entry_block0_y),
        fsm.wait_event("UP_JUECE_DONE"),
    )
    log.info("EntryGrab: 块1 nav=%s grab=%s", nav_r.success, grab_r.success)

    # ── 块 1: 稳固 + 倒回 ──
    await fsm.wait(10.0)
    log.info("EntryGrab: 块1 倒回")
    await fsm.nav_to(cfg.entry_approach_x, cfg.entry_block0_y)
    act.stop_zone2_grab()
    await fsm.wait(10.0)

    # ── 块 2: 同样流程 ──
    log.info("EntryGrab: nav→块2 approach (%.2f, %.2f)", cfg.entry_stair1_x, cfg.entry_block2_y)
    await fsm.nav_to(cfg.entry_stair1_x, cfg.entry_block2_y)

    log.info("EntryGrab: 块2 grab(is_finsh=%d) + nav 并行", cfg.entry_block2_is_finsh)
    act.start_zone2_grab(cfg.entry_block2_is_finsh)
    nav_r, grab_r = await asyncio.gather(
        fsm.nav_to(cfg.entry_block2_x, cfg.entry_block2_y),
        fsm.wait_event("UP_JUECE_DONE"),
    )
    log.info("EntryGrab: 块2 nav=%s grab=%s", nav_r.success, grab_r.success)

    await fsm.wait(10.0)
    log.info("EntryGrab: 块2 倒回")
    await fsm.nav_to(cfg.entry_stair1_x, cfg.entry_block2_y)
    act.stop_zone2_grab()
    await fsm.wait(10.0)

    # ── 台阶 + 转向 ──
    log.info("EntryGrab: 上台阶 #1")
    await fsm.up_stairs()

    log.info("EntryGrab: nav→转向点 (%.2f, %.2f)", cfg.entry_rotate_x, cfg.entry_rotate_y)
    await fsm.nav_to(cfg.entry_rotate_x, cfg.entry_rotate_y)

    log.info("EntryGrab: 转 90° (蓝区顺, 红区逆)")
    qz = -cfg.turn_sign * 0.707
    await fsm.nav_to(cfg.entry_rotate_x, cfg.entry_rotate_y, 0, 0, 0, qz, 0.707)

    log.info("EntryGrab: 上台阶 #2")
    await fsm.up_stairs()

    log.info("EntryGrab: 转回 90°")
    if cfg.zone2_tasks:
        t = cfg.zone2_tasks[0]
        qz = cfg.turn_sign * 0.707
        await fsm.nav_to(t.approach_x, t.approach_y, 0, 0, 0, qz, 0.707)

    log.info("EntryGrab: 上台阶 #3")
    await fsm.up_stairs()

    log.info("EntryGrab: 完成")


# ==========================================================================
# Zone2 固定路线抓取
# ==========================================================================

async def zone2_grab_fixed(fsm: FSM, act, task: Zone2Task, cfg: Config):
    """
    固定路线中段抓取.

    C++ 对应: tickGrab + handleSubEvent GRAB (~100 行) → 这里 ~20 行
    """
    log.info("Zone2Grab point %d: START is_finsh=%d", task.id, task.grab_is_finsh)

    act.start_zone2_grab(task.grab_is_finsh)
    grab_result = await fsm.wait_event("UP_JUECE_DONE")

    if grab_result.success:
        yaw = math.atan2(2 * (task.qw * task.qz + task.qx * task.qy),
                         1 - 2 * (task.qy * task.qy + task.qz * task.qz))
        bx = task.approach_x - math.cos(yaw) * cfg.zone2_fixed_backoff
        by = task.approach_y - math.sin(yaw) * cfg.zone2_fixed_backoff
        log.info("Zone2Grab point %d: retreat to (%.2f, %.2f)", task.id, bx, by)
        await fsm.nav_to(bx, by, 0, task.qx, task.qy, task.qz, task.qw)

    act.stop_zone2_grab()
    log.info("Zone2Grab point %d: done", task.id)


# ==========================================================================
# Point0 多步序列
# ==========================================================================

async def point0_sequence(fsm: FSM, act, task: Zone2Task):
    """
    Point0 特殊流程.

    C++ 对应: handlePoint0Substep (~80 行) → 这里 ~25 行
    """
    rx = task.rotate_x if task.rotate_x != 0.0 else task.approach_x
    ry = task.rotate_y if task.rotate_y != 0.0 else task.approach_y

    log.info("Point0: 上台阶 #1")
    await fsm.up_stairs()

    log.info("Point0: nav→旋转点 (%.2f, %.2f)", rx, ry)
    await fsm.nav_to(rx, ry)

    log.info("Point0: 顺时针转 90°")
    await fsm.nav_to(rx, ry, 0, task.rqx, task.rqy, task.rqz, task.rqw)

    log.info("Point0: 上台阶 #2")
    await fsm.up_stairs()

    log.info("Point0: 逆时针转 90°")
    await fsm.nav_to(rx, ry, 0, task.qx, task.qy, task.qz, task.qw)

    log.info("Point0: 上台阶 #3")
    await fsm.up_stairs()

    log.info("Point0: 完成")


# ==========================================================================
# Zone2 主流程
# ==========================================================================

async def zone2(fsm: FSM, act, cfg: Config, state: State):
    """
    二区完整流程.

    C++ 对应: Zone2State (~600 行) → 这里 ~120 行
    """
    if not cfg.zone2_tasks:
        log.warning("Zone2: no tasks, done")
        return

    log.info("Zone2: %d tasks", len(cfg.zone2_tasks))

    # ── 入口抓取 ──
    if cfg.use_fixed_route:
        await entry_grab(fsm, act, cfg, state)

    # ── 逐点处理 ──
    for idx, task in enumerate(cfg.zone2_tasks):
        if cfg.use_fixed_route:
            # 导航
            has_approach = (task.approach_x != 0.0 or task.approach_y != 0.0)
            if has_approach:
                await fsm.nav_to(task.approach_x, task.approach_y, task.z,
                                 task.qx, task.qy, task.qz, task.qw)
                await fsm.nav_to(task.x, task.y, task.z,
                                 task.qx, task.qy, task.qz, task.qw)
            else:
                await fsm.nav_to(task.x, task.y, task.z,
                                 task.qx, task.qy, task.qz, task.qw)

            # 抓取
            if task.grab_is_finsh != 0:
                if task.id == 0:
                    await zone2_grab_fixed(fsm, act, task, cfg)
                    await point0_sequence(fsm, act, task)
                else:
                    await zone2_grab_fixed(fsm, act, task, cfg)

            # 台阶
            if task.stair_cmd == 1:
                if task.use_rotate and (task.rotate_x != 0.0 or task.rotate_y != 0.0):
                    await fsm.nav_to(task.rotate_x, task.rotate_y, task.z,
                                     task.qx, task.qy, task.qz, task.qw)
                    await fsm.nav_to(task.rotate_x, task.rotate_y, task.z,
                                     task.rqx, task.rqy, task.rqz, task.rqw)
                await fsm.up_stairs()
            elif task.stair_cmd == 2:
                await fsm.down_stairs()

            # 纯旋转
            if task.stair_cmd == 0 and task.use_rotate:
                rx = task.rotate_x if task.rotate_x != 0.0 else task.x
                ry = task.rotate_y if task.rotate_y != 0.0 else task.y
                await fsm.nav_to(rx, ry, task.z, task.rqx, task.rqy, task.rqz, task.rqw)

        else:
            # ── 动态路线 ──
            await fsm.nav_to(task.x, task.y, task.z, task.qx, task.qy, task.qz, task.qw)

            if task.grab_adjacent_block >= 0:
                adj = task.grab_adjacent_block
                dx = state.zone2_blocks[adj].x - task.x
                dy = state.zone2_blocks[adj].y - task.y
                yaw = math.atan2(dy, dx)
                await fsm.rotate_to(task.x, task.y, yaw)
                result = await fsm.arm_and_wait(task.grab_is_finsh)
                if not result.success:
                    log.warning("Zone2 arm failed, retry once")
                    await fsm.arm_and_wait(task.grab_is_finsh)
            else:
                act.enable_grab_scene(True, task.grab_scene)
                scene_r = await fsm.wait_event("SCENE_READY", timeout=cfg.scene_confirm_timeout_s)
                act.enable_grab_scene(False)
                if scene_r.success:
                    result = await fsm.arm_and_wait(task.arm_command)
                    if not result.success:
                        log.warning("Zone2 arm failed, retry once")
                        await fsm.arm_and_wait(task.arm_command)
                else:
                    log.warning("Zone2: scene timeout, skip block %d", task.id)
                    continue

            if task.stair_cmd == 1:
                await fsm.up_stairs()
            elif task.stair_cmd == 2:
                await fsm.down_stairs()

    act.enable_grab_scene(False)
    log.info("Zone2: FINISH")


# ==========================================================================
# 完整任务
# ==========================================================================

async def run_mission(fsm: FSM, act, cfg: Config, state: State):
    """
    完整比赛流程.

    C++ 对应: BootState→WaitStart→Zone1→Zone2→Exit→Done (~100 行) → 这里 ~20 行
    """
    act.enable_spear(False)
    act.enable_lightboard(False)
    log.info("Waiting START button...")
    await fsm.wait_event("START_PRESSED")

    await zone1(fsm, act, cfg, state)
    await zone2(fsm, act, cfg, state)

    if not cfg.use_fixed_route:
        log.info("Exit: nav to (%.2f, %.2f)", cfg.mf_exit_x, cfg.mf_exit_y)
        await fsm.nav_to(cfg.mf_exit_x, cfg.mf_exit_y, cfg.mf_exit_z)

    act.enable_spear(False)
    act.enable_lightboard(False)
    act.enable_grab_scene(False)
    log.info("=== MISSION COMPLETE ===")
