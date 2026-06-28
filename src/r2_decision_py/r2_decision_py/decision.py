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
class GrabPoint:
    """二区入口抓取点."""
    approach_x: float = 0.0
    approach_y: float = 0.0
    grab_x: float = 0.0
    grab_y: float = 0.0
    dt35_x: float = 0.0
    dt35_y: float = 0.0
    is_finsh: int = 0


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

        # ── DT35 目标: 不镜像, 传感器读数不随红蓝区变 ──

        # ── Zone2 方块坐标 ──
        for b in self.zone2_blocks:
            b.y = -b.y

        # ── Zone2 任务 (只改 Y, 四元数不变) ──
        for t in self.zone2_tasks:
            t.y = -t.y
            t.approach_y = -t.approach_y
            t.rotate_y = -t.rotate_y

        # ── 入口抓取坐标 (只镜像 Y, DT35 不镜像) ──
        for gp in self.grab_points:
            gp.approach_y = -gp.approach_y
            gp.grab_y = -gp.grab_y

        # ── 台阶起始点 ──
        self.stairs_start_y = -self.stairs_start_y

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

    # 入口抓取 (三个固定抓块点)
    grab_points: list[GrabPoint] = field(default_factory=lambda: [
        GrabPoint(approach_x=-0.7, approach_y=1.9, grab_x=-0.7, grab_y=2.15,
                  dt35_x=0.428, dt35_y=1.516, is_finsh=2),
        GrabPoint(approach_x=-1.9, approach_y=1.9, grab_x=-1.9, grab_y=2.15,
                  dt35_x=0.428, dt35_y=2.716, is_finsh=1),
        GrabPoint(approach_x=-3.1, approach_y=1.9, grab_x=-3.1, grab_y=2.15,
                  dt35_x=0.428, dt35_y=3.919, is_finsh=2),
    ])
    grab_qz: float = 0.707    # 抓块时的四元数 z 分量 (蓝区)
    grab_qw: float = 0.707    # 抓块时的四元数 w 分量

    # 台阶起始点 (三个块吸完后回到这里)
    stairs_start_x: float = -1.9
    stairs_start_y: float = 1.9

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

        # ── 等对接完成 (TODO: 之后改成事件驱动) ──
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
    入口抓取: 三个固定点依次吸块, 吸完回到台阶起始点.

    流水线:
      1. 导航到 approach → DT35 微调
      2. 发 is_finsh, 导航到 grab 点
      3. 等 XIPAN_GRABBED (status=1) → 吸到了, 开始走下一个 approach (但不发 is_finsh)
      4. 等 XIPAN_DONE (status=2) → 彻底完成, 可以发下一个 is_finsh
    """

    nav_task = None  # 后台导航任务

    for i, gp in enumerate(cfg.grab_points):
        log.info("EntryGrab: 块%d approach=(%.2f, %.2f) grab=(%.2f, %.2f) is_finsh=%d",
                 i, gp.approach_x, gp.approach_y, gp.grab_x, gp.grab_y, gp.is_finsh)

        # ── 等待上一个点的导航完成 (如果有) ──
        if nav_task is not None:
            await nav_task
            nav_task = None

        # ── DT35 微调 ──
        await fsm.fine_tune(
            gp.dt35_x, gp.dt35_y,
            cfg.fine_tune_xy_threshold,
            cfg.fine_tune_speed_x, cfg.fine_tune_speed_y,
            cfg.fine_tune_stable_required, cfg.fine_tune_timeout_s,
            lambda: (state.dt35_x, state.dt35_y),
        )

        # ── 发 is_finsh + 导航到 grab 点 ──
        act.publish_cmd(0, gp.is_finsh, 0, 2)
        await fsm.nav_to(gp.grab_x, gp.grab_y, 0,
                         0, 0, cfg.grab_qz, cfg.grab_qw)

        # ── 等 XIPAN_GRABBED (status=1): 吸到了, 可以走位 ──
        await fsm.wait_event("XIPAN_GRABBED")
        log.info("EntryGrab: 块%d grabbed, start moving to next approach", i)

        # ── 吸到了就开始走下一个 approach (不发 is_finsh) ──
        if i + 1 < len(cfg.grab_points):
            nxt = cfg.grab_points[i + 1]
            nav_task = asyncio.ensure_future(
                fsm.nav_to(nxt.approach_x, nxt.approach_y, 0,
                           0, 0, cfg.grab_qz, cfg.grab_qw)
            )

        # ── 等 XIPAN_DONE (status=2): 彻底完成 ──
        await fsm.wait_event("XIPAN_DONE")
        log.info("EntryGrab: 块%d grab fully done", i)

    # ── 等最后一个点的导航完成 ──
    if nav_task is not None:
        await nav_task
        nav_task = None

    # ── 回到台阶起始点 ──
    log.info("EntryGrab: nav→stairs_start (%.2f, %.2f)", cfg.stairs_start_x, cfg.stairs_start_y)
    await fsm.nav_to(cfg.stairs_start_x, cfg.stairs_start_y, 0,
                     0, 0, cfg.grab_qz, cfg.grab_qw)
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

    1. 入口抓取: 三个固定点依次吸块
    2. 回到台阶起始点, 上台阶进入梅花林
    3. 按 zone2_tasks 逐点处理 (台阶/抓取)
    """

    # ── 入口抓取 (三个固定点) ──
    await entry_grab(fsm, act, cfg, state)

    # ── 上台阶进入梅花林 ──
    if cfg.zone2_tasks:
        for idx, task in enumerate(cfg.zone2_tasks):
            log.info("Zone2 task %d: stair=%d", idx, task.stair_cmd)

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
