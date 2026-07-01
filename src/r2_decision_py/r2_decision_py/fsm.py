"""
异步 FSM 框架 — 让决策逻辑写成线性 async 函数

核心思想:
  每个 await = 一次状态切换. 不需要 step 编号, 不需要 switch-case.
  "等什么事件" 就写 await fsm.wait_event("NAV_DONE"), 直白自文档化.

用法:
  fsm = FSM(action_dispatcher)
  async def my_decision(fsm, act):
      await fsm.nav_to(1.0, 2.0)        # 发导航, 等 NAV_DONE
      await fsm.wait(5.0)                # 等 5 秒
      act.start_zone2_grab(1)            # 发抓取 (不等)
      await fsm.wait_event("ARM_DONE")   # 等机械臂完成
      await fsm.up_stairs()              # 发台阶, 等 DOWN_JUECE_DONE
"""

import asyncio
import logging
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger("fsm")


# ==========================================================================
# 事件
# ==========================================================================

@dataclass
class Event:
    type: str           # "NAV_DONE", "ARM_DONE", "UP_JUECE_DONE", ...
    success: bool = True
    command: int = 0


# ==========================================================================
# FSM
# ==========================================================================

class FSM:
    """
    异步状态机运行器.

    决策逻辑写成 async 函数, 用以下原语等待:
      - await fsm.nav_to(x, y, ...)     → 发导航, 等 NAV_DONE
      - await fsm.wait(seconds)          → 等 N 秒
      - await fsm.wait_event("XXX")      → 等特定事件
      - await fsm.wait_events("A", "B")  → 等两个事件都到
      - await fsm.up_stairs()            → 发台阶, 等 DOWN_JUECE_DONE
      - await fsm.fine_tune(...)         → DT35 bang-bang 闭环

    也支持 asyncio.gather 并行等待:
      await asyncio.gather(
          fsm.nav_to(x, y),
          fsm.wait_event("UP_JUECE_DONE"),
      )
    """

    def __init__(self):
        self._waiters: list[tuple[str, asyncio.Future]] = []
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._last_nav_x: float = 0.0
        self._last_nav_y: float = 0.0

        # ActionDispatcher 在 node.py 中设置
        self.act: object = None  # 实际类型是 ActionDispatcher

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    # ── 事件分发 ────────────────────────────────────────────────

    def post_event(self, event: Event):
        """
        从任意线程投递事件. 唤醒所有匹配的 awaiter.
        线程安全: 通过 call_soon_threadsafe.
        """
        if self._loop is None:
            log.warning("post_event(%s) dropped: loop not set", event.type)
            return

        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._dispatch_event, event)
        else:
            self._loop.call_soon(self._dispatch_event, event)

    def _dispatch_event(self, event: Event):
        """在 asyncio 线程中分发事件."""
        log.info("EVENT: %s success=%s cmd=%d", event.type, event.success, event.command)

        still_waiting = []
        for event_type, future in self._waiters:
            if event_type == event.type:
                if not future.done():
                    future.set_result(event)
            else:
                still_waiting.append((event_type, future))
        self._waiters = still_waiting

    # ── async 原语 ─────────────────────────────────────────────

    async def wait_event(self, event_type: str, timeout: Optional[float] = None) -> Event:
        """等待指定类型的事件."""
        future = self._loop.create_future()
        self._waiters.append((event_type, future))

        try:
            if timeout is not None:
                return await asyncio.wait_for(future, timeout)
            return await future
        except asyncio.TimeoutError:
            log.warning("wait_event(%s) timeout after %.1fs", event_type, timeout)
            return Event(event_type, success=False)

    async def wait_events(self, *event_types: str, timeout: Optional[float] = None) -> list[Event]:
        """等待多个事件全部到达."""
        futures = {et: self._loop.create_future() for et in event_types}
        for et, fut in futures.items():
            self._waiters.append((et, fut))

        try:
            if timeout is not None:
                await asyncio.wait(
                    list(futures.values()), timeout=timeout,
                    return_when=asyncio.ALL_COMPLETED)
            else:
                await asyncio.gather(*futures.values())
            return [futures[et].result() if futures[et].done() else Event(et, success=False)
                    for et in event_types]
        except asyncio.TimeoutError:
            return [Event(et, success=False) for et in event_types]

    async def wait(self, seconds: float):
        """等待指定秒数."""
        await asyncio.sleep(seconds)

    # ── 导航 ───────────────────────────────────────────────────

    async def nav_to(self, x: float, y: float, z: float = 0.0,
                     qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0) -> Event:
        """发送导航目标到 /command, 等待到达."""
        self._last_nav_x = x
        self._last_nav_y = y
        self.act.send_navigate(x, y, z, qx, qy, qz, qw)
        return await self.wait_event("NAV_DONE")

    async def rotate_to(self, x: float, y: float, yaw: float, z: float = 0.0) -> Event:
        """原地旋转到指定朝向."""
        import math
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return await self.nav_to(x, y, z, 0, 0, qz, qw)

    # ── 台阶 ───────────────────────────────────────────────────

    async def up_stairs(self) -> Event:
        """上台阶, 等待完成."""
        self.act.start_stair(1)
        return await self.wait_event("DOWN_JUECE_DONE")

    async def down_stairs(self) -> Event:
        """下台阶, 等待完成."""
        self.act.start_stair(2)
        return await self.wait_event("DOWN_JUECE_DONE")

    # ── 机械臂 / 抓取 ──────────────────────────────────────────

    async def spearhead_and_wait(self, cmd: int) -> Event:
        """发送矛头指令, 等待完成."""
        await self.act.send_spearhead_command(cmd)
        return await self.wait_event("ARM_DONE")

    async def arm_and_wait(self, cmd: int) -> Event:
        """发送机械臂指令 (动态路线), 等待完成."""
        await self.act.send_arm_command(cmd)
        return await self.wait_event("ARM_DONE")

    # ── DT35 微调 ──────────────────────────────────────────────

    async def fine_tune(self, target_x: float, target_y: float,
                        threshold: float, speed_x: float, speed_y: float,
                        stable_required: int, timeout: float,
                        get_dt35, y_sign: float = 1.0):
        """DT35 + cmd_vel 闭环对齐. y_sign=-1 翻转 Y 轴方向 (红区)."""
        stable_count = 0
        start = asyncio.get_event_loop().time()

        while True:
            dt35_x, dt35_y = get_dt35()
            err_x = dt35_x - target_x
            err_y = target_y - dt35_y

            vx = speed_x * (1 if err_x > 0 else -1) if abs(err_x) > threshold else 0.0
            vy = speed_y * y_sign * (1 if err_y > 0 else -1) if abs(err_y) > threshold else 0.0

            log.info("FINE_TUNE dt35=(%.3f,%.3f) target=(%.3f,%.3f) err=(%.3f,%.3f) vel=(%.3f,%.3f)",
                     dt35_x, dt35_y, target_x, target_y, err_x, err_y, vx, vy)
            self.act.publish_cmd_vel(vx, vy)

            if abs(err_x) < threshold and abs(err_y) < threshold:
                stable_count += 1
            else:
                stable_count = max(0, stable_count - 1)  # 偶尔跳出扣 1, 不归零

            if stable_count >= stable_required:
                self.act.stop_cmd_vel()
                log.info("FINE_TUNE aligned at dt35(%.3f,%.3f)", dt35_x, dt35_y)
                return

            elapsed = asyncio.get_event_loop().time() - start
            if elapsed > timeout:
                self.act.stop_cmd_vel()
                log.warning("FINE_TUNE timeout (%.1fs) at dt35(%.3f,%.3f)", elapsed, dt35_x, dt35_y)
                return

            await asyncio.sleep(0.02)

    async def dt35_correct(self, nav_x: float, nav_y: float,
                           dt35_target_x: float, dt35_target_y: float,
                           get_dt35,
                           qx: float = 0.0, qy: float = 0.0,
                           qz: float = 0.0, qw: float = 1.0):
        """
        DT35 一次性坐标修正.

        读 DT35 当前值, 算误差, 加到导航目标上, 发修正后的目标到 /command.
        err = dt35_target - dt35_current
        new_goal = nav + err
        """
        # 等待一小段时间让DT35值稳定
        await asyncio.sleep(0.3)

        # 读取DT35当前值
        dt35_x, dt35_y = get_dt35()
        err_x = dt35_target_x - dt35_x
        err_y = dt35_target_y - dt35_y

        corrected_x = nav_x + err_x
        corrected_y = nav_y + err_y

        log.info("DT35_CORRECT dt35=(%.3f,%.3f) target=(%.3f,%.3f) err=(%.3f,%.3f) "
                 "nav=(%.2f,%.2f)→(%.2f,%.2f)",
                 dt35_x, dt35_y, dt35_target_x, dt35_target_y, err_x, err_y,
                 nav_x, nav_y, corrected_x, corrected_y)

        # 导航到修正后的坐标
        self.act.send_navigate(corrected_x, corrected_y, 0.0, qx, qy, qz, qw)
        result = await self.wait_event("NAV_DONE")

        return result

    # ── 运行入口 ───────────────────────────────────────────────

    async def run(self, coro):
        """运行决策协程."""
        self._loop = asyncio.get_event_loop()
        try:
            await coro
        except asyncio.CancelledError:
            log.info("FSM coroutine cancelled")
        except Exception:
            log.exception("FSM coroutine exception")
