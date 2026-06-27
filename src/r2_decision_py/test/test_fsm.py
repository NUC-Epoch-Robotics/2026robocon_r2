"""
独立测试: 演示 async FSM 工作方式, 不需要 ROS2.

运行: python -m test.test_fsm
"""

import asyncio
import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from r2_decision_py.fsm import FSM, Event

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(message)s')
log = logging.getLogger("test")


class MockActionDispatcher:
    """模拟 ActionDispatcher, 打印日志代替真实动作."""

    def __init__(self):
        self.fsm: FSM = None

    def send_navigate(self, x, y, z=0, qx=0, qy=0, qz=0, qw=1):
        log.info("  [ACT] NAV to (%.1f, %.1f)", x, y)
        async def done():
            await asyncio.sleep(0.5)
            self.fsm.post_event(Event("NAV_DONE", success=True))
        asyncio.create_task(done())

    def start_zone2_grab(self, is_finsh):
        log.info("  [ACT] GRAB is_finsh=%d", is_finsh)
        async def done():
            await asyncio.sleep(0.3)
            self.fsm.post_event(Event("UP_JUECE_DONE", success=True))
        asyncio.create_task(done())

    def stop_zone2_grab(self):
        log.info("  [ACT] STOP_GRAB")

    def start_stair(self, cmd):
        log.info("  [ACT] STAIR cmd=%d", cmd)
        async def done():
            await asyncio.sleep(0.4)
            self.fsm.post_event(Event("DOWN_JUECE_DONE", success=True))
        asyncio.create_task(done())

    def stop_stair(self):
        log.info("  [ACT] STOP_STAIR")

    def send_arm_command(self, cmd):
        log.info("  [ACT] ARM cmd=%d", cmd)
        async def done():
            await asyncio.sleep(0.3)
            self.fsm.post_event(Event("ARM_DONE", success=True))
        asyncio.create_task(done())

    def send_spearhead_command(self, cmd):
        log.info("  [ACT] SPEARHEAD cmd=%d", cmd)
        async def done():
            await asyncio.sleep(0.3)
            self.fsm.post_event(Event("ARM_DONE", success=True))
        asyncio.create_task(done())

    def set_hold_cmd(self, cmd):
        pass

    def suppress_heartbeat_flag(self, s):
        pass

    def enable_spear(self, e):
        log.info("  [ACT] spear %s", "ON" if e else "OFF")

    def enable_lightboard(self, e):
        log.info("  [ACT] lightboard %s", "ON" if e else "OFF")

    def enable_grab_scene(self, e, scene=0):
        log.info("  [ACT] grab_scene %s (scene=%d)", "ON" if e else "OFF", scene)

    def publish_cmd(self, s, i=0, z=0, a=0):
        log.info("  [ACT] CMD status=%d is_finsh=%d zhuangtai=%d area=%d", s, i, z, a)

    def publish_cmd_with_area(self, s, i=0, z=0):
        log.info("  [ACT] CMD_WITH_AREA status=%d is_finsh=%d zhuangtai=%d", s, i, z)

    def publish_cmd_vel(self, vx, vy, vz=0):
        log.info("  [ACT] CMD_VEL (%.2f, %.2f)", vx, vy)

    def stop_cmd_vel(self):
        log.info("  [ACT] STOP_CMD_VEL")


# ==========================================================================
# 测试
# ==========================================================================

async def test_entry_grab(fsm: FSM, act: MockActionDispatcher):
    """模拟入口抓取简化版."""
    log.info("=== 入口抓取测试 ===")

    log.info("步骤 1: 导航到 approach")
    await fsm.nav_to(1.6, 0.289)

    log.info("步骤 2: 抓取 + 导航并行")
    act.start_zone2_grab(2)
    nav_r, grab_r = await asyncio.gather(
        fsm.nav_to(2.0, 0.289),
        fsm.wait_event("UP_JUECE_DONE"),
    )
    log.info("  nav=%s grab=%s", nav_r.success, grab_r.success)

    log.info("步骤 3: 等稳固")
    await fsm.wait(0.5)

    log.info("步骤 4: 倒回")
    await fsm.nav_to(1.6, 0.289)
    act.stop_zone2_grab()

    log.info("步骤 5: 上台阶")
    await fsm.up_stairs()

    log.info("=== 测试完成 ===")


async def test_parallel_events(fsm: FSM, act: MockActionDispatcher):
    """测试并行事件."""
    log.info("=== 并行事件测试 ===")

    async def delayed_nav():
        await asyncio.sleep(0.3)
        fsm.post_event(Event("NAV_DONE", success=True))

    async def delayed_arm():
        await asyncio.sleep(0.5)
        fsm.post_event(Event("ARM_DONE", success=True))

    asyncio.create_task(delayed_nav())
    asyncio.create_task(delayed_arm())

    results = await fsm.wait_events("NAV_DONE", "ARM_DONE")
    log.info("NAV_DONE: %s, ARM_DONE: %s", results[0].success, results[1].success)
    log.info("=== 并行测试完成 ===")


async def test_grab_with_retry(fsm: FSM, act: MockActionDispatcher):
    """测试抓取失败重试."""
    log.info("=== 重试测试 ===")

    attempt = 0

    async def failing_grab():
        nonlocal attempt
        attempt += 1
        await asyncio.sleep(0.2)
        ok = attempt >= 3
        log.info("  [模拟] 抓取 %s (attempt %d)", "成功" if ok else "失败", attempt)
        fsm.post_event(Event("ARM_DONE", success=ok))

    for retry in range(3):
        asyncio.create_task(failing_grab())
        result = await fsm.wait_event("ARM_DONE")
        if result.success:
            log.info("抓取成功!")
            break
        log.warning("重试 %d/3", retry + 1)

    log.info("=== 重试测试完成 ===")


# ==========================================================================

async def main():
    act = MockActionDispatcher()
    fsm = FSM()
    fsm.act = act
    act.fsm = fsm

    await fsm.run(test_entry_grab(fsm, act))
    print()
    await fsm.run(test_parallel_events(fsm, act))
    print()
    await fsm.run(test_grab_with_retry(fsm, act))


if __name__ == '__main__':
    asyncio.run(main())
