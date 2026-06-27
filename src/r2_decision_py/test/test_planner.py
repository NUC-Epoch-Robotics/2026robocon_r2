"""
测试 Zone2 动态路线规划器.

运行: python -m test.test_planner
"""

import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from r2_decision_py.planner import build_zone2_route, find_action, HEIGHT, ADJ
from r2_decision_py.decision import Config, Zone2BlockInfo

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(message)s')
log = logging.getLogger("test")


def make_config() -> Config:
    """创建测试用配置 (坐标来自 launch 文件)."""
    cfg = Config()
    # 梅花林 4×3 网格坐标 (简化, 实际从 launch 参数读)
    coords = [
        (1.0, 0.0), (2.0, 0.0), (3.0, 0.0),     # 行0
        (1.0, 1.0), (2.0, 1.0), (3.0, 1.0),     # 行1
        (1.0, 2.0), (2.0, 2.0), (3.0, 2.0),     # 行2
        (1.0, 3.0), (2.0, 3.0), (3.0, 3.0),     # 行3
    ]
    scenes = [2, 1, 2, 1, 2, 3, 2, 3, 2, 1, 2, 1]
    cfg.zone2_blocks = [
        Zone2BlockInfo(x=x, y=y, z=0.0, grab_scene=s)
        for (x, y), s in zip(coords, scenes)
    ]
    return cfg


def test_basic_route():
    """测试: 全部 R2 方块, 无障碍."""
    log.info("=== 测试 1: 全部可通行 ===")
    cfg = make_config()
    # 灯板: 1=可通行, 2=R2目标
    lightboard = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
    tasks = build_zone2_route(cfg, lightboard)
    log.info("任务数: %d", len(tasks))
    assert len(tasks) > 0
    log.info("")


def test_with_obstacles():
    """测试: 有障碍物 (假方块)."""
    log.info("=== 测试 2: 有障碍 ===")
    cfg = make_config()
    # 格子 4 和 7 是障碍
    lightboard = [2, 2, 2, 2, 0, 2, 2, 0, 2, 2, 2, 2]
    tasks = build_zone2_route(cfg, lightboard)
    log.info("任务数: %d", len(tasks))
    # 不应该经过格子 4 或 7
    task_ids = [t.id for t in tasks]
    assert 4 not in task_ids, "不应该经过障碍格子4"
    assert 7 not in task_ids, "不应该经过障碍格子7"
    log.info("")


def test_sparse_r2():
    """测试: 只有少量 R2 方块."""
    log.info("=== 测试 3: 稀疏 R2 ===")
    cfg = make_config()
    # 只有格子 1, 5, 11 有 R2
    lightboard = [0, 2, 0, 0, 0, 2, 0, 0, 0, 0, 0, 2]
    tasks = build_zone2_route(cfg, lightboard)
    log.info("任务数: %d", len(tasks))
    log.info("")


def test_height_actions():
    """测试: 高度差 → 抓取指令."""
    log.info("=== 测试 4: 高度表 ===")
    for i in range(12):
        for j in ADJ[i]:
            if j < 0:
                continue
            action = find_action(i, j)
            if action:
                dh = HEIGHT[j] - HEIGHT[i]
                log.info("  %d→%d: dh=%+d stair=%d grab=%d rot=%d",
                         i, j, dh, action.stair, action.grab, action.rotation)
    log.info("")


def test_adjacency():
    """测试: 邻接表正确性."""
    log.info("=== 测试 5: 邻接表 ===")
    for i in range(12):
        neighbors = [n for n in ADJ[i] if n >= 0]
        log.info("  格子 %d: 邻居 %s", i, neighbors)
        # 验证对称性
        for n in neighbors:
            assert i in ADJ[n], f"邻接不对称: {i}→{n} 但 {n} 不连 {i}"
    log.info("邻接表对称性验证通过 ✓")


if __name__ == '__main__':
    test_adjacency()
    test_height_actions()
    test_basic_route()
    test_with_obstacles()
    test_sparse_r2()
    log.info("全部测试通过 ✓")
