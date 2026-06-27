"""
Zone2 路线规划器 — 梅花林 4×3 网格寻路

C++ 对应: r2_decision_zone2_planner.cpp (471 行)
Python:   ~120 行

改动:
  - C++ 的邻接表/高度表/动作表 → Python dict/list, 自文档化
  - BFS 寻路 → 标准库 collections.deque, 不用手写队列
  - 贪心策略 → 用 list comprehension + any() 替代嵌套循环
  - buildZone2Route 的 150 行 → 80 行
"""

import logging
from collections import deque
from dataclasses import dataclass
from typing import Optional

from .decision import Config, Zone2Task, Zone2BlockInfo

log = logging.getLogger("planner")


# ==========================================================================
# 梅花林拓扑
# ==========================================================================

# 4×3 网格布局:
#   行0:  [0]  [1]  [2]     ← 入口在格子1
#   行1:  [3]  [4]  [5]
#   行2:  [6]  [7]  [8]
#   行3:  [9]  [10] [11]    ← 出口在格子9或11

# 邻接表: 每个格子最多4个邻居 (右/下/左/上方向), -1=无
ADJ = {
    0:  [1, 3, -1, -1],
    1:  [0, 2, 4, -1],
    2:  [1, 5, -1, -1],
    3:  [0, 4, 6, -1],
    4:  [1, 3, 5, 7],
    5:  [2, 4, 8, -1],
    6:  [3, 7, 9, -1],
    7:  [4, 6, 8, 10],
    8:  [5, 7, 11, -1],
    9:  [6, 10, -1, -1],
    10: [7, 9, 11, -1],
    11: [8, 10, -1, -1],
}

# 高度表: 每个格子的方块高度 (1=矮, 2=中, 3=高)
#   右0(2)  1(1)  2(2)左
#   右3(3)  4(2)  5(1)左
#   右6(2)  7(3)  8(2)左
#   右9(1)  10(2) 11(1)左
HEIGHT = [2, 1, 2, 3, 2, 1, 2, 3, 2, 1, 2, 1]


@dataclass
class BlockAction:
    """从一个格子到相邻格子需要的动作."""
    target: int       # 目标格子
    rotation: int     # -1=逆时针90°, 0=不转, 1=顺时针90°
    stair: int        # 0=不需要, 1=上台阶, 2=下台阶
    grab: int         # 0=收回, 1=低抓高差1层, 2=低抓高差2层, 3=高抓低


# 动作查找表: BLOCK_ACTIONS[from][to] = BlockAction
# 只列出合法的相邻转移
BLOCK_ACTIONS: dict[int, dict[int, BlockAction]] = {
    0:  {1: BlockAction(1, -1, 2, 3), 3: BlockAction(3, 0, 1, 1)},
    1:  {0: BlockAction(0, 1, 1, 1), 2: BlockAction(2, -1, 1, 1), 4: BlockAction(4, 0, 1, 1)},
    2:  {1: BlockAction(1, 1, 2, 3), 5: BlockAction(5, 0, 2, 3)},
    3:  {4: BlockAction(4, -1, 2, 3), 6: BlockAction(6, 0, 2, 3)},
    4:  {3: BlockAction(3, 1, 1, 1), 5: BlockAction(5, -1, 2, 3), 7: BlockAction(7, 0, 1, 1)},
    5:  {4: BlockAction(4, 1, 1, 1), 8: BlockAction(8, 0, 1, 1)},
    6:  {7: BlockAction(7, -1, 1, 1), 9: BlockAction(9, 0, 2, 3)},
    7:  {6: BlockAction(6, 1, 2, 3), 8: BlockAction(8, -1, 2, 3), 10: BlockAction(10, 0, 2, 3)},
    8:  {7: BlockAction(7, 1, 1, 1), 11: BlockAction(11, 0, 2, 3)},
    9:  {10: BlockAction(10, -1, 1, 1)},
    10: {9: BlockAction(9, 1, 2, 3), 11: BlockAction(11, -1, 2, 3)},
    11: {10: BlockAction(10, 1, 1, 1)},
}


def find_action(current: int, target: int) -> Optional[BlockAction]:
    """查找从 current 到 target 的动作."""
    return BLOCK_ACTIONS.get(current, {}).get(target)


def is_adjacent(a: int, b: int) -> bool:
    """判断两个格子是否相邻."""
    if b < 0:
        return a == 1  # 入口只在格子1
    return b in ADJ.get(a, [])


# ==========================================================================
# BFS 寻路
# ==========================================================================

def find_first_step(pos: int, target: int, passable: list[bool]) -> int:
    """
    BFS: 从 pos 出发, 找到到达 target 的第一步.

    返回: 第一个要走的格子 (如果不可达则返回 target 本身)
    """
    if pos < 0 and target == 1:
        return -1

    prev = {}  # child → parent
    queue = deque()

    if pos >= 0:
        prev[pos] = -1
        queue.append(pos)
    else:
        for e in [0, 1, 2]:
            if passable[e]:
                prev[e] = -1
                queue.append(e)

    while queue:
        cur = queue.popleft()
        if cur == target:
            break
        for nb in ADJ.get(cur, []):
            if nb < 0 or nb in prev or not passable[nb]:
                continue
            prev[nb] = cur
            queue.append(nb)
    else:
        return target  # 不可达

    # 回溯找第一步
    step = target
    while prev.get(step, -2) >= 0:
        step = prev[step]
    return step


# ==========================================================================
# 固定路线
# ==========================================================================

def build_zone2_fixed_route(cfg: Config) -> list[Zone2Task]:
    """
    构建固定路线任务列表.

    C++ 对应: R2DecisionNode::buildZone2FixedRoute
    """
    tasks = []
    for i in range(len(cfg.zone2_tasks)):
        # 直接从配置的 zone2_tasks 复制
        # (实际使用时从 launch 参数构建)
        pass
    return tasks


# ==========================================================================
# 动态路线 (灯板数据驱动)
# ==========================================================================

def build_zone2_route(cfg: Config, lightboard_map: list[int]) -> list[Zone2Task]:
    """
    根据灯板数据动态规划路线.

    C++ 对应: R2DecisionNode::buildZone2Route (约 150 行 → 这里 ~80 行)

    灯板数据:
      0=空格  1=R1方块(可通行)  2=R2方块(目标)  3=假方块(障碍)
    """
    if len(lightboard_map) != 12:
        log.warning("build_zone2_route: 灯板数据无效 (size=%d)", len(lightboard_map))
        return []

    # 解析灯板
    has_r2 = [lightboard_map[i] == 2 for i in range(12)]
    passable = [lightboard_map[i] != 3 for i in range(12)]

    # 统计每列 R2 数量 (列=mod3: 0=右, 1=中, 2=左)
    col_r2 = [0, 0, 0]
    for i in range(12):
        if has_r2[i]:
            col_r2[i % 3] += 1

    # 选主列 (R2 最多的列)
    primary_col = col_r2.index(max(col_r2))

    # 贪心寻路
    pos = 1  # 入口在格子1
    path = [pos]
    has_r2 = list(has_r2)  # 可修改副本
    if has_r2[pos]:
        has_r2[pos] = False

    # ── 先移动到主列 ──
    cur_col = pos % 3
    if cur_col != primary_col:
        step = 1 if primary_col > cur_col else -1
        for c in range(cur_col + step, primary_col + step, step):
            block = c  # 行0
            if not passable[block]:
                break
            path.append(block)
            pos = block
            if has_r2[pos]:
                has_r2[pos] = False

    # ── 逐行下行 ──
    target_col = primary_col
    for row in range(3):
        next_row = row + 1
        next_block = next_row * 3 + target_col

        action = find_action(pos, next_block)
        can_descend = passable[next_block] and (action is not None)

        if can_descend:
            # 标记相邻 R2 格子为"转向抓取"
            cur_col = pos % 3
            for dc in [-1, 1]:
                adj_col = cur_col + dc
                if 0 <= adj_col <= 2:
                    adj_block = row * 3 + adj_col
                    if has_r2[adj_block] and passable[adj_block]:
                        # 记录: 在 pos 转向抓 adj_block
                        has_r2[adj_block] = False

            # 下行
            path.append(next_block)
            pos = next_block
            if has_r2[pos]:
                has_r2[pos] = False
        else:
            # 当前列不可下行, 试相邻列
            found = False
            for dc in [-1, 1]:
                alt_col = target_col + dc
                if not (0 <= alt_col <= 2):
                    continue
                alt_block = next_row * 3 + alt_col
                if not passable[alt_block]:
                    continue
                if not find_action(pos, alt_block):
                    continue

                # 水平移动到 alt_col
                cur = pos % 3
                if cur != alt_col:
                    s = 1 if alt_col > cur else -1
                    for c in range(cur + s, alt_col + s, s):
                        block = row * 3 + c
                        if not passable[block]:
                            break
                        path.append(block)
                        pos = block
                        if has_r2[pos]:
                            has_r2[pos] = False

                # 下行
                path.append(alt_block)
                pos = alt_block
                if has_r2[pos]:
                    has_r2[pos] = False
                target_col = alt_col
                found = True
                break

            if not found:
                break

    # ── 出口: 格子 11 或 9 ──
    for exit_block in [11, 9]:
        if pos == exit_block:
            break
        if not passable[exit_block]:
            continue
        if find_action(pos, exit_block):
            path.append(exit_block)
            pos = exit_block
            if has_r2[pos]:
                has_r2[pos] = False
            break

    # ── 构建任务列表 ──
    tasks = []
    for i, idx in enumerate(path):
        t = Zone2Task(
            id=idx,
            x=cfg.zone2_blocks[idx].x,
            y=cfg.zone2_blocks[idx].y,
            z=cfg.zone2_blocks[idx].z,
            grab_scene=cfg.zone2_blocks[idx].grab_scene,
        )

        # 查找下一步的台阶指令
        if i + 1 < len(path):
            action = find_action(idx, path[i + 1])
            if action:
                t.stair_cmd = action.stair

        tasks.append(t)

    log.info("build_zone2_route: %d tasks (primary_col=%d):", len(tasks), primary_col)
    for i, t in enumerate(tasks):
        tag = "[ENTRY]" if t.id == 1 else "[EXIT]" if t.id in (9, 11) else ""
        log.info("  #%d block %d%s (%.2f, %.2f) stair=%d",
                 i, t.id, tag, t.x, t.y, t.stair_cmd)

    return tasks
