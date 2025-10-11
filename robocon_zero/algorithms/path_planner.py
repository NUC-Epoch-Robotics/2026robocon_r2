# algorithms/path_planner.py
"""
路径规划算法
实现A*算法用于机器人路径规划
"""

import heapq
import math
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
from kf_decision.types import RobotPose


@dataclass
class Node:
    """路径规划节点"""
    x: int
    y: int
    g: float  # 从起点到当前节点的实际代价
    h: float  # 启发式代价（到终点的估计代价）
    f: float  # 总代价 f = g + h
    parent: Optional['Node'] = None
    
    def __lt__(self, other):
        return self.f < other.f


class PathPlanner:
    """路径规划器"""
    
    def __init__(self, grid_width: int, grid_height: int, resolution: int = 100):
        """
        初始化路径规划器
        :param grid_width: 网格宽度（毫米）
        :param grid_height: 网格高度（毫米）
        :param resolution: 网格分辨率（毫米）
        """
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.resolution = resolution
        
        # 计算网格尺寸
        self.grid_cols = grid_width // resolution
        self.grid_rows = grid_height // resolution
        
        # 障碍物集合 (grid_x, grid_y)
        self.obstacles: Set[Tuple[int, int]] = set()
        
    def add_obstacle(self, x: float, y: float):
        """添加障碍物"""
        grid_x = int(x // self.resolution)
        grid_y = int(y // self.resolution)
        if 0 <= grid_x < self.grid_cols and 0 <= grid_y < self.grid_rows:
            self.obstacles.add((grid_x, grid_y))
            
    def remove_obstacle(self, x: float, y: float):
        """移除障碍物"""
        grid_x = int(x // self.resolution)
        grid_y = int(y // self.resolution)
        self.obstacles.discard((grid_x, grid_y))
        
    def plan_path(self, start: RobotPose, goal: RobotPose) -> List[Tuple[float, float]]:
        """
        规划路径
        :param start: 起点
        :param goal: 终点
        :return: 路径点列表 [(x, y), ...]
        """
        # 转换为网格坐标
        start_x = int(start.x // self.resolution)
        start_y = int(start.y // self.resolution)
        goal_x = int(goal.x // self.resolution)
        goal_y = int(goal.y // self.resolution)
        
        # 检查起点和终点是否在网格内
        if not (0 <= start_x < self.grid_cols and 0 <= start_y < self.grid_rows):
            return []
        if not (0 <= goal_x < self.grid_cols and 0 <= goal_y < self.grid_rows):
            return []
            
        # 检查起点和终点是否是障碍物
        if (start_x, start_y) in self.obstacles or (goal_x, goal_y) in self.obstacles:
            return []
            
        # A*算法
        open_list: List[Node] = []
        closed_set: Set[Tuple[int, int]] = set()
        
        # 创建起始节点
        start_node = Node(start_x, start_y, 0, self._heuristic(start_x, start_y, goal_x, goal_y), 0)
        heapq.heappush(open_list, start_node)
        
        # 8方向移动（包括对角线）
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 四个方向
            (1, 1), (1, -1), (-1, -1), (-1, 1)  # 对角线方向
        ]
        direction_costs = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]  # 对角线代价为√2
        
        while open_list:
            # 取出f值最小的节点
            current = heapq.heappop(open_list)
            
            # 检查是否到达终点
            if current.x == goal_x and current.y == goal_y:
                # 重构路径
                path = self._reconstruct_path(current)
                # 转换为实际坐标
                return [(x * self.resolution + self.resolution // 2, 
                        y * self.resolution + self.resolution // 2) for x, y in path]
            
            # 添加到已访问集合
            closed_set.add((current.x, current.y))
            
            # 探索邻居节点
            for i, (dx, dy) in enumerate(directions):
                neighbor_x = current.x + dx
                neighbor_y = current.y + dy
                
                # 检查边界
                if not (0 <= neighbor_x < self.grid_cols and 0 <= neighbor_y < self.grid_rows):
                    continue
                    
                # 检查是否已访问
                if (neighbor_x, neighbor_y) in closed_set:
                    continue
                    
                # 检查是否是障碍物
                if (neighbor_x, neighbor_y) in self.obstacles:
                    continue
                    
                # 计算代价
                tentative_g = current.g + direction_costs[i]
                
                # 创建邻居节点
                neighbor = Node(
                    neighbor_x, 
                    neighbor_y, 
                    tentative_g, 
                    self._heuristic(neighbor_x, neighbor_y, goal_x, goal_y),
                    0  # 将在下面计算
                )
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current
                
                # 检查是否已在开放列表中
                existing_node = self._find_node_in_open_list(open_list, neighbor_x, neighbor_y)
                if existing_node and tentative_g >= existing_node.g:
                    continue
                    
                heapq.heappush(open_list, neighbor)
        
        # 没有找到路径
        return []
        
    def _heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """启发式函数（使用欧几里得距离）"""
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        
    def _reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """重构路径"""
        path = []
        current = node
        while current:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]  # 反转路径
        
    def _find_node_in_open_list(self, open_list: List[Node], x: int, y: int) -> Optional[Node]:
        """在开放列表中查找节点"""
        for node in open_list:
            if node.x == x and node.y == y:
                return node
        return None
        
    def set_obstacles_from_mf_blocks(self, mf_blocks: List[Tuple[float, float, int]]):
        """
        根据梅花林区块设置障碍物
        :param mf_blocks: 区块列表 [(x, y, level), ...]
        """
        for x, y, level in mf_blocks:
            # 三级区块视为障碍物
            if level == 3:
                self.add_obstacle(x, y)


def plan_simple_path(start: RobotPose, goal: RobotPose) -> List[Tuple[float, float]]:
    """
    简单路径规划（直线路径）
    :param start: 起点
    :param goal: 终点
    :return: 路径点列表
    """
    # 简单的直线路径，中间插入几个点
    path = [(start.x, start.y)]
    
    # 计算中间点
    dx = goal.x - start.x
    dy = goal.y - start.y
    steps = 10  # 插入10个中间点
    
    for i in range(1, steps):
        t = i / steps
        x = start.x + dx * t
        y = start.y + dy * t
        path.append((x, y))
        
    path.append((goal.x, goal.y))
    return path


# 全局路径规划器实例
planner = PathPlanner(12000, 8000)  # 根据场地尺寸创建规划器


def get_path_planner() -> PathPlanner:
    """获取全局路径规划器实例"""
    return planner
