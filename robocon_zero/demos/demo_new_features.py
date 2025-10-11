# demos/demo_new_features.py
"""
演示新添加的功能
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kf_decision.types import GameState, RobotPose, TeamSide, ArenaGridCell
from config.field_config import FIELD_WIDTH, FIELD_HEIGHT
from utils.logger import RoboconLogger, log_game_state
from utils.rule_validator import RuleValidator
from algorithms.path_planner import PathPlanner, plan_simple_path
from algorithms.decision_tree import DecisionTreePolicy


def demo_field_config():
    """演示场地配置"""
    print("=== 场地配置演示 ===")
    print(f"场地宽度: {FIELD_WIDTH} mm")
    print(f"场地高度: {FIELD_HEIGHT} mm")
    print()


def demo_logger():
    """演示日志系统"""
    print("=== 日志系统演示 ===")
    logger = RoboconLogger("demo_logger")
    
    # 创建一个游戏状态
    state = GameState(
        time_left_ms=180000,
        phase="MC",
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(x=0.0, y=0.0, yaw=0.0),
        r1_has_weapon=False,
        r1_weapon_count=0,
        r1_kfs=[],
        r1_busy=False,
        r1_last_action="",
        r1_last_action_time=0,
        r2_pose=RobotPose(x=0.0, y=0.0, yaw=0.0),
        r2_has_weapon=False,
        r2_kfs=[],
        r2_side=TeamSide.LEFT,
        weapons=[],
        staff_rack={},
        spearhead_rack={},
        mf_slots={},
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(0.0, 0.0),
        packages=[],
        retry_requested=False
    )
    
    # 记录游戏状态
    log_game_state(state, "demo")
    print("日志已记录到控制台和文件")
    print()


def demo_rule_validator():
    """演示规则验证器"""
    print("=== 规则验证器演示 ===")
    validator = RuleValidator()
    
    # 创建一个游戏状态
    state = GameState(
        time_left_ms=180000,
        phase="MC",
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(x=0.0, y=0.0, yaw=0.0),
        r1_has_weapon=False,
        r1_weapon_count=0,
        r1_kfs=[],
        r1_busy=False,
        r1_last_action="",
        r1_last_action_time=0,
        r2_pose=RobotPose(x=0.0, y=0.0, yaw=0.0),
        r2_has_weapon=False,
        r2_kfs=[],
        r2_side=TeamSide.LEFT,
        weapons=[],
        staff_rack={},
        spearhead_rack={},
        mf_slots={},
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(0.0, 0.0),
        packages=[],
        retry_requested=False
    )
    
    # 验证R1在MC阶段抓取矛头的行为（应该违规）
    result = validator.validate_all(state, "grab_spearhead", "r1")
    print(f"R1在MC阶段抓取矛头是否合规: {result}")
    if not result:
        print(f"违规行为: {validator.get_violations()}")
    print()


def demo_path_planner():
    """演示路径规划器"""
    print("=== 路径规划器演示 ===")
    
    # 创建路径规划器
    planner = PathPlanner(1000, 1000, 100)
    
    # 设置起点和终点
    start = RobotPose(x=0.0, y=0.0, yaw=0.0)
    goal = RobotPose(x=500.0, y=500.0, yaw=0.0)
    
    # 规划路径
    path = planner.plan_path(start, goal)
    print(f"规划路径点数: {len(path)}")
    print(f"起点: {path[0]}")
    print(f"终点: {path[-1]}")
    print()
    
    # 演示简单路径规划器
    simple_path = plan_simple_path(start, goal)
    print(f"简单路径点数: {len(simple_path)}")
    print(f"起点: {simple_path[0]}")
    print(f"终点: {simple_path[-1]}")
    print()


def demo_decision_tree():
    """演示决策树策略"""
    print("=== 决策树策略演示 ===")
    
    # 创建决策树策略
    policy = DecisionTreePolicy()
    
    # 创建一个游戏状态
    state = GameState(
        time_left_ms=180000,
        phase="MC",
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(x=0.0, y=0.0, yaw=0.0),
        r1_has_weapon=False,
        r1_weapon_count=0,
        r1_kfs=[],
        r1_busy=False,
        r1_last_action="",
        r1_last_action_time=0,
        r2_pose=RobotPose(x=0.0, y=0.0, yaw=0.0),
        r2_has_weapon=False,
        r2_kfs=[],
        r2_side=TeamSide.LEFT,
        weapons=[],
        staff_rack={},
        spearhead_rack={},
        mf_slots={},
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(0.0, 0.0),
        packages=[],
        retry_requested=False
    )
    
    # 做出决策
    commands = policy.decide(state)
    print(f"生成的命令数: {len(commands)}")
    if commands:
        print(f"第一个命令类型: {type(commands[0]).__name__}")
        print(f"命令详情: {commands[0]}")
    print()


def main():
    """主函数"""
    print("新功能演示程序")
    print("=" * 50)
    
    demo_field_config()
    demo_logger()
    demo_rule_validator()
    demo_path_planner()
    demo_decision_tree()
    
    print("演示完成！")


if __name__ == "__main__":
    main()
