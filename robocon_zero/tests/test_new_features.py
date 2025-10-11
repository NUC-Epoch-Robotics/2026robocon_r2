# tests/test_new_features.py
"""
测试新添加的功能
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest
from kf_decision.types import GameState, RobotPose, TeamSide, KungFuScroll, KFSType, ArenaGridCell
from config.field_config import FIELD_WIDTH, FIELD_HEIGHT, COLORS
from utils.logger import RoboconLogger, log_game_state
from utils.rule_validator import RuleValidator
from algorithms.path_planner import PathPlanner, plan_simple_path
from algorithms.decision_tree import DecisionTreePolicy, BehaviorTreePolicy


def test_field_config():
    """测试场地配置"""
    assert FIELD_WIDTH == 12000
    assert FIELD_HEIGHT == 8000
    assert "RED_START_ZONE" in COLORS


def test_logger():
    """测试日志系统"""
    logger = RoboconLogger("test_logger", level=100)  # 设置高日志级别，避免输出
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
    
    logger.info("Test message", game_state=state)
    logs = logger.get_logs()
    assert len(logs) == 1
    assert logs[0].message == "Test message"


def test_rule_validator():
    """测试规则验证器"""
    validator = RuleValidator()
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
    
    # 测试R1在MC阶段抓取矛头应该违规
    result = validator.validate_all(state, "grab_spearhead", "r1")
    assert not result
    assert "R1_CANNOT_GRAB_SPEARHEAD_IN_MC" in validator.get_violations()


def test_path_planner():
    """测试路径规划器"""
    planner = PathPlanner(1000, 1000, 100)
    start = RobotPose(x=0.0, y=0.0, yaw=0.0)
    goal = RobotPose(x=500.0, y=500.0, yaw=0.0)
    
    path = planner.plan_path(start, goal)
    assert len(path) > 0
    assert path[0] == (50.0, 50.0)  # 起点
    assert path[-1] == (550.0, 550.0)  # 终点


def test_simple_path_planner():
    """测试简单路径规划器"""
    start = RobotPose(x=0.0, y=0.0, yaw=0.0)
    goal = RobotPose(x=100.0, y=100.0, yaw=0.0)
    
    path = plan_simple_path(start, goal)
    assert len(path) == 11  # 起点 + 9个中间点 + 终点
    assert path[0] == (0.0, 0.0)
    assert path[-1] == (100.0, 100.0)


def test_decision_tree_policy():
    """测试决策树策略"""
    policy = DecisionTreePolicy()
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
    
    commands = policy.decide(state)
    assert len(commands) > 0


def test_behavior_tree_policy():
    """测试行为树策略"""
    policy = BehaviorTreePolicy()
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
    
    commands = policy.decide(state)
    assert isinstance(commands, list)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
