# tests/test_r1_policy.py
import json
from pathlib import Path
from kf_decision import get_policy, list_policies
from kf_decision.abc import Policy
from kf_decision.types import GameState, RobotPose, TeamSide, KungFuScroll, KFSType, ArenaGridCell
from commands import RobotCommand, CmdR1StrategySuggestion, CmdR1PathSuggestion


def test_r1_policy_registration():
    """测试R1策略是否正确注册"""
    assert "r1_simple" in list_policies()
    Klass = get_policy("r1_simple")
    assert issubclass(Klass, Policy)


def test_r1_policy_mc_phase():
    """测试R1策略在武器组装阶段的行为"""
    # 创建测试游戏状态
    mf_slots = {
        "A": KungFuScroll(id="scroll-A", slot_name="A", kfs_type=KFSType.REAL),
        "B": KungFuScroll(id="scroll-B", slot_name="B", kfs_type=KFSType.FAKE),
        "C": KungFuScroll(id="scroll-C", slot_name="C", kfs_type=KFSType.REAL),
    }
    
    state = GameState(
        time_left_ms=180000,
        phase="MC",  # 武器组装阶段
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
        mf_slots=mf_slots,
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(0.0, 0.0),
        packages=[],
        retry_requested=False
    )
    
    # 获取R1策略并生成决策
    policy = get_policy("r1_simple")()
    cmds: list[RobotCommand] = policy.decide(state)
    
    # 验证生成的命令
    assert len(cmds) > 0
    assert isinstance(cmds[0], CmdR1StrategySuggestion)
    assert cmds[0].strategy == "grab_staff"


def test_r1_policy_mf_phase():
    """测试R1策略在梅花林阶段的行为"""
    # 创建测试游戏状态
    mf_slots = {
        "A": KungFuScroll(id="scroll-A", slot_name="A", kfs_type=KFSType.REAL),
        "B": KungFuScroll(id="scroll-B", slot_name="B", kfs_type=KFSType.FAKE),
        "C": KungFuScroll(id="scroll-C", slot_name="C", kfs_type=KFSType.REAL),
    }
    
    state = GameState(
        time_left_ms=180000,
        phase="MF",  # 梅花林阶段
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(x=0.0, y=0.0, yaw=0.0),
        r1_has_weapon=True,
        r1_weapon_count=1,
        r1_kfs=[],  # 没有收集KFS
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
        mf_slots=mf_slots,
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(0.0, 0.0),
        packages=[],
        retry_requested=False
    )
    
    # 获取R1策略并生成决策
    policy = get_policy("r1_simple")()
    cmds: list[RobotCommand] = policy.decide(state)
    
    # 验证生成的命令
    assert len(cmds) > 0
    # 可能生成路径建议或策略建议
    assert isinstance(cmds[0], (CmdR1PathSuggestion, CmdR1StrategySuggestion))


def test_r1_policy_arena_phase():
    """测试R1策略在竞技场阶段的行为"""
    # 创建测试游戏状态
    mf_slots = {
        "A": KungFuScroll(id="scroll-A", slot_name="A", kfs_type=KFSType.REAL),
        "B": KungFuScroll(id="scroll-B", slot_name="B", kfs_type=KFSType.FAKE),
        "C": KungFuScroll(id="scroll-C", slot_name="C", kfs_type=KFSType.REAL),
    }
    
    state = GameState(
        time_left_ms=180000,
        phase="ARENA",  # 竞技场阶段
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(x=0.0, y=0.0, yaw=0.0),
        r1_has_weapon=True,
        r1_weapon_count=1,
        r1_kfs=["kfs-1"],  # 已收集KFS
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
        mf_slots=mf_slots,
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(0.0, 0.0),
        packages=[],
        retry_requested=False
    )
    
    # 获取R1策略并生成决策
    policy = get_policy("r1_simple")()
    cmds: list[RobotCommand] = policy.decide(state)
    
    # 验证生成的命令
    assert len(cmds) > 0
    assert isinstance(cmds[0], CmdR1StrategySuggestion)
    assert cmds[0].strategy in ["place_kfs", "attack"]
