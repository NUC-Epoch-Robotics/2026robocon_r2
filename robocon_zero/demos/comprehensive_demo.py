# demos/comprehensive_demo.py
"""
综合演示程序
演示所有已实现的功能
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kf_decision.types import GameState, RobotPose, TeamSide, ArenaGridCell, KungFuScroll, KFSType
from kf_decision import get_policy
from config.field_config import *
from utils.logger import RoboconLogger, log_game_state, log_violation
from utils.rule_validator import RuleValidator, validate_action
from utils.analyzer import DataAnalyzer, PerformanceMetrics
from algorithms.path_planner import PathPlanner, plan_simple_path, get_path_planner
from algorithms.decision_tree import DecisionTreePolicy, BehaviorTreePolicy
from commands import CmdR1StrategySuggestion, CmdR1PathSuggestion


def demo_field_configuration():
    """演示场地配置"""
    print("=" * 60)
    print("场地配置演示")
    print("=" * 60)
    print(f"场地总尺寸: {FIELD_WIDTH}mm x {FIELD_HEIGHT}mm")
    print(f"武器组装区: {MC_WIDTH}mm x {MC_HEIGHT}mm")
    print(f"梅花林区: {MF_WIDTH}mm x {MF_HEIGHT}mm")
    print(f"竞技场区: {ARENA_WIDTH}mm x {ARENA_HEIGHT}mm")
    print(f"梅花林区块大小: {MF_BLOCK_SIZE}mm x {MF_BLOCK_SIZE}mm")
    print(f"KFS尺寸: {KFS_SIZE}mm³")
    print(f"R1起始尺寸: {R1_START_SIZE_X}mm x {R1_START_SIZE_Y}mm x {R1_START_SIZE_Z}mm")
    print(f"R2起始尺寸: {R2_START_SIZE_X}mm x {R2_START_SIZE_Y}mm x {R2_START_SIZE_Z}mm")
    print()


def demo_logging_system():
    """演示日志系统"""
    print("=" * 60)
    print("日志系统演示")
    print("=" * 60)
    
    logger = RoboconLogger("comprehensive_demo")
    
    # 创建游戏状态
    state = GameState(
        time_left_ms=180000,
        phase="MC",
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(x=500.0, y=300.0, yaw=0.0),
        r1_has_weapon=False,
        r1_weapon_count=0,
        r1_kfs=[],
        r1_busy=False,
        r1_last_action="",
        r1_last_action_time=0,
        r2_pose=RobotPose(x=1000.0, y=200.0, yaw=0.0),
        r2_has_weapon=False,
        r2_kfs=[],
        r2_side=TeamSide.LEFT,
        weapons=[],
        staff_rack={"staff_1": True, "staff_2": True},
        spearhead_rack={"spearhead_1": "FIST", "spearhead_2": "PALM"},
        mf_slots={
            "A": KungFuScroll(id="scroll-A", slot_name="A", kfs_type=KFSType.REAL, x=4000.0, y=1000.0),
            "B": KungFuScroll(id="scroll-B", slot_name="B", kfs_type=KFSType.FAKE, x=5000.0, y=2000.0)
        },
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(9000.0, 2000.0),
        packages=[],
        retry_requested=False
    )
    
    # 记录游戏状态
    log_game_state(state, "demo")
    
    # 记录违规行为
    log_violation("TEST_VIOLATION", "This is a test violation")
    
    print("日志已记录到控制台和文件")
    print()


def demo_rule_validation():
    """演示规则验证"""
    print("=" * 60)
    print("规则验证演示")
    print("=" * 60)
    
    # 创建游戏状态
    state = GameState(
        time_left_ms=180000,
        phase="MC",
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(x=500.0, y=300.0, yaw=0.0),
        r1_has_weapon=False,
        r1_weapon_count=0,
        r1_kfs=[],
        r1_busy=False,
        r1_last_action="",
        r1_last_action_time=0,
        r2_pose=RobotPose(x=1000.0, y=200.0, yaw=0.0),
        r2_has_weapon=False,
        r2_kfs=[],
        r2_side=TeamSide.LEFT,
        weapons=[],
        staff_rack={"staff_1": True, "staff_2": True},
        spearhead_rack={"spearhead_1": "FIST", "spearhead_2": "PALM"},
        mf_slots={
            "A": KungFuScroll(id="scroll-A", slot_name="A", kfs_type=KFSType.REAL, x=4000.0, y=1000.0),
            "B": KungFuScroll(id="scroll-B", slot_name="B", kfs_type=KFSType.FAKE, x=5000.0, y=2000.0)
        },
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(9000.0, 2000.0),
        packages=[],
        retry_requested=False
    )
    
    # 测试合规操作
    result = validate_action(state, "grab_staff", "r1")
    print(f"R1抓取棍子 (合规): {result}")
    
    # 测试违规操作
    result = validate_action(state, "grab_spearhead", "r1")
    print(f"R1抓取矛头 (违规): {result}")
    
    if not result:
        validator = RuleValidator()
        print(f"违规详情: {validator.get_violations()}")
    
    print()


def demo_path_planning():
    """演示路径规划"""
    print("=" * 60)
    print("路径规划演示")
    print("=" * 60)
    
    # 创建路径规划器
    planner = PathPlanner(2000, 2000, 100)
    
    # 设置起点和终点
    start = RobotPose(x=100.0, y=100.0, yaw=0.0)
    goal = RobotPose(x=1500.0, y=1500.0, yaw=0.0)
    
    # 规划路径
    path = planner.plan_path(start, goal)
    print(f"A*算法路径点数: {len(path)}")
    if path:
        print(f"起点: {path[0]}")
        print(f"终点: {path[-1]}")
    
    # 演示简单路径规划器
    simple_path = plan_simple_path(start, goal)
    print(f"简单路径点数: {len(simple_path)}")
    print(f"起点: {simple_path[0]}")
    print(f"终点: {simple_path[-1]}")
    
    print()


def demo_decision_making():
    """演示决策系统"""
    print("=" * 60)
    print("决策系统演示")
    print("=" * 60)
    
    # 创建游戏状态
    state = GameState(
        time_left_ms=180000,
        phase="MC",
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(x=500.0, y=300.0, yaw=0.0),
        r1_has_weapon=False,
        r1_weapon_count=0,
        r1_kfs=[],
        r1_busy=False,
        r1_last_action="",
        r1_last_action_time=0,
        r2_pose=RobotPose(x=1000.0, y=200.0, yaw=0.0),
        r2_has_weapon=False,
        r2_kfs=[],
        r2_side=TeamSide.LEFT,
        weapons=[],
        staff_rack={"staff_1": True, "staff_2": True},
        spearhead_rack={"spearhead_1": "FIST", "spearhead_2": "PALM"},
        mf_slots={
            "A": KungFuScroll(id="scroll-A", slot_name="A", kfs_type=KFSType.REAL, x=4000.0, y=1000.0),
            "B": KungFuScroll(id="scroll-B", slot_name="B", kfs_type=KFSType.FAKE, x=5000.0, y=2000.0)
        },
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
        used_weapons_area=[],
        arena_ramp=(9000.0, 2000.0),
        packages=[],
        retry_requested=False
    )
    
    # 演示决策树策略
    decision_tree_policy = DecisionTreePolicy()
    commands = decision_tree_policy.decide(state)
    print(f"决策树策略生成命令数: {len(commands)}")
    if commands:
        print(f"第一个命令: {commands[0]}")
    
    # 演示行为树策略
    behavior_tree_policy = BehaviorTreePolicy()
    commands = behavior_tree_policy.decide(state)
    print(f"行为树策略生成命令数: {len(commands)}")
    
    # 演示R1策略
    r1_policy = get_policy("r1_simple")()
    commands = r1_policy.decide(state)
    print(f"R1策略生成命令数: {len(commands)}")
    if commands:
        print(f"第一个命令: {commands[0]}")
    
    print()


def demo_data_analysis():
    """演示数据分析"""
    print("=" * 60)
    print("数据分析演示")
    print("=" * 60)
    
    # 创建分析器
    analyzer = DataAnalyzer()
    
    # 添加一些示例数据
    metrics = PerformanceMetrics(
        total_time=150000.0,
        total_distance=5000.0,
        total_commands=100,
        violations=2,
        kfs_collected=5,
        weapons_assembled=3,
        arena_kfs_placed=2
    )
    
    run_data = {
        "metrics": metrics,
        "success": True
    }
    
    analyzer.add_run_data(run_data)
    
    # 分析单次运行
    analysis = analyzer.analyze_multiple_runs("demo_strategy")
    print(f"策略名称: {analysis.strategy_name}")
    print(f"运行次数: {analysis.run_count}")
    print(f"成功率: {analysis.success_rate:.2%}")
    print(f"平均时间: {analysis.avg_metrics.total_time:.2f} ms")
    print(f"平均KFS收集: {analysis.avg_metrics.kfs_collected}")
    print(f"平均武器组装: {analysis.avg_metrics.weapons_assembled}")
    
    print()


def main():
    """主函数"""
    print("ABU Robocon 2026 - Kung Fu Quest 综合演示")
    print("=" * 60)
    
    demo_field_configuration()
    demo_logging_system()
    demo_rule_validation()
    demo_path_planning()
    demo_decision_making()
    demo_data_analysis()
    
    print("=" * 60)
    print("所有功能演示完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
