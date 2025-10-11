# test_visualizer.py
"""
测试场地可视化工具
"""

import sys
import os

# 添加项目根目录到 Python 路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from kf_decision.types import GameState, RobotPose, KungFuScroll, KFSType, TeamSide, Weapon, ArenaGridCell, Package
from visualization.field_visualizer import visualize_game_state
from config.field_config import *

def create_test_game_state():
    """创建一个测试游戏状态"""
    # 创建机器人位置
    r1_pose = RobotPose(x=1000, y=1000, yaw=0.0)
    r2_pose = RobotPose(x=5000, y=3000, yaw=0.0)
    
    # 创建KFS
    kfs_a = KungFuScroll(id="A", slot_name="A", kfs_type=KFSType.REAL, x=4000, y=2000)
    kfs_b = KungFuScroll(id="B", slot_name="B", kfs_type=KFSType.FAKE, x=5000, y=2500)
    
    # 创建武器
    weapon1 = Weapon(id="W1", weapon_type="FIST", is_assembled=True, is_used=False, carried_by="r1", x=1500, y=1500)
    weapon2 = Weapon(id="W2", weapon_type="SPEAR", is_assembled=True, is_used=False, carried_by="r2", x=5500, y=3500)
    
    # 创建竞技场网格
    arena_grid = [
        [ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None), 
         ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None), 
         ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None)],
        [ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None), 
         ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None), 
         ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None)],
        [ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None), 
         ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None), 
         ArenaGridCell(occupied_by=None, kfs_id=None, kfs_type=None)]
    ]
    
    # 创建游戏状态
    state = GameState(
        time_left_ms=180000,
        phase="MF",
        retry_timer=0,
        violations=0,
        r1_pose=r1_pose,
        r1_has_weapon=True,
        r1_weapon_count=1,
        r1_kfs=["A"],
        r1_busy=False,
        r1_last_action="",
        r1_last_action_time=0,
        r2_pose=r2_pose,
        r2_has_weapon=True,
        r2_kfs=["B"],
        r2_side=TeamSide.LEFT,
        weapons=[weapon1, weapon2],
        staff_rack={"S1": True, "S2": False},
        spearhead_rack={"SH1": "TYPE_A", "SH2": "TYPE_B"},
        mf_slots={"A": kfs_a, "B": kfs_b},
        mf_r1_path=[],
        mf_r2_path=[],
        arena_grid=arena_grid,
        used_weapons_area=[],
        arena_ramp=(8000, 2000),
        packages=[],
        retry_requested=False,
        retry_robot=None
    )
    
    return state

def main():
    """主函数"""
    print("创建测试游戏状态...")
    state = create_test_game_state()
    
    print("运行场地可视化工具...")
    visualize_game_state(state)
    
    print("可视化完成。")

if __name__ == "__main__":
    main()
