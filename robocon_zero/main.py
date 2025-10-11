from kf_decision.types import GameState, RobotPose, TeamSide, KungFuScroll, KFSType, ArenaGridCell
from kf_decision.policy import pick_policy
from kf_decision import get_policy
from executor.core import FakeRobot, Executor
from commands import CmdVel, CmdGrab, CmdPlace, CmdDone, CmdR1StrategySuggestion

def build_task():
    mc, mf = ["1","2","3"], ["4","5"]
    return pick_policy(mc, mf)

def demo_r1_policy():
    """演示R1策略的使用"""
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
    cmds = policy.decide(state)
    
    return cmds

if __name__ == "__main__":
    # 运行原来的任务构建示例
    print("运行原来的任务构建示例:")
    bot = FakeRobot()
    for cmd in build_task():
        bot.run(cmd)
    bot.run(CmdDone(slot="done"))
    
    print("\n" + "="*50)
    print("演示R1策略:")
    # 演示R1策略
    r1_cmds = demo_r1_policy()
    for cmd in r1_cmds: 
        bot.run(cmd)
