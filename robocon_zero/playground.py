# playground.py
from kf_decision.types import GameState, RobotPose, TeamSide
from kf_decision import get_policy

def fake_run():
    # 获取已注册的策略
    SimplePolicy = get_policy("simple")
    policy = SimplePolicy()
    
    state = GameState(
        time_left_ms=180000,
        phase="MC",
        retry_timer=0,
        violations=0,
        r1_pose=RobotPose(0, 0, 0),
        r1_has_weapon=False,
        r1_busy=False,
        r2_pose=RobotPose(0, 0, 0),
        r2_has_weapon=False,
        r2_kfs=[],
        r2_side=TeamSide.LEFT,
        assembled_weapons=[],
        mf_slots={},
        arena_grid=[],
        packages=[],
    )
    while True:
        acts = policy.decide(state)
        print("\n=== 当前状态 ===")
        print(f"phase={state.phase}  weapon={state.r2_has_weapon}  r2_kfs={state.r2_kfs}")
        print("=== 决策动作 ===")
        for a in acts:
            print(" -", a)

        cmd = input("\n输入回车执行第一个动作 (q退出)> ").strip()
        if cmd == 'q':
            break
        # 极简"执行"：只把状态改掉
        if acts:
            a = acts[0]
            # 这里需要根据实际的命令类型来更新状态
            # 为了简化，我们只更新一些基本状态
            if state.phase == "MC":
                if not state.r2_has_weapon:
                    state.r2_has_weapon = True
                else:
                    state.phase = "MF"
            elif state.phase == "MF":
                state.r2_kfs.append("A")  # 模拟抓取了一个KFS
                state.phase = "ARENA"
            elif state.phase == "ARENA":
                state.phase = "FINISHED"

if __name__ == "__main__":
    fake_run()