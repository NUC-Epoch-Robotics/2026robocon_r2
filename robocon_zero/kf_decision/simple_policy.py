# kf_decision/simple_policy.py
from typing import List
from kf_decision.abc import Policy
from kf_decision.registry import register
from kf_decision.types import GameState
from commands import RobotCommand, CmdGrab, CmdVel, CmdPlace, CmdNavTo

@register("simple")
class SimplePolicy(Policy):
    def decide(self, state: GameState) -> List[RobotCommand]:
        cmds: List[RobotCommand] = []

        # 第一阶段：MC准备
        if state.phase == "MC":
            if not state.r2_has_weapon:
                # R2 引导 R1 组装武器
                print("[R2] 引导 R1 组装武器...")
                # 这里发送命令给底层去驱动 R1（模拟）
                cmds.extend([
                    CmdGrab(slot="weapon_part_1"),
                    CmdGrab(slot="weapon_part_2"),
                    CmdPlace(slot="assemble_station")
                ])
                return cmds
            else:
                # 武器已就绪，准备进入下一阶段
                print("[R2] 武器就绪，准备前往 MF")
                cmds.append(CmdNavTo(x=1000, y=1000))  # 假设 MF 入口坐标
                return cmds

        # 第二阶段：MF收集KFS
        elif state.phase == "MF":
            print("[R2] 进入 MF 阶段，开始搜索 KFS")
            # 示例逻辑：前往第一个 slot
            cmds.append(CmdNavTo(x=1500, y=1500))  # Slot A 坐标
            cmds.append(CmdGrab(slot="A"))
            return cmds

        # 第三阶段：ARENA对战
        elif state.phase == "ARENA":
            print("[R2] 进入 ARENA 阶段，开始占位策略")
            # 占据中心位置
            cmds.append(CmdNavTo(x=2000, y=2000))  # Arena 中心
            cmds.append(CmdPlace(slot="center"))
            return cmds

        return cmds
