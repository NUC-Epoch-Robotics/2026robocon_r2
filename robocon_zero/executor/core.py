# executor/core.py
from __future__ import annotations
import time
from dataclasses import dataclass, field
from typing import List

from commands import RobotCommand, CmdVel, CmdGrab, CmdPlace, CmdDone, CmdR1StatusQuery, CmdR1PathSuggestion, CmdR1ViolationWarning, CmdR1StrategySuggestion
# 用你现在真实存在的类
from kf_decision.types import GameState, RobotPose

@dataclass
class _Step:
    ts: float
    state: GameState
    command: RobotCommand


class Executor:
    # ---- 基本状态管理+时间追踪 ----
    def __init__(self, init: GameState, robot_dt: float = 0.2):
        self.state: GameState = init
        self.history: List[_Step] = []
        self._robot = FakeRobot(dt=robot_dt)

        self._total_mm = 0.0
        self._cmd_cnt = 0
        
        # 初始化武器列表（如果为空）
        if not self.state.weapons:
            # 创建默认武器
            from kf_decision.types import Weapon
            self.state.weapons = [
                Weapon(id="weapon_1", weapon_type="FIST", is_assembled=False, is_used=False),
                Weapon(id="weapon_2", weapon_type="PALM", is_assembled=False, is_used=False),
                Weapon(id="weapon_3", weapon_type="SPEAR", is_assembled=False, is_used=False),
            ]
        
        # 初始化棍子架状态（如果为空）
        if not self.state.staff_rack:
            self.state.staff_rack = {"staff_1": True, "staff_2": True, "staff_3": True, "staff_4": True}
        
        # 初始化矛头架状态（如果为空）
        if not self.state.spearhead_rack:
            self.state.spearhead_rack = {
                "spearhead_1": "FIST", 
                "spearhead_2": "PALM", 
                "spearhead_3": "SPEAR",
                "spearhead_4": "FIST", 
                "spearhead_5": "PALM", 
                "spearhead_6": "SPEAR"
            }

    # ---- 外部接口 ----
    # 每次 step 都传入一个命令，更新状态并"发到底层"
    # 主执行入口
    def step(self, cmd: RobotCommand):
        self._apply(cmd)
        self._robot.run(cmd)
        self.history.append(_Step(time.perf_counter(), self.state, cmd))
        self._cmd_cnt += 1

    # 判停条件
    def is_done(self) -> bool:
        # 先随意给一个简单判停：时间到 或 收到 CmdDone
        return self.state.time_left_ms <= 0

    def total_distance(self) -> int:
        return int(self._total_mm)

    def command_count(self) -> int:
        return self._cmd_cnt

    # ---- 内部 ----
    # 简单模拟执行命令对 GameState 的影响
    def _apply(self, cmd: RobotCommand):
        pose = self.state.r1_pose
        match cmd:
            case CmdVel(vx, vy, _):
                dx, dy = vx * 0.2, vy * 0.2
                new_x, new_y = pose.x + dx, pose.y + dy
                self._total_mm += (dx * dx + dy * dy) ** 0.5
                self.state.r1_pose.x = new_x
                self.state.r1_pose.y = new_y

            case CmdGrab(slot):
                # slot 是端头类型字符串，比如 "weapon_part_1"
                # 这里简单记录抓取了哪个类型的端头
                print(f"[INFO] 抓取端头类型: {slot}")
                # 将抓取的KFS ID添加到列表中
                self.state.r2_kfs.append(slot)
            case CmdPlace(slot):
                # slot 是放置位置，比如 "assemble_station"
                print(f"[INFO] 放置到位置: {slot}")
                pass

            case CmdDone():
                self.state.time_left_ms = 0
                
            # R1专用命令的处理
            case CmdR1StatusQuery():
                # 状态查询命令，不需要改变游戏状态
                print(f"[INFO] R1状态查询")
                pass
                
            case CmdR1PathSuggestion(waypoints):
                # 路径建议命令，更新R1的路径信息
                print(f"[INFO] R1路径建议: {waypoints}")
                # 这里可以更新R1的路径信息，但暂时不改变游戏状态
                
            case CmdR1ViolationWarning(violation_type, details):
                # 违规警告命令，增加违规计数
                print(f"[WARNING] R1违规警告: {violation_type} - {details}")
                self.state.violations += 1
                
            case CmdR1StrategySuggestion(strategy, details):
                # 策略建议命令，不需要改变游戏状态
                print(f"[INFO] R1策略建议: {strategy} - {details}")
                pass

class FakeRobot:
    def __init__(self, dt: float = 0.2):
        self.dt = dt

    def run(self, cmd: RobotCommand):
        print(f'>>> dt = {self.dt} 秒', flush=True)
        match cmd:
            case CmdVel(vx, vy, omega):
                print(f"[FakeRobot] 移动  vx={vx:.2f}  vy={vy:.2f}  ω={omega:.2f}")
            case CmdGrab(slot):
                print(f"[FakeRobot] 抓取  槽位={slot}")
            case CmdPlace(slot):
                print(f"[FakeRobot] 放置  槽位={slot}")
            case CmdDone():
                print("[FakeRobot] 任务完成，停机！")
            # R1专用命令的处理
            case CmdR1StatusQuery():
                print("[FakeRobot] R1状态查询")
            case CmdR1PathSuggestion(waypoints):
                print(f"[FakeRobot] R1路径建议: {waypoints}")
            case CmdR1ViolationWarning(violation_type, details):
                print(f"[FakeRobot] R1违规警告: {violation_type} - {details}")
            case CmdR1StrategySuggestion(strategy, details):
                print(f"[FakeRobot] R1策略建议: {strategy} - {details}")
        time.sleep(self.dt)
