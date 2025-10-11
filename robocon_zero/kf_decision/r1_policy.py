# kf_decision/r1_policy.py
from typing import List, Tuple
from kf_decision.abc import Policy
from kf_decision.registry import register
from kf_decision.types import GameState, KFSType
from commands import RobotCommand, CmdR1StatusQuery, CmdR1PathSuggestion, CmdR1ViolationWarning, CmdR1StrategySuggestion


@register("r1_simple")
class R1SimplePolicy(Policy):
    """R1简单决策策略"""
    
    def decide(self, state: GameState) -> List[RobotCommand]:
        """为R1生成决策命令"""
        cmds: List[RobotCommand] = []
        
        # 根据当前阶段生成不同的决策
        if state.phase == "MC":
            cmds.extend(self._mc_phase_decisions(state))
        elif state.phase == "MF":
            cmds.extend(self._mf_phase_decisions(state))
        elif state.phase == "ARENA":
            cmds.extend(self._arena_phase_decisions(state))
            
        return cmds
    
    def _mc_phase_decisions(self, state: GameState) -> List[RobotCommand]:
        """武器组装阶段的决策"""
        cmds: List[RobotCommand] = []
        
        # 检查是否需要组装武器
        if not state.r1_has_weapon:
            # 建议R1去拿棍子
            cmds.append(CmdR1StrategySuggestion(
                strategy="grab_staff",
                details="建议R1前往Staff Rack抓取棍子"
            ))
        else:
            # 检查是否已经拿到武器
            if state.r1_weapon_count > 0:
                # 建议R1准备与R2协作组装武器
                cmds.append(CmdR1StrategySuggestion(
                    strategy="prepare_assemble",
                    details="建议R1准备与R2协作组装武器"
                ))
        
        return cmds
    
    def _mf_phase_decisions(self, state: GameState) -> List[RobotCommand]:
        """梅花林阶段的决策"""
        cmds: List[RobotCommand] = []
        
        # 检查R1是否需要收集KFS
        if len(state.r1_kfs) < 3:  # 假设最多需要3个R1 KFS
            # 建议路径规划
            waypoints = self._plan_r1_kfs_collection_path(state)
            if waypoints:
                cmds.append(CmdR1PathSuggestion(
                    waypoints=waypoints
                ))
        
        # 检查是否收集了足够的KFS
        if len(state.r1_kfs) >= 1:  # 至少需要1个KFS才能进入竞技场
            cmds.append(CmdR1StrategySuggestion(
                strategy="prepare_arena",
                details="R1已收集足够KFS，准备进入竞技场"
            ))
            
        return cmds
    
    def _arena_phase_decisions(self, state: GameState) -> List[RobotCommand]:
        """竞技场阶段的决策"""
        cmds: List[RobotCommand] = []
        
        # 检查R1是否携带武器和KFS
        if state.r1_has_weapon and state.r1_weapon_count > 0 and len(state.r1_kfs) > 0:
            # 建议放置KFS策略
            cmds.append(CmdR1StrategySuggestion(
                strategy="place_kfs",
                details="建议R1将R1 KFS放置到井字棋底部行"
            ))
        elif state.r1_has_weapon and state.r1_weapon_count > 0:
            # 建议攻击策略
            cmds.append(CmdR1StrategySuggestion(
                strategy="attack",
                details="建议R1使用武器攻击对手的KFS"
            ))
            
        return cmds
    
    def _plan_r1_kfs_collection_path(self, state: GameState) -> List[Tuple[float, float]]:
        """为R1规划KFS收集路径"""
        # 这里只是一个简单的示例，实际实现需要更复杂的路径规划算法
        waypoints: List[Tuple[float, float]] = []
        
        # 根据MF slots中的R1 KFS位置规划路径
        for slot_name, kfs in state.mf_slots.items():
            if kfs.kfs_type == KFSType.REAL and kfs.is_grabbed_by is None:
                # 这里需要根据slot_name计算实际坐标
                # 暂时使用占位符坐标
                waypoints.append((1000.0, 1000.0))
                
        return waypoints
    
    def _check_violations(self, state: GameState) -> List[RobotCommand]:
        """检查可能的违规行为"""
        cmds: List[RobotCommand] = []
        
        # 检查R1是否进入了错误区域
        if self._is_r1_in_opponent_area(state):
            cmds.append(CmdR1ViolationWarning(
                violation_type="enter_opponent_area",
                details="R1进入了对手区域"
            ))
            
        # 检查R1是否触碰了错误类型的KFS
        if self._is_r1_touching_wrong_kfs(state):
            cmds.append(CmdR1ViolationWarning(
                violation_type="touch_wrong_kfs",
                details="R1触碰了错误类型的KFS"
            ))
            
        return cmds
    
    def _is_r1_in_opponent_area(self, state: GameState) -> bool:
        """检查R1是否在对手区域"""
        # 这里需要根据实际场地尺寸和R1位置判断
        # 暂时返回False
        return False
    
    def _is_r1_touching_wrong_kfs(self, state: GameState) -> bool:
        """检查R1是否触碰了错误类型的KFS"""
        # R1只能触碰R1 KFS，不能触碰R2 KFS或Fake KFS
        # 这里需要根据实际实现判断
        # 暂时返回False
        return False
