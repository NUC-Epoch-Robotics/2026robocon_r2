# utils/rule_validator.py
"""
规则验证系统
根据ABU Robocon 2026规则书验证操作是否合规
"""

from typing import List, Tuple, Optional
from kf_decision.types import GameState, KFSType, RobotPose
from config.field_config import *
from utils.logger import log_violation


class RuleValidator:
    """规则验证器"""
    
    def __init__(self):
        self.violations: List[str] = []
        
    def validate_all(self, state: GameState, action: str, robot: str) -> bool:
        """验证所有相关规则"""
        self.violations.clear()
        
        # 根据不同阶段和动作验证不同规则
        if state.phase == "MC":
            self._validate_mc_rules(state, action, robot)
        elif state.phase == "MF":
            self._validate_mf_rules(state, action, robot)
        elif state.phase == "ARENA":
            self._validate_arena_rules(state, action, robot)
            
        # 验证通用规则
        self._validate_general_rules(state, action, robot)
        
        # 记录违规行为
        for violation in self.violations:
            log_violation(violation, f"Action: {action}, Robot: {robot}")
            
        return len(self.violations) == 0
        
    def _validate_mc_rules(self, state: GameState, action: str, robot: str):
        """验证武器组装阶段规则"""
        if robot == "r1":
            # R1在MC阶段的规则
            if action == "grab_spearhead":
                self.violations.append("R1_CANNOT_GRAB_SPEARHEAD_IN_MC")
                
            if action == "enter_mf" and not state.r1_has_weapon:
                self.violations.append("R1_MUST_HAVE_WEAPON_TO_ENTER_MF")
                
        elif robot == "r2":
            # R2在MC阶段的规则
            if action == "grab_staff":
                self.violations.append("R2_CANNOT_GRAB_STAFF_IN_MC")
                
            if action == "enter_mf" and state.r1_pose.x >= MC_WIDTH:  # 简化的R1是否已离开判断
                self.violations.append("R2_CANNOT_ENTER_MF_BEFORE_R1_LEAVES")
                
    def _validate_mf_rules(self, state: GameState, action: str, robot: str):
        """验证梅花林阶段规则"""
        if robot == "r1":
            # R1在MF阶段的规则
            if action == "grab_kfs":
                # 检查是否在正确的区域
                if not self._is_r1_in_mf_perimeter(state.r1_pose):
                    self.violations.append("R1_CANNOT_GRAB_KFS_OUTSIDE_MF_PERIMETER")
                    
            if action == "grab_r2_kfs":
                self.violations.append("R1_CANNOT_GRAB_R2_KFS")
                
            if action == "grab_fake_kfs":
                self.violations.append("R1_CANNOT_GRAB_FAKE_KFS")
                
            if action == "enter_arena" and len(state.r1_kfs) < 1:
                self.violations.append("R1_MUST_HAVE_AT_LEAST_ONE_KFS_TO_ENTER_ARENA")
                
        elif robot == "r2":
            # R2在MF阶段的规则
            if action == "grab_kfs":
                # 检查是否在正确的区域
                if not self._is_r2_in_mf_area(state.r2_pose):
                    self.violations.append("R2_CANNOT_GRAB_KFS_OUTSIDE_MF_AREA")
                    
                # 检查是否从正确的入口进入
                if not self._is_r2_in_entrance_zone(state.r2_pose) and len(state.r2_kfs) == 0:
                    self.violations.append("R2_MUST_START_FROM_ENTRANCE_ZONE")
                    
            if action == "grab_r1_kfs":
                self.violations.append("R2_CANNOT_GRAB_R1_KFS")
                
            if action == "grab_fake_kfs":
                self.violations.append("R2_CANNOT_GRAB_FAKE_KFS")
                
            if action == "enter_arena" and len(state.r2_kfs) < 1:
                self.violations.append("R2_MUST_HAVE_AT_LEAST_ONE_KFS_TO_ENTER_ARENA")
                
            # 检查相邻规则
            if action == "grab_kfs" and not self._is_r2_adjacent_to_kfs(state.r2_pose):
                self.violations.append("R2_CAN_ONLY_GRAB_ADJACENT_KFS")
                
    def _validate_arena_rules(self, state: GameState, action: str, robot: str):
        """验证竞技场阶段规则"""
        if robot == "r1":
            # R1在竞技场阶段的规则
            if action == "use_weapon" and state.r1_has_weapon:
                # 检查武器是否已使用
                # 这里需要更复杂的逻辑来跟踪武器使用状态
                pass
                
        elif robot == "r2":
            # R2在竞技场阶段的规则
            pass
            
    def _validate_general_rules(self, state: GameState, action: str, robot: str):
        """验证通用规则"""
        # 检查是否进入对手区域
        if self._is_in_opponent_area(state.r1_pose if robot == "r1" else state.r2_pose, robot):
            self.violations.append(f"{robot.upper()}_ENTERED_OPPONENT_AREA")
            
        # 检查尺寸限制
        if not self._validate_size_limits(state, robot):
            self.violations.append(f"{robot.upper()}_SIZE_LIMIT_EXCEEDED")
            
    def _is_r1_in_mf_perimeter(self, pose: RobotPose) -> bool:
        """检查R1是否在梅花林外围路径上"""
        # 简化的实现，实际需要更精确的区域判断
        return MC_WIDTH <= pose.x <= (MC_WIDTH + MF_WIDTH) and 0 <= pose.y <= MF_HEIGHT
        
    def _is_r2_in_mf_area(self, pose: RobotPose) -> bool:
        """检查R2是否在梅花林区域内"""
        # 简化的实现
        return MC_WIDTH <= pose.x <= (MC_WIDTH + MF_WIDTH) and 0 <= pose.y <= MF_HEIGHT
        
    def _is_r2_in_entrance_zone(self, pose: RobotPose) -> bool:
        """检查R2是否在入口区域"""
        # 简化的实现，实际需要根据具体入口区域定义
        entrance_y = MF_BLOCK_SIZE * 3  # 假设前3个区块是入口区域
        return MC_WIDTH <= pose.x <= (MC_WIDTH + MF_WIDTH) and 0 <= pose.y <= entrance_y
        
    def _is_r2_adjacent_to_kfs(self, pose: RobotPose) -> bool:
        """检查R2是否与KFS相邻"""
        # 简化的实现，实际需要根据具体KFS位置判断
        return True  # 暂时返回True
        
    def _is_in_opponent_area(self, pose: RobotPose, robot: str) -> bool:
        """检查是否进入对手区域"""
        # 简化的实现
        if robot == "r1":
            # R1的对手区域在场地右侧
            return pose.x > FIELD_WIDTH / 2
        else:
            # R2的对手区域在场地左侧
            return pose.x < FIELD_WIDTH / 2
            
    def _validate_size_limits(self, state: GameState, robot: str) -> bool:
        """验证尺寸限制"""
        if robot == "r1":
            # 检查R1尺寸
            return (state.r1_pose.x <= R1_MAX_SIZE_X and 
                   state.r1_pose.y <= R1_MAX_SIZE_Y and 
                   state.r1_pose.yaw <= R1_MAX_SIZE_Z)  # 简化实现
        else:
            # 检查R2尺寸
            return (state.r2_pose.x <= R2_MAX_SIZE_X and 
                   state.r2_pose.y <= R2_MAX_SIZE_Y and 
                   state.r2_pose.yaw <= R2_MAX_SIZE_Z)  # 简化实现
                   
    def get_violations(self) -> List[str]:
        """获取违规列表"""
        return self.violations[:]
        

# 全局规则验证器实例
validator = RuleValidator()


def validate_action(state: GameState, action: str, robot: str) -> bool:
    """验证动作是否合规"""
    return validator.validate_all(state, action, robot)
