# algorithms/decision_tree.py
"""
决策树算法
实现基于规则的决策树用于机器人行为决策
"""

from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass
from kf_decision.types import GameState, RobotPose
from kf_decision.abc import Policy
from commands import RobotCommand


@dataclass
class DecisionNode:
    """决策树节点"""
    name: str
    condition: Optional[Callable[[GameState], bool]] = None
    action: Optional[Callable[[GameState], List[RobotCommand]]] = None
    children: List['DecisionNode'] = None
    parent: Optional['DecisionNode'] = None
    
    def __post_init__(self):
        if self.children is None:
            self.children = []


class DecisionTreePolicy(Policy):
    """基于决策树的策略"""
    
    def __init__(self):
        self.root = self._build_decision_tree()
        
    def decide(self, state: GameState) -> List[RobotCommand]:
        """根据决策树做出决策"""
        return self._traverse_tree(self.root, state)
        
    def _traverse_tree(self, node: DecisionNode, state: GameState) -> List[RobotCommand]:
        """遍历决策树"""
        # 如果有动作函数，执行它
        if node.action:
            return node.action(state)
            
        # 如果有条件函数，检查条件
        if node.condition:
            if node.condition(state):
                # 条件为真，遍历子节点
                for child in node.children:
                    result = self._traverse_tree(child, state)
                    if result:
                        return result
            else:
                # 条件为假，返回空列表
                return []
                
        # 如果没有条件和动作，继续遍历子节点
        for child in node.children:
            result = self._traverse_tree(child, state)
            if result:
                return result
                
        return []
        
    def _build_decision_tree(self) -> DecisionNode:
        """构建决策树"""
        # 创建根节点
        root = DecisionNode("Root")
        
        # 武器组装阶段决策
        mc_node = DecisionNode(
            "MC_Phase",
            condition=lambda state: state.phase == "MC"
        )
        root.children.append(mc_node)
        
        # R1在MC阶段的决策
        r1_mc_node = DecisionNode(
            "R1_MC",
            condition=lambda state: not state.r1_has_weapon
        )
        mc_node.children.append(r1_mc_node)
        
        r1_mc_action = DecisionNode(
            "Grab_Staff",
            action=self._grab_staff_action
        )
        r1_mc_node.children.append(r1_mc_action)
        
        # R1已有武器的决策
        r1_mc_has_weapon_node = DecisionNode(
            "R1_MC_Has_Weapon",
            condition=lambda state: state.r1_has_weapon and state.r1_weapon_count > 0
        )
        mc_node.children.append(r1_mc_has_weapon_node)
        
        r1_mc_prepare_assemble = DecisionNode(
            "Prepare_Assemble",
            action=self._prepare_assemble_action
        )
        r1_mc_has_weapon_node.children.append(r1_mc_prepare_assemble)
        
        # 梅花林阶段决策
        mf_node = DecisionNode(
            "MF_Phase",
            condition=lambda state: state.phase == "MF"
        )
        root.children.append(mf_node)
        
        # R1在MF阶段收集KFS的决策
        r1_mf_collect_node = DecisionNode(
            "R1_MF_Collect",
            condition=lambda state: len(state.r1_kfs) < 3
        )
        mf_node.children.append(r1_mf_collect_node)
        
        r1_mf_collect_action = DecisionNode(
            "Collect_KFS",
            action=self._collect_kfs_action
        )
        r1_mf_collect_node.children.append(r1_mf_collect_action)
        
        # R1在MF阶段准备进入竞技场的决策
        r1_mf_prepare_arena_node = DecisionNode(
            "R1_MF_Prepare_Arena",
            condition=lambda state: len(state.r1_kfs) >= 1
        )
        mf_node.children.append(r1_mf_prepare_arena_node)
        
        r1_mf_prepare_arena_action = DecisionNode(
            "Prepare_Arena",
            action=self._prepare_arena_action
        )
        r1_mf_prepare_arena_node.children.append(r1_mf_prepare_arena_action)
        
        # 竞技场阶段决策
        arena_node = DecisionNode(
            "Arena_Phase",
            condition=lambda state: state.phase == "ARENA"
        )
        root.children.append(arena_node)
        
        # R1在竞技场放置KFS的决策
        r1_arena_place_node = DecisionNode(
            "R1_Arena_Place",
            condition=lambda state: state.r1_has_weapon and state.r1_weapon_count > 0 and len(state.r1_kfs) > 0
        )
        arena_node.children.append(r1_arena_place_node)
        
        r1_arena_place_action = DecisionNode(
            "Place_KFS",
            action=self._place_kfs_action
        )
        r1_arena_place_node.children.append(r1_arena_place_action)
        
        # R1在竞技场攻击的决策
        r1_arena_attack_node = DecisionNode(
            "R1_Arena_Attack",
            condition=lambda state: state.r1_has_weapon and state.r1_weapon_count > 0
        )
        arena_node.children.append(r1_arena_attack_node)
        
        r1_arena_attack_action = DecisionNode(
            "Attack",
            action=self._attack_action
        )
        r1_arena_attack_node.children.append(r1_arena_attack_action)
        
        return root
        
    def _grab_staff_action(self, state: GameState) -> List[RobotCommand]:
        """抓取棍子动作"""
        from commands import CmdR1StrategySuggestion
        return [CmdR1StrategySuggestion(
            strategy="grab_staff",
            details="建议R1前往Staff Rack抓取棍子"
        )]
        
    def _prepare_assemble_action(self, state: GameState) -> List[RobotCommand]:
        """准备组装动作"""
        from commands import CmdR1StrategySuggestion
        return [CmdR1StrategySuggestion(
            strategy="prepare_assemble",
            details="建议R1准备与R2协作组装武器"
        )]
        
    def _collect_kfs_action(self, state: GameState) -> List[RobotCommand]:
        """收集KFS动作"""
        from commands import CmdR1PathSuggestion
        # 这里应该调用路径规划算法
        waypoints = [(1000.0, 1000.0)]  # 占位符
        return [CmdR1PathSuggestion(waypoints=waypoints)]
        
    def _prepare_arena_action(self, state: GameState) -> List[RobotCommand]:
        """准备进入竞技场动作"""
        from commands import CmdR1StrategySuggestion
        return [CmdR1StrategySuggestion(
            strategy="prepare_arena",
            details="R1已收集足够KFS，准备进入竞技场"
        )]
        
    def _place_kfs_action(self, state: GameState) -> List[RobotCommand]:
        """放置KFS动作"""
        from commands import CmdR1StrategySuggestion
        return [CmdR1StrategySuggestion(
            strategy="place_kfs",
            details="建议R1将R1 KFS放置到井字棋底部行"
        )]
        
    def _attack_action(self, state: GameState) -> List[RobotCommand]:
        """攻击动作"""
        from commands import CmdR1StrategySuggestion
        return [CmdR1StrategySuggestion(
            strategy="attack",
            details="建议R1使用武器攻击对手的KFS"
        )]


class BehaviorTreePolicy(Policy):
    """行为树策略（更高级的决策系统）"""
    
    def __init__(self):
        self.behaviors = [
            self._check_violations,
            self._check_phase_transitions,
            self._execute_phase_behavior
        ]
        
    def decide(self, state: GameState) -> List[RobotCommand]:
        """执行行为树"""
        commands = []
        for behavior in self.behaviors:
            result = behavior(state)
            if result:
                commands.extend(result)
        return commands
        
    def _check_violations(self, state: GameState) -> List[RobotCommand]:
        """检查违规行为"""
        # 这里应该调用规则验证系统
        return []
        
    def _check_phase_transitions(self, state: GameState) -> List[RobotCommand]:
        """检查阶段转换"""
        # 根据游戏状态决定是否需要转换阶段
        return []
        
    def _execute_phase_behavior(self, state: GameState) -> List[RobotCommand]:
        """执行当前阶段的行为"""
        if state.phase == "MC":
            return self._execute_mc_behavior(state)
        elif state.phase == "MF":
            return self._execute_mf_behavior(state)
        elif state.phase == "ARENA":
            return self._execute_arena_behavior(state)
        return []
        
    def _execute_mc_behavior(self, state: GameState) -> List[RobotCommand]:
        """执行MC阶段行为"""
        # 实现MC阶段的具体行为
        return []
        
    def _execute_mf_behavior(self, state: GameState) -> List[RobotCommand]:
        """执行MF阶段行为"""
        # 实现MF阶段的具体行为
        return []
        
    def _execute_arena_behavior(self, state: GameState) -> List[RobotCommand]:
        """执行竞技场阶段行为"""
        # 实现竞技场阶段的具体行为
        return []


# 注册策略
from kf_decision.registry import register

@register("decision_tree")
def create_decision_tree_policy():
    return DecisionTreePolicy()
    
@register("behavior_tree")
def create_behavior_tree_policy():
    return BehaviorTreePolicy()
