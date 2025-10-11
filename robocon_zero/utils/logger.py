# utils/logger.py
"""
日志和调试工具
"""

import logging
import time
import os
from datetime import datetime
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, asdict
from kf_decision.types import GameState, RobotPose


@dataclass
class LogEntry:
    """日志条目"""
    timestamp: float
    level: str
    message: str
    module: str
    game_state: Optional[Dict[str, Any]] = None


class RoboconLogger:
    """机器人竞赛日志记录器"""
    
    def __init__(self, name: str = "robocon", level: int = logging.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)
        
        # 创建控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)
        
        # 创建文件处理器
        file_handler = logging.FileHandler(f"logs/robocon_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        file_handler.setLevel(level)
        
        # 创建格式器
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        console_handler.setFormatter(formatter)
        file_handler.setFormatter(formatter)
        
        # 添加处理器
        self.logger.addHandler(console_handler)
        self.logger.addHandler(file_handler)
        
        # 日志条目存储
        self.log_entries: List[LogEntry] = []
        
    def debug(self, message: str, module: str = "main", game_state: Optional[GameState] = None):
        """记录调试信息"""
        self.logger.debug(message)
        self._add_log_entry("DEBUG", message, module, game_state)
        
    def info(self, message: str, module: str = "main", game_state: Optional[GameState] = None):
        """记录一般信息"""
        self.logger.info(message)
        self._add_log_entry("INFO", message, module, game_state)
        
    def warning(self, message: str, module: str = "main", game_state: Optional[GameState] = None):
        """记录警告信息"""
        self.logger.warning(message)
        self._add_log_entry("WARNING", message, module, game_state)
        
    def error(self, message: str, module: str = "main", game_state: Optional[GameState] = None):
        """记录错误信息"""
        self.logger.error(message)
        self._add_log_entry("ERROR", message, module, game_state)
        
    def _add_log_entry(self, level: str, message: str, module: str, game_state: Optional[GameState] = None):
        """添加日志条目到存储"""
        entry = LogEntry(
            timestamp=time.time(),
            level=level,
            message=message,
            module=module,
            game_state=self._serialize_game_state(game_state) if game_state else None
        )
        self.log_entries.append(entry)
        
    def _serialize_game_state(self, state: GameState) -> Dict[str, Any]:
        """序列化游戏状态用于日志记录"""
        return {
            "time_left_ms": state.time_left_ms,
            "phase": state.phase,
            "violations": state.violations,
            "r1_pose": {
                "x": state.r1_pose.x,
                "y": state.r1_pose.y,
                "yaw": state.r1_pose.yaw
            },
            "r2_pose": {
                "x": state.r2_pose.x,
                "y": state.r2_pose.y,
                "yaw": state.r2_pose.yaw
            },
            "r1_has_weapon": state.r1_has_weapon,
            "r2_has_weapon": state.r2_has_weapon,
            "r1_kfs_count": len(state.r1_kfs),
            "r2_kfs_count": len(state.r2_kfs)
        }
        
    def get_logs(self, level: Optional[str] = None) -> List[LogEntry]:
        """获取日志条目"""
        if level:
            return [entry for entry in self.log_entries if entry.level == level]
        return self.log_entries[:]
        
    def save_logs(self, filename: str):
        """保存日志到文件"""
        import json
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump([asdict(entry) for entry in self.log_entries], f, indent=2, ensure_ascii=False)


# 全局日志记录器实例
logger = RoboconLogger()


def log_robot_pose(pose: RobotPose, robot_name: str, action: str = "position"):
    """记录机器人位置"""
    logger.info(
        f"{robot_name} {action} at ({pose.x:.2f}, {pose.y:.2f}, {pose.yaw:.2f})", 
        module="robot_position"
    )


def log_game_state(state: GameState, phase: str = "unknown"):
    """记录游戏状态"""
    logger.info(
        f"Game state - Phase: {state.phase}, Time: {state.time_left_ms}ms, Violations: {state.violations}",
        module="game_state",
        game_state=state
    )


def log_violation(violation_type: str, details: str):
    """记录违规行为"""
    logger.warning(
        f"Violation: {violation_type} - {details}",
        module="violation_detector"
    )
