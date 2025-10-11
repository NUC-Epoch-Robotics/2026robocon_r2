# kf_decision/abc.py
from abc import ABC, abstractmethod
from typing import List
from commands import RobotCommand
from kf_decision.types import GameState


class Policy(ABC):
    @abstractmethod
    def decide(self, state: GameState) -> List[RobotCommand]:
        """根据当前游戏状态，决定下一步要执行的一组命令"""
        pass
