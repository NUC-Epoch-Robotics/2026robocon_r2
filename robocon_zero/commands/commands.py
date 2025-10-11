from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Any, Tuple

@dataclass
class RobotCommand:
    """所有机器人指令的基类"""
    name: str = field(default="RobotCommand", init=False)

    # --------- 供快照测试读的4个统一字段 ---------
    @property
    def type(self) -> str:
        """命令类型字符串，默认用类名小写"""
        return self.__class__.__name__.lower()

    @property
    def actor(self) -> str:
        """执行者，默认机器人本体"""
        return "robot"

    @property
    def target(self) -> Any:
        """操作对象，子类按需覆盖"""
        return None

    @property
    def param(self) -> Dict[str, Any]:
        """额外参数 dict，子类按需覆盖"""
        return {}

    # --------- 子类必须实现的“执行”钩子 ---------
    def execute(self) -> None:
        raise NotImplementedError

    def __str__(self) -> str:
        """给人看的单行日志"""
        return f"[{self.actor}] {self.name} -> {self.target or 'none'} {self.param}"

# ------------------------------------------------------------------
# 下面3个子类只改__init__，其余字段用基类默认或覆盖即可
# ------------------------------------------------------------------
@dataclass
class CmdVel(RobotCommand):
    vx: float
    vy: float
    omega: float
    name: str = field(default="CmdVel", init=False)

    @property
    def param(self) -> Dict[str, Any]:
        return {"vx": self.vx, "vy": self.vy, "omega": self.omega}

    def execute(self) -> None:
        # 假装发到底层
        print(self)

@dataclass
class CmdGrab(RobotCommand):
    slot: str
    name: str = field(default="CmdGrab", init=False)

    @property
    def target(self) -> str:
        return self.slot
    def execute(self) -> None:
        print(self)

@dataclass
class CmdPlace(RobotCommand):
    slot: str
    name: str = field(default="CmdPlace", init=False)

    @property
    def target(self) -> str:
        return self.slot
    def execute(self) -> None:
        print(self)

@dataclass
class CmdDone(RobotCommand):
    slot: str
    name: str = field(default="CmdDone", init=False)

    @property
    def target(self) -> str:
        return self.slot
    def execute(self) -> None:
        print(self)

@dataclass
class CmdNavTo(RobotCommand):
    x: float
    y: float
    name: str = field(default="CmdNavTo", init=False)

    @property
    def target(self) -> tuple:
        return (self.x, self.y)

    @property
    def param(self) -> Dict[str, Any]:
        return {"x": self.x, "y": self.y}

    def execute(self) -> None:
        print(self)

# ------------------------------------------------------------------
# R1专用命令
# ------------------------------------------------------------------
@dataclass
class CmdR1StatusQuery(RobotCommand):
    """R1状态查询命令"""
    name: str = field(default="CmdR1StatusQuery", init=False)
    actor: str = field(default="r1", init=False)

    def execute(self) -> None:
        print(self)

@dataclass
class CmdR1PathSuggestion(RobotCommand):
    """R1路径建议命令"""
    waypoints: List[Tuple[float, float]]
    name: str = field(default="CmdR1PathSuggestion", init=False)
    actor: str = field(default="r1", init=False)

    @property
    def target(self) -> List[Tuple[float, float]]:
        return self.waypoints

    @property
    def param(self) -> Dict[str, Any]:
        return {"waypoints": self.waypoints}

    def execute(self) -> None:
        print(self)

@dataclass
class CmdR1ViolationWarning(RobotCommand):
    """R1违规警告命令"""
    violation_type: str
    details: str
    name: str = field(default="CmdR1ViolationWarning", init=False)
    actor: str = field(default="r1", init=False)

    @property
    def target(self) -> str:
        return self.violation_type

    @property
    def param(self) -> Dict[str, Any]:
        return {"violation_type": self.violation_type, "details": self.details}

    def execute(self) -> None:
        print(self)

@dataclass
class CmdR1StrategySuggestion(RobotCommand):
    """R1策略建议命令"""
    strategy: str
    details: str
    name: str = field(default="CmdR1StrategySuggestion", init=False)
    actor: str = field(default="r1", init=False)

    @property
    def target(self) -> str:
        return self.strategy

    @property
    def param(self) -> Dict[str, Any]:
        return {"strategy": self.strategy, "details": self.details}

    def execute(self) -> None:
        print(self)
