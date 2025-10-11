# kf_decision/types.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import Enum

class KFSType(Enum):
    REAL = "REAL"
    FAKE = "FAKE"

class TeamSide(Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"

@dataclass
class RobotPose:
    x: float        # 单位：毫米
    y: float
    yaw: float      # 弧度

@dataclass
class KungFuScroll:
    id: str
    slot_name: str               # 如 "A", "B"
    kfs_type: KFSType           # REAL / FAKE
    is_grabbed_by: Optional[str] = None   # r1 / r2 / None
    x: float = 0.0              # 位置坐标（毫米）
    y: float = 0.0

@dataclass
class ArenaGridCell:
    occupied_by: Optional[str]   # team_side name
    kfs_id: Optional[str]
    kfs_type: Optional[KFSType] = None  # KFS类型（REAL/FAKE）
    is_r1_kfs: bool = False     # 是否为R1 KFS
    is_r2_kfs: bool = False     # 是否为R2 KFS

@dataclass
class Package:
    id: str
    src: str               # 比如 "A"
    dst: str               # 比如 "B"

@dataclass
class Weapon:
    id: str
    weapon_type: str       # "FIST", "PALM", "SPEAR"
    is_assembled: bool     # 是否已组装完成
    is_used: bool          # 是否已在竞技场中使用
    carried_by: Optional[str] = None  # "r1" 或 "r2" 或 None
    x: float = 0.0         # 位置坐标（毫米）
    y: float = 0.0

@dataclass
class GameState:
    # 时间与阶段
    time_left_ms: int
    phase: str                    # MC / MF / ARENA / FINISHED
    retry_timer: int              # 最近一次违规后的冷却倒计时
    violations: int               # 总违规次数
    
    # R2 状态（主决策机器人）
    r2_pose: RobotPose
    r2_has_weapon: bool
    r2_kfs: List[str]             # 已抓取的 KFS ID 列表
    r2_side: TeamSide             # 左/右阵营
    
    # R1 状态（手动控制但需要状态跟踪）
    r1_pose: RobotPose            # 实时位置反馈
    r1_has_weapon: bool           # 是否携带武器
    r1_weapon_count: int          # 携带的武器数量
    r1_kfs: List[str]             # R1携带的KFS ID列表
    r1_busy: bool                 # 是否正在执行任务
    r1_last_action: str           # 最后执行的动作，用于违规检测
    r1_last_action_time: int      # 最后动作的时间戳

    # 武器相关
    weapons: List[Weapon]         # 所有武器的详细信息
    staff_rack: Dict[str, bool]   # 棍子架状态（slot_id: 是否有棍子）
    spearhead_rack: Dict[str, str]  # 矛头架状态（slot_id: 矛头类型）
    
    # MF 阶段区域管理
    mf_slots: Dict[str, KungFuScroll]  # 每个 slot 对应的 KFS 信息
    mf_r1_path: List[str]         # R1在梅花林外围的路径
    mf_r2_path: List[str]         # R2在梅花林内部的路径

    # ARENA 阶段布局
    arena_grid: List[List[ArenaGridCell]]  # 3x3 表格用于 Tic Tac Toe
    used_weapons_area: List[str]  # 已使用武器区域（武器ID列表）
    arena_ramp: Tuple[float, float]  # 竞技场坡道位置

    # Packages
    packages: List[Package]
    
    # 重试机制
    retry_requested: bool         # 是否请求了重试
    retry_robot: Optional[str] = None  # 请求重试的机器人("r1"/"r2")
