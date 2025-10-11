# bench/runner.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Any
import time
import json
import pathlib

from commands import RobotCommand
from executor.core import Executor
from kf_decision import get_policy
from kf_decision.types import GameState, RobotPose, Package, TeamSide, ArenaGridCell, KungFuScroll, KFSType

@dataclass
class Stats:
    duration_ms: int
    dist_mm: int
    n_commands: int
    trace: List[Dict] = field(default_factory=list)

class BenchmarkRunner:
    def __init__(self, track_file: str):
        raw = json.loads(pathlib.Path(track_file).read_text())

        # 1. 读取json初始化地图 & 包裹
        self.map = raw["map"]
        self.packages = [Package(**p) for p in raw.get("packages", [])]

        # 2. 构造初始 GameState（全面围绕 R2 建模）
        r2_pose_data = raw.get("robot", {"x": 500.0, "y": 600.0, "yaw": 0.0})
        r1_pose_data = raw.get("r1", {"x": 0.0, "y": 0.0, "yaw": 0.0})

        # 初始化 MF 阶段 scroll slots（模拟假数据）
        mf_slots = {
            "A": KungFuScroll(id="scroll-A", slot_name="A", kfs_type=KFSType.REAL),
            "B": KungFuScroll(id="scroll-B", slot_name="B", kfs_type=KFSType.FAKE),
            "C": KungFuScroll(id="scroll-C", slot_name="C", kfs_type=KFSType.REAL),
        }

        self.init_state = GameState(
            time_left_ms=180_000,
            phase="MC",  # 👈 起始阶段切换回 MC
            retry_timer=0,
            violations=0,
            r1_pose=RobotPose(**r1_pose_data),
            r1_has_weapon=False,
            r1_busy=False,
            r2_pose=RobotPose(**r2_pose_data),
            r2_has_weapon=False,
            r2_kfs=[],
            r2_side=TeamSide.LEFT,
            assembled_weapons=[],
            mf_slots=mf_slots,
            arena_grid=[[ArenaGridCell(None, None) for _ in range(3)] for _ in range(3)],
            packages=self.packages
        )

    # ---------- 运行 ----------
    def run(self, policy_name: str) -> Stats:
        policy = get_policy(policy_name)()
        exe = Executor(self.init_state)
        start = time.perf_counter()
        step_cnt = 0
        dt_ms = int(exe._robot.dt * 1000)

        while not exe.is_done():
            step_cnt += 1
            exe.state.time_left_ms = max(0, exe.state.time_left_ms - dt_ms)

            # 超时退出
            if exe.state.time_left_ms <= 0:
                print("[Runner] 时间结束")
                break

            # 步数限制保护
            if step_cnt >= 100_000:
                print('[Runner] 最大步数 reached, force quit')
                break

            # 用 R2 状态来驱动决策 👇
            cmds: List[RobotCommand] = policy.decide(exe.state)

            # 执行命令
            for c in cmds:
                exe.step(c)

        elapsed = int((time.perf_counter() - start) * 1000)

        # ---- 把 trace 转成 JSON 可序列化 dict ----
        trace_dict: List[Dict[str, Any]] = [
            {
                "ts": s.ts,
                "state": {
                    "time_left_ms": s.state.time_left_ms,
                    "phase": s.state.phase,
                    "r2_pose": {
                        "x": s.state.r2_pose.x,
                        "y": s.state.r2_pose.y,
                        "yaw": s.state.r2_pose.yaw,
                    },
                    "r2_kfs": s.state.r2_kfs,
                    "r2_has_weapon": s.state.r2_has_weapon,
                    "r1_has_weapon": s.state.r1_has_weapon,
                },
                "command": s.command.__dict__,
            }
            for s in exe.history
        ]

        return Stats(
            duration_ms=elapsed,
            dist_mm=exe.total_distance(),
            n_commands=exe.command_count(),
            trace=trace_dict,
        )
