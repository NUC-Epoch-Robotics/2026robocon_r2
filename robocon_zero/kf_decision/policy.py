# policy.py
from typing import List
from commands import RobotCommand, CmdVel, CmdGrab, CmdPlace

def pick_policy(src: List[str], dst: List[str]) -> List[RobotCommand]:
    """
    最简单的"一次抓一个、顺序放"策略。
    src: 待抓取的槽号列表，如 ["1","2","3"]
    dst: 目标放置槽号列表，如 ["4","5"] （长度可以 ≥ src）
    返回: 可直接发给机器人的命令序列
    """
    cmd_seq: List[RobotCommand] = []
    for i, s in enumerate(src):
        cmd_seq.append(CmdGrab(slot=s))
        # 假装每次移动 0.5 m/s 直走 1 秒
        cmd_seq.append(CmdVel(vx=0.5, vy=0.0, omega=0.0))
        # 放置到对应目标槽（循环使用 dst 列表）
        cmd_seq.append(CmdPlace(slot=dst[i % len(dst)]))
    return cmd_seq
