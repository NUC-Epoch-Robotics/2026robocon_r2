# tests/test_policy.py
import json
from pathlib import Path
from kf_decision.policy import pick_policy
from commands import RobotCommand
from kf_decision import get_policy, list_policies
from kf_decision.abc import Policy

def cmd_to_dict(cmd: RobotCommand) -> dict:
    """把 RobotCommand 拍平成可 JSON 序列化的 dict"""
    return {
        "type": cmd.type,
        "actor": cmd.actor,
        "target": cmd.target,
        "param": cmd.param,
    }

def test_policy_snapshot(snapshot):
    """
    用 pytest-snapshot 做 JSON 比对。
    第一次跑会自动写入 tests/snapshots/test_policy_snapshot/snapshot.json
    以后只要输出不一致就会失败；需要更新策略时加 --snapshot-update
    """
    src, dst = ["1", "2", "3"], ["7", "8", "9"]            # 固定输入，别用随机
    cmds: list[RobotCommand] = pick_policy(src, dst)

    # 转成 dict 列表方便 JSON 比对
    data = [cmd_to_dict(c) for c in cmds]

    snapshot_dir = Path(__file__).parent / "snapshots" / "test_policy_snapshot"
    snapshot.snapshot_dir = snapshot_dir
    snapshot.assert_match(json.dumps(data, indent=2) + "\n", "snapshot.json")
def test_plugin_contract():
    assert "simple" in list_policies()
    Klass = get_policy("simple")
    assert issubclass(Klass, Policy)
    
    # 测试R1策略是否正确注册
    assert "r1_simple" in list_policies()
    Klass = get_policy("r1_simple")
    assert issubclass(Klass, Policy)
