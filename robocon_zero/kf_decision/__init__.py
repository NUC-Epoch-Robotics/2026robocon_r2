# kf_decision/__init__.py
# 先扫 impl 包，让装饰器跑到
from . import simple_policy, r1_policy   # noqa: F401
# 再导出工具函数
from .registry import get_policy, list_policies

__all__ = ["get_policy", "list_policies"]
