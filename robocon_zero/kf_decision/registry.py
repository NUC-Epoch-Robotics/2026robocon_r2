# kf_decision/registry.py
from typing import Dict, Type
from kf_decision.abc import Policy

_REGISTRY: Dict[str, Type[Policy]] = {}

def register(name: str):
    def _decorator(cls: Type[Policy]) -> Type[Policy]:
        if name in _REGISTRY:
            raise ValueError(f"Policy {name} already registered")
        _REGISTRY[name] = cls
        return cls
    return _decorator

def get_policy(name: str) -> Type[Policy]:
    try:
        return _REGISTRY[name]
    except KeyError:
        raise KeyError(f"No policy '{name}'. Loaded: {list(_REGISTRY)}")

def list_policies():
    return list(_REGISTRY)