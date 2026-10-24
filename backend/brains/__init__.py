"""
SwarmTech3_UpRising: Pluggable Guidance Brain Architecture
Defines BaseBrain interface and registry for hot-swappable guidance plugins.
"""

from typing import Dict, Type
from backend.brains.base import BaseBrain
from backend.brains.leader_follower import LeaderFollowerBrain
from backend.brains.gossip_consensus import GossipConsensusBrain

BRAIN_REGISTRY: Dict[str, Type[BaseBrain]] = {
    "LEADER_FOLLOWER": LeaderFollowerBrain,
    "GOSSIP_CONSENSUS": GossipConsensusBrain,
}


def register_brain(name: str):
    """Decorator to register a guidance brain plugin."""
    def decorator(cls: Type[BaseBrain]):
        BRAIN_REGISTRY[name.upper()] = cls
        cls.brain_name = name.upper()
        return cls
    return decorator


__all__ = [
    "BaseBrain",
    "BRAIN_REGISTRY",
    "register_brain",
    "LeaderFollowerBrain",
    "GossipConsensusBrain",
]
