"""
SwarmTech3_UpRising: Pluggable Guidance Brain Architecture (Backward-Compatible Facade)
Re-exports from Bubble 2: OBC.backend.brains
"""

from OBC.backend.brains import (
    BaseBrain,
    BRAIN_REGISTRY,
    register_brain,
    LeaderFollowerBrain,
    GossipConsensusBrain,
)

__all__ = [
    "BaseBrain",
    "BRAIN_REGISTRY",
    "register_brain",
    "LeaderFollowerBrain",
    "GossipConsensusBrain",
]
