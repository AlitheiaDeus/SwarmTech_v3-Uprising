# [TKT-43] Dynamic Hungarian Formation Inversion & U-Turn Slot Flipping

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Formation Kinematics & Optimization (`Todo`) | **Priority**: High
* **Target Files**: `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`
* **Technical Blueprint**:
  - Distance-minimizing slot assignment on heading reversals ($> 120^\circ$, $\mathbf{v}_{\text{old}} \cdot \mathbf{v}_{\text{new}} < -0.5$).
  - Solves $\min \sum \|p_i - s_j\|^2$ using greedy nearest-neighbor or Hungarian matching.
  - Trailing wingtip drones immediately claim the new vanguard slots in the reversed direction, while forward drones ease back into the trail.
  - Eliminates 80m cross-flight drag races, formation distortion, and prop clash risks during sharp U-turns.
* **Targeted Test**: `python -m unittest tests/test_brains.py`
