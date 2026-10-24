# SwarmTech_v3: UpRising

<p align="center">
  <strong>Autonomous Drone Swarm Simulator & Hardware-Accelerated 3D WebGL Tactical GCS</strong><br>
  <em>Built on a high-performance 50/50 Architecture: Pure Python Vectorized Kinematics & GPU-Native Three.js Rendering.</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.12-blue?logo=python&logoColor=white" alt="Python 3.12">
  <img src="https://img.shields.io/badge/WebGL-Three.js_r128-00F0FF?logo=three.js&logoColor=white" alt="Three.js">
  <img src="https://img.shields.io/badge/Architecture-50%2F50_Decoupled-00FFAA" alt="50/50 Decoupled">
  <img src="https://img.shields.io/badge/Physics-60_FPS_Vectorized-FFB800" alt="60 FPS">
  <img src="https://img.shields.io/badge/Tests-26%2F26_Passing-brightgreen" alt="Tests 26/26 Passing">
  <img src="https://img.shields.io/badge/License-MIT-gray" alt="MIT License">
</p>

---

## 1. System Mission & The 50/50 Architecture

Previous desktop swarm simulators suffered from severe frame drops, sluggish UI thread freezing, and inaccurate CPU software projections (e.g. `QPainter` 3D projection math). 

**`SwarmTech_v3-Uprising`** solves this by strictly enforcing a **50/50 Architecture**:
* **Backend (Python 3.12 / NumPy / aiohttp)**: High-performance vectorized 60 FPS Reynolds boid kinematics, sane physical safety zones, realistic 10 Hz radio comms with Last Known Position (LKP) recovery, catenary tether dynamics, and non-blocking UDP/MAVLink loopback.
* **Frontend (Three.js / WebGL / Vanilla CSS)**: Hardware-accelerated 3D viewport running 100% on the GPU. Cyberpunk tactical HUD with live in-place telemetry cards, formation hot-switching, 3D ground-click target dispatch, and casualty simulation controls.

> **Invariable Rule**: Guidance intelligence, physical bounds, and networking remain in Python; 3D rendering and operator interaction run in WebGL on the GPU. Neither side leaks into the other.

---

## 2. Key Capabilities & Technical Features

### Sane 4-Zone Spatial Safety (No "Truck-Sized" Bubbles)
Replaces over-engineered 15m clash radii and force spikes with true-to-scale physics:
* **Physical Airframe**: $R_{\text{body}} = 0.75\text{m}$ (Wingspan $= 1.5\text{m}$).
* **Zone 1 ($d > 8.0\text{m}$)**: Nominal flight toward formation slots or waypoints.
* **Zone 2 ($3.5\text{m} < d \le 8.0\text{m}$)**: Smooth linear separation steering.
* **Zone 3 ($1.5\text{m} < d \le 3.5\text{m}$)**: **Active Relative-Velocity Damping** brakes incoming momentum and prevents discrete pinball penetration:
  $$\mathbf{a}_{\text{evade}} = \left[ 12.0 + 8.0 \cdot \left(\frac{3.5 - d_{ij}}{3.5 - 1.5}\right) \right] \hat{\mathbf{n}}_{ij} - 1.5 (\mathbf{v}_i - \mathbf{v}_j)_{\parallel}$$
* **Zone 4 ($d \le 1.5\text{m}$)**: Prop clash occurs. Motors disengage (`crashed = True`) and airframe enters ballistic gravity drop ($g = -9.81\text{m/s}^2$).

### Pluggable Guidance Brains (Centralized & Decentralized)
* **`LeaderFollowerBrain`**: Star topology where tethered leader transmits heartbeats and followers hold staggered formation slots.
* **`GossipConsensusBrain`**: Fully decentralized P2P mesh network. Target waypoints propagate via multi-hop gossip echo waves until a $75\%$ quorum consensus barrier is achieved, prompting simultaneous swarm maneuver.
* Hot-swappable at runtime with zero physics hitching.

### Realistic Radio Comms ("Baud") & LKP Failover
* Leader broadcasts state packets at $10\text{ Hz}$ with a $120\text{m}$ Euclidean radio range.
* Drones that lose heartbeats for $> 1.0\text{s}$ enter `LOST_LINK` and autonomously seek their **Last Known Position (LKP)** at conservative cruise speed until rejoining the fleet.

### Catenary Tether Dynamics & Autonomic Succession
* GCS mast anchor at $(0, 0, 1.5)$ connects to the active leader via a dynamic 16-point catenary curve calculating gravitational sag.
* Followers maintain a $3.0\text{m}$ clearance envelope. Physical contact ($d_{\perp} \le 1.0\text{m}$) snaps the cable, cuts leader power, and triggers immediate **autonomic leadership succession** based on tier priority.

### UDP Network Bridge & MAVLink Loopback
* **Port 9000 (TX)**: Non-blocking UDP broadcast of swarm telemetry JSON array.
* **Port 9001 (RX)**: External UDP command ingestion.
* **Port 14550 (MAVLink v2 Loopback)**: Emits binary MAVLink `HEARTBEAT` and `GLOBAL_POSITION_INT` messages with GPS projection for integration with QGroundControl and SITL tools.

### GPU-Native WebGL Tactical GCS
* **True 3D Geometry**: Detailed quadcopter airframes with spinning rotor discs, carbon arms, and orientation LEDs.
* **Spatial Depth**: Concentric perspective range rings ($25\text{m}, 50\text{m}, 100\text{m}$) and 3D dashed altitude drop-lines projecting to $(X, Y, 0)$ ground reticles.
* **Ground-Click Dispatch**: Left-click anywhere on the 3D ground grid to dispatch a target waypoint with an expanding pulse ripple animation.
* **Interactive Command Deck**: In-place telemetry cards (ID, altitude, speed, battery gauge), formation switcher (*Echelon*, *Boids*, *Vortex*, *Patrol*), and a Casualty Lab (*Kill Leader*, *Cut Tether*, *Reboot*).

---

## 3. Quick Start & Execution

### Prerequisites
* Windows 10/11
* Python 3.12 (Anaconda / Miniconda environment recommended)

### One-Click Startup (PowerShell)
```powershell
# Run the automated 2.0s headless smoke test (exits code 0)
.\run.ps1 -SmokeTest

# Launch simulation backend and auto-open WebGL GCS in default browser
.\run.ps1
```

### Direct Python Invocation
Thanks to self-healing Conda DLL and path bootstraps, you can launch directly:
```powershell
# Interactive mode (opens browser to http://127.0.0.1:8080)
python run.py

# Custom swarm size or port
python run.py --count 12 --port 8080

# Headless verification mode
python run.py --smoke-test
```

---

## 4. Automated Test Suite

Run the full automated unit test suite covering kinematics, 4-zone safety, tether dynamics, pluggable brains, UDP bridge, and aiohttp WebSockets:

```powershell
python -m unittest discover tests
```

```text
..........................
----------------------------------------------------------------------
Ran 26 tests in 0.549s

OK
```

---

## 5. Repository Topology

```
SwarmTech_v3-Uprising/
├── README.md                       # Master documentation & technical guide
├── AGENTS.md                       # Architectural landmark & design invariant guide
├── SWARM_UPRISING_SPEC.md          # Complete mathematical & architectural specification
├── kanban.md                       # Strict single-WIP project lifecycle board
├── run.py                          # Unified backend launcher, browser opener & smoke test runner
├── run.ps1                         # Zero-config PowerShell launcher with DLL path injection
├── backend/                        # Python Simulation & Networking Backend
│   ├── __init__.py                 # Self-healing Conda DLL and sys.path bootstrap
│   ├── sim_core.py                 # Vectorized 60 FPS boids, 4-zone safety, tether, LKP
│   ├── mavlink_bridge.py           # UDP ports 9000, 9001, and 14550 MAVLink loopback
│   ├── web_server.py               # aiohttp static web server & 60 FPS WebSocket streamer
│   └── brains/                     # Pluggable Guidance Brain Modules
│       ├── __init__.py             # Brain registry and factory
│       ├── base.py                 # BaseBrain plugin interface
│       ├── leader_follower.py      # Tethered leader-follower with LKP recovery
│       └── gossip_consensus.py     # Decentralized P2P mesh with quorum consensus
├── frontend/                       # Three.js WebGL Tactical GCS
│   ├── index.html                  # GCS entry point & layout
│   ├── css/
│   │   └── style.css               # Cyberpunk dark navy (#061019) & neon HUD styling
│   └── js/
│       ├── app.js                  # Three.js 3D scene, quadcopters, drop-lines, camera
│       ├── hud.js                  # Telemetry cards, mission dispatch, casualty lab
│       └── ws_client.js            # Duplex WebSocket state synchronization client
└── tests/                          # Automated unit test suite
    ├── __init__.py
    ├── test_sim_core.py            # Physics, 4-zone avoidance, tether snap, LKP tests
    ├── test_brains.py              # Brain plugins, P2P gossip wave, quorum tests
    ├── test_mavlink_bridge.py      # UDP packet dispatch and MAVLink parsing tests
    └── test_web_server.py          # WebSocket streaming and static asset serving tests
```

---

## 6. Controls Reference

| Control | Action |
| :--- | :--- |
| **Left-Click + Drag** | Orbit camera around tactical space |
| **Right-Click + Drag** | Pan tactical camera |
| **Scroll Wheel** | Zoom in / out |
| **Left-Click on Ground** | Dispatch target waypoint to swarm |
| **`[C]` Key** | Toggle Chase Cam mode |
| **`[R]` Key** | Reset camera to tactical overview |

---

## 7. License

Distributed under the MIT License. Built with passion for autonomous multi-agent robotics.
