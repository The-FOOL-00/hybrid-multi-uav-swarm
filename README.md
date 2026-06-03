# Attention-Guided Decentralized Multi-UAV Surveillance
## Real-Time Crowd Monitoring in Smart Cities using POMDP Framework

**Project:** STIRS-2025  
**Institution:** SSN College of Engineering  
**Researchers:** Sharruk S, Sundareswaran, Jeswin Joel  
**Guide:** Dr. E. Selvam  
**Simulator:** Webots R2025a

---

## Project Overview

This simulation implements a **multi-UAV surveillance system** for smart-city crowd monitoring:

- **5 DJI Mavic2Pro UAVs** patrolling in adaptive circular formation
- **POMDP attention mechanism** that biases UAVs toward crowd density (α = 0.15)
- **5 distinct smart-city scenarios** — downtown, event, residential, mixed, industrial
- **Boids-based bird flocking** for environmental realism
- **Pedestrian crowd simulation** using Webots built-in Pedestrian PROTO

### Research Focus

| ✅ In Scope | ❌ Out of Scope |
|-------------|----------------|
| Multi-UAV coordination | Image / object detection |
| Autonomous navigation | Computer vision pipeline |
| Coverage optimization | ROS2 integration |
| Collision avoidance | Suspicious activity detection |
| Altitude optimization | |
| Dynamic crowd movement | |

---

## Project Structure

```
hybrid-multi-uav-swarm/
├── worlds/                         # Webots world files (5 scenarios)
│   ├── downtown.wbt                # Dense urban, 3×3 road grid, 15 buildings
│   ├── event.wbt                   # Central plaza, 40 crowd agents
│   ├── residential.wbt             # Low-density, parks, 8 houses
│   ├── mixed.wbt                   # Mixed commercial/residential, 35 agents
│   └── industrial.wbt              # Warehouse, containers, 10 workers
│
├── controllers/                    # Webots controllers (DO NOT MOVE)
│   ├── uav_swarm_controller/       # Main supervisor: POMDP + UAV movement
│   ├── bird_controller/            # Per-bird Boids orbit (self-supervisor)
│   ├── crowd_controller/           # CrowdAgent state machine (optional)
│   └── uav_camera/                 # Per-drone camera + propeller controller
│
├── rl/                             # RL training skeleton (not yet active)
│   ├── gym_wrapper/                # OpenAI Gym-compatible env interface
│   ├── rewards/                    # Coverage, tracking, collision rewards
│   ├── training/                   # Training loop + experiment management
│   └── algorithms/                 # PPO → MAPPO → QMIX roadmap
│
├── experiments/                    # Simulation outputs
│   ├── metrics/                    # JSON metrics per run
│   ├── screenshots/                # PNG figures for paper
│   ├── videos/                     # Recorded simulations
│   └── logs/                       # Text logs
│
├── configs/                        # Canonical configuration (with RL section)
│   └── environment_config.yaml
│
├── protos/                         # Custom Webots PROTOs
│   ├── Bird.proto                  # Flying bird (supervisor Robot)
│   └── CrowdAgent.proto            # Pedestrian agent (supervisor Robot)
│
├── models/                         # Saved RL model checkpoints
│
├── docs/                           # Project documentation
│   ├── architecture.md             # System design, data flow, POMDP details
│   ├── workflow.md                 # When to use GUI vs headless mode
│   └── current_status.md          # What works, what's next
│
├── run_simulation.py               # Main launch script (GUI + headless)
└── README.md
```

---

## Git Repository & Branches

This repository uses a multi-branch workflow to organize research, feature migrations, and reinforcement learning stabilization:

* **`main`**: The primary production-ready branch. It integrates the decentralized Multi-UAV Swarm surveillance system, including all 5 world scenarios, the Attention-Guided POMDP patrol framework, and the completed Phase 3 RL Gym environment wrapper integrated with Stable-Baselines3 PPO.
* **`webots-research-architecture`**: Established the baseline directory layouts, base controllers, custom PROTO files, and environment configs before incorporating Mavic-specific components or reinforcement learning frameworks.
* **`dji-mavic-migration`**: Migrated the simulation from generic UAV models to DJI Mavic 2 Pro drones, enabling onboard camera systems, propeller rotation visualizations, and addressing gimbal-related physics/warning messages.
* **`phase1-rl-webots-bridge`**: Implemented Phases 1 and 2 of the reinforcement learning integration, laying down the synchronized Gym interface IPC architecture and observation/action space mapping, while solving drone tumbling issues.
* **`rl-stability-improvements`**: Developed and integrated stability patches for the multi-agent RL training environment and added metric/benchmark logging pipelines to analyze swarm coverage performance.
* **`single-drone-navigation-baseline`**: A research branch containing navigation, path-planning, and collision-avoidance algorithms for a single UAV. It includes a custom test environment (`worlds/single_drone_downtown.wbt`) to contrast single-drone baseline performance against the multi-UAV swarm.

---

## Quick Start

### Prerequisites

- **Webots R2025a** installed at `C:\Program Files\Webots\`
- **Python 3.8+** (for the launch script only — no extra packages needed)

### Launch Simulation

```powershell
# GUI mode — full 3D visualization (default scenario: downtown)
python run_simulation.py

# GUI mode — specific scenario
python run_simulation.py --scenario downtown
python run_simulation.py --scenario event
python run_simulation.py --scenario residential
python run_simulation.py --scenario mixed
python run_simulation.py --scenario industrial

# Headless mode — no window, faster (for experiments)
python run_simulation.py --headless
python run_simulation.py --scenario event --headless

# List all scenarios
python run_simulation.py --list

# Help
python run_simulation.py --help
```

### Launch Directly in Webots

1. Open Webots R2025a
2. **File → Open World**
3. Navigate to `worlds/` and select a `.wbt` file
4. Press the **Play (▶)** button

---

## Two Execution Modes

### Mode 1 — GUI Mode (Demo / Debugging)

```powershell
python run_simulation.py
python run_simulation.py --scenario event
```

**Use for:**
- Demo to guide/supervisor
- Debugging drone movement
- Taking screenshots for paper (Webots → Tools → Screenshot → `experiments/screenshots/`)
- Recording videos (`experiments/videos/`)

**Result:** Full Webots window with 3D environment, UAVs, crowd, birds.

---

### Mode 2 — Headless Mode (Training / Research)

```powershell
python run_simulation.py --headless
python run_simulation.py --scenario industrial --headless
```

**Use for:**
- Faster metric collection
- Reward tuning experiments
- Batch runs across all scenarios
- Future RL training

**Result:** No GUI window. Same controllers, same world, same metrics in console.

> **Same codebase. No duplicate logic. Only rendering changes.**

---

## Scenarios

| Scenario | Buildings | Crowd | UAV Alt | Description |
|----------|-----------|-------|---------|-------------|
| `downtown` | 15 | 15 | 15 m | Dense urban 3×3 road grid |
| `event` | 5 | 40 | 12 m | Public event, central plaza |
| `residential` | 8 | 8 | 10 m | Low-density with parks |
| `mixed` | 14 | 35 | 15 m | Mixed commercial + residential |
| `industrial` | 2 | 10 | 12 m | Port, containers, cranes |

All worlds use **ENU coordinates** (East=X, North=Y, Up=Z). UAVs patrol at Z = 10–15 m.

---

## Controllers

### `uav_swarm_controller` — Main Supervisor

Runs once per world. Controls ALL 5 UAVs via Supervisor API.

1. Computes **circular patrol target** per UAV (staggered formation)
2. Applies **POMDP attention bias** toward crowd centroid (α = 0.15)
3. Moves UAVs via `setSFVec3f` (teleport — bypasses drone physics)
4. Logs **coverage %** and UAV positions every 125 steps

### `uav_camera` — Per-Drone Camera Controller

Runs on each Mavic2Pro drone:
- Enables front camera
- Sets propellers to `68.0 rad/s` (visual hover effect)

### `bird_controller` — Per-Bird Self-Supervisor

Each bird runs its own controller:
- Unique Boids parameters per bird ID
- Orbit center drifts slowly for realistic flock behavior
- Altitude clamped: 4–12 m

### `crowd_controller` — Optional CrowdAgent Supervisor

State machine: **WALK → WAIT → GATHER**. Used with `CrowdAgent.proto`.
Default worlds use the built-in Webots `pedestrian` controller instead.

---

## Configuration

Edit `configs/environment_config.yaml` to change:

```yaml
pomdp:
  attention_alpha: 0.15        # crowd-centroid pull strength (tune this)
  coverage_cell_size: 5.0      # metres per grid cell
  observation_radius: 12.0     # UAV sensor radius

uav:
  patrol_altitude: 15.0        # ENU Z in metres (per scenario)
  patrol_radius: 20.0

birds:
  altitude_min: 4.0
  altitude_max: 12.0
```

---

## Team Workflow

```
Edit → GUI test → Headless metrics → Analyze → Paper
```

1. **Implement** changes in `controllers/`
2. **Test visually** with `python run_simulation.py`
3. **Collect metrics** with `python run_simulation.py --headless`
4. **Save screenshots** to `experiments/screenshots/`
5. **Document results** in `docs/current_status.md`

### Adding a New Controller

```
controllers/
└── my_controller/
    └── my_controller.py      ← must match folder name
```

In `.wbt` file:
```
controller "my_controller"
```

### Adding a New World Scenario

1. Create `worlds/new_scenario.wbt` in Webots
2. Add to `SCENARIOS` in `run_simulation.py`
3. Add to `configs/environment_config.yaml`
4. Test: `python run_simulation.py --scenario new_scenario`

---

## POMDP Framework

```
State:      UAV positions (5×3) + crowd centroid (2)
Obs:        Grid cells visible from current altitude
Action:     Circular patrol + attention bias
Reward:     Coverage % (logged; not yet used for learning)

Attention: x_target = x_patrol × (1 - α) + x_crowd_centroid × α
```

Coverage metric: `|unique 5m grid cells covered| / 1296 total × 100%`

---

## RL Roadmap

> **Status:** Architecture skeleton ready. Not yet training.

```
Phase 1 (current): Simulation baseline + paper metrics
Phase 2 (next):    Implement UAVSwarmEnv IPC bridge + PPO
Phase 3 (future):  MAPPO (multi-agent) + QMIX comparison
```

See `rl/algorithms/README.md` for algorithm comparison table.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `Webots not found` | Webots auto-detected in common paths. Use `--webots-exe` to override |
| `DEF UAV_0 not found` | World file not open or DEF name mismatch in `.wbt` |
| `Pedestrians not walking` | Controller must be `"pedestrian"` with `--trajectory` args |
| `Birds fall to ground` | Bird PROTO requires `supervisor TRUE` (already set in `Bird.proto`) |
| `Black screen / no background` | TexturedBackground needs internet on first Webots load |
| `EXTERNPROTO download fails` | Check internet connection; Webots downloads PROTOs on first use |

---

## Research Notes

- UAV motion uses **Supervisor teleport** (`setSFVec3f`) — bypasses drone physics for speed
- To enable physics-based flight: replace `controller "void"` with a PID hover controller
- Pedestrian PROTO provides **motion-captured** walking animations
- Crowd density metrics from `node.getPosition()` in supervisor
- See `docs/architecture.md` for full system design
