# System Architecture

**Project:** Attention-Guided Decentralized Multi-UAV Surveillance  
**Simulator:** Webots R2025a | **Coord. System:** ENU (East=X, North=Y, Up=Z)  
**Institution:** SSN College of Engineering · STIRS-2025

---

## High-Level Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         run_simulation.py                            │
│   CLI launcher — selects scenario, launches Webots (GUI/headless)    │
└───────────────────────────────┬──────────────────────────────────────┘
                                │ subprocess (Webots R2025a)
                                ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        Webots World (.wbt)                           │
│                                                                      │
│  ┌─────────────────┐   ┌─────────────────┐   ┌──────────────────┐  │
│  │ uav_swarm_ctrl  │   │  uav_camera ×5  │   │  bird_ctrl ×8    │  │
│  │ (Supervisor)    │   │  (per-drone)    │   │  (self-supv.)    │  │
│  │                 │   │                 │   │                  │  │
│  │ · POMDP patrol  │   │ · camera enable │   │ · boids orbit    │  │
│  │ · UAV positions │   │ · propeller spin│   │ · drift center   │  │
│  │ · Bird fallback │   │                 │   │                  │  │
│  │ · Coverage log  │   └─────────────────┘   └──────────────────┘  │
│  └────────┬────────┘                                                 │
│           │ Supervisor API (setSFVec3f)                              │
│           ▼                                                          │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │   Scene Graph                                                 │   │
│  │   · 5× Mavic2Pro UAVs (DEF UAV_0…UAV_4)                     │   │
│  │   · 5-8× Bird PROTOs (DEF BIRD_0…BIRD_7)                    │   │
│  │   · 15-40× Pedestrian nodes                                  │   │
│  │   · Buildings, roads, terrain                                │   │
│  └──────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Controller Architecture

### `uav_swarm_controller` (Supervisor)

The main brain of the simulation. Runs once per world, controls ALL UAVs.

```
MultiUAVSurveillance.__init__()
    ├── getFromDef("UAV_0") … getFromDef("UAV_4")   → uav_trans[]
    ├── getFromDef("BIRD_0") … getFromDef("BIRD_7")  → bird_trans[]
    └── _collect_crowd() → scan root children for Pedestrian/CrowdAgent

MultiUAVSurveillance.run()  [main loop, every 8 ms]
    ├── update_uavs()
    │     └── _patrol_target(i, t)
    │           ├── Circular orbit: r·cos(ωt + φ_i), r·sin(ωt + φ_i)
    │           └── POMDP attention: x = x*(1-α) + crowd_cx*α  [α=0.15]
    ├── update_birds()  [fallback if bird_controller inactive]
    └── log_metrics()   [every 125 steps: coverage%, UAV positions]
```

**Motion model:** Supervisor teleport (`setSFVec3f`) — bypasses drone physics
for simulation speed. No PID controller needed.

### `uav_camera` (per-drone Robot)

Simple controller per Mavic2Pro drone:
- Enables front camera at simulation timestep
- Sets propellers to `HOVER_RPM = 68.0 rad/s` for visual realism
- Position is fully controlled by `uav_swarm_controller` supervisor

### `bird_controller` (per-Bird self-supervisor)

Each Bird PROTO runs its own instance:
- Unique parameters derived from bird ID (speed, radius, phase)
- Orbit center drifts slowly (drift_speed ≈ 0.0003 rad/s)
- Altitude clamped: 4–12 m (ENU Z)

### `crowd_controller` (optional Supervisor)

State machine for CrowdAgent PROTOs:
- **WALK** → follows hexagonal waypoint loop
- **WAIT** → random pause (40–180 ticks, 12% probability)
- **GATHER** → converge on plaza (steps 500–700)

> Not active in default worlds — Webots built-in `pedestrian` controller is used.

---

## POMDP Framework

```
State  S:  UAV positions (5×3) + crowd centroid (2)
Obs    O:  Grid cells visible from current altitude
Action A:  Target position update (patrol formula + attention)
Reward R:  Coverage % (logged, not yet used for learning)

Attention: x_target = x_patrol*(1-α) + x_crowd_centroid*α   [α=0.15]
```

### Metrics computed every 125 steps

| Metric | Formula |
|--------|---------|
| Coverage % | `|unique_5m_cells_covered| / total_cells × 100` |
| Footprint | `altitude × 0.7` metres radius per UAV |
| Arena | 200×200 m grid (±90 m, cell=5 m → 1296 total cells) |

---

## World Scenarios

| Scenario | World File | Crowd | UAV Alt | Area |
|----------|-----------|-------|---------|------|
| downtown | downtown.wbt | 15 | 15 m | 40 000 m² |
| event | event.wbt | 40 | 12 m | 25 600 m² |
| residential | residential.wbt | 8 | 10 m | 25 600 m² |
| mixed | mixed.wbt | 35 | 15 m | 32 400 m² |
| industrial | industrial.wbt | 10 | 12 m | 32 400 m² |

---

## Custom PROTOs

### `Bird.proto`

```
Robot (supervisor=TRUE)
├── Box body:       0.28 × 0.14 × 0.08 m
├── Left wing:      0.22 × 0.02 × 0.09 m (rotated +0.35 rad)
├── Right wing:     same (rotated -0.35 rad)
├── Tail:           0.14 × 0.08 × 0.04 m
├── Physics:        mass=0.18 kg
└── Controller:     bird_controller
```

### `CrowdAgent.proto`

```
Robot (supervisor=TRUE)
├── Capsule body + head + torso + legs
├── Physics: mass=70 kg
└── Controller: crowd_controller
```

---

## RL Integration (Future)

```
rl/
├── gym_wrapper/uav_swarm_env.py   ← wraps Webots sim as Gym env
│     observation_space: Box(40,) — 8 features × 5 UAVs
│     action_space:      Box(15,) — Δx,Δy,Δz × 5 UAVs
├── rewards/reward_functions.py    ← CoverageReward + CollisionPenalty + TrackingReward
├── training/trainer.py            ← episode loop, metric logging, checkpointing
└── algorithms/                    ← PPO → MAPPO → QMIX roadmap
```

**Integration point:** `uav_swarm_controller` will accept actions via IPC when
`rl.enabled=true` in configs/environment_config.yaml.

---

## Data Flow Diagram

```
run_simulation.py
      │
      │ --scenario downtown [--headless]
      ▼
Webots R2025a subprocess
      │
      ├── uav_swarm_controller (Supervisor, 8ms loop)
      │     ├── READ:  crowd positions via getPosition()
      │     ├── COMPUTE: patrol_target() + attention_bias()
      │     ├── WRITE:  uav_trans[i].setSFVec3f(target)
      │     └── LOG:   coverage%, UAV pos → stdout (every 125 steps)
      │
      ├── uav_camera ×5 (per drone, 8ms loop)
      │     └── camera.enable() + propeller.setVelocity(68.0)
      │
      └── bird_controller ×8 (per bird, 8ms loop)
            └── orbit + drift → trans_field.setSFVec3f()
```
