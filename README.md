# Attention-Guided Decentralized Multi-UAV Surveillance
## Real-Time Crowd Monitoring in Smart Cities using POMDP Framework

**Project:** STIRS-2025  
**Institution:** SSN College of Engineering  
**Researchers:** Sharruk S, Sundareswaran, Jeswin Joel  
**Guide:** Dr. E. Selvam  
**Simulator:** Webots R2025a  

---

## Project Overview

This simulation implements a multi-UAV surveillance system using:
- **5 DJI Mavic 2 Pro UAVs** patrolling in adaptive circular formation
- **POMDP-based attention mechanism** that biases UAVs toward crowd density
- **5 distinct smart-city scenarios** (downtown, event, residential, mixed, industrial)
- **Boids-based bird flocking** for environmental realism
- **Built-in pedestrian animation** using Webots Pedestrian PROTO

### Coordinate System
All world files use **ENU (East-North-Up)**:
- X = East (horizontal)
- Y = North (horizontal)
- Z = Up / Altitude

UAVs patrol at **Z = 10–15 m altitude**.

---

## Project Structure

```
multi-uav-webots/
├── worlds/
│   ├── downtown.wbt       # Dense urban, 3×3 road grid, 15 buildings
│   ├── event.wbt          # Central plaza, 40 crowd agents
│   ├── residential.wbt    # Low-density, parks, 8 houses
│   ├── mixed.wbt          # Mixed commercial/residential, 35 agents
│   └── industrial.wbt     # Warehouse, containers, 10 workers
├── controllers/
│   ├── uav_swarm_controller/
│   │   └── uav_swarm_controller.py  # Supervisor: moves all UAVs + birds
│   ├── bird_controller/
│   │   └── bird_controller.py       # Per-bird Boids orbit (self-supervisor)
│   └── crowd_controller/
│       └── crowd_controller.py      # CrowdAgent state-machine supervisor
├── protos/
│   ├── Bird.proto          # Custom flying bird (supervisor Robot)
│   └── CrowdAgent.proto    # Custom pedestrian agent (supervisor Robot)
├── config/
│   └── environment_config.yaml
├── logs/                   # Simulation output logs
├── run_simulation.py       # Launch script
└── README.md
```

---

## Quick Start

### Prerequisites
- Webots R2025a installed at `C:\Program Files\Webots\`
- Python 3.8+ (for the launch script)

### Launch via Script
```bash
# From project root
python run_simulation.py --scenario downtown
python run_simulation.py --scenario event
python run_simulation.py --scenario residential
python run_simulation.py --scenario mixed
python run_simulation.py --scenario industrial
```

### Launch Directly in Webots
1. Open Webots R2025a
2. File → Open World
3. Navigate to `worlds/` and select a `.wbt` file
4. Press the Play button

---

## Scenarios

| Scenario | Buildings | Crowd | UAV Alt. | Description |
|----------|-----------|-------|----------|-------------|
| downtown | 15 | 15 | 15 m | Dense urban 3×3 road grid |
| event | 5 | 40 | 12 m | Public event, central plaza |
| residential | 8 | 8 | 10 m | Low-density with parks |
| mixed | 14 | 35 | 15 m | Mixed commercial + residential |
| industrial | 2 | 10 | 12 m | Port, containers, cranes |

---

## Controllers

### `uav_swarm_controller.py`
Runs as a **Supervisor** Robot node. Every timestep:
1. Computes circular patrol target for each UAV (staggered formation)
2. Applies POMDP attention bias toward crowd centroid
3. Moves all 5 UAVs and 5-8 birds via `setSFVec3f`
4. Logs coverage % and UAV positions every 125 steps

### `bird_controller.py`
Runs per-bird (Bird PROTO has `supervisor TRUE`). Each bird:
1. Orbits a slowly-drifting center point
2. Maintains altitude between 4–12 m
3. Uses unique per-bird parameters for flock variation

### `crowd_controller.py`
Runs as an optional second Supervisor. Manages `CrowdAgent` PROTO instances:
- **WALK**: follows waypoint loop
- **WAIT**: pauses randomly (10-15% chance at each waypoint)
- **GATHER**: converges on a plaza point during steps 500-700

> The built-in Pedestrian controller (used by default in all worlds) provides
> walking animation without this supervisor. Use `crowd_controller` with
> `CrowdAgent.proto` for research-grade custom crowd models.

---

## Custom PROTOs

### `Bird.proto`
A simple flying bird with:
- Box body (0.28 × 0.14 m) + wing panels
- Physics: 0.18 kg mass
- Self-supervising via hardcoded `supervisor TRUE`
- Controller: `bird_controller`

### `CrowdAgent.proto`
A human-shaped agent:
- Capsule body with head, torso, legs
- Physics: 70 kg mass
- Self-supervising via hardcoded `supervisor TRUE`
- Controller: `crowd_controller`

---

## POMDP Framework

The `uav_swarm_controller` implements a simplified POMDP:

**State**: UAV positions + crowd centroid  
**Observation**: Grid cells visible from current altitude  
**Action**: Target position update (circular + attention bias)  
**Reward proxy**: Coverage % logged every 125 steps

Attention mechanism:
```
x_target = x_patrol * (1 - α) + x_crowd_centroid * α
```
where α = 0.15 (configurable in `environment_config.yaml`).

---

## Configuration

Edit `config/environment_config.yaml` to change:
- `pomdp.attention_alpha` — crowd-centroid pull strength
- `pomdp.coverage_cell_size` — grid cell size for coverage metric
- `uav.patrol_altitude` — per-scenario UAV altitude
- `birds.altitude_min/max` — bird flight band

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| EXTERNPROTO download fails | Check internet connection; Webots downloads PROTOs on first use |
| "DEF UAV_0 not found" in console | World file not open or DEF name mismatch |
| Pedestrians not walking | Ensure controller is "pedestrian" and `controllerArgs` has `--trajectory` |
| Birds fall to ground | Bird PROTO requires `supervisor TRUE` (already set in `Bird.proto`) |
| Webots not found | Edit `WEBOTS_EXE` path in `run_simulation.py` |
| Black screen / no background | TexturedBackground needs internet for first load |

---

## Research Notes

- UAV motion is teleported via Supervisor API (bypasses drone physics for simulation speed)
- To enable physics-based flight, replace `controller "void"` with a PID hover controller
- The built-in Pedestrian PROTO provides motion-captured walking animations
- Crowd density metrics are computed from `node.getPosition()` in the supervisor
