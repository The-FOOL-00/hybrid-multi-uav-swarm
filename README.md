# Hybrid Multi-UAV Swarm Surveillance System

A Webots-based research project for **multi-UAV autonomous surveillance**, combining:

* Multi-drone coordination
* Urban environment simulation
* Crowd monitoring
* Path planning
* Collision avoidance
* Reinforcement Learning (PPO → future MAPPO/QMIX)

---

# Project Goal

The long-term goal is to build a **research-grade autonomous UAV swarm surveillance system** capable of:

* monitoring urban crowds
* navigating dense city environments
* avoiding buildings and dynamic obstacles
* coordinating multiple drones
* optimizing surveillance using Reinforcement Learning

The final system should support:

```text
single drone
→ multi-drone swarm
→ RL optimization
→ adaptive surveillance
```

---

# Current Project Status

Current maturity level:

### Engineering Prototype

This is **NOT yet a research-grade swarm system**.

The project currently contains:

✅ stable simulation infrastructure

✅ DJI Mavic drone integration

✅ urban environments

✅ crowd simulation

✅ RL infrastructure

✅ deterministic single-drone baseline

But still lacks:

❌ swarm intelligence

❌ multi-agent reinforcement learning (MAPPO)

---

# Repository Structure

```text
hybrid-multi-uav-swarm/

├── worlds/
│   ├── downtown.wbt
│   ├── event.wbt
│   ├── residential.wbt
│   ├── mixed.wbt
│   └── industrial.wbt
│
├── controllers/
│   ├── uav_swarm_controller/
│   ├── uav_camera/
│   ├── bird_controller/
│   └── crowd_controller/
│
├── rl/
│   ├── gym_wrapper/
│   ├── rewards/
│   ├── training/
│   └── algorithms/
│
├── experiments/
│   ├── metrics/
│   ├── screenshots/
│   ├── videos/
│   └── logs/
│
├── configs/
│   └── environment_config.yaml
│
├── docs/
│
├── models/
│
└── run_simulation.py
```

---

# What Works Today

## 1. DJI Mavic Integration

Each UAV uses:

```text
DJI Mavic 2 Pro
```

with:

* GPS
* IMU
* Gyroscope
* Camera

---

## 2. Urban Simulation Environments

Available worlds:

### downtown

Dense urban city.

Primary research environment.

### event

Crowd-heavy surveillance scenario.

### residential

Low-density suburban environment.

### mixed

Mixed urban planning.

### industrial

Warehouse/industrial navigation.

---

## 3. Crowd Simulation

Pedestrians are implemented and move through predefined trajectories.

Used for:

```text
future crowd surveillance
future target tracking
```

---

## 4. Single Drone Navigation Baseline

Current development focus.

Implemented:

### Planners
* A* (grid-based)
* Dijkstra (grid-based)
* Bidirectional RRT (sampling-based)

### Collision Avoidance
* Reactive 8-ray obstacle avoidance
* Building-distance safety layer

### Telemetry & Dashboard
* Comprehensive metrics infrastructure
* React-based Research Dashboard (HTML)

### Fixed Mission

```text
Start:
(-40, 48, 15)

Target:
(50, 48, 15)
```

### Stable Flight

* deterministic movement
* constant altitude
* smooth navigation

### Purpose

Provides a measurable benchmark before:

```text
swarm coordination
RL
```

---

## 5. RL Infrastructure

PPO infrastructure exists.

Includes:

* Gym wrapper
* reward system
* trainer
* metrics
* checkpointing

However:

### RL is temporarily frozen.

Reason:

Navigation foundations must be implemented first.

---

# RL Fixes Completed

Major RL bugs were fixed.

Previously:

```text
coverage reward = broken
tracking reward = broken
collision penalty = broken
episode reset = inconsistent
reward visibility = poor
```

Now fixed:

✅ Coverage reward

✅ Tracking reward

✅ Collision penalty

✅ Environment reset

✅ Reward diagnostics

✅ Stable config parsing

Result:

Future RL experiments are now reproducible and reliable.

---

# Technical Reality (Important)

Current drone movement is:

### Supervisor-based movement

Drone position is controlled via:

```text
setSFVec3f()
```

This means:

### NOT real UAV physics yet.

The current baseline is:

```text
controlled deterministic navigation
```

NOT full flight autonomy.

This is intentional for engineering validation.

---

# Development Roadmap

The roadmap is intentionally sequential.

We are NOT skipping steps.

---

# Phase 1 — Single Drone Baseline (Current)

Goal:

Build reliable navigation fundamentals.

Environment:

```text
downtown.wbt
```

Only:

```text
UAV_0
```

Tasks:

### 1. Fixed Start → Target

Deterministic benchmark.

### 2. Stable Altitude

Maintain:

```text
15m
```

### 3. Obstacle Interaction Testing

Verify:

```text
collision behavior
building clipping
physics reactions
```

### 4. Collision Avoidance

Implement:

```text
safety radius
distance threshold
repulsive steering
```

### 5. Path Planning

Implement:

```text
A*
RRT
paper algorithms
```

Success criteria:

Drone reaches target while safely navigating around buildings.

---

# Phase 2 — Dynamic Environment

Introduce:

```text
moving pedestrians
dynamic obstacles
```

Goal:

Stress-test navigation.

Tasks:

* obstacle awareness
* moving-object avoidance
* route adaptation

---

# Phase 3 — Swarm Scaling

Reactivate:

```text
UAV_1–UAV_4
```

Move from:

```text
single drone
```

to:

```text
multi-drone coordination
```

Tasks:

* decentralized movement
* drone-drone avoidance
* formation logic
* distributed surveillance

---

# Phase 4 — Reinforcement Learning

RL becomes:

### Optimization Layer

NOT primary navigation.

RL will optimize:

* coverage
* target tracking
* coordination
* surveillance efficiency
* adaptive routing

Algorithms roadmap:

```text
PPO
→ MAPPO
→ QMIX
```

---

# How To Run

## List Scenarios

```bash
python run_simulation.py --list
```

---

## Downtown (default)

```bash
python run_simulation.py
```

or

```bash
python run_simulation.py --scenario downtown
```

---

## Event Scenario

```bash
python run_simulation.py --scenario event
```

---

## Headless Mode

```bash
python run_simulation.py --headless
```

---

# Branch Strategy

### main

Stable branch.

Always runnable.

---

### single-drone-navigation

Current experimental branch.

Contains:

```text
single drone baseline
navigation testing
collision experiments
```

---

# Team Workflow

Before working:

```bash
git checkout single-drone-navigation
git pull origin single-drone-navigation
```

Run:

```bash
python run_simulation.py
```

Observe:

* drone motion
* building interactions
* console warnings
* collision behavior

Do NOT start RL training now.

Current priority:

```text
single drone navigation
→ collision avoidance
→ path planning
→ swarm
→ RL
```

---

# Current Status Summary

We are no longer randomly experimenting.

We now follow a controlled engineering roadmap:

```text
single drone
→ collision avoidance
→ path planning
→ dynamic environment
→ swarm
→ RL optimization
```

The current milestone is:

### A deterministic single-drone navigation baseline inside an urban environment.

