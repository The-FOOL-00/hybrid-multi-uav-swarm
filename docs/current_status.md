# Current Project Status

**Last Updated:** May 2026  
**Project:** Attention-Guided Decentralized Multi-UAV Surveillance — STIRS-2025  
**Institution:** SSN College of Engineering  
**Researchers:** Sharruk S, Sundareswaran, Jeswin Joel  
**Guide:** Dr. E. Selvam

---

## ✅ What Is Working

### Simulation Core

| Component | Status | Notes |
|-----------|--------|-------|
| Webots R2025a launches | ✅ Working | Via `python run_simulation.py` |
| All 5 world scenarios load | ✅ Working | downtown, event, residential, mixed, industrial |
| UAV movement (5 drones) | ✅ Working | Circular formation, POMDP attention |
| Bird flocking (5–8 birds) | ✅ Working | Boids-lite orbit with drift |
| Crowd pedestrians | ✅ Working | Built-in Webots `pedestrian` controller |
| Coverage metric logging | ✅ Working | Printed every 125 steps |
| GUI mode | ✅ Working | `python run_simulation.py` |
| Headless mode | ✅ Working | `python run_simulation.py --headless` |

### Controllers

| Controller | Status | Notes |
|-----------|--------|-------|
| `uav_swarm_controller.py` | ✅ Working | Supervisor, POMDP attention α=0.15 |
| `uav_camera.py` | ✅ Working | Camera + propeller spin |
| `bird_controller.py` | ✅ Working | Per-bird self-supervising |
| `crowd_controller.py` | ✅ Present | For CrowdAgent PROTOs (optional) |

### Custom PROTOs

| PROTO | Status |
|-------|--------|
| `Bird.proto` | ✅ Working |
| `CrowdAgent.proto` | ✅ Present |

---

## 🏗️ What Was Added (This Restructuring)

### New Folder Structure

| Folder | Purpose | Status |
|--------|---------|--------|
| `rl/` | RL training skeleton | 🟡 Skeleton only |
| `rl/gym_wrapper/` | OpenAI Gym env stub | 🟡 Interface defined, IPC pending |
| `rl/rewards/` | Reward functions | 🟡 Stubs with TODO implementations |
| `rl/training/` | Training loop | 🟡 Architecture ready, IPC pending |
| `rl/algorithms/` | Algorithm roadmap | 🟡 Documentation, no code yet |
| `experiments/metrics/` | JSON metrics output | ✅ Ready to use |
| `experiments/screenshots/` | Paper figures | ✅ Ready to use |
| `experiments/videos/` | Paper supplementary | ✅ Ready to use |
| `experiments/logs/` | Simulation logs | ✅ Ready to use |
| `configs/` | Canonical config location | ✅ Config with RL section added |
| `models/` | Saved RL models | ✅ Ready to use |
| `docs/` | Project documentation | ✅ Complete |

---

## 📋 What Remains To Do

### Research Phase (Next Steps in Order)

1. **Collect baseline metrics** — Run all 5 scenarios headless, record coverage %
   ```powershell
   python run_simulation.py --scenario downtown --headless
   python run_simulation.py --scenario event --headless
   # ... etc.
   ```

2. **Paper figures** — Run GUI mode for each scenario, take screenshots
   ```powershell
   python run_simulation.py --scenario event
   # In Webots: Tools → Take Screenshot → save to experiments/screenshots/
   ```

3. **Tune POMDP parameters** — Experiment with `attention_alpha` in `configs/environment_config.yaml`
   - Currently: `alpha = 0.15`
   - Try: 0.10, 0.20, 0.30 — measure coverage change

4. **Implement RL (Phase 2)** — After baseline paper results:
   - Complete `rl/gym_wrapper/uav_swarm_env.py` IPC bridge
   - Implement `rl/rewards/reward_functions.py` compute() methods
   - Implement `rl/training/trainer.py` run() loop
   - Start with PPO from Stable-Baselines3

5. **MAPPO (Phase 3)** — Multi-agent RL after PPO baseline is established

---

## 📐 Key Parameters

| Parameter | Value | Location |
|-----------|-------|----------|
| `attention_alpha` | 0.15 | `uav_swarm_controller.py` L86 |
| `patrol_radius` | 20.0 m | `uav_swarm_controller.py` L18 |
| `patrol_altitude` | 15.0 m | `uav_swarm_controller.py` L19 |
| `coverage_cell_size` | 5.0 m | `configs/environment_config.yaml` |
| `metrics_interval` | 125 steps | `configs/environment_config.yaml` |
| `timestep` | 8 ms | Webots WorldInfo |
| `num_uavs` | 5 | All worlds |

---

## 🚀 How to Run Right Now

```powershell
# GUI (demo)
python run_simulation.py

# GUI specific scenario
python run_simulation.py --scenario event

# Headless (metrics collection)
python run_simulation.py --headless

# See all options
python run_simulation.py --help

# List scenarios
python run_simulation.py --list
```

---

## ⚠️ Known Limitations

| Limitation | Impact | Resolution |
|-----------|--------|-----------|
| No Webots IPC bridge | RL training cannot start | Phase 2 work |
| Coverage logged to stdout only | No persistent metrics yet | Redirect stdout or use `Trainer` |
| No multi-run comparison tooling | Manual result comparison | Add after RL phase |
| `crowd_controller` not active by default | CrowdAgent PROTO unused in standard worlds | Add supervisor Robot node to world if needed |
| Webots path hardcoded | Breaks on different machines | Edit `WEBOTS_EXE` in `run_simulation.py` |

---

## 📁 Project Structure (Current)

```
hybrid-multi-uav-swarm/
├── worlds/                      ✅ 5 scenarios
│   ├── downtown.wbt
│   ├── event.wbt
│   ├── industrial.wbt
│   ├── mixed.wbt
│   └── residential.wbt
├── controllers/                 ✅ All working
│   ├── uav_swarm_controller/
│   ├── crowd_controller/
│   ├── bird_controller/
│   └── uav_camera/
├── rl/                          🟡 Skeleton ready
│   ├── gym_wrapper/
│   ├── rewards/
│   ├── training/
│   └── algorithms/
├── experiments/                 ✅ Ready to receive output
│   ├── metrics/
│   ├── screenshots/
│   ├── videos/
│   └── logs/
├── protos/                      ✅ 2 custom PROTOs
│   ├── Bird.proto
│   └── CrowdAgent.proto
├── configs/                     ✅ Canonical config (with RL section)
│   └── environment_config.yaml
├── models/                      ✅ Ready for RL checkpoints
├── docs/                        ✅ Complete documentation
│   ├── architecture.md
│   ├── workflow.md
│   └── current_status.md
├── run_simulation.py            ✅ Enhanced launcher
└── README.md                    ✅ Updated
```
