# Workflow Guide

**Project:** Hybrid Multi-UAV Swarm — STIRS-2025  
**Simulator:** Webots R2025a

---

## Two Execution Modes

The simulation has a **single codebase** with two modes controlled by a flag.
No logic is duplicated. Only the rendering behavior changes.

---

## Mode 1 — GUI Mode (Presentation & Debugging)

### Purpose

| Use case | Why GUI? |
|----------|----------|
| Demo to guide / supervisor | Full 3D visualization |
| Debugging drone movement | See exact positions in real-time |
| Observing crowd behavior | Watch pedestrians animate |
| Screenshots for paper | Webots screenshot tools |
| Videos for paper | Webots animation recorder |
| First-time scenario check | Verify world loads correctly |

### How to Run

```powershell
# Default scenario (downtown)
python run_simulation.py

# Choose a specific scenario
python run_simulation.py --scenario downtown
python run_simulation.py --scenario event
python run_simulation.py --scenario residential
python run_simulation.py --scenario mixed
python run_simulation.py --scenario industrial
```

### What You See

- Full Webots window opens
- 3D city environment with buildings, roads, terrain
- 5 DJI Mavic2Pro drones flying in circular formation
- Crowd pedestrians walking along trajectories
- Birds flocking in the environment
- Console output showing coverage metrics every 125 steps

### Taking Screenshots (for paper figures)

In Webots:
- `Tools → Take Screenshot` — saves PNG
- Save to `experiments/screenshots/`
- Name format: `<scenario>_step_<N>.png`

### Recording Video (for paper supplementary)

In Webots:
- `Tools → Start Animation` → saves `.html` or `.x3d`
- For MP4: use OBS or Webots' built-in animation recorder
- Save to `experiments/videos/`

---

## Mode 2 — Headless Mode (Training & Research)

### Purpose

| Use case | Why Headless? |
|----------|--------------|
| RL reward tuning | 3–5× faster without rendering |
| Coverage optimization experiments | Batch overnight runs |
| Collision avoidance testing | No GUI overhead |
| Metrics collection | Same controllers, same world |
| Multiple scenario sweeps | Run all 5 scenarios sequentially |

### How to Run

```powershell
# Headless (no GUI window)
python run_simulation.py --headless

# Headless with specific scenario
python run_simulation.py --scenario event --headless

# Both flags work the same
python run_simulation.py --no-rendering --scenario industrial
```

### What Changes in Headless Mode

Webots is launched with these flags:
```
--batch --minimize --no-rendering
```

- No graphical window
- Faster simulation steps (no render latency)
- Same controllers execute identically
- Same world loads, same agents move
- Same metrics logged to stdout

### What Does NOT Change

- `uav_swarm_controller.py` — identical
- `bird_controller.py` — identical
- `uav_camera.py` — identical
- `crowd_controller.py` — identical
- All 5 world files — unchanged
- Coverage computation — unchanged
- POMDP attention mechanism — unchanged

---

## Team Workflow

### Daily Development

```
Day's work → GUI mode to verify → headless mode for experiments
```

1. **Edit controller logic** → test in GUI mode first
2. **Verify visually** → check drone paths look correct
3. **Switch to headless** → collect metrics for analysis
4. **Save metrics** → `experiments/metrics/`

### For Paper Results

```
python run_simulation.py --scenario downtown --headless   # collect metrics
python run_simulation.py --scenario event --headless      # collect metrics
python run_simulation.py --scenario residential --headless
python run_simulation.py --scenario mixed --headless
python run_simulation.py --scenario industrial --headless
```

Then aggregate results from Webots console output.

### For Demo to Guide

```powershell
python run_simulation.py --scenario event
```

Show the event scenario — highest crowd density, most visible UAV tracking.

---

## Adding a New Scenario

1. Create `worlds/new_scenario.wbt` in Webots
2. Add entry to `SCENARIOS` dict in `run_simulation.py`:
   ```python
   "new_scenario": "worlds/new_scenario.wbt"
   ```
3. Add info entry to `INFO` dict:
   ```python
   "new_scenario": {"buildings": N, "crowd": N, "uavs": 5, "birds": N}
   ```
4. Add to `configs/environment_config.yaml` under `scenarios:`
5. Test: `python run_simulation.py --scenario new_scenario`

## Adding a New Controller

1. Create folder: `controllers/my_controller/`
2. Create file: `controllers/my_controller/my_controller.py`
3. In `.wbt` file, set `controller "my_controller"` on the relevant Robot node
4. No import path changes needed — Webots resolves by folder name

---

## File Output Locations

| Output type | Location |
|-------------|----------|
| Coverage metrics (console) | stdout during simulation |
| Experiment JSON metrics | `experiments/metrics/<run_id>/` |
| Simulation logs | `experiments/logs/<run_id>/` |
| Screenshots | `experiments/screenshots/` |
| Videos | `experiments/videos/` |
| Trained RL models | `models/<run_id>/` |

---

## Common Commands Reference

```powershell
# List available scenarios
python run_simulation.py --list

# GUI mode (default scenario: downtown)
python run_simulation.py

# GUI mode (specific scenario)
python run_simulation.py --scenario event

# Headless mode
python run_simulation.py --headless

# Headless + specific scenario
python run_simulation.py --scenario industrial --headless

# Help
python run_simulation.py --help
```
