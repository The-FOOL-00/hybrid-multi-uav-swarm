This directory stores single-drone navigation run metrics.

Each run creates a timestamped subdirectory:

  experiments/single_drone/YYYYMMDD_HHMMSS/metrics.json

metrics.json fields:
  phase                    — "Phase 1 — Single Drone Navigation Baseline"
  world                    — world file used
  start                    — [x, y, z] start position
  target                   — [x, y, z] target position
  reached_target           — true/false
  travel_time_s            — simulation seconds to reach target
  distance_travelled_m     — total 3-D path length (m)
  collision_count          — number of steps within SAFETY_RADIUS of a building
  altitude_stability_std_m — standard deviation of altitude readings (smaller = more stable)
  total_steps              — total simulation steps run
  step_log                 — sparse trajectory [{step, x, y, z, state, dist_to_target}, ...]

To run the single drone world:
  Open Webots → File → Open World → worlds/single_drone_downtown.wbt
  Press Play ▶
