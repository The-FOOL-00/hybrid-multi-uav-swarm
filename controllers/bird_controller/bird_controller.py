"""
Bird flocking controller — runs on each Bird PROTO instance.
Bird PROTO must have supervisor=TRUE (it does in Bird.proto).
Coordinate system: ENU (East=X, North=Y, Up=Z)
"""
from controller import Supervisor
import math
import random


def run():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    self_node = robot.getSelf()
    trans_field = self_node.getField("translation")

    # Seed per-bird variation from name hash
    name = robot.getName()
    try:
        bird_id = int(name.split("_")[-1])
    except (ValueError, IndexError):
        bird_id = abs(hash(name)) % 8

    random.seed(bird_id * 31 + 7)

    # Read initial world position
    init_pos = list(self_node.getPosition())
    cx = init_pos[0]    # East center
    cy = init_pos[1]    # North center
    alt = max(init_pos[2], 5.0)  # Z altitude; ensure at least 5m

    # Per-bird boids parameters
    radius = 10.0 + bird_id * 2.2
    speed_h = 0.28 + bird_id * 0.018   # rad/s horizontal
    speed_v = 0.007 + bird_id * 0.002  # rad/s vertical bob
    phase = bird_id * 0.8              # spread birds in time

    # Gentle drift of the orbit center
    drift_angle = random.uniform(0, 2 * math.pi)
    drift_speed = 0.0003 + bird_id * 0.00005

    step = 0
    while robot.step(timestep) != -1:
        step += 1
        t = step * timestep / 1000.0  # seconds

        # Drift orbit center slowly
        cx += drift_speed * math.cos(drift_angle + t * 0.01)
        cy += drift_speed * math.sin(drift_angle + t * 0.01)

        # Clamp to arena
        cx = max(-80.0, min(80.0, cx))
        cy = max(-80.0, min(80.0, cy))

        # Circular orbit
        angle = speed_h * t + phase
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)

        # Vertical bob
        z = alt + 2.5 * math.sin(speed_v * 2 * math.pi * t + phase)
        z = max(4.0, min(12.0, z))   # stay within altitude band

        trans_field.setSFVec3f([x, y, z])


if __name__ == "__main__":
    run()
