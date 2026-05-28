"""
Crowd supervisor controller — runs as a standalone supervisor Robot node.
Moves CrowdAgent nodes (custom PROTO with supervisor=TRUE) via state machine.
Coordinate system: ENU (East=X, North=Y, Up=Z). Pedestrian Z=1.27 for ground.

To activate: add this Robot node to a world file:
  Robot {
    name "crowd_supervisor"
    controller "crowd_controller"
    supervisor TRUE
    children [] boundingObject NULL physics NULL
  }
And use DEF CROWD_N CrowdAgent { ... controller "void" } for each agent.
"""
from controller import Supervisor
import math
import random


GROUND_Z = 1.27  # ENU Z for pedestrian center at ground level


class AgentState:
    WALK = "WALK"
    WAIT = "WAIT"
    GATHER = "GATHER"

    def __init__(self, node, agent_id):
        self.node = node
        self.trans = node.getField("translation")
        self.id = agent_id

        pos = list(node.getPosition())
        self.x = pos[0]
        self.y = pos[1]

        random.seed(agent_id * 17 + 3)
        self.speed = random.uniform(0.8, 1.6)
        self.state = AgentState.WALK
        self.timer = 0
        self.wait_ticks = random.randint(40, 180)

        # Generate waypoint loop
        angle_start = random.uniform(0, 2 * math.pi)
        r = random.uniform(8, 28)
        self.waypoints = [
            (self.x + r * math.cos(angle_start + i * 2 * math.pi / 6),
             self.y + r * math.sin(angle_start + i * 2 * math.pi / 6))
            for i in range(6)
        ]
        self.wp_idx = 0
        self.gather_x = random.uniform(-15, 15)
        self.gather_y = random.uniform(-15, 15)

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _move_toward(self, tx, ty, dt):
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.8:
            return True
        step = self.speed * dt
        self.x += dx / dist * step
        self.y += dy / dist * step
        self.x = self._clamp(self.x, -85, 85)
        self.y = self._clamp(self.y, -85, 85)
        return False

    def update(self, dt, gather_mode=False):
        self.timer += 1

        if gather_mode:
            self.state = AgentState.GATHER

        if self.state == AgentState.WALK:
            tx, ty = self.waypoints[self.wp_idx]
            reached = self._move_toward(tx, ty, dt)
            if reached:
                self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
                if random.random() < 0.12:
                    self.state = AgentState.WAIT
                    self.timer = 0
                    self.wait_ticks = random.randint(40, 180)

        elif self.state == AgentState.WAIT:
            if self.timer >= self.wait_ticks:
                self.state = AgentState.WALK
                self.timer = 0

        elif self.state == AgentState.GATHER:
            reached = self._move_toward(self.gather_x, self.gather_y, dt)
            if reached:
                # wander near gather point
                self.gather_x += random.uniform(-3, 3)
                self.gather_y += random.uniform(-3, 3)

        try:
            self.trans.setSFVec3f([self.x, self.y, GROUND_Z])
        except Exception:
            pass


def run():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0

    # Discover CrowdAgent nodes with DEF prefix CROWD_
    agents = []
    root = robot.getRoot()
    children = root.getField("children")
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node is None:
            continue
        def_name = node.getDef()
        if def_name and def_name.upper().startswith("CROWD_"):
            agents.append(AgentState(node, len(agents)))

    print(f"[Crowd Controller] Managing {len(agents)} CrowdAgent nodes")

    step = 0
    while robot.step(timestep) != -1:
        step += 1
        # Enable gather mode between step 500-700 (simulates event)
        gather = (500 < step < 700)
        for agent in agents:
            agent.update(dt, gather_mode=gather)

        if step % 200 == 0:
            states = {}
            for a in agents:
                states[a.state] = states.get(a.state, 0) + 1
            print(f"  Step {step}: {states}")


if __name__ == "__main__":
    run()
