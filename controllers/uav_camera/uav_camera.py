"""
UAV Camera Controller for Mavic2Pro drones.
Enables the front camera and spins propellers for visual realism.
Position/movement is handled entirely by the swarm_supervisor via Supervisor API.
"""
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Enable front camera
camera = robot.getDevice("camera")
if camera:
    camera.enable(timestep)
    print(f"[{robot.getName()}] Camera {camera.getWidth()}x{camera.getHeight()} enabled")
else:
    print(f"[{robot.getName()}] WARNING: camera device not found")

# Spin propellers (visual effect; supervisor overrides position each step)
HOVER_RPM = 68.0  # rad/s — realistic hover speed for Mavic2Pro
for name in ["front left propeller", "front right propeller",
             "rear left propeller", "rear right propeller"]:
    motor = robot.getDevice(name)
    if motor:
        motor.setPosition(float("inf"))
        motor.setVelocity(HOVER_RPM)

while robot.step(timestep) != -1:
    pass
