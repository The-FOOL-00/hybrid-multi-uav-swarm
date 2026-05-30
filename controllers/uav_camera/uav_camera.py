"""
UAV Camera + Sensor Controller for DJI Mavic 2 Pro drones.

Role in architecture:
    Runs on each of the 5 Mavic2Pro drones as a per-robot controller.
    Position/movement is handled entirely by the swarm_supervisor via
    Supervisor API (setSFVec3f) — this controller handles only:
        - Propeller spin (visual realism)
        - Camera enable (placeholder; no CV processing)
        - GPS, IMU, Gyro, Compass enable (sensors available for RL future use)
        - LED status blink (visual drone ID)

Sensors enabled (all built into Mavic2Pro PROTO body):
    GPS          - altitude and XY position readback
    InertialUnit - roll/pitch/yaw for orientation logging
    Gyro         - angular velocity
    Compass      - heading

NOTE: DO NOT add computer vision, object detection, or image analysis here.
      This controller is intentionally minimal.
"""
from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())
drone_name = robot.getName()

# ── Propellers ────────────────────────────────────────────────────────────────
# Spin at realistic hover speed; supervisor overrides translation each step.
# Counter-rotating pairs: FL+RR spin positive, FR+RL spin negative.
HOVER_RPM = 68.0  # rad/s — empirically correct for Mavic2Pro hover

propeller_names = [
    "front left propeller",
    "front right propeller",
    "rear left propeller",
    "rear right propeller",
]
propeller_signs = [1.0, -1.0, -1.0, 1.0]  # FL, FR, RL, RR counter-rotation

for name, sign in zip(propeller_names, propeller_signs):
    motor = robot.getDevice(name)
    if motor:
        motor.setPosition(float("inf"))
        motor.setVelocity(sign * HOVER_RPM)

# ── Camera ────────────────────────────────────────────────────────────────────
# Enable front camera as placeholder (no image processing performed).
camera = robot.getDevice("camera")
if camera:
    camera.enable(timestep)
    print(f"[{drone_name}] Camera {camera.getWidth()}x{camera.getHeight()} enabled (placeholder)")
else:
    print(f"[{drone_name}] WARNING: camera device not found")

# ── GPS ───────────────────────────────────────────────────────────────────────
gps = robot.getDevice("gps")
if gps:
    gps.enable(timestep)

# ── Inertial Unit (IMU) ───────────────────────────────────────────────────────
imu = robot.getDevice("inertial unit")
if imu:
    imu.enable(timestep)

# ── Gyro ──────────────────────────────────────────────────────────────────────
gyro = robot.getDevice("gyro")
if gyro:
    gyro.enable(timestep)

# ── Compass ───────────────────────────────────────────────────────────────────
compass = robot.getDevice("compass")
if compass:
    compass.enable(timestep)

# ── Camera gimbal stabilization motors (if available) ────────────────────────
camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")

# ── LEDs for visual drone identification ──────────────────────────────────────
front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")

print(f"[{drone_name}] DJI Mavic 2 Pro online | "
      f"GPS={'OK' if gps else 'N/A'} | "
      f"IMU={'OK' if imu else 'N/A'} | "
      f"Gyro={'OK' if gyro else 'N/A'}")

# ── Main loop ─────────────────────────────────────────────────────────────────
step_count = 0
LOG_INTERVAL = 125  # log every 125 steps to match supervisor metrics interval

while robot.step(timestep) != -1:
    step_count += 1
    t = step_count * timestep / 1000.0  # simulation time in seconds

    # ── LED status blink (alternating, 1 Hz) ─────────────────────────────────
    led_state = int(t) % 2
    if front_left_led:
        front_left_led.set(led_state)
    if front_right_led:
        front_right_led.set(1 - led_state)

    # ── Camera gimbal stabilization ───────────────────────────────────────────
    # Stabilize gimbal based on gyro feedback (keeps horizon level)
    if gyro and camera_roll_motor and camera_pitch_motor:
        gyro_vals = gyro.getValues()
        if gyro_vals:
            # Clamp roll and pitch target positions to joint limits [-0.49, 0.49] rad
            # to prevent out-of-bounds warnings and stabilize camera view
            roll_target = max(-0.49, min(0.49, -0.115 * gyro_vals[0]))
            pitch_target = max(-0.49, min(0.49, -0.1 * gyro_vals[1]))
            camera_roll_motor.setPosition(roll_target)
            camera_pitch_motor.setPosition(pitch_target)

    # ── Periodic sensor log (for research metrics) ────────────────────────────
    # NOTE: Position truth comes from supervisor (getPosition()).
    #       GPS here confirms sensor data is flowing for RL future use.
    if step_count % LOG_INTERVAL == 0 and gps and imu:
        gps_vals = gps.getValues()
        rpy = imu.getRollPitchYaw()
        if gps_vals and rpy:
            alt = gps_vals[2]  # ENU Z = altitude
            roll_deg = math.degrees(rpy[0])
            pitch_deg = math.degrees(rpy[1])
            print(f"  [{drone_name}] Alt={alt:5.1f}m "
                  f"Roll={roll_deg:5.1f}deg Pitch={pitch_deg:5.1f}deg")
