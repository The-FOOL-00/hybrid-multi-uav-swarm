"""
altitude_control
================
Phase 1 placeholder — future altitude controllers.

Phase 1 (current): Simple P-controller in SingleDroneController._altitude_correction()
    correction = Kp * (TARGET_ALT - current_z)
    clamped to ±MAX_ALT_CORRECT per step

Phase 2 plan:
    - PIDController  : full PID with integral wind-up protection
    - AdaptivePID    : gain scheduling based on altitude error magnitude
    - LQRController  : Linear Quadratic Regulator for vertical axis

Usage (planned):
    from altitude_control.pid import PIDController
    pid = PIDController(kp=0.08, ki=0.002, kd=0.01)
    correction = pid.update(error=TARGET_ALT - current_z, dt=timestep_s)

Notes:
    Since we use supervisor teleport (setSFVec3f), altitude "control" is simply
    choosing the Z component of the next translation. A true PID over physics
    motors would be needed for a physics-based flight model.
"""
# Nothing extra implemented yet — see SingleDroneController._altitude_correction()
