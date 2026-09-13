# Fixed-camera controller V1

## Purpose

This version keeps the existing detector messages and MAVROS velocity command
interface. It only adds a new controller for a forward-looking camera rigidly
attached to the aircraft.

Reference: Yang et al., *High-Speed Interception Multicopter Control by
Image-based Visual Servoing*, arXiv:2404.08296.

## Data flow

```text
kcf_msgs/Bbox or detection_msgs/Detection
                  |
                  v
       camera optical LOS at t-delay
                  |
       fixed camera-to-body transform
                  |
       MAVROS attitude history at t-delay
                  |
                  v
        inertial LOS + LOS angular rate
                  |
       predict LOS to current control time
                  |
       current attitude -> current body LOS
                  |
        FOV barrier + PNG/LOS control
                  |
                  v
/mavros/setpoint_velocity/cmd_vel
```

## What removes aircraft-motion interference

The controller does not subtract yaw/pitch/roll as independent Euler angles.
It rotates the camera ray into the MAVROS local frame with the aircraft
quaternion sampled from a short attitude history. LOS angular rate is estimated
in that inertial frame, so aircraft yaw, pitch and roll do not appear as target
motion. The estimated inertial LOS is then transformed back with the current
attitude to obtain the LOS that the fixed camera should see now.

The attitude history is fed by `/mavros/imu/data` and also accepts
`/mavros/local_position/pose` as a fallback. Both topics carry the flight
controller attitude through MAVROS; no raw MAVLink parser is needed.

## Fixed camera coordinate convention

- Camera optical frame: `+z` forward through the lens, `+x` image-right,
  `+y` image-down.
- Aircraft body frame: ROS FLU, `+x` forward, `+y` left, `+z` up.
- With zero installation correction, camera `+z` maps to body `+x`.
- The configured installation is roll `0 deg`, yaw `0 deg`, and the optical
  axis raised `20 deg`. Under the FLU right-hand convention this is represented
  by `camera_mount_pitch_deg: -20.0`.

The center optical ray therefore points in body coordinates approximately as
`[cos(20 deg), 0, sin(20 deg)] = [0.9397, 0, 0.3420]`.

`image_delay_sec` selects the attitude-history sample associated with the image.
This preserves the headerless visual messages while providing a practical
delay correction. Accurate calibration of this delay is important.

## FOV constraint

The paper constrains target LOS by choosing a desired LOS inside the camera FOV
and using a barrier Lyapunov term. V1 keeps the same idea while retaining the
existing velocity interface:

- the desired LOS defaults to the optical center;
- the measured half-angle limits are used directly: horizontal `+/-26.6 deg`
  and vertical `+/-20.6 deg` (full FOV `53.2 x 41.2 deg`);
- horizontal and vertical logarithmic barrier terms are evaluated separately;
- control gain grows as predicted LOS approaches either boundary;
- yaw control bypasses the normal speed gate near the boundary;
- forward speed decreases near the boundary to leave time for recentering.

The debug topic `/fixed_camera_ibvs/fov_state` publishes horizontal ratio,
vertical ratio and applied barrier scale. A ratio of `1.0` is the configured
safe boundary.

For each axis, `rho` is the absolute predicted image angle divided by its
half-angle limit. The implemented gain multiplier is
`1 + k * rho^2 / (1 - rho^2)`, capped by `fov_barrier_max` to respect actuator
limits. This is the rectangular, axis-wise form of the paper's logarithmic
barrier idea. `rho = 1` represents the measured usable FOV boundary.

## Important limitation

The paper outputs desired thrust and full body angular velocity on SO(3). The
existing project outputs `geometry_msgs/TwistStamped` velocity and yaw-rate
commands. V1 intentionally keeps that interface to minimize changes, so pitch
FOV correction is achieved indirectly through vertical velocity rather than a
full attitude/thrust controller. A later SO(3) version should be a separate node
and should only be introduced after the low-level autopilot command interface is
confirmed.

## First test order

1. Set the true camera intrinsics and physical horizontal/vertical FOV.
2. Verify image-center sign conventions while stationary.
3. Calibrate camera mount roll/pitch/yaw corrections.
4. Rotate the aircraft on the ground and confirm `los_world` remains nearly
   fixed for a stationary target.
5. Calibrate `image_delay_sec` using timestamped logs.
6. Start with low `V_default`, `max_yaw_rate`, and `max_vz` in SITL/tethered
   tests before outdoor interception.
