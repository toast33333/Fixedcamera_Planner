# Fixed-camera controller V1

## Purpose

This version keeps the existing detector messages and MAVROS velocity command
interface. It only adds a new controller for a forward-looking camera rigidly
attached to the aircraft.

Reference: Yang et al., *High-Speed Interception Multicopter Control by
Image-based Visual Servoing*, arXiv:2404.08296.

## Data flow

```text
detection_msgs/Detection.header.stamp = camera acquisition time
                  |
                  v
       camera optical LOS at t_capture
                  |
       fixed camera-to-body transform
                  |
       interpolate IMU attitude at t_capture
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

The attitude history is fed by `/mavros/imu/data`. The detector and IMU must
use the same ROS time domain; if they run on different computers, their clocks
must be synchronized (for example with chrony/PTP).

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

The controller reads `Detection.header.stamp` and interpolates the IMU
quaternion at that exact acquisition time. Inference and ROS transport latency
may vary without requiring a manually estimated `image_delay_sec`. The legacy
headerless `/object_kcf` input remains available only as a compatibility path;
it falls back to message receipt time and receives no assumed delay correction.

## FOV constraint

The paper constrains target LOS by choosing a desired LOS inside the camera FOV
and using a barrier Lyapunov term. V1 keeps the same idea while retaining the
existing velocity interface:

- the desired LOS defaults to the optical center;
- the measured half-angle limits are used directly: horizontal `+/-26.6 deg`
  and vertical `+/-20.6 deg` (full FOV `53.2 x 41.2 deg`);
- the base interception equations are unchanged in the central 50% of the
  usable image area;
- horizontal and vertical additive guard terms are evaluated separately only
  outside that central region;
- the guard magnitude grows smoothly as predicted LOS approaches a boundary;
- yaw control bypasses the normal speed gate near the boundary;
- forward speed can optionally decrease near the boundary to leave time for
  recentering (disabled by the current `fov_min_forward_speed_ratio: 1.0`).

The debug topic `/fixed_camera_ibvs/fov_state` publishes horizontal ratio,
vertical ratio and the maximum dimensionless guard shape. A horizontal or
vertical ratio of `1.0` is the configured warning boundary.

For each axis, `rho` is the absolute predicted pixel error divided by the
physical usable half-extent. Let `a = sqrt(fov_guard_center_area_ratio)` and
`q = clip((rho-a)/(1-a), 0, 0.999)`. The guard is exactly zero for `rho <= a`.
Outside it, the additive magnitude is
`gain * min(q^2/(1-q^2), fov_guard_shape_max)`. The two independent gains are
`fov_guard_yaw_gain_radps` and `fov_guard_vertical_gain_mps`, so yaw and vertical
tuning have explicit units. The sign points toward the image center, and the
result is added to the base yaw-rate or vertical-speed command rather than
multiplying any base interception gain.

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
5. Verify `/object_detections/header/stamp` and `/mavros/imu/data/header/stamp`
   use the same synchronized ROS clock.
6. Start with low `V_default`, `max_yaw_rate`, and `max_vz` in SITL/tethered
   tests before outdoor interception.
