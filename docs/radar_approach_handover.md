# Radar approach and IBVS handover

## Scope

This node implements approach only. It does not contain a search state and it
does not fuse radar and camera measurements into one estimator. Radar provides
the target motion estimate; the camera is used only as the final handover
confirmation and by the existing IBVS controller after handover.

## Radar prediction

For each radar position measurement `z` received at about 3 s intervals, an
alpha-beta constant-velocity filter applies:

```text
p_pred = p + v * dt
r      = z - p_pred
p      = p_pred + alpha * r
v      = v + beta / dt * r
```

At the 20 Hz control rate, the current target position is extrapolated as
`p_target(now) = p + v * (now - t_radar)`. A 7 s timeout stops the approach if
two expected radar updates are missed.

Global radar GPS is converted into the aircraft MAVROS local ENU frame using
the current aircraft GPS and local pose. A direct local `PoseStamped` input can
be selected with `radar_input_mode: local_pose`.

## Terminal geometry

The ideal target LOS is the camera optical center. With a 20 deg upward fixed
camera and a 20 m camera-to-target slant range, the desired camera position is
approximately 18.79 m behind and 6.84 m below the predicted target. Desired
aircraft yaw follows the target horizontal velocity direction. The configured
camera body offset is then removed to obtain the desired aircraft position.

The approach command is a local-ENU velocity command:

```text
v_test = v_target + Kp * (p_terminal - p_test)
```

The first term follows the moving target; the second closes the error to the
ideal terminal point. Yaw rate is proportional to desired yaw error. All axes
are limited by configuration parameters.

The forward speed supplied to IBVS is:

```text
V_forward = clamp(|v_target,xy| + closing_margin, V_min, V_max)
```

## Handover and command ownership

Handover is latched only when all conditions are true:

- radar estimate is initialized and fresh;
- aircraft is within the configured terminal-position tolerance;
- yaw error is within the configured tolerance;
- the existing detector reports the target near image center for the required
  consecutive detections.

`fixed_camera_ibvs` publishes to
`/fixed_camera_ibvs/cmd_vel_candidate` in the integrated launch. The approach
node is the only publisher to `/mavros/setpoint_velocity/cmd_vel`: it publishes
the radar approach command before handover and forwards the IBVS candidate
after handover. Therefore PX4 never receives two competing command streams.

## Diagnostics

- `/radar_approach/predicted_target`
- `/radar_approach/handover_pose`
- `/radar_approach/estimated_target_velocity`
- `/radar_approach/state`
- `/radar_approach/handover_complete`
