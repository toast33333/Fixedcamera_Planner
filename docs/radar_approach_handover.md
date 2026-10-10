# Radar approach and IBVS handover

## Scope

This node implements approach only. It does not contain a search state and it
does not fuse radar and camera measurements into one estimator. Radar provides
the target motion estimate; the camera is used only as the final handover
confirmation and by the existing IBVS controller after handover.

## Radar prediction

Four timestamped radar positions initialize a robust horizontal
constant-velocity fit. Subsequent measurements update a covariance
constant-velocity Kalman filter with innovation gating and robust weighting.
At control time, the current target position is extrapolated as:

```text
p_target(now) = p_filter + v_filter * (now - t_filter)
```

The default 9.5 s timeout tolerates two missed nominal 3 s updates while still
stopping stale-data approach commands. The default measurement standard
deviation of 5.7735 m per axis represents 10 m three-dimensional RMS error.

Global radar GPS is converted into the aircraft MAVROS local ENU frame using
the current aircraft GPS and local pose. A direct local `PoseStamped` input can
be selected with `radar_input_mode: local_pose`.

## Terminal geometry

The ideal target LOS is the camera optical center. With a 20 deg upward fixed
camera and a 25 m camera-to-target slant range, the desired camera position is
approximately 23.49 m behind and 8.55 m below the predicted target. Desired
aircraft yaw follows the target horizontal velocity direction. The configured
camera body offset is then removed to obtain the desired aircraft position.

The approach command is a local-ENU velocity command:

```text
v_test = v_target + Kp * (p_terminal - p_test)
```

The first term follows the moving target; the second closes the error to the
ideal terminal point. The horizontal correction limit follows a continuous
braking profile based on the remaining horizontal error to that point:

```text
v_correction,max = min(v_max, sqrt(2 * a_brake * max(e_xy - d_buffer, 0)))
```

This replaces the old behavior that slowed down merely because target range
crossed a fixed threshold. Yaw rate is proportional to desired yaw error. All
axes remain limited by configuration parameters.

The forward speed supplied to IBVS is:

```text
V_forward = clamp(|v_target,xy| + closing_margin, V_min, V_max)
```

## Handover and command ownership

Handover is latched only when all conditions are true:

- radar estimate is initialized and fresh;
- estimated target range is within the configured 18--38 m safety window;
- aircraft is within the configured terminal-position tolerance;
- yaw error is within the configured tolerance;
- commanded yaw rate and estimated closing speed are below their handover
  limits;
- the existing detector reports the target near image center for the required
  consecutive detections. An invalid frame, an off-center frame, or a gap
  longer than the detection timeout resets the consecutive count.

`fixed_camera_ibvs` publishes to
`/fixed_camera_ibvs/cmd_vel_candidate` in the integrated launch. The approach
node is the only publisher to `/mavros/setpoint_velocity/cmd_vel`: it publishes
the radar approach command before handover and forwards the IBVS candidate
after handover. Radar freshness is no longer used to gate control after this
one-way handover; only IBVS candidate freshness is checked. Therefore PX4
never receives two competing command streams.

## Diagnostics

- `/radar_approach/predicted_target`
- `/radar_approach/handover_pose`
- `/radar_approach/estimated_target_velocity`
- `/radar_approach/state`
- `/radar_approach/handover_complete`
