# Camera mount update v0.3.0

The fixed camera optical axis is configured `20 deg` upward relative to the
aircraft forward axis, with no roll or yaw installation offset.

Coordinate convention:

```text
camera optical: +z forward, +x right, +y down
aircraft body:  +x forward, +y left, +z up (ROS FLU)
```

The nominal optical-to-body mapping is:

```text
[x_body, y_body, z_body] = [z_camera, -x_camera, -y_camera]
```

The installation correction then rotates this vector about body `+y` by
`-20 deg`. Consequently, the optical center ray in body coordinates is:

```text
[cos(20 deg), 0, sin(20 deg)] ~= [0.9397, 0, 0.3420]
```

Configuration:

```yaml
camera_mount_roll_deg: 0.0
camera_mount_pitch_deg: -20.0
camera_mount_yaw_deg: 0.0
```
