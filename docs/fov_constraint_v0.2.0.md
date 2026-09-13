# FOV constraint update v0.2.0

The test-aircraft viewing limits are now the direct IBVS constraints:

- horizontal half angle: `+/-26.6 deg`;
- vertical half angle: `+/-20.6 deg`;
- equivalent full FOV: `53.2 x 41.2 deg`.

With the current `fx=fy=640`, the constraints correspond to approximately
`+/-320.5 px` horizontally and `+/-240.6 px` vertically from image center. For
the current 960x540 geometry these limits remain inside the processed image.

No additional hidden `0.8` margin is applied. The limits are configured by
`fov_horizontal_limit_deg` and `fov_vertical_limit_deg`.

Let

```text
rho_h = abs(horizontal image angle) / 26.6 deg
rho_v = abs(vertical image angle) / 20.6 deg
```

For each axis, the controller uses the logarithmic barrier shape from the
paper, adapted to a rectangular camera FOV:

```text
B(rho) = -0.5 log(1 - rho^2),  abs(rho) < 1
```

The corresponding control multiplier is implemented as

```text
gain(rho) = 1 + k_fov * rho^2 / (1 - rho^2)
```

and is capped by `fov_barrier_max` because the velocity/yaw-rate interface has
finite actuation authority. Horizontal and vertical gains are independent.
Forward speed reduction and emergency yaw activation use
`max(rho_h, rho_v)`.

The numerical limits are reasonable when they describe the usable processed
image area after crop/resize, rather than only the lens datasheet FOV. If the
detector later changes image crop or resolution, these limits must be measured
again.
