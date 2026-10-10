#!/usr/bin/env python3
"""Generate a Gazebo Classic world using PX4's standard wind plugin."""

import argparse
import math
import os
import shutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("base_world")
    parser.add_argument("output_world")
    parser.add_argument("--enabled", action="store_true")
    parser.add_argument("--mean-mps", type=float, default=0.0)
    parser.add_argument("--max-mps", type=float, default=5.0)
    parser.add_argument("--variance", type=float, default=0.0)
    parser.add_argument("--direction", nargs=3, type=float,
                        metavar=("X", "Y", "Z"), default=(0.0, 1.0, 0.0))
    parser.add_argument("--publish-rate-hz", type=float, default=20.0)
    parser.add_argument("--demo-gui", action="store_true",
                        help="add a fullscreen tracking camera for video capture")
    parser.add_argument("--demo-camera-model", default="iris_1")
    parser.add_argument("--demo-camera-xyz", nargs=3, type=float,
                        metavar=("X", "Y", "Z"), default=(-80.0, -50.0, 60.0))
    args = parser.parse_args()

    scalar_values = (
        args.mean_mps, args.max_mps, args.variance, args.publish_rate_hz,
        *args.direction, *args.demo_camera_xyz)
    if not all(math.isfinite(value) for value in scalar_values):
        raise SystemExit("wind parameters must all be finite")
    if args.mean_mps < 0.0 or args.max_mps < args.mean_mps:
        raise SystemExit("wind speed must satisfy 0 <= mean <= maximum")
    if args.variance < 0.0:
        raise SystemExit("wind velocity variance must be non-negative")
    if args.publish_rate_hz <= 0.0:
        raise SystemExit("wind publish rate must be positive")
    direction_norm = math.sqrt(sum(value * value for value in args.direction))
    if direction_norm <= 1.0e-9:
        raise SystemExit("wind direction must be a non-zero vector")
    normalized_direction = tuple(
        value / direction_norm for value in args.direction)

    os.makedirs(os.path.dirname(os.path.abspath(args.output_world)), exist_ok=True)
    if not args.enabled and not args.demo_gui:
        shutil.copyfile(args.base_world, args.output_world)
        return

    with open(args.base_world, encoding="utf-8") as stream:
        world = stream.read()
    if args.enabled and "libgazebo_wind_plugin.so" in world:
        raise SystemExit("base world already contains the PX4 wind plugin")
    marker = "    <physics "
    if marker not in world:
        raise SystemExit("physics element not found in base world")
    direction = "%.9g %.9g %.9g" % normalized_direction
    additions = ""
    if args.demo_gui:
        camera_xyz = "%.9g %.9g %.9g" % tuple(args.demo_camera_xyz)
        additions += """    <gui fullscreen='true'>
      <camera name='user_camera'>
        <view_controller>orbit</view_controller>
        <track_visual>
          <name>{model}</name>
          <static>true</static>
          <use_model_frame>true</use_model_frame>
          <xyz>{camera_xyz}</xyz>
          <inherit_yaw>false</inherit_yaw>
        </track_visual>
      </camera>
    </gui>
""".format(model=args.demo_camera_model, camera_xyz=camera_xyz)
    if args.enabled:
        additions += """    <plugin name='wind_plugin' filename='libgazebo_wind_plugin.so'>
      <frameId>base_link</frameId>
      <robotNamespace/>
      <publishRate>{publish_rate:.9g}</publishRate>
      <windVelocityMean>{mean:.9g}</windVelocityMean>
      <windVelocityMax>{maximum:.9g}</windVelocityMax>
      <windVelocityVariance>{variance:.9g}</windVelocityVariance>
      <windDirectionMean>{direction}</windDirectionMean>
      <windDirectionVariance>0</windDirectionVariance>
      <windGustStart>0</windGustStart>
      <windGustDuration>0</windGustDuration>
      <windGustVelocityMean>0</windGustVelocityMean>
      <windGustVelocityMax>0</windGustVelocityMax>
      <windGustVelocityVariance>0</windGustVelocityVariance>
      <windGustDirectionMean>1 0 0</windGustDirectionMean>
      <windGustDirectionVariance>0</windGustDirectionVariance>
      <windPubTopic>world_wind</windPubTopic>
    </plugin>
""".format(
            publish_rate=args.publish_rate_hz,
            mean=args.mean_mps,
            maximum=args.max_mps,
            variance=args.variance,
            direction=direction)
    world = world.replace(marker, additions + marker, 1)
    with open(args.output_world, "w", encoding="utf-8") as stream:
        stream.write(world)


if __name__ == "__main__":
    main()
