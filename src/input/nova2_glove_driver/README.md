# Nova2 UDP Glove Driver

This package is the Ubuntu-side bridge for SenseGlove Nova 2 teleoperation. It no longer links SGCore on Ubuntu. A Windows host reads Nova 2 through the official SenseGlove SDK, sends UDP JSON, and this ROS 2 node converts that JSON to the existing `manus_ros2_msgs/msg/ManusGlove` topics.

```text
Windows Nova 2 reader
  -> UDP JSON
  -> nova2_udp_bridge.py
  -> /manus_glove_0 and /manus_glove_1
  -> manus_revo2_retarget
  -> revo2_pid_controller
  -> MuJoCo / RViz / Revo2 hardware
```

The existing MANUS and Hex input paths are not changed.

## Why UDP

The Linux SGCore binaries supplied with the available SDK copies do not currently match Ubuntu 22.04 / ROS 2 Humble cleanly:

- one Linux SDK uses a different C++ standard library ABI;
- the Ubuntu SDK copy in `senseglove_ros` needs newer `GLIBC` / `GLIBCXX` symbols than Ubuntu 22.04 provides.

So the supported route in this workspace is:

```text
Windows: SenseCom + SGCore + Nova 2
Ubuntu: ROS 2 Humble + UDP receiver + Revo2 stack
```

## Windows Sender

Use the prepared Windows-side package:

```text
nova2_windows_bridge/README_CN.md
```

The sender publishes UDP JSON to the Ubuntu machine. Default port:

```text
15020
```

Expected JSON shape:

```json
{
  "source": "senseglove_nova2",
  "version": 3,
  "timestamp_ms": 123456789,
  "hands": [
    {
      "side": "right",
      "glove_id": 1,
      "connected": true,
      "sensor_channels": {
        "thumb_flexion": 0,
        "index_flexion_proximal": 0,
        "index_flexion_distal": 0,
        "middle_flexion": 0,
        "ring_flexion": 0,
        "thumb_abduction": 0
      },
      "hand_angles_rad": [],
      "normalized_flexion": [],
      "joint_positions_mm": [],
      "joint_rotations_xyzw": []
    }
  ]
}
```

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
python -m colcon build --symlink-install --packages-select \
  manus_ros2_msgs nova2_glove_driver
source install/setup.bash
```

This build does not require SGCore headers or SenseGlove `.so` files on Ubuntu.

## Launch With Revo2 Simulation

Start the Ubuntu receiver and include the existing Revo2 retarget pipeline:

```bash
ros2 launch nova2_glove_driver nova2_glove_pipeline.launch.py \
  hand_mode:=right \
  if_sim:=true \
  udp_port:=15020 \
  launch_plot:=false
```

Then start the Windows reader and send to the Ubuntu IP address on the same port.

## Topics

Default outputs:

```text
/manus_glove_0  left ManusGlove
/manus_glove_1  right ManusGlove
```

The retarget node also checks `msg.side`, so keep both the topic and `side` consistent.

## Parameters

Main parameters live in `config/nova2_mapping.yaml`:

```yaml
listen_host: 0.0.0.0
udp_port: 15020
poll_rate_hz: 500.0
enable_left: true
enable_right: true
left_topic: /manus_glove_0
right_topic: /manus_glove_1
position_scale: 0.001
left_flex_sign: 1.0
right_flex_sign: 1.0
left_thumb_spread_sign: 1.0
right_thumb_spread_sign: -1.0
timeout_ms: 200
publish_raw_nodes: true
publish_ergonomics: true
```

Each ergonomics channel supports:

```text
mapping.<Field>.offset
mapping.<Field>.scale
mapping.<Field>.sign
mapping.<Field>.min
mapping.<Field>.max
```

Formula:

```text
output_degrees = clamp(sign * (raw_degrees + offset) * scale, min, max)
```

Do real Nova 2 direction, zero, and range tuning in YAML.

## Safety

When UDP times out, the bridge stops publishing new `ManusGlove` messages. It does not repeatedly publish an all-zero hand pose, because the downstream Revo2 stack may treat that as a valid target.

Keep `if_sim:=true` until `/manus_glove_1` or `/manus_glove_0` is stable and the MuJoCo/RViz model moves in the expected direction.

## Mapping Status

The bridge maps:

- `hand_angles_rad`: radians to degrees for `ManusErgonomics`;
- `joint_positions_mm`: millimeters to meters using `position_scale`;
- `joint_rotations_xyzw`: copied to ROS quaternions as `x, y, z, w`.

The receiver uses the SDK order already emitted by the Windows reader:

- finger order: thumb, index, middle, ring, pinky;
- `hand_angles_rad`: finger -> joint -> Euler XYZ, in radians;
- `joint_positions_mm`: finger -> node -> XYZ, in millimeters, scaled by `position_scale`;
- `joint_rotations_xyzw`: finger -> node -> quaternion `x, y, z, w`;
- `normalized_flexion`: finger order above, values from 0 to 1.

The receiver selects each hand by `hands[].side`, so left/right/both sender modes are supported. It does not depend on the array index to identify the hand. `sensor_channels`, IMU, battery, and charging fields are accepted in the UDP frame but are not currently published into `ManusGlove` messages.

The exact signs, open-hand offsets, thumb spread direction, and comfortable limits still need validation on physical Nova 2 gloves.
