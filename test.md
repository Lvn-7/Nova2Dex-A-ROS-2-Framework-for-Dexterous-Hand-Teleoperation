# Test and Validation Command Reference

Run commands from the repository root unless a section says otherwise. Real
hardware can move unexpectedly; complete the build, message, and simulation
checks first.

## 1. Discover the workspace

```bash
source /opt/ros/humble/setup.bash
colcon list
```

Expected ROS packages include:

```text
manus_ros2_msgs
nova2_glove_driver
manus_revo2_retarget
nova2_inspire_retarget
revo2_description
revo2_driver
brainco_hand_dds_bridge
inspire_hand_dds_bridge
```

The standalone `inspire_service` CMake project may also be discovered by
colcon, depending on the colcon CMake extension installed on the host.

## 2. Python syntax check

```bash
python3 -m compileall -q src windows
```

Generated `__pycache__` directories are ignored by Git.

## 3. Inspire mapping unit test

The test uses the Python standard-library `unittest` runner:

```bash
PYTHONPATH=src/retargeting/nova2_inspire_retarget \
python3 src/retargeting/nova2_inspire_retarget/test/test_mapping.py
```

It checks open/closed values, clamping, midpoint mapping, and invalid input.

## 4. Revo2 thumb retargeting test

Install `requirements.txt` first, then run:

```bash
PYTHONPATH=src/retargeting/manus_revo2_retarget \
python3 src/retargeting/manus_revo2_retarget/test/test_revo3_thumb.py
```

This exercises range limiting, thumb IK response, ergonomics mapping,
smoothing, PIP constraints, and calibration.

## 5. ROS package tests

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The full build needs every optional hardware SDK. Build a subset when those
SDKs are intentionally absent.

### Nova 2 and Inspire simulation subset

```bash
colcon build --symlink-install --packages-select \
  manus_ros2_msgs nova2_glove_driver nova2_inspire_retarget
```

### Revo2 packages

```bash
colcon build --symlink-install --packages-up-to \
  revo2_driver manus_revo2_retarget
```

The pinned BrainCo SDK is included. Run `download_sdk.sh` only when that copy
needs to be refreshed or repaired.

### DDS bridges

After installing `unitree_sdk2`:

```bash
colcon build --symlink-install --packages-select \
  brainco_hand_dds_bridge inspire_hand_dds_bridge
```

## 6. Windows reader validation

On Windows, after placing the SenseGlove SDK and Qt as documented:

```powershell
cd windows\nova2_bridge\reader
$bridgeRoot = Split-Path -Parent (Get-Location)
$qtRoot = Join-Path $bridgeRoot "third_party\Qt\6.8.2\msvc2022_64"
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 "-DCMAKE_PREFIX_PATH=$qtRoot"
cmake --build build --config Release
```

Start SenseCom, run the reader, and direct UDP to the Ubuntu host on port
`15020`.

## 7. Nova 2 ROS input checks

```bash
ros2 topic hz /manus_glove_1
ros2 topic echo /manus_glove_1 --field ergonomics --once
```

For a left glove, replace `_1` with `_0`. Confirm `msg.side` agrees with the
topic.

## 8. Revo2 simulation validation

```bash
ros2 launch nova2_glove_driver nova2_glove_pipeline.launch.py \
  hand_mode:=right \
  enable_left:=false \
  enable_right:=true \
  if_sim:=true \
  udp_port:=15020 \
  launch_plot:=false
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers \
  -c /revo2_right/controller_manager
```

Optional viewer:

```bash
ros2 run manus_revo2_retarget mujoco_manus_overlay_viewer \
  --hand-mode right
```

## 9. Inspire simulation validation

```bash
ros2 launch nova2_inspire_retarget sim_pipeline.launch.py \
  hand_mode:=right \
  udp_port:=15020 \
  launch_viewer:=true
```

Check the mapped target:

```bash
ros2 topic echo /inspire_right/retarget/joint_states --once
```

## 10. Revo2 hardware-driver setup check

Run before connecting control output:

```bash
bash src/hardware/brainco/revo2_driver/setup/check_revo2_setup.sh
```

Direct right-hand Modbus launch:

```bash
ros2 launch revo2_driver revo2_system.launch.py \
  hand_side:=right protocol:=modbus if_sim:=false
```

Do not run this until port identity, joint direction, and command limits have
been checked.

## 11. BrainCo Revo2 DDS validation

```bash
ros2 launch brainco_hand_dds_bridge nova2_right_dds_pipeline.launch.py \
  udp_port:=15020 \
  network_interface:=<ubuntu-interface>
```

Check the target before starting the G1 serial service:

```bash
ros2 topic echo \
  /revo2_right/revo2_pid_controller/target_joint_states --once
```

## 12. Inspire DDS validation

```bash
ros2 launch inspire_hand_dds_bridge nova2_inspire_dds_pipeline.launch.py \
  hand_mode:=right \
  udp_port:=15020 \
  network_interface:=<ubuntu-interface>
```

Build and test the G1-side service separately:

```bash
cd src/hardware/inspire/inspire_g1_service
cmake -S . -B build
cmake --build build -j6
./build/hand_all_test right open
./build/hand_all_test right close
```

Only issue movement tests after confirming the correct hand, serial bus, DDS
interface, and clear physical workspace.
