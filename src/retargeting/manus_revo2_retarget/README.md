# manus_revo2_retarget in Nova2Dex

This package keeps its historical name because the Nova2 route reuses the
validated `ManusGlove` message contract and Revo2 retargeting code.

In this slim workspace it is used for:

```text
/manus_glove_0 or /manus_glove_1
  -> retarget algorithm
  -> /revo2_<side>/revo2_pid_controller/target_joint_states
```

The MANUS and Hex glove drivers are not included. `launch_manus_publisher` is
kept only as a compatibility launch argument and is ignored here.

Run the complete Nova2 pipeline from the repository root:

```bash
ros2 launch nova2_glove_driver nova2_glove_pipeline.launch.py \
  hand_mode:=right \
  enable_left:=false \
  enable_right:=true \
  if_sim:=true \
  udp_port:=15020 \
  launch_plot:=false
```
