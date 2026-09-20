# Nova2 → Inspire direct retargeting

This ROS 2 package is intentionally independent from `manus_revo2_retarget`.
It reuses `/manus_glove_0` and `/manus_glove_1`, then publishes:

- `/inspire_left/retarget/joint_states`
- `/inspire_right/retarget/joint_states`

The six-position order follows AnyDexRetarget and the G1 Inspire service:
`pinky, ring, middle, index, thumb pitch, thumb yaw`. `JointState.name` always
contains the complete URDF joint names, so consumers should match by name.

The retargeter uses the Nova 2 `normalized_flexion` values for finger
closure. The UDP bridge transports those values as `0..100` in the existing
`ManusGlove` ergonomics fields, and this package converts them directly to the
Inspire hand's six joint positions in radians. Thumb yaw still comes from the
Nova 2 hand angle. This keeps the `JointState` interface unchanged for both
MuJoCo and the G1 DDS bridge.

- Four fingers and thumb pitch: one normalized flexion value per finger.
- Thumb yaw: `ThumbMCPSpread`, derived from the Nova 2 thumb joint-0 X angle,
  matching `arm_hand_teleop`.

Ranges and zero points are isolated in `config/direct_mapping.yaml`.

## Model provenance

The files under `models/inspire_hand` are the minimal left URDF and right
MuJoCo MJCF/STL dependencies copied from
[qqsq12321/AnyDexRetarget](https://github.com/qqsq12321/AnyDexRetarget),
commit `745a8358f0ff86e90991fc5eb7e85073e56a8818`. The upstream MIT license and
model attribution are retained next to the model.

The right viewer intentionally uses `inspire_hand_right_mujoco.xml`, not the
generic URDF: MuJoCo discards URDF visual-only GLB meshes by default and would
otherwise display the simplified collision geometry.
