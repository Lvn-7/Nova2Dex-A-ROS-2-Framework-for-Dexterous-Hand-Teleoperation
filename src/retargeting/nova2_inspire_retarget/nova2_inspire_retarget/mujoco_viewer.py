"""MuJoCo viewer for one Inspire target JointState topic."""

import threading
import time

import mujoco
import mujoco.viewer
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState


MIMIC = {
    "thumb_intermediate_joint": ("thumb_proximal_pitch_joint", 1.334, 0.0),
    "thumb_distal_joint": ("thumb_proximal_pitch_joint", 0.667, 0.0),
    "index_intermediate_joint": ("index_proximal_joint", 1.06399, -0.04545),
    "middle_intermediate_joint": ("middle_proximal_joint", 1.06399, -0.04545),
    "ring_intermediate_joint": ("ring_proximal_joint", 1.06399, -0.04545),
    "pinky_intermediate_joint": ("pinky_proximal_joint", 1.06399, -0.04545),
}


class InspireViewerNode(Node):
    def __init__(self):
        super().__init__("inspire_mujoco_viewer")
        self.declare_parameter("side", "right")
        side = str(self.get_parameter("side").value).lower()
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.declare_parameter("joint_state_topic", f"/inspire_{side}/retarget/joint_states")
        model_file = (
            "inspire_hand_right_mujoco.xml"
            if side == "right"
            else "inspire_hand_left.urdf"
        )
        model_path = (
            get_package_share_directory("nova2_inspire_retarget")
            + f"/models/inspire_hand/{model_file}"
        )
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.targets: dict[str, float] = {}
        self.lock = threading.Lock()
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_target,
            10,
        )
        self.get_logger().info(
            f"Inspire {side} MuJoCo model listening on "
            f"{self.get_parameter('joint_state_topic').value}"
        )

    def _on_target(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        with self.lock:
            self.targets = dict(zip(message.name, message.position))

    def apply_targets(self) -> None:
        with self.lock:
            targets = dict(self.targets)
        for mimic_name, (source, multiplier, offset) in MIMIC.items():
            if source in targets:
                targets[mimic_name] = targets[source] * multiplier + offset
        for name, value in targets.items():
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                continue
            qpos_address = self.model.jnt_qposadr[joint_id]
            if self.model.jnt_limited[joint_id]:
                low, high = self.model.jnt_range[joint_id]
                value = min(high, max(low, value))
            self.data.qpos[qpos_address] = value
        mujoco.mj_forward(self.model, self.data)


def main(args=None):
    rclpy.init(args=args)
    node = InspireViewerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            viewer.cam.lookat[:] = node.model.stat.center
            viewer.cam.distance = max(0.35, 2.5 * node.model.stat.extent)
            while viewer.is_running() and rclpy.ok():
                node.apply_targets()
                viewer.sync()
                time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
