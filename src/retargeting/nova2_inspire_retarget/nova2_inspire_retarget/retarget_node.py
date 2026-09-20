"""ROS 2 node publishing direct Nova2-to-Inspire JointState targets."""

import time

import rclpy
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node
from sensor_msgs.msg import JointState

from nova2_inspire_retarget.mapping import JOINT_NAMES, direct_map


class DirectRetargetNode(Node):
    def __init__(self):
        super().__init__("nova2_inspire_retarget")
        self.declare_parameter("hand_mode", "right")
        self.declare_parameter("left_input_topic", "/manus_glove_0")
        self.declare_parameter("right_input_topic", "/manus_glove_1")
        self.declare_parameter("left_output_topic", "/inspire_left/retarget/joint_states")
        self.declare_parameter("right_output_topic", "/inspire_right/retarget/joint_states")
        self.declare_parameter("finger_zero_deg", 0.0)
        self.declare_parameter("finger_range_deg", 84.0)
        self.declare_parameter("thumb_pitch_zero_deg", 0.0)
        self.declare_parameter("thumb_pitch_range_deg", 80.0)
        self.declare_parameter("thumb_yaw_zero_deg", 0.0)
        self.declare_parameter("thumb_yaw_range_deg", 45.0)
        self.declare_parameter("thumb_yaw_sign", 1.0)
        self.declare_parameter("warning_period_sec", 2.0)

        mode = str(self.get_parameter("hand_mode").value).lower()
        if mode not in ("left", "right", "both"):
            raise ValueError("hand_mode must be left, right, or both")
        self.enabled_sides = {"left", "right"} if mode == "both" else {mode}
        self.mapping_config = {
            name: float(self.get_parameter(name).value)
            for name in (
                "finger_zero_deg",
                "finger_range_deg",
                "thumb_pitch_zero_deg",
                "thumb_pitch_range_deg",
                "thumb_yaw_zero_deg",
                "thumb_yaw_range_deg",
                "thumb_yaw_sign",
            )
        }
        self.warning_period = max(0.1, float(self.get_parameter("warning_period_sec").value))
        self.last_warning: dict[str, float] = {}

        self._publishers_by_side = {
            side: self.create_publisher(
                JointState,
                str(self.get_parameter(f"{side}_output_topic").value),
                10,
            )
            for side in self.enabled_sides
        }
        # Subscribe to both stable Nova2 topics and trust msg.side for routing.
        self._input_subscriptions = [
            self.create_subscription(
                ManusGlove,
                str(self.get_parameter(f"{topic_side}_input_topic").value),
                self._on_glove,
                10,
            )
            for topic_side in ("left", "right")
        ]
        self.get_logger().info(
            f"Nova2 -> Inspire direct mapping enabled for {sorted(self.enabled_sides)}; "
            f"joint order={list(JOINT_NAMES)}"
        )

    def _warn_throttled(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning.get(key, 0.0) >= self.warning_period:
            self.last_warning[key] = now
            self.get_logger().warning(message)

    def _on_glove(self, glove: ManusGlove) -> None:
        side = glove.side.strip().lower()
        if side not in self.enabled_sides:
            return
        ergonomics = {item.type: float(item.value) for item in glove.ergonomics}
        try:
            targets = direct_map(ergonomics, self.mapping_config)
        except (KeyError, TypeError, ValueError) as exc:
            self._warn_throttled(side, f"Skipping invalid {side} glove frame: {exc}")
            return

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(targets)
        self._publishers_by_side[side].publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DirectRetargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
