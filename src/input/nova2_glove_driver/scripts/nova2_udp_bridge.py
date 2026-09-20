#!/usr/bin/env python3
"""Receive Nova2 UDP JSON frames and publish ManusGlove topics."""

from __future__ import annotations

import json
import math
import socket
import time
from typing import Any

from geometry_msgs.msg import Pose
from manus_ros2_msgs.msg import ManusErgonomics, ManusGlove, ManusRawNode
import rclpy
from rclpy.node import Node


FINGERS = (
    ("Thumb", "thumb", 1),
    ("Index", "index", 6),
    ("Middle", "middle", 11),
    ("Ring", "ring", 16),
    ("Pinky", "pinky", 21),
)
JOINT_TYPES = ("mcp", "pip", "dip", "tip")
ERGONOMICS_FIELDS = (
    "ThumbMCPStretch", "ThumbMCPSpread", "ThumbPIPStretch", "ThumbDIPStretch",
    "IndexMCPStretch", "IndexSpread", "IndexPIPStretch", "IndexDIPStretch",
    "MiddleMCPStretch", "MiddleSpread", "MiddlePIPStretch", "MiddleDIPStretch",
    "RingMCPStretch", "RingSpread", "RingPIPStretch", "RingDIPStretch",
    "PinkyMCPStretch", "PinkySpread", "PinkyPIPStretch", "PinkyDIPStretch",
)
RAD_TO_DEG = 180.0 / math.pi
THUMB_FLEXION_MAX = 0.733333
FOUR_FINGER_FLEXION_MAX = 0.928571


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def number_at(values: Any, *indices: int, default: float = 0.0) -> float:
    node = values
    try:
        for index in indices:
            node = node[index]
        return float(node)
    except (TypeError, ValueError, IndexError):
        return default


class Nova2UdpBridge(Node):
    def __init__(self) -> None:
        super().__init__("nova2_glove_driver")

        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("udp_port", 15020)
        self.declare_parameter("poll_rate_hz", 500.0)
        self.declare_parameter("enable_left", True)
        self.declare_parameter("enable_right", True)
        self.declare_parameter("left_topic", "/manus_glove_0")
        self.declare_parameter("right_topic", "/manus_glove_1")
        self.declare_parameter("position_scale", 0.001)
        self.declare_parameter("left_flex_sign", 1.0)
        self.declare_parameter("right_flex_sign", 1.0)
        self.declare_parameter("left_thumb_spread_sign", 1.0)
        self.declare_parameter("right_thumb_spread_sign", 1.0)
        self.declare_parameter("timeout_ms", 200)
        self.declare_parameter("publish_raw_nodes", True)
        self.declare_parameter("publish_ergonomics", True)
        self.declare_parameter("use_normalized_flexion", True)
        self.declare_parameter("normalized_flexion_max_degrees", 100.0)
        self.declare_parameter("normalized_thumb_flexion_max", THUMB_FLEXION_MAX)
        self.declare_parameter("normalized_four_finger_flexion_max", FOUR_FINGER_FLEXION_MAX)

        for field in ERGONOMICS_FIELDS:
            self.declare_parameter(f"mapping.{field}.offset", 0.0)
            self.declare_parameter(f"mapping.{field}.scale", 1.0)
            self.declare_parameter(f"mapping.{field}.sign", 1.0)
            self.declare_parameter(f"mapping.{field}.min", -360.0)
            self.declare_parameter(f"mapping.{field}.max", 360.0)

        self.listen_host = self.get_parameter("listen_host").value
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.position_scale = float(self.get_parameter("position_scale").value)
        self.timeout_sec = max(0.01, float(self.get_parameter("timeout_ms").value) / 1000.0)
        self.publish_raw_nodes = as_bool(self.get_parameter("publish_raw_nodes").value)
        self.publish_ergonomics = as_bool(self.get_parameter("publish_ergonomics").value)
        self.use_normalized_flexion = as_bool(self.get_parameter("use_normalized_flexion").value)
        self.normalized_flexion_max_degrees = float(
            self.get_parameter("normalized_flexion_max_degrees").value
        )
        self.normalized_flexion_max = {
            "thumb": float(self.get_parameter("normalized_thumb_flexion_max").value),
            "finger": float(self.get_parameter("normalized_four_finger_flexion_max").value),
        }
        self.enabled = {
            "left": as_bool(self.get_parameter("enable_left").value),
            "right": as_bool(self.get_parameter("enable_right").value),
        }
        self.flex_sign = {
            "left": float(self.get_parameter("left_flex_sign").value),
            "right": float(self.get_parameter("right_flex_sign").value),
        }
        self.thumb_spread_sign = {
            "left": float(self.get_parameter("left_thumb_spread_sign").value),
            "right": float(self.get_parameter("right_thumb_spread_sign").value),
        }
        self.mapping = self._load_mapping()

        self.glove_publishers = {
            "left": self.create_publisher(ManusGlove, str(self.get_parameter("left_topic").value), 10),
            "right": self.create_publisher(ManusGlove, str(self.get_parameter("right_topic").value), 10),
        }
        self.last_packet_time = None
        self.last_warn_time = 0.0
        self.packet_count = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((str(self.listen_host), self.udp_port))
        self.sock.setblocking(False)

        poll_rate_hz = max(1.0, float(self.get_parameter("poll_rate_hz").value))
        self.timer = self.create_timer(1.0 / poll_rate_hz, self._poll)
        self.get_logger().info(
            f"Nova2 UDP bridge listening on {self.listen_host}:{self.udp_port}; "
            f"left={self.enabled['left']} right={self.enabled['right']}"
        )

    def _load_mapping(self) -> dict[str, dict[str, float]]:
        mapping: dict[str, dict[str, float]] = {}
        for field in ERGONOMICS_FIELDS:
            mapping[field] = {
                "offset": float(self.get_parameter(f"mapping.{field}.offset").value),
                "scale": float(self.get_parameter(f"mapping.{field}.scale").value),
                "sign": float(self.get_parameter(f"mapping.{field}.sign").value),
                "min": float(self.get_parameter(f"mapping.{field}.min").value),
                "max": float(self.get_parameter(f"mapping.{field}.max").value),
            }
        return mapping

    def _poll(self) -> None:
        received = False
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(262144)
            except BlockingIOError:
                break
            received = True
            self._handle_packet(data, addr)

        now = time.monotonic()
        if not received and self.last_packet_time is not None:
            if now - self.last_packet_time > self.timeout_sec and now - self.last_warn_time > 1.0:
                self.last_warn_time = now
                self.get_logger().warning("Nova2 UDP timeout; no new ManusGlove messages published.")

    def _handle_packet(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            packet = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            now = time.monotonic()
            if now - self.last_warn_time > 1.0:
                self.last_warn_time = now
                self.get_logger().warning(f"Invalid Nova2 UDP JSON from {addr}: {exc}")
            return

        hands = packet.get("hands")
        if not isinstance(hands, list):
            return

        self.last_packet_time = time.monotonic()
        self.packet_count += 1
        if self.packet_count == 1:
            self.get_logger().info(f"Received first Nova2 UDP packet from {addr[0]}:{addr[1]}")

        for hand in hands:
            if not isinstance(hand, dict):
                continue
            side = str(hand.get("side", "")).strip().lower()
            if side not in ("left", "right") or not self.enabled[side]:
                continue
            if not as_bool(hand.get("connected", True)):
                continue
            msg = self._hand_to_msg(hand, side)
            self.glove_publishers[side].publish(msg)

    def _hand_to_msg(self, hand: dict[str, Any], side: str) -> ManusGlove:
        msg = ManusGlove()
        msg.glove_id = int(hand.get("glove_id", 1 if side == "right" else 0))
        msg.side = side
        msg.raw_sensor_orientation.w = 1.0

        positions = hand.get("joint_positions_mm", [])
        rotations = hand.get("joint_rotations_xyzw", [])
        angles = hand.get("hand_angles_rad", [])
        normalized_flexion = hand.get("normalized_flexion", [])

        if self.publish_raw_nodes:
            palm = ManusRawNode()
            palm.node_id = 0
            palm.parent_node_id = -1
            palm.joint_type = "palm"
            palm.chain_type = "palm"
            palm.pose.orientation.w = 1.0
            msg.raw_nodes.append(palm)

            for finger_index, (_name, chain, node_base) in enumerate(FINGERS):
                finger_positions = positions[finger_index] if finger_index < len(positions) else []
                for joint_index, joint_type in enumerate(JOINT_TYPES):
                    if joint_index >= len(finger_positions):
                        continue
                    node = ManusRawNode()
                    node.node_id = node_base + joint_index
                    node.parent_node_id = 0 if joint_index == 0 else node.node_id - 1
                    node.joint_type = joint_type
                    node.chain_type = chain
                    node.pose = self._pose_from(positions, rotations, finger_index, joint_index)
                    msg.raw_nodes.append(node)
        msg.raw_node_count = len(msg.raw_nodes)

        if self.publish_ergonomics:
            for finger_index, (name, _chain, _node_base) in enumerate(FINGERS):
                spread_name = "ThumbMCPSpread" if finger_index == 0 else f"{name}Spread"
                if self.use_normalized_flexion:
                    flex_degrees = self._normalized_flexion_deg(normalized_flexion, finger_index)
                    self._append_ergo(msg, side, f"{name}MCPStretch", flex_degrees)
                    spread_degrees = (
                        self._thumb_yaw_deg(angles)
                        if finger_index == 0
                        else self._spread_deg(angles, finger_index)
                    )
                    self._append_ergo(msg, side, spread_name, spread_degrees)
                    self._append_ergo(msg, side, f"{name}PIPStretch", flex_degrees)
                    self._append_ergo(msg, side, f"{name}DIPStretch", flex_degrees)
                    continue
                self._append_ergo(msg, side, f"{name}MCPStretch", self._angle_y_deg(angles, finger_index, 0))
                self._append_ergo(msg, side, spread_name, self._spread_deg(angles, finger_index))
                self._append_ergo(msg, side, f"{name}PIPStretch", self._angle_y_deg(angles, finger_index, 1))
                self._append_ergo(msg, side, f"{name}DIPStretch", self._angle_y_deg(angles, finger_index, 2))
        msg.ergonomics_count = len(msg.ergonomics)
        msg.raw_sensor_count = 0
        return msg

    def _pose_from(self, positions: Any, rotations: Any, finger: int, joint: int) -> Pose:
        pose = Pose()
        pose.position.x = number_at(positions, finger, joint, 0) * self.position_scale
        pose.position.y = number_at(positions, finger, joint, 1) * self.position_scale
        pose.position.z = number_at(positions, finger, joint, 2) * self.position_scale
        pose.orientation.x = number_at(rotations, finger, joint, 0)
        pose.orientation.y = number_at(rotations, finger, joint, 1)
        pose.orientation.z = number_at(rotations, finger, joint, 2)
        pose.orientation.w = number_at(rotations, finger, joint, 3, default=1.0)
        return pose

    @staticmethod
    def _angle_y_deg(angles: Any, finger: int, joint: int) -> float:
        return number_at(angles, finger, joint, 1) * RAD_TO_DEG

    @staticmethod
    def _spread_deg(angles: Any, finger: int) -> float:
        return -number_at(angles, finger, 0, 2) * RAD_TO_DEG

    @staticmethod
    def _thumb_yaw_deg(angles: Any) -> float:
        # arm_hand_teleop maps hand_angles_rad[0] (thumb joint-0 X axis)
        # from 0..-0.3 rad to open..closed. Keep that convention here.
        return -number_at(angles, 0, 0, 0) * RAD_TO_DEG

    def _normalized_flexion_deg(self, values: Any, finger: int) -> float:
        flexion = min(max(number_at(values, finger, default=0.0), 0.0), 1.0)
        max_flexion = self.normalized_flexion_max["thumb" if finger == 0 else "finger"]
        if max_flexion <= 0.0:
            return 0.0
        # Transport the normalized value through the existing ergonomics message.
        return min(1.0, flexion / max_flexion) * self.normalized_flexion_max_degrees

    def _append_ergo(self, msg: ManusGlove, side: str, field: str, raw_degrees: float) -> None:
        value = self._mapped_value(side, field, raw_degrees)
        ergo = ManusErgonomics()
        ergo.type = field
        ergo.value = float(value)
        msg.ergonomics.append(ergo)

    def _mapped_value(self, side: str, field: str, raw_degrees: float) -> float:
        mapping = self.mapping[field]
        sign = mapping["sign"]
        if "Stretch" in field:
            sign *= self.flex_sign[side]
        if field == "ThumbMCPSpread":
            sign *= self.thumb_spread_sign[side]
        value = sign * (raw_degrees + mapping["offset"]) * mapping["scale"]
        return min(max(value, mapping["min"]), mapping["max"])


def main() -> None:
    rclpy.init()
    node = Nova2UdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
