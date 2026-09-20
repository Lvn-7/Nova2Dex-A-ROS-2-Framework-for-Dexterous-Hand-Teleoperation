"""Pure direct-mapping logic shared by the ROS node and tests."""

from math import isfinite


# This order matches AnyDexRetarget's Inspire command adapter and the G1 service.
JOINT_NAMES = (
    "pinky_proximal_joint",
    "ring_proximal_joint",
    "middle_proximal_joint",
    "index_proximal_joint",
    "thumb_proximal_pitch_joint",
    "thumb_proximal_yaw_joint",
)
JOINT_LIMITS_RAD = (1.47, 1.47, 1.47, 1.47, 0.6, 1.308)
def _unit(value: float, zero: float, span: float, sign: float = 1.0) -> float:
    if not all(isfinite(number) for number in (value, zero, span, sign)):
        raise ValueError("mapping inputs must be finite")
    if abs(span) < 1e-6:
        raise ValueError("mapping range must not be zero")
    return min(1.0, max(0.0, sign * (value - zero) / abs(span)))


def direct_map(ergonomics: dict[str, float], config: dict[str, float]) -> tuple[float, ...]:
    """Map normalized-flexion-backed ergonomics values to Inspire joint radians.

    The UDP bridge encodes normalized flexion as 0..100 in all three stretch
    fields.  Reading MCP only keeps this equivalent to arm_hand_teleop while
    preserving the existing ManusGlove topic and Inspire JointState output.
    """
    fingers = []
    for finger in ("Pinky", "Ring", "Middle", "Index"):
        key = f"{finger}MCPStretch"
        if key not in ergonomics:
            raise ValueError(f"missing ergonomics field: {key}")
        angle = float(ergonomics[key])
        if not isfinite(angle):
            raise ValueError(f"{key} must be finite")
        normalized = _unit(
            angle,
            config["finger_zero_deg"],
            config["finger_range_deg"],
        )
        fingers.append(normalized * 1.47)

    thumb_key = "ThumbMCPStretch"
    if thumb_key not in ergonomics:
        raise ValueError(f"missing ergonomics field: {thumb_key}")
    thumb_angle = float(ergonomics[thumb_key])
    if not isfinite(thumb_angle):
        raise ValueError(f"{thumb_key} must be finite")
    thumb_pitch = _unit(
        thumb_angle,
        config["thumb_pitch_zero_deg"],
        config["thumb_pitch_range_deg"],
    ) * 0.6

    if "ThumbMCPSpread" not in ergonomics:
        raise ValueError("missing ergonomics field: ThumbMCPSpread")
    thumb_yaw = _unit(
        float(ergonomics["ThumbMCPSpread"]),
        config["thumb_yaw_zero_deg"],
        config["thumb_yaw_range_deg"],
        config["thumb_yaw_sign"],
    ) * 1.308
    return tuple(fingers) + (thumb_pitch, thumb_yaw)
