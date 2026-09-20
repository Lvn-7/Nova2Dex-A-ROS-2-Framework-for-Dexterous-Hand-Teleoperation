import math
import unittest

from nova2_inspire_retarget.mapping import JOINT_LIMITS_RAD, direct_map


CONFIG = {
    "finger_zero_deg": 0.0,
    "finger_range_deg": 100.0,
    "thumb_pitch_zero_deg": 0.0,
    "thumb_pitch_range_deg": 100.0,
    "thumb_yaw_zero_deg": 0.0,
    "thumb_yaw_range_deg": 17.1887,
    "thumb_yaw_sign": 1.0,
}


def ergonomics(value):
    result = {"ThumbMCPSpread": value}
    for finger in ("Thumb", "Index", "Middle", "Ring", "Pinky"):
        for joint in ("MCP", "PIP", "DIP"):
            result[f"{finger}{joint}Stretch"] = value
    return result


class DirectMappingTest(unittest.TestCase):
    def test_normalized_flexion_open_closed_and_clamping(self):
        self.assertEqual(direct_map(ergonomics(0.0), CONFIG), (0.0,) * 6)
        self.assertEqual(direct_map(ergonomics(1000.0), CONFIG), JOINT_LIMITS_RAD)
        halfway_values = ergonomics(50.0)
        halfway_values["ThumbMCPSpread"] = 8.59435
        halfway = direct_map(halfway_values, CONFIG)
        for actual in halfway[:4]:
            self.assertAlmostEqual(actual, 0.735)
        self.assertTrue(math.isclose(halfway[5], 1.308 * 50.0 / 100.0))

    def test_missing_field_rejects_frame(self):
        values = ergonomics(0.0)
        del values["IndexMCPStretch"]
        with self.assertRaisesRegex(ValueError, "IndexMCPStretch"):
            direct_map(values, CONFIG)


if __name__ == "__main__":
    unittest.main()
