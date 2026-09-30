import unittest
from pathlib import Path

from tools.generate_calibration import CHECKS, HEADER, empty_profile, render, validate


def measured_profile():
    profile = empty_profile()
    axes = (("FL", "proximal"), ("FL", "distal"),
            ("RL", "proximal"), ("RL", "distal"),
            ("FR", "proximal"), ("FR", "distal"),
            ("RR", "proximal"), ("RR", "distal"))
    for index, (leg, joint) in enumerate(axes):
        profile["axes"][index].update({
            "leg": leg, "joint": joint, "angle_sign": 1, "home_dir": -1,
            "limit_pin": 42 + index, "limit_pressed_level": "LOW",
            "home_angle_deg": -10, "min_angle_deg": -9, "max_angle_deg": 10,
            "stand_angle_deg": 0, "sit_angle_deg": 5,
            "trot_amplitude_deg": 3, "max_home_steps": 3000,
        })
    return profile


class GeneratorTests(unittest.TestCase):
    def test_requires_unique_limit_inputs_and_axes(self):
        profile = measured_profile()
        self.assertEqual(validate(profile)[1], list(range(8)))
        profile["axes"][1]["limit_pin"] = 42
        with self.assertRaisesRegex(ValueError, "every.*input"):
            validate(profile)
        profile["axes"][1]["limit_pin"] = 43
        profile["axes"][1]["joint"] = "proximal"
        with self.assertRaisesRegex(ValueError, "duplicate anatomical"):
            validate(profile)

    def test_crossed_inputs_and_normally_closed_switch(self):
        profile = measured_profile()
        profile["axes"][0]["limit_pin"] = 43
        profile["axes"][1]["limit_pin"] = 42
        profile["axes"][0]["limit_pressed_level"] = "HIGH"
        source = HEADER.read_text(encoding="utf-8")
        locked = render(profile, source)
        self.assertIn("LIMIT_INPUT_FOR_J[8] = {1,0,2,3,4,5,6,7}", locked)
        self.assertIn("LIMIT_ACTIVE_LOW_FOR_J[8] = {false,true", locked)
        self.assertIn("ROBOT_CALIBRATED = false", locked)
        with self.assertRaisesRegex(ValueError, "not confirmed"):
            render(profile, source, unlock=True)
        for key in CHECKS:
            profile["verified"][key] = True
        unlocked = render(profile, source, unlock=True)
        self.assertIn("ROBOT_CALIBRATED = true", unlocked)
        self.assertIn("ENABLE_EXPERIMENTAL_TROT = false", unlocked)
        self.assertIn("// J1 D43", unlocked)

    def test_rejects_unsafe_retract(self):
        profile = measured_profile()
        profile["axes"][0]["home_dir"] = 1
        with self.assertRaisesRegex(ValueError, "away from switch"):
            validate(profile)


if __name__ == "__main__":
    unittest.main()
