"""Create and validate a Mega calibration from measurements of one robot.

The shipped firmware remains locked. --unlock requires explicit confirmation of
all physical checks in the local profile; --enable-trot is a separate step.
"""
import argparse
import json
import re
from decimal import Decimal, InvalidOperation
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "firmware" / "RoboDogMegaFinal" / "calibration.h"
LEGS = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}
JOINTS = {"proximal": 0, "distal": 1}
CHECKS = (
    "hardware_motor_cut", "estop_signal", "rail_sensing", "driver_rsense",
    "limits_press_release", "motor_direction", "mechanical_bounds", "manual_home_path",
)
ANGLES = ("home_angle_deg", "min_angle_deg", "max_angle_deg",
          "stand_angle_deg", "sit_angle_deg", "trot_amplitude_deg")


def empty_profile():
    return {
        "gear_ratio": 12, "microsteps": 16,
        "verified": {key: False for key in CHECKS},
        "axes": [{"channel": f"J{i}", "leg": None, "joint": None,
                  "angle_sign": None, "home_dir": None, "limit_pin": None,
                  "limit_pressed_level": None,
                  **{key: None for key in ANGLES}, "max_home_steps": None}
                 for i in range(1, 9)],
    }


def _integer(value, label, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{label}: expected integer {low}..{high}")
    return value


def _centidegrees(value, label):
    if value is None or isinstance(value, bool):
        raise ValueError(f"{label}: measured angle is required")
    try:
        amount = Decimal(str(value)) * 100
    except (InvalidOperation, ValueError):
        raise ValueError(f"{label}: invalid angle") from None
    if not amount.is_finite() or amount != amount.to_integral_value():
        raise ValueError(f"{label}: use at most two decimal places")
    return int(amount)


def _steps(c, angle):
    numerator = (angle - c[4]) * 38400 * c[2]
    return (1 if numerator >= 0 else -1) * (abs(numerator) // 36000)


def validate(profile):
    if profile.get("gear_ratio") != 12 or profile.get("microsteps") != 16:
        raise ValueError("profile must match compiled 12:1 gearing and 1/16 microsteps")
    axes = profile.get("axes")
    if not isinstance(axes, list) or len(axes) != 8:
        raise ValueError("exactly eight axes J1..J8 are required")
    rows, inputs, active_low, seen_axes = [], [], [], set()
    for index, axis in enumerate(axes, 1):
        label = f"J{index}"
        if not isinstance(axis, dict) or axis.get("channel") != label:
            raise ValueError(f"axis row {index} must be {label}")
        leg, joint = axis.get("leg"), axis.get("joint")
        if leg not in LEGS or joint not in JOINTS:
            raise ValueError(f"{label}: choose FL/FR/RL/RR and proximal/distal")
        if (leg, joint) in seen_axes:
            raise ValueError(f"{label}: duplicate anatomical axis {leg}/{joint}")
        seen_axes.add((leg, joint))
        sign = _integer(axis.get("angle_sign"), f"{label}.angle_sign", -1, 1)
        home_dir = _integer(axis.get("home_dir"), f"{label}.home_dir", -1, 1)
        if sign == 0 or home_dir == 0:
            raise ValueError(f"{label}: signs must be +1 or -1")
        pin = _integer(axis.get("limit_pin"), f"{label}.limit_pin", 42, 49)
        level = axis.get("limit_pressed_level")
        if level not in ("LOW", "HIGH"):
            raise ValueError(f"{label}: limit_pressed_level must be LOW or HIGH")
        angles = [_centidegrees(axis.get(key), f"{label}.{key}") for key in ANGLES]
        home, low, high, stand, sit, amplitude = angles
        max_steps = _integer(axis.get("max_home_steps"), f"{label}.max_home_steps", 200, 12800)
        if not (-18000 <= home <= 18000 and -18000 <= low < high <= 18000):
            raise ValueError(f"{label}: invalid mechanical angles")
        if not (low <= stand <= high and low <= sit <= high and
                0 <= amplitude <= 500 and low <= stand-amplitude and
                stand+amplitude <= high):
            raise ValueError(f"{label}: pose/amplitude lies outside soft bounds")
        row = (LEGS[leg], JOINTS[joint], sign, home_dir, home, low, high,
               stand, sit, amplitude, max_steps)
        retract = -home_dir * 200
        bound_a, bound_b = sorted((_steps(row, low), _steps(row, high)))
        if not (bound_a <= retract <= bound_b and
                _steps(row, low)*home_dir < 0 and _steps(row, high)*home_dir < 0):
            raise ValueError(f"{label}: both soft bounds and 200-step retract must be away from switch")
        rows.append(row)
        inputs.append(pin-42)
        active_low.append(level == "LOW")
    if len(seen_axes) != 8 or len(set(inputs)) != 8:
        raise ValueError("every anatomical axis and every D42..D49 input must appear once")
    sides = [row[0] & 1 for row in rows]
    if len(set(sides[:4])) != 1 or len(set(sides[4:])) != 1 or sides[0] == sides[4]:
        raise ValueError("J1..J4 and J5..J8 must each cover one opposite side")
    checks = profile.get("verified")
    if not isinstance(checks, dict) or any(type(checks.get(key)) is not bool for key in CHECKS):
        raise ValueError("verified must contain true/false for every physical check")
    return rows, inputs, active_low


def _replace_once(source, pattern, replacement):
    result, count = re.subn(pattern, lambda _: replacement, source, count=1, flags=re.S)
    if count != 1:
        raise ValueError(f"cannot locate firmware field: {pattern}")
    return result


def render(profile, source, unlock=False, enable_trot=False):
    rows, inputs, active_low = validate(profile)
    if enable_trot and not unlock:
        raise ValueError("--enable-trot requires --unlock")
    if unlock and not all(profile["verified"][key] for key in CHECKS):
        missing = ", ".join(key for key in CHECKS if not profile["verified"][key])
        raise ValueError("physical checks are not confirmed: " + missing)
    gates = {
        "ROBOT_CALIBRATED": unlock,
        "ESTOP_SIGNAL_VERIFIED": unlock,
        "RAIL_SENSING_VERIFIED": unlock,
        "DRIVER_RSENSE_VERIFIED": unlock,
        "HARDWARE_MOTOR_CUT_VERIFIED": unlock,
        "ENABLE_EXPERIMENTAL_TROT": enable_trot,
    }
    for key, value in gates.items():
        source = _replace_once(source,
            rf"constexpr bool {key} = (?:true|false);",
            f"constexpr bool {key} = {'true' if value else 'false'};")
    source = _replace_once(source, r"const uint8_t LIMIT_INPUT_FOR_J\[8\] = \{[^}]*\};",
        "const uint8_t LIMIT_INPUT_FOR_J[8] = {" + ",".join(map(str, inputs)) + "};")
    source = _replace_once(source, r"const bool LIMIT_ACTIVE_LOW_FOR_J\[8\] = \{[^}]*\};",
        "const bool LIMIT_ACTIVE_LOW_FOR_J[8] = {" +
        ",".join("true" if item else "false" for item in active_low) + "};")
    entries = ["  {" + ",".join(map(str, row)) + f"}}, // J{index} D{inputs[index-1]+42}"
               for index, row in enumerate(rows, 1)]
    source = _replace_once(source, r"const JointCalibration JOINTS\[8\] = \{.*?\n\};",
        "const JointCalibration JOINTS[8] = {\n" + ",\n".join(entries) + "\n};")
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--init", metavar="JSON", type=Path, help="create a local measurement form")
    parser.add_argument("--input", type=Path, help="completed local measurement form")
    parser.add_argument("--output", type=Path, default=HEADER)
    parser.add_argument("--unlock", action="store_true", help="set firmware gates after physical verification")
    parser.add_argument("--enable-trot", action="store_true", help="enable experimental gait after supported testing")
    args = parser.parse_args()
    if args.init:
        if args.input or args.unlock or args.enable_trot:
            parser.error("--init cannot be combined with generation options")
        with args.init.open("x", encoding="utf-8") as file:
            json.dump(empty_profile(), file, ensure_ascii=False, indent=2)
            file.write("\n")
        print(f"Created {args.init}; record physical measurements before generating firmware")
        return
    if not args.input:
        parser.error("--input is required unless --init is used")
    profile = json.loads(args.input.read_text(encoding="utf-8"))
    source = render(profile, HEADER.read_text(encoding="utf-8"), args.unlock, args.enable_trot)
    args.output.write_text(source, encoding="utf-8")
    print(f"Wrote {args.output}; motor ARM {'unlocked' if args.unlock else 'locked'}; "
          f"trot {'enabled' if args.enable_trot else 'disabled'}")


if __name__ == "__main__":
    main()
