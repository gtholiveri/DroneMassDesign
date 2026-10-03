"""Fit the rule for a typical motor from the scraped Tyto tests (run tools/scrape_tyto.py first).

Each test logs electrical power P_e into the ESC, and shaft torque Q and speed omega out of
the motor. Everything lost in between is
    loss = eta_esc * P_e - Q * omega = Q_0(omega) * omega + ((Q + Q_0(omega)) / K_m)^2
         (drag: friction and iron)   (copper)
with Q_0(omega) = Q_0,ref * (omega / omega_ref)^k, the same model as drone_sizing.motor. Fitting
K_m and Q_0,ref to each motor's tests needs no Kv, R_m or I_0, so motors of any winding are
comparable. Regressing them against motor mass across many motors gives a rule for any motor whose
listing gives only its mass.

The prop rule comes from tools/fit_props.py: Tyto has almost no props as small as ours.

Updates the motor part of catalog/scaling.json and prints how well the rule fits.
Run from the project folder:  python tools/fit_tyto.py
"""

import json
import math
import statistics
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from drone_sizing.catalog import MOTOR_DATASHEET_COLUMNS, fully_listed_motor, read_rows  # noqa: E402
from drone_sizing.constants import GRAVITY_M_PER_S2  # noqa: E402
from drone_sizing.motor import REFERENCE_SPEED_RAD_S  # noqa: E402
from drone_sizing.numerics import PowerLaw, minimize  # noqa: E402

DATA = PROJECT / "data" / "tyto"
OUTPUT = PROJECT / "catalog" / "scaling.json"

# The stands measure power going into the ESC. Assume the ESC passes on this fraction of it.
ESC_EFFICIENCY = 0.95
SPEED_EXPONENTS = (0.0, 0.25, 0.5, 0.75, 1.0)
CHOSEN_SPEED_EXPONENT = 0.5  # what drone_sizing assumes; the fit below reports how the data compares

SMALL_MOTOR_MAX_KG = 0.060  # the motor rule is fitted to motors this size or smaller

MIN_ROWS = 6
MIN_SPEED_FRACTION = 0.3  # of each test's top speed, for the motor fit
COPPER_FLOOR = 1e-12  # the fit's lower bound on 1 / K_m^2
MIN_LOSS_FIT_R2 = 0.8  # a motor whose losses the model can't follow is noise or a bad log


def rows(dataset: dict) -> list[dict]:
    """The usable rows of a test, in SI: speed, torque, thrust, electrical and shaft power."""
    def column(name: str) -> list:
        return (dataset.get(name) or {}).get("values") or []

    rpm, torque, thrust, epower = column("rotation_speed"), column("torque"), column("thrust"), column("epower")
    usable = []
    for i in range(min(len(rpm), len(torque), len(thrust), len(epower))):
        if None in (rpm[i], torque[i], thrust[i], epower[i]):
            continue
        speed_rad_s = abs(rpm[i]) * 2 * math.pi / 60
        torque_nm = abs(torque[i])  # sign depends on spin direction
        shaft_power_w = torque_nm * speed_rad_s
        if speed_rad_s < 50 or torque_nm <= 0 or epower[i] <= 0 or shaft_power_w >= epower[i]:
            continue
        usable.append(
            {
                "speed_rad_s": speed_rad_s,
                "torque_nm": torque_nm,
                "thrust_n": abs(thrust[i]) * GRAVITY_M_PER_S2,  # logged in kgf
                "electrical_power_w": epower[i],
                "shaft_power_w": shaft_power_w,
            }
        )
    return usable


def fit_motor(test_rows: list[dict], speed_exponent: float) -> dict | None:
    """Fit K_m and Q_0,ref to a set of rows' losses. Returns them with the fit's R^2, or None."""
    if len(test_rows) < MIN_ROWS:
        return None

    speeds = [r["speed_rad_s"] for r in test_rows]
    torques = [r["torque_nm"] for r in test_rows]
    losses = [ESC_EFFICIENCY * r["electrical_power_w"] - r["shaft_power_w"] for r in test_rows]
    weights = [1 / r["electrical_power_w"] ** 2 for r in test_rows]  # fit relative, not absolute, error
    shapes = [(speed / REFERENCE_SPEED_RAD_S) ** speed_exponent for speed in speeds]

    def best_copper_coefficient(drag_ref_nm: float) -> float:
        """For a given drag, 1 / K_m^2 is a weighted linear least-squares fit, so solve it exactly."""
        g = [(q + drag_ref_nm * s) ** 2 for q, s in zip(torques, shapes)]
        numerator = sum(w * (loss - drag_ref_nm * speed * s) * gi
                        for w, loss, speed, s, gi in zip(weights, losses, speeds, shapes, g))
        denominator = sum(w * gi**2 for w, gi in zip(weights, g))
        return max(numerator / denominator, COPPER_FLOOR)

    def weighted_error(drag_ref_nm: float) -> float:
        c = best_copper_coefficient(drag_ref_nm)
        return sum(
            w * (loss - drag_ref_nm * speed * s - c * (q + drag_ref_nm * s) ** 2) ** 2
            for w, loss, speed, s, q in zip(weights, losses, speeds, shapes, torques)
        )

    # Drag can't cost more than the whole loss at the slowest row, so search below that.
    drag_limit_nm = max(loss / (speed * s) for loss, speed, s in zip(losses, speeds, shapes) if loss > 0)
    drag_ref_nm = minimize(weighted_error, 0.0, drag_limit_nm)
    copper_coefficient = best_copper_coefficient(drag_ref_nm)

    mean_loss = sum(w * loss for w, loss in zip(weights, losses)) / sum(weights)
    total = sum(w * (loss - mean_loss) ** 2 for w, loss in zip(weights, losses))
    r_squared = 1 - weighted_error(drag_ref_nm) / total if total > 0 else 0.0

    at_bound = (
        copper_coefficient <= COPPER_FLOOR
        or drag_ref_nm <= 1e-3 * drag_limit_nm
        or drag_ref_nm >= 0.999 * drag_limit_nm
    )
    return {
        "motor_constant_nm_per_sqrt_w": 1 / math.sqrt(copper_coefficient),
        "drag_torque_nm": drag_ref_nm,
        "r_squared": r_squared,
        "at_bound": at_bound,
        "relative_error": math.sqrt(weighted_error(drag_ref_nm) / len(test_rows)),
    }


def log_scatter(predictions: list[float], actuals: list[float]) -> float:
    """Typical gap between rule and data: the standard deviation of ln(actual / predicted)."""
    return statistics.pstdev(math.log(actual / predicted) for predicted, actual in zip(predictions, actuals))


def log_r_squared(predictions: list[float], actuals: list[float]) -> float:
    mean_log = sum(math.log(actual) for actual in actuals) / len(actuals)
    total = sum((math.log(actual) - mean_log) ** 2 for actual in actuals)
    unexplained = sum(math.log(actual / predicted) ** 2 for predicted, actual in zip(predictions, actuals))
    return 1 - unexplained / total


def load_tests() -> list[tuple[dict, list[dict]]]:
    """Every single-motor test with torque data, as (index entry, usable rows)."""
    index = {test["hash"]: test for test in json.loads((DATA / "index.json").read_text(encoding="utf-8"))}
    tests = []
    for path in sorted((DATA / "tests").glob("*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        meta = index.get(record["hash"])
        # Dual-motor pages mix two powertrains, so leave them out.
        if meta is None or len(meta["powertrains"]) != 1 or len(record["datasets"]) != 1:
            continue
        test_rows = rows(record["datasets"][0])
        if test_rows:
            tests.append((meta, test_rows))
    return tests


def fit_motors(tests: list[tuple[dict, list[dict]]]) -> list[dict]:
    """K_m and drag torque for every linked motor with a listed mass.

    Every test of the same motor is pooled into one fit. Different props load the motor
    differently, which is what lets the fit tell copper loss (grows with torque) from drag (grows
    with speed).
    """
    by_motor: dict[str, dict] = {}
    for meta, test_rows in tests:
        motor = meta["powertrains"][0].get("motor")
        mass_g = motor and (motor["measures"].get("weight") or {}).get("value")
        if not mass_g:
            continue
        entry = by_motor.setdefault(
            motor["hash"], {"title": motor["title"], "mass_kg": mass_g / 1000, "rows": [], "tests": 0}
        )
        # Skip the slowest rows: sensor noise and ESC idle losses dominate there.
        top_speed = max(r["speed_rad_s"] for r in test_rows)
        entry["rows"] += [r for r in test_rows if r["speed_rad_s"] >= MIN_SPEED_FRACTION * top_speed]
        entry["tests"] += 1

    motors = []
    for entry in by_motor.values():
        fit = fit_motor(entry["rows"], CHOSEN_SPEED_EXPONENT)
        # Reject fits the model can't follow, and fits that put all the loss in one term.
        if not fit or fit["r_squared"] < MIN_LOSS_FIT_R2 or fit["at_bound"]:
            continue
        motors.append(
            {
                "title": entry["title"],
                "mass_kg": entry["mass_kg"],
                "motor_constant": fit["motor_constant_nm_per_sqrt_w"],
                "drag_torque": fit["drag_torque_nm"],
                "source": f"{entry['tests']} Tyto tests, loss fit R^2 {fit['r_squared']:.2f}",
            }
        )
    return motors


def datasheet_motors() -> list[dict]:
    """Motors in catalog/motors.csv whose datasheets list R_m and I_0: K_m and drag straight from those."""
    motors = []
    for row in read_rows(PROJECT / "catalog" / "motors.csv"):
        if all(row.get(column) for column in MOTOR_DATASHEET_COLUMNS):
            motor = fully_listed_motor(row)
            motors.append(
                {
                    "title": motor.name,
                    "mass_kg": motor.mass_kg,
                    "motor_constant": motor.motor_constant_nm_per_sqrt_w,
                    "drag_torque": motor.drag_torque_nm(REFERENCE_SPEED_RAD_S, CHOSEN_SPEED_EXPONENT),
                    "source": "datasheet",
                }
            )
    return motors


def main() -> None:
    tests = load_tests()
    print(f"{len(tests)} single-motor tests with torque data")

    # Which drag-speed exponent k fits the whole database best?
    print("\nDrag-speed exponent k: median relative error of the loss fit across tests")
    for k in SPEED_EXPONENTS:
        errors = [fit["relative_error"] for _, r in tests if (fit := fit_motor(r, k))]
        print(f"  k = {k:.2f}: {statistics.median(errors):.4f}  ({len(errors)} tests)")

    # Motors. The rule is fitted to small motors only: across the whole range the slope is
    # steeper, and extrapolating that down to a few grams underestimates small motors.
    all_motors = fit_motors(tests) + datasheet_motors()
    motors = sorted((m for m in all_motors if m["mass_kg"] <= SMALL_MOTOR_MAX_KG), key=lambda m: m["mass_kg"])
    masses = [m["mass_kg"] for m in motors]
    motor_constants = [m["motor_constant"] for m in motors]
    drags = [m["drag_torque"] for m in motors]
    motor_constant_law = PowerLaw.fit(masses, motor_constants)
    drag_law = PowerLaw.fit(masses, drags)
    motor_constant_scatter = log_scatter([motor_constant_law(m) for m in masses], motor_constants)
    drag_scatter = log_scatter([drag_law(m) for m in masses], drags)

    print(f"\nMotors: {len(all_motors)} with a good fit; the rule uses the {len(motors)} of "
          f"{SMALL_MOTOR_MAX_KG * 1000:.0f} g or less ({min(masses) * 1000:.1f} g to {max(masses) * 1000:.0f} g)")
    print(f"  K_m = {motor_constant_law.coefficient:.4g} * m^{motor_constant_law.exponent:.3f}   "
          f"log R^2 {log_r_squared([motor_constant_law(m) for m in masses], motor_constants):.2f}, "
          f"scatter x/÷ {math.exp(motor_constant_scatter):.2f}   (same-shape scaling predicts exponent 0.83)")
    print(f"  Q_0 = {drag_law.coefficient:.4g} * m^{drag_law.exponent:.3f}   "
          f"log R^2 {log_r_squared([drag_law(m) for m in masses], drags):.2f}, "
          f"scatter x/÷ {math.exp(drag_scatter):.2f}   (same-shape scaling predicts exponent 1.00)")
    for m in motors:
        print(f"    {m['mass_kg'] * 1000:6.2f} g  K_m {m['motor_constant'] * 1000:6.2f} mNm/sqrt(W) "
              f"(rule {motor_constant_law(m['mass_kg']) * 1000:6.2f})  Q_0 {m['drag_torque'] * 1000:6.3f} mNm "
              f"(rule {drag_law(m['mass_kg']) * 1000:6.3f})  {m['title']}  [{m['source']}]")

    scaling = json.loads(OUTPUT.read_text(encoding="utf-8")) if OUTPUT.exists() else {}
    scaling.update(
        {
            "source": "Motors: Tyto Robotics test database plus datasheet motors, fitted by tools/fit_tyto.py",
            "speed_exponent": CHOSEN_SPEED_EXPONENT,
            "esc_efficiency_assumed": ESC_EFFICIENCY,
            "motor": {
                "motor_constant": {
                    "coefficient": motor_constant_law.coefficient,
                    "exponent": motor_constant_law.exponent,
                },
                "drag_torque": {"coefficient": drag_law.coefficient, "exponent": drag_law.exponent},
                "motor_constant_log_scatter": motor_constant_scatter,
                "drag_torque_log_scatter": drag_scatter,
                "motors_used": len(motors),
                "mass_range_kg": [min(masses), max(masses)],
            },
        }
    )
    OUTPUT.write_text(json.dumps(scaling, indent=2), encoding="utf-8")
    print(f"\nUpdated the motor rule in {OUTPUT.relative_to(PROJECT)}")


if __name__ == "__main__":
    main()
