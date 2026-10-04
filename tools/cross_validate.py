"""How well do the fitted rules predict parts they have never seen?

The fit tools report how closely each rule follows the data it was fitted to. That flatters a
rule: every point helped pull the line toward itself. This tool checks the honest way. It leaves
parts out, refits the rule on the rest, and predicts the ones left out:

    one at a time     each motor or prop predicted by a rule fitted to all the others
    a group at a time each family of props, or each source of data, predicted by a rule fitted
                      without any of it. Props of one family share a blade shape, so leaving out
                      one of them still leaves its siblings in the fit. This is the harder test,
                      and the closer one to predicting a part from a maker we have no data on.

Errors are ratios (measured / predicted), since the rules are fitted on logs. "x/÷ 1.20" means
predictions are typically within a factor of 1.20 either way (one standard deviation).

Run from the project folder:  python tools/cross_validate.py
"""

import math
import statistics
import sys
from collections.abc import Callable, Sequence
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

from dataclasses import replace  # noqa: E402

import fit_props  # noqa: E402
import fit_tyto  # noqa: E402
from drone_sizing.build import Build  # noqa: E402
from drone_sizing.numerics import PowerLaw  # noqa: E402
from drone_sizing.report import table  # noqa: E402
from parts import AIRFRAMES, BATTERIES, MOTOR_SCALING, MOTORS, PROPELLERS  # noqa: E402
from scenario import REQUIREMENTS, TECHNOLOGY  # noqa: E402

CLOSE = 1.25  # a prediction within this factor of the truth counts as close
MICRO_MOTOR_MAX_KG = 0.007  # the motors a drone this size would use

# A fit from the points kept, returning a function that predicts a point.
Fit = Callable[[Sequence], Callable[[object], float]]


def held_out_errors(points: Sequence, groups: Sequence[str], fit: Fit, truth: Callable[[object], float]) -> list[float]:
    """Log of measured / predicted for every point, each predicted by a fit that left its whole group out."""
    errors = []
    for group in sorted(set(groups)):
        kept = [point for point, its_group in zip(points, groups) if its_group != group]
        predict = fit(kept)
        errors += [
            math.log(truth(point) / predict(point)) for point, its_group in zip(points, groups) if its_group == group
        ]
    return errors


def summary(label: str, errors: list[float], fitted_errors: list[float]) -> list[str]:
    close = sum(abs(error) <= math.log(CLOSE) for error in errors) / len(errors)
    return [
        label,
        f"x/÷ {math.exp(statistics.pstdev(fitted_errors)):.2f}",
        f"x/÷ {math.exp(math.sqrt(statistics.fmean(error**2 for error in errors))):.2f}",
        f"{math.exp(statistics.median(errors)):.2f}",
        f"{close:.0%}",
        f"x {math.exp(max(errors, key=abs)):.2f}",
    ]


HEADERS = ["", "fitted to all", "left out", "median measured / predicted", f"within x/÷ {CLOSE}", "worst miss"]


def motor_rules() -> None:
    motors = sorted(
        (m for m in fit_tyto.fit_motors(fit_tyto.load_tests()) + fit_tyto.datasheet_motors() if m["mass_kg"] <= fit_tyto.SMALL_MOTOR_MAX_KG),
        key=lambda m: m["mass_kg"],
    )
    names = [m["title"] for m in motors]
    sources = ["datasheet" if m["source"] == "datasheet" else "Tyto thrust stand" for m in motors]

    def rule_for(key: str) -> Fit:
        def fit(kept: Sequence) -> Callable[[object], float]:
            law = PowerLaw.fit([m["mass_kg"] for m in kept], [m[key] for m in kept])
            return lambda m: law(m["mass_kg"])

        return fit

    rows = []
    for label, key in (("K_m from mass", "motor_constant"), ("drag torque from mass", "drag_torque")):
        truth = lambda m, key=key: m[key]  # noqa: E731
        fitted = held_out_errors(motors, ["all"] * len(motors), lambda kept, key=key: rule_for(key)(motors), truth)
        rows.append(summary(f"{label}: one motor at a time", held_out_errors(motors, names, rule_for(key), truth), fitted))
        rows.append(summary(f"{label}: one source at a time", held_out_errors(motors, sources, rule_for(key), truth), fitted))

    datasheet = sources.count("datasheet")
    print(f"MOTOR RULE: {len(motors)} motors of {fit_tyto.SMALL_MOTOR_MAX_KG * 1000:.0f} g or less "
          f"({datasheet} from datasheets, {len(motors) - datasheet} from Tyto thrust-stand tests)")
    print(table(HEADERS, rows, "lrrrrr"))


def prop_family(test: fit_props.StaticTest) -> str:
    """Props that share a blade design: the UIUC name up to its size, or the maker's range."""
    if test.source == "UIUC":
        return test.name.split("_")[0]
    words = test.name.split()
    return " ".join(words[:2]) if words[1] == "Hurricane" else words[0]


def prop_rules() -> None:
    tests = [t for t in fit_props.load_uiuc_tests() + fit_props.load_modern_tests() if t.diameter_m <= fit_props.MAX_DIAMETER_M]
    names = [f"{index} {test.name}" for index, test in enumerate(tests)]
    families = [prop_family(test) for test in tests]
    sources = [test.source for test in tests]

    def rule_for(value: Callable[[fit_props.StaticTest], float]) -> Fit:
        def fit(kept: Sequence) -> Callable[[object], float]:
            rule = fit_props.Rule.fit(kept, [value(test) for test in kept])
            return lambda test: rule(test.pitch_ratio, test.blades, test.diameter_m)

        return fit

    thrust = lambda test: test.thrust_coefficient  # noqa: E731
    power = lambda test: test.power_coefficient  # noqa: E731

    rows = []
    for label, groups in (("one prop at a time", names), ("one family at a time", families), ("one source at a time", sources)):
        thrust_errors = held_out_errors(tests, groups, rule_for(thrust), thrust)
        power_errors = held_out_errors(tests, groups, rule_for(power), power)
        fitted_thrust = held_out_errors(tests, ["all"] * len(tests), lambda kept: rule_for(thrust)(tests), thrust)
        fitted_power = held_out_errors(tests, ["all"] * len(tests), lambda kept: rule_for(power)(tests), power)
        # Figure of merit goes as C_T^1.5 / C_P, so its error is this mix of the other two.
        merit = [1.5 * t - p for t, p in zip(thrust_errors, power_errors)]
        fitted_merit = [1.5 * t - p for t, p in zip(fitted_thrust, fitted_power)]
        rows += [
            summary(f"C_T: {label}", thrust_errors, fitted_thrust),
            summary(f"C_P: {label}", power_errors, fitted_power),
            summary(f"figure of merit: {label}", merit, fitted_merit),
        ]

    print(f"PROP RULE: {len(tests)} static tests in {len(set(families))} families from {len(set(sources))} sources")
    print(table(HEADERS, rows, "lrrrrr"))


def motors_on_a_drone() -> None:
    """What the motor rule's error does to a drone: fly each listed micro motor, then its stand-in.

    The stand-in knows only the motor's mass and Kv. Its resistance and no-load current come from
    the rule, refitted without that motor. Both fly the same frame, prop and battery.
    """
    fitted = [
        m for m in fit_tyto.fit_motors(fit_tyto.load_tests()) + fit_tyto.datasheet_motors()
        if m["mass_kg"] <= fit_tyto.SMALL_MOTOR_MAX_KG
    ]
    small_prop = next(p for p in PROPELLERS if p.name == "Bitcraze 55-35")
    large_prop = next(p for p in PROPELLERS if p.name.startswith("Gemfan 65mm"))
    pack = next(b for b in BATTERIES if b.name == "GNB LiHV 850 mAh 60C")
    requirements = replace(REQUIREMENTS, flight_time_s=10 * 60)

    flight_errors, thrust_errors = [], []
    listed_motors = [m for m in MOTORS if not m.estimated_fields and m.mass_kg < MICRO_MOTOR_MAX_KG]
    for listed in listed_motors:
        others = [m for m in fitted if m["title"] != listed.name]
        masses_kg = [m["mass_kg"] for m in others]
        scaling = replace(
            MOTOR_SCALING,
            motor_constant=PowerLaw.fit(masses_kg, [m["motor_constant"] for m in others]),
            drag_torque=PowerLaw.fit(masses_kg, [m["drag_torque"] for m in others]),
        )
        stand_in = scaling.motor_with_kv(listed.name, listed.kv_rpm_per_v, listed.mass_kg)

        # A drone that suits the motor's size: small motors on the 55 mm prop, bigger ones on the 65 mm.
        airframe, prop = (AIRFRAMES[0], small_prop) if listed.mass_kg < 0.003 else (AIRFRAMES[1], large_prop)
        real = Build(airframe, listed, prop, pack).evaluate(requirements, TECHNOLOGY)
        guess = Build(airframe, stand_in, prop, pack).evaluate(requirements, TECHNOLOGY)
        flight_errors.append(math.log(real.flight_time_s / guess.flight_time_s))
        thrust_errors.append(math.log(real.thrust_to_weight / guess.thrust_to_weight))

    def row(label: str, errors: list[float]) -> list[str]:
        return [
            label,
            f"x/÷ {math.exp(math.sqrt(statistics.fmean(error**2 for error in errors))):.2f}",
            f"{math.exp(statistics.median(errors)):.2f}",
            f"{sum(abs(error) <= math.log(1.10) for error in errors) / len(errors):.0%}",
            f"x {math.exp(max(errors, key=abs)):.2f}",
        ]

    print(f"ON A DRONE: {len(listed_motors)} fully listed motors under {MICRO_MOTOR_MAX_KG * 1000:.0f} g, "
          "each against a stand-in known only by its mass and Kv")
    print(table(
        ["", "typical error", "median with listed / with stand-in", "within 10%", "worst miss"],
        [row("flight time", flight_errors), row("thrust-to-weight with sag", thrust_errors)],
        "lrrrr",
    ))


def main() -> None:
    print("Each rule's typical error, when fitted to every part and when predicting parts left out of the fit.")
    print()
    motor_rules()
    print()
    motors_on_a_drone()
    print()
    prop_rules()


if __name__ == "__main__":
    main()
