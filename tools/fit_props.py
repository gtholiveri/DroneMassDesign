"""Fit the rule for a typical small prop from static tests made with a torque cell.

Two sources, both in data/:
    data/uiuc/                         UIUC Propeller Database, Volume 2 (run tools/fetch_uiuc.py):
                                       C_T and C_P against RPM for park-flyer and research props.
    data/cox_dantsker_2026/table2.csv  Table 2 of Cox and Dantsker, "Performance Testing of Small
                                       Multi-Rotor UAV Propellers", AIAA 2026-2306: thrust and torque
                                       at top speed for 18 current FPV props, typed in from the paper.

The rule predicts C_T and C_P from what a listing gives: pitch / diameter, blade count and diameter,
    C = coefficient * (pitch / diameter)^a * (blades / 2)^b * diameter^c
fitted by least squares on the logs. Only props of 140 mm or less are used: their blades run at
the Reynolds numbers ours do, where props are less efficient than the big ones in most
thrust-stand databases. The diameter term is that same effect inside the range.

Both sources are read near the top of each prop's speed range. At hover speed C_T is a few
percent lower and C_P about the same, so a real prop hovers a little worse than the rule says.

Where a prop's name gives no pitch, the pitch comes from its measured blade angle at 75% radius.

Updates the "propeller" part of catalog/scaling.json and prints how well the rule fits.
Run from the project folder:  python tools/fit_props.py
"""

import csv
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))

import reference  # noqa: E402
from drone_sizing.constants import METERS_PER_INCH, METERS_PER_MILLIMETER  # noqa: E402
from drone_sizing.report import table  # noqa: E402
from scenario import TECHNOLOGY  # noqa: E402

UIUC = PROJECT / "data" / "uiuc"
MODERN = PROJECT / "data" / "cox_dantsker_2026" / "table2.csv"
OUTPUT = PROJECT / "catalog" / "scaling.json"

MAX_DIAMETER_M = 0.140
PITCH_STATION = 0.75  # fraction of the radius where a prop's pitch is defined
PITCH_BANDS = ((0.30, 0.45), (0.45, 0.60), (0.60, 0.75), (0.75, 0.90), (0.90, 1.10))
DIAMETER_BANDS_MM = ((40, 50), (50, 60), (60, 70), (70, 90), (90, 141))
BLADE_COUNTS_IN_DATA = (3, 4)  # besides two

# The paper measured air density during each test but doesn't list it. Bloomington is about
# 235 m up, so at room temperature its air is a little thinner than at sea level.
MODERN_TEST_AIR_DENSITY_KG_PER_M3 = 1.17

INCH_NAMED = re.compile(r"(?:apcff|apcsp|da4002|da4022|da4052|gwsdd|mi|mit)_([\d.]+)x([\d.]+)")
MILLIMETER_NAMED = re.compile(r"(?:ef|kpf|pl|vp)_(\d+)x(\d+)")
ANGLE_NAMED = re.compile(r"nr640_(\d+)_(\d+)deg")  # diameter in inches, blade angle at 75% radius
UNNAMED_DIAMETERS_M = {"cfnq": 1.85 * METERS_PER_INCH, "union_u80": 0.080, "pl_triturbo": 2.55 * METERS_PER_INCH}
THREE_BLADED = ("mit_", "pl_triturbo")


@dataclass(frozen=True)
class StaticTest:
    """One prop on the stand: its size as a listing would give it, and what it did."""

    name: str
    source: str
    diameter_m: float
    pitch_ratio: float
    blades: int
    thrust_coefficient: float
    power_coefficient: float

    @property
    def figure_of_merit(self) -> float:
        return figure_of_merit(self.thrust_coefficient, self.power_coefficient)


def figure_of_merit(thrust_coefficient: float, power_coefficient: float) -> float:
    """Ideal hover power over actual: C_T^1.5 / (sqrt(pi / 2) * C_P)."""
    return thrust_coefficient**1.5 / (math.sqrt(math.pi / 2) * power_coefficient)


def pitch_from_angle_m(angle_deg: float, diameter_m: float) -> float:
    """The distance a blade set at this angle at 75% radius would screw forward in one turn."""
    return math.pi * PITCH_STATION * diameter_m * math.tan(math.radians(angle_deg))


def read_table(path: Path) -> list[tuple[float, ...]]:
    """The numeric rows of a UIUC data file, skipping its header line."""
    lines = path.read_text(encoding="utf-8").splitlines()[1:]
    return [tuple(float(cell) for cell in line.split()) for line in lines if line.strip()]


def blade_angle_deg(prop: str) -> float | None:
    """The measured blade angle at 75% radius, if this prop's geometry was measured."""
    path = UIUC / f"{prop}_geom.txt"
    if not path.exists():
        return None
    rows = read_table(path)  # r/R, chord / R, blade angle
    for (inner, _, inner_angle), (outer, _, outer_angle) in zip(rows, rows[1:]):
        if inner <= PITCH_STATION <= outer:
            fraction = (PITCH_STATION - inner) / (outer - inner)
            return inner_angle + fraction * (outer_angle - inner_angle)
    return None


def size_from_name(name: str) -> tuple[float, float | None]:
    """Diameter and nominal pitch in meters. The pitch is None if the name doesn't give one."""
    if match := INCH_NAMED.match(name):
        return float(match.group(1)) * METERS_PER_INCH, float(match.group(2)) * METERS_PER_INCH
    if match := MILLIMETER_NAMED.match(name):
        return float(match.group(1)) * METERS_PER_MILLIMETER, float(match.group(2)) * METERS_PER_MILLIMETER
    if match := ANGLE_NAMED.match(name):
        diameter_m = float(match.group(1)) * METERS_PER_INCH
        return diameter_m, pitch_from_angle_m(float(match.group(2)), diameter_m)
    for prefix, diameter_m in UNNAMED_DIAMETERS_M.items():
        if name.startswith(prefix):
            return diameter_m, None
    raise ValueError(f"don't know the size of {name}")


def load_uiuc_tests() -> list[StaticTest]:
    tests = []
    for path in sorted(UIUC.glob("*_static_*.txt")):
        name = path.stem.split("_static_")[0]
        blades = 3 if name.startswith(THREE_BLADED) or name.endswith("_3b") else 4 if name.endswith("_4b") else 2
        diameter_m, pitch_m = size_from_name(name)
        if pitch_m is None:
            angle_deg = blade_angle_deg(re.sub(r"_[34]b$", "", name))
            if angle_deg is None:
                continue
            pitch_m = pitch_from_angle_m(angle_deg, diameter_m)

        # Average over the top half of the speed range, where the readings are steadiest.
        rows = sorted(read_table(path))  # RPM, C_T, C_P
        fast = rows[len(rows) // 2 :]
        tests.append(
            StaticTest(
                name=name,
                source="UIUC",
                diameter_m=diameter_m,
                pitch_ratio=pitch_m / diameter_m,
                blades=blades,
                thrust_coefficient=statistics.mean(row[1] for row in fast),
                power_coefficient=statistics.mean(row[2] for row in fast),
            )
        )
    return tests


def load_modern_tests() -> list[StaticTest]:
    """The paper's props, from thrust T and torque Q at top speed n (rev/s):
    C_T = T / (rho n^2 D^4) and C_P = 2 pi Q / (rho n^2 D^5). Props with no stated pitch are skipped.
    """
    if not MODERN.exists():
        return []
    density = MODERN_TEST_AIR_DENSITY_KG_PER_M3
    tests = []
    with open(MODERN, newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            if not row["pitch_in"]:
                continue
            diameter_m = float(row["diameter_mm"]) * METERS_PER_MILLIMETER
            speed_rev_s = float(row["max_rpm"]) / 60
            dynamic = density * speed_rev_s**2 * diameter_m**4
            tests.append(
                StaticTest(
                    name=row["name"],
                    source="Cox and Dantsker",
                    diameter_m=diameter_m,
                    pitch_ratio=float(row["pitch_in"]) * METERS_PER_INCH / diameter_m,
                    blades=int(row["blades"]),
                    thrust_coefficient=float(row["max_thrust_n"]) / dynamic,
                    power_coefficient=2 * math.pi * float(row["max_torque_nm"]) / (dynamic * diameter_m),
                )
            )
    return tests


def solve(matrix: list[list[float]], right_side: list[float]) -> list[float]:
    """Solve a small linear system by Gauss-Jordan elimination with pivoting."""
    size = len(matrix)
    rows = [row[:] + [value] for row, value in zip(matrix, right_side)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda r: abs(rows[r][column]))
        rows[column], rows[pivot] = rows[pivot], rows[column]
        for r in range(size):
            if r != column:
                factor = rows[r][column] / rows[column][column]
                rows[r] = [a - factor * b for a, b in zip(rows[r], rows[column])]
    return [rows[i][size] / rows[i][i] for i in range(size)]


@dataclass(frozen=True)
class Rule:
    """coefficient * (pitch / diameter)^pitch_exponent * (blades / 2)^blade_exponent * diameter_m^diameter_exponent."""

    coefficient: float
    pitch_exponent: float
    blade_exponent: float
    diameter_exponent: float

    def __call__(self, pitch_ratio: float, blades: int, diameter_m: float) -> float:
        return (
            self.coefficient
            * pitch_ratio**self.pitch_exponent
            * (blades / 2) ** self.blade_exponent
            * diameter_m**self.diameter_exponent
        )

    @classmethod
    def fit(cls, tests: list[StaticTest], values: list[float]) -> "Rule":
        """Least squares on the logs: ln(value) = ln(coefficient) + a ln(P/D) + b ln(blades / 2) + c ln(D)."""
        inputs = [
            (1.0, math.log(test.pitch_ratio), math.log(test.blades / 2), math.log(test.diameter_m)) for test in tests
        ]
        targets = [math.log(value) for value in values]
        unknowns = range(len(inputs[0]))
        normal_matrix = [[sum(x[i] * x[j] for x in inputs) for j in unknowns] for i in unknowns]
        normal_right_side = [sum(x[i] * target for x, target in zip(inputs, targets)) for i in unknowns]
        log_coefficient, *exponents = solve(normal_matrix, normal_right_side)
        return cls(math.exp(log_coefficient), *exponents)

    def log_errors(self, tests: list[StaticTest], values: list[float]) -> list[float]:
        return [
            math.log(value / self(test.pitch_ratio, test.blades, test.diameter_m))
            for test, value in zip(tests, values)
        ]

    def describe(self) -> str:
        return (
            f"{self.coefficient:.4f} * (P/D)^{self.pitch_exponent:.2f} * (blades/2)^{self.blade_exponent:.2f}"
            f" * D^{self.diameter_exponent:.2f}"
        )


def correlation(xs: list[float], ys: list[float]) -> float:
    mean_x, mean_y = statistics.mean(xs), statistics.mean(ys)
    covariance = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    return covariance / math.sqrt(sum((x - mean_x) ** 2 for x in xs) * sum((y - mean_y) ** 2 for y in ys))


def spread(log_errors: list[float]) -> str:
    return f"x/÷ {math.exp(statistics.pstdev(log_errors)):.2f}"


def median_and_range(values: list[float]) -> tuple[str, str]:
    return f"{statistics.median(values):.2f}", f"{min(values):.2f} to {max(values):.2f}"


def main() -> None:
    tests = [test for test in load_uiuc_tests() + load_modern_tests() if test.diameter_m <= MAX_DIAMETER_M]
    thrust = [test.thrust_coefficient for test in tests]
    power = [test.power_coefficient for test in tests]
    thrust_rule, power_rule = Rule.fit(tests, thrust), Rule.fit(tests, power)
    thrust_errors, power_errors = thrust_rule.log_errors(tests, thrust), power_rule.log_errors(tests, power)
    # FM goes as C_T^1.5 / C_P, so its error is this mix of the other two.
    merit_errors = [1.5 * t - p for t, p in zip(thrust_errors, power_errors)]

    diameters_mm = [test.diameter_m / METERS_PER_MILLIMETER for test in tests]
    print(f"{len(tests)} static tests of props from {min(diameters_mm):.0f} to {max(diameters_mm):.0f} mm\n")
    print(table(
        ("", "rule (D in meters)", "scatter"),
        [
            ("C_T", thrust_rule.describe(), spread(thrust_errors)),
            ("C_P", power_rule.describe(), spread(power_errors)),
            ("figure of merit", "from the two above", spread(merit_errors)),
        ],
        "lll",
    ))
    print(f"C_T and C_P miss in the same direction (correlation {correlation(thrust_errors, power_errors):.2f}): "
          "a wide blade raises both.\nSo the figure of merit, which sets hover power, is better known than either.\n")

    # Does one rule serve both kinds of prop? A ratio of 1.00 means no bias for that source.
    by_source = []
    for source in sorted({test.source for test in tests}):
        chosen = [i for i, test in enumerate(tests) if test.source == source]
        by_source.append(
            (source, len(chosen))
            + tuple(
                f"{math.exp(statistics.mean(errors[i] for i in chosen)):.2f}"
                for errors in (thrust_errors, power_errors, merit_errors)
            )
        )
    print(table(("source", "tests", "C_T / rule", "C_P / rule", "figure of merit / rule"), by_source, "lrrrr"))

    two_bladed = [test for test in tests if test.blades == 2]
    pitch_rows = []
    for low, high in PITCH_BANDS:
        merits = [test.figure_of_merit for test in two_bladed if low <= test.pitch_ratio < high]
        pitch_rows.append((f"{low:.2f} to {high:.2f}", len(merits), *median_and_range(merits)))
    print()
    print(table(("pitch / diameter, two blades", "tests", "median figure of merit", "range"), pitch_rows, "lrrr"))

    diameter_rows = []
    for low, high in DIAMETER_BANDS_MM:
        merits = [test.figure_of_merit for test, mm in zip(tests, diameters_mm) if low <= mm < high]
        diameter_rows.append((f"{low} to {high} mm", len(merits), *median_and_range(merits)))
    print()
    print(table(("diameter, any blade count", "tests", "median figure of merit", "range"), diameter_rows, "lrrr"))

    crazyflie = reference.propeller(TECHNOLOGY.no_load_current_speed_exponent)
    rule_thrust = thrust_rule(35 / 55, 2, crazyflie.diameter_m)
    rule_power = power_rule(35 / 55, 2, crazyflie.diameter_m)
    print(f"\nCheck against the {crazyflie.name} (not in the fit; from Bitcraze's thrust table and our motor model):")
    print(table(
        ("", "C_T", "C_P", "figure of merit"),
        [
            ("thrust table", f"{crazyflie.thrust_coefficient:.4f}", f"{crazyflie.power_coefficient:.4f}", f"{crazyflie.figure_of_merit:.2f}"),
            ("rule", f"{rule_thrust:.4f}", f"{rule_power:.4f}", f"{figure_of_merit(rule_thrust, rule_power):.2f}"),
        ],
        "lrrr",
    ))

    def law_entry(rule: Rule) -> dict:
        return {
            "coefficient": rule.coefficient,
            "pitch_ratio_exponent": rule.pitch_exponent,
            "diameter_exponent": rule.diameter_exponent,
        }

    scaling = json.loads(OUTPUT.read_text(encoding="utf-8")) if OUTPUT.exists() else {}
    scaling["propeller"] = {
        "source": "UIUC Propeller Database Volume 2 and Cox and Dantsker (AIAA 2026-2306), fitted by tools/fit_props.py",
        "thrust_coefficient": law_entry(thrust_rule),
        "power_coefficient": law_entry(power_rule),
        "thrust_coefficient_log_scatter": statistics.pstdev(thrust_errors),
        "power_coefficient_log_scatter": statistics.pstdev(power_errors),
        "figure_of_merit_log_scatter": statistics.pstdev(merit_errors),
        "blade_factors": {
            str(blades): {
                "thrust": (blades / 2) ** thrust_rule.blade_exponent,
                "power": (blades / 2) ** power_rule.blade_exponent,
            }
            for blades in BLADE_COUNTS_IN_DATA
        },
        "tests_used": len(tests),
        "diameter_range_m": [min(test.diameter_m for test in tests), max(test.diameter_m for test in tests)],
    }
    OUTPUT.write_text(json.dumps(scaling, indent=2), encoding="utf-8")
    print(f"\nUpdated the propeller rule in {OUTPUT.relative_to(PROJECT)}")


if __name__ == "__main__":
    main()
