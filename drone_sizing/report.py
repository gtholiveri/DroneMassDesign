"""Turn sizing results into readable text tables. Unit conversion for humans happens here, not in the physics."""

from collections import Counter
from collections.abc import Sequence

from tabulate import SEPARATING_LINE, tabulate

from drone_sizing.build import BuildResult, Check
from drone_sizing.closure import ClosureResult
from drone_sizing.constants import GRAVITY_M_PER_S2, JOULES_PER_WATT_HOUR, RPM_PER_RAD_PER_S
from drone_sizing.inputs import Requirements, Technology
from drone_sizing.motor import OperatingPoint
from drone_sizing.search import Evaluation, SearchResult

GRAMS_PER_KG = 1000.0
MILLINEWTON_METERS_PER_NM = 1000.0
MILLIAMP_HOURS_PER_AMP_HOUR = 1000.0
SECONDS_PER_MINUTE = 60.0

TABLE_FORMAT = "outline"


def table(headers: Sequence[str], rows: Sequence, align: str) -> str:
    """An ASCII table of already-formatted cells.

    `align` has one letter per column: l for left, r for right. A row of SEPARATING_LINE draws a
    rule, for setting a total apart.
    """
    column_alignment = ["left" if letter == "l" else "right" for letter in align]
    return tabulate(rows, headers=headers, tablefmt=TABLE_FORMAT, colalign=column_alignment, disable_numparse=True)


# How each quantity is shown to a human.


def grams(mass_kg: float) -> str:
    return f"{mass_kg * GRAMS_PER_KG:.1f} g"


def grams_force(thrust_n: float) -> float:
    """Thrust expressed as the mass it could hold up, in grams, the way thrust tables quote it."""
    return thrust_n / GRAVITY_M_PER_S2 * GRAMS_PER_KG


def rpm(speed_rad_s: float) -> str:
    return f"{speed_rad_s * RPM_PER_RAD_PER_S:,.0f}"


def minutes(time_s: float) -> str:
    return f"{time_s / SECONDS_PER_MINUTE:.1f} min"


def price(price_usd: float | None) -> str:
    return "?" if price_usd is None else f"${price_usd:.2f}"


def milliamp_hours(energy_wh: float, cell_count: int, nominal_cell_voltage_v: float) -> str:
    capacity_ah = energy_wh / (cell_count * nominal_cell_voltage_v)
    return f"{capacity_ah * MILLIAMP_HOURS_PER_AMP_HOUR:.0f} mAh"


# Pieces shared by the reports.


def format_mass_breakdown(mass_breakdown_kg: dict[str, float]) -> str:
    """Every part's mass and share, with the margin's size spelled out and a total."""
    total_mass_kg = sum(mass_breakdown_kg.values())
    margin_kg = mass_breakdown_kg.get("margin", 0.0)
    parts_kg = total_mass_kg - margin_kg - mass_breakdown_kg.get("payload", 0.0)

    rows = []
    for part, mass_kg in mass_breakdown_kg.items():
        if part == "payload" and mass_kg == 0:
            continue  # nothing carried, so nothing to show
        if part == "margin":
            # The margin is a fraction of the parts, which is a smaller share of the total.
            part = f"margin ({margin_kg / parts_kg:.0%} of the parts above)"
        rows.append([part, grams(mass_kg), f"{mass_kg / total_mass_kg:.1%}"])
    rows += [SEPARATING_LINE, ["total", grams(total_mass_kg), "100.0%"]]
    return table(["part", "mass", "share of total"], rows, "lrr")


def format_mass_history(mass_history_kg: list[float]) -> str:
    """Show the sequence of guesses, shortened in the middle if it's long."""
    masses_g = [f"{mass_kg * GRAMS_PER_KG:.1f}" for mass_kg in mass_history_kg]
    if len(masses_g) > 8:
        masses_g = masses_g[:5] + ["..."] + masses_g[-2:]
    return " -> ".join(masses_g) + " g"


# The continuous model (main.py).


def format_designs(results: list[ClosureResult]) -> str:
    """One row per continuous design, for comparing props."""
    rows = []
    for result in results:
        design = result.design
        rows.append(
            [
                design.choices.airframe.frame.name,
                design.choices.propeller.name,
                grams(design.built_mass_kg),
                f"{design.motor.mass_kg * GRAMS_PER_KG:.2f} g",
                f"{design.motor.kv_rpm_per_v:,.0f}",
                f"{design.hover.efficiency:.0%}",
                milliamp_hours(design.battery.energy_wh, design.choices.cell_count, design.tech.nominal_cell_voltage_v),
            ]
        )
    headers = ["frame", "prop", "total mass", "motor mass", "motor Kv", "motor efficiency at hover", "battery"]
    return table(headers, rows, "llrrrrr")


def format_report(result: ClosureResult) -> str:
    """The continuous model's answer in full: the consistent design and what to shop for."""
    design = result.design
    rotor = design.rotor
    propeller = rotor.propeller
    motor = design.motor
    fresh = design.fresh_full_throttle
    cell_count = design.choices.cell_count
    electronics_power_w = design.choices.airframe.electronics_power_w

    rotor_table = table(
        ["", "hover", "full throttle"],
        [
            ["thrust per rotor", f"{grams_force(rotor.hover_thrust_n):.1f} g", f"{grams_force(rotor.max_thrust_n):.1f} g"],
            ["speed (RPM)", rpm(rotor.hover_speed_rad_s), rpm(rotor.max_speed_rad_s)],
            [
                "torque",
                f"{rotor.hover_torque_nm * MILLINEWTON_METERS_PER_NM:.3f} mNm",
                f"{rotor.max_torque_nm * MILLINEWTON_METERS_PER_NM:.3f} mNm",
            ],
            ["shaft power", f"{rotor.hover_shaft_power_w:.2f} W", f"{rotor.max_shaft_power_w:.2f} W"],
        ],
        "lrr",
    )
    motor_table = table(
        ["motor to shop for", f"x{design.choices.airframe.rotor_count}"],
        [
            ["mass", f"{motor.mass_kg * GRAMS_PER_KG:.2f} g"],
            ["Kv", f"{motor.kv_rpm_per_v:,.0f} RPM/V on {cell_count}S"],
            ["K_m (this or higher)", f"{motor.motor_constant_nm_per_sqrt_w * MILLINEWTON_METERS_PER_NM:.3f} mNm/sqrt(W)"],
            ["winding resistance at that Kv", f"{motor.resistance_ohm:.3f} ohm"],
            ["current at hover", f"{design.hover.current_a:.2f} A"],
            ["efficiency at hover", f"{design.hover.efficiency:.0%}"],
            ["current rating needed (full throttle, fresh pack)", f"{fresh.current_a:.2f} A"],
        ],
        "lr",
    )
    power_table = table(
        ["power and battery", ""],
        [
            [
                f"motors, hover power x{design.requirements.average_power_factor} for maneuvering (battery side)",
                f"{design.average_battery_power_w - electronics_power_w:.2f} W",
            ],
            ["electronics", f"{electronics_power_w:.2f} W"],
            ["average battery power", f"{design.average_battery_power_w:.2f} W"],
            ["power loading", f"{design.built_mass_kg * GRAMS_PER_KG / design.average_battery_power_w:.1f} g/W"],
            [
                f"battery for {minutes(design.requirements.flight_time_s)}",
                f"{design.battery.energy_wh:.2f} Wh = "
                f"{milliamp_hours(design.battery.energy_wh, cell_count, design.tech.nominal_cell_voltage_v)} at {cell_count}S",
            ],
        ],
        "lr",
    )

    return "\n".join(
        [
            f"Consistent design: {grams(design.built_mass_kg)} after {len(result.mass_history_kg)} guesses",
            f"Guesses: {format_mass_history(result.mass_history_kg)}",
            "",
            format_mass_breakdown(design.mass_breakdown_kg),
            "",
            f"Rotor: {propeller.name}, figure of merit {propeller.figure_of_merit:.2f}, "
            f"disk loading {rotor.hover_thrust_n / propeller.disk_area_m2:.1f} N/m^2",
            rotor_table,
            "",
            motor_table,
            "",
            power_table,
        ]
    )


def format_kv_windows(
    windows: dict[tuple[float, float], tuple[float, float] | None],
    masses_kg: list[float],
    max_currents_a: list[float],
) -> str:
    """Usable Kv ranges: one row per motor mass, one column per rated max current."""
    rows = []
    for mass_kg in masses_kg:
        row = [f"{mass_kg * GRAMS_PER_KG:.1f} g"]
        for current_a in max_currents_a:
            window = windows[(mass_kg, current_a)]
            row.append("none" if window is None else f"{window[0]:,.0f} to {window[1]:,.0f}")
        rows.append(row)
    headers = ["motor mass"] + [f"Kv if rated {current_a:g} A" for current_a in max_currents_a]
    return table(headers, rows, "l" + "r" * len(max_currents_a))


# The catalog search (select_parts.py).


def status(evaluation: Evaluation) -> str:
    if evaluation.passes_worst_case:
        return "robust"
    return "passes" if evaluation.passes else "fails"


def format_search(result: SearchResult, ranked: list[Evaluation], limit: int) -> str:
    """How many builds made it, the first `limit` of them in the order given, and why others don't fit."""
    passing = [evaluation for evaluation in result.evaluations if evaluation.passes]
    robust = [evaluation for evaluation in passing if evaluation.passes_worst_case]
    funnel = table(
        ["combinations", "count"],
        [
            ["tried", f"{result.combination_count:,}"],
            ["parts don't fit together", f"{len(result.incompatible):,}"],
            ["evaluated", f"{len(result.evaluations):,}"],
            ["pass every check on catalog numbers", f"{len(passing):,}"],
            ["still pass in the worst case (robust)", f"{len(robust):,}"],
        ],
        "lr",
    )

    rows = []
    for rank, evaluation in enumerate(ranked[:limit], start=1):
        nominal = evaluation.nominal
        tightest = nominal.tightest_check
        rows.append(
            [
                str(rank),
                status(evaluation),
                price(evaluation.build.price_usd),
                grams(nominal.total_mass_kg),
                f"{nominal.thrust_to_weight:.2f}",
                minutes(nominal.flight_time_s),
                f"{tightest.name} {tightest.margin:+.0%}",
                *evaluation.build.part_names,
            ]
        )
    headers = ["#", "status", "price", "mass", "TWR with sag", "flight", "tightest check", "frame", "motor", "prop", "battery"]
    lines = [funnel, "", table(headers, rows, "rlrrrrlllll")]

    if len(ranked) > limit:
        lines.append(f"... and {len(ranked) - limit:,} more")
    if any(evaluation.build.uses_estimates for evaluation in ranked[:limit]):
        lines.append("* numbers for this part came from a model or a fitted rule, not its own datasheet or test")

    if result.incompatible:
        reasons = Counter(problem for _, problems in result.incompatible for problem in problems)
        reason_rows = [[reason, f"{count:,}"] for reason, count in reasons.most_common()]
        lines += ["", table(["why parts don't fit together", "combinations"], reason_rows, "lr")]
    return "\n".join(lines)


def format_operating_points(result: BuildResult, tech: Technology) -> str:
    """One motor at hover and at full throttle on a tired and a fresh pack."""
    build = result.build

    def row(label: str, point: OperatingPoint, note: str) -> list[str]:
        return [
            label,
            f"{grams_force(build.propeller.thrust_at_speed_n(point.speed_rad_s)):.1f} g",
            rpm(point.speed_rad_s),
            f"{point.current_a:.2f} A",
            f"{point.voltage_v:.2f} V",
            note,
        ]

    tired_resting_v = build.battery.cell_count * tech.tired_cell_voltage_v
    fresh_resting_v = build.battery.fresh_voltage_v(tech.fresh_cell_voltage_v)
    rows = [
        row(
            "hover",
            result.hover,
            f"{result.hover_throttle:.0%} throttle on a tired pack, motor efficiency {result.hover.efficiency:.0%}",
        ),
        row("full throttle, tired pack", result.tired_full_throttle, f"pack sags from {tired_resting_v:.2f} V at rest"),
        row("full throttle, fresh pack", result.fresh_full_throttle, f"pack sags from {fresh_resting_v:.2f} V at rest"),
    ]
    return table(["per motor", "thrust", "RPM", "current", "voltage", ""], rows, "lrrrrl")


def format_power(result: BuildResult, requirements: Requirements, tech: Technology) -> str:
    build = result.build
    airframe = build.airframe
    electronics_power_w = airframe.electronics_power_w
    electronics = ", ".join(["board"] + [component.name for component in airframe.components])
    usable_energy_wh = (
        tech.usable_battery_fraction * build.battery.energy_j(tech.nominal_cell_voltage_v) / JOULES_PER_WATT_HOUR
    )
    rows = [
        [f"{airframe.rotor_count} motors at hover, at the battery", f"{result.propulsion_power_w:.2f} W"],
        [
            f"  x {requirements.average_power_factor} allowance for maneuvering",
            f"{requirements.average_power_factor * result.propulsion_power_w:.2f} W",
        ],
        [f"electronics ({electronics})", f"{electronics_power_w:.2f} W"],
        ["average draw", f"{result.average_battery_power_w:.2f} W"],
        SEPARATING_LINE,
        [f"usable energy ({tech.usable_battery_fraction:.0%} of the pack)", f"{usable_energy_wh:.2f} Wh"],
        ["flight time", minutes(result.flight_time_s)],
        ["grams lifted per watt", f"{result.total_mass_kg * GRAMS_PER_KG / result.average_battery_power_w:.1f} g/W"],
    ]
    return table(["power and flight time", ""], rows, "lr")


def check_value(value: float, unit: str) -> str:
    if unit == "s":
        return minutes(value)
    if unit in ("A", "V"):
        return f"{value:.2f} {unit}"
    return f"{value:.2f}"


def check_margin(check: Check) -> str:
    return f"{check.margin:+.0%}" + ("" if check.passed else " FAIL")


def format_checks(evaluation: Evaluation) -> str:
    """Every check side by side: on the catalog numbers, and with the uncertain numbers pushed the wrong way."""
    rows = []
    for nominal, worst in zip(evaluation.nominal.checks, evaluation.worst_case.checks):
        relation = "at least" if nominal.is_minimum else "at most"
        rows.append(
            [
                nominal.name,
                f"{relation} {check_value(nominal.limit, nominal.unit)}",
                check_value(nominal.value, nominal.unit),
                check_margin(nominal),
                check_value(worst.value, worst.unit),
                check_margin(worst),
            ]
        )
    headers = ["check", "must be", "catalog numbers", "margin", "worst case", "margin"]
    return table(headers, rows, "llrrrr")


def format_margins(evaluation: Evaluation, requirements: Requirements, tech: Technology) -> str:
    """How much safety the planning margins add up to, for the two requirements that matter most.

    The margins act on different things (mass, power, energy, voltage) and feed through
    nonlinearly, so they can't be rolled into one input factor. What can be compared is the
    output: the requirement, the best estimate with no margins, the design value with them, and
    the worst case.
    """
    best = evaluation.build.evaluate_without_margins(requirements, tech)
    nominal, worst = evaluation.nominal, evaluation.worst_case
    rows = [
        [
            "flight time",
            minutes(requirements.flight_time_s),
            minutes(best.flight_time_s),
            minutes(nominal.flight_time_s),
            minutes(worst.flight_time_s),
            f"{best.flight_time_s / requirements.flight_time_s:.2f}",
        ],
        [
            "thrust-to-weight",
            f"{requirements.thrust_to_weight:.2f}",
            f"{best.fresh_thrust_to_weight:.2f}",
            f"{nominal.thrust_to_weight:.2f}",
            f"{worst.thrust_to_weight:.2f}",
            f"{best.fresh_thrust_to_weight / requirements.thrust_to_weight:.2f}",
        ],
    ]
    headers = ["", "required", "best estimate", "as designed", "worst case", "best estimate / required"]
    return table(headers, rows, "lrrrrr")


def format_build(evaluation: Evaluation, requirements: Requirements, tech: Technology) -> str:
    """Everything about one build: mass, operating points, power, every check, and the margins."""
    nominal = evaluation.nominal
    build = nominal.build
    return "\n".join(
        [
            " | ".join(build.part_names),
            f"Price of motors, props and battery: {price(build.price_usd)}",
            "",
            format_mass_breakdown(nominal.mass_breakdown_kg),
            "",
            format_operating_points(nominal, tech),
            "",
            format_power(nominal, requirements, tech),
            "",
            format_checks(evaluation),
            "",
            format_margins(evaluation, requirements, tech),
            "best estimate: no mass margin, steady hover, the whole pack, thrust on a fresh pack",
            "as designed:   with the mass margin, the maneuvering allowance, the landing reserve, thrust on a tired pack",
            "worst case:    as designed, with every uncertain number pushed the wrong way",
        ]
    )
