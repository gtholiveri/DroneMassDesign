"""Turn sizing results into readable text tables.

Nothing here computes physics: every number is read off a result and formatted for a human.
Unit conversion for people (grams, minutes, RPM) happens here and nowhere else.
"""

from collections import Counter
from collections.abc import Sequence

from tabulate import SEPARATING_LINE, tabulate

from drone_sizing.battery import BatteryKind, BatteryPack
from drone_sizing.build import BuildResult, Check
from drone_sizing.constants import GRAVITY_M_PER_S2, JOULES_PER_WATT_HOUR, METERS_PER_INCH, METERS_PER_MILLIMETER, RPM_PER_RAD_PER_S
from drone_sizing.design import DroneDesign
from drone_sizing.inputs import Requirements, Scenario, Technology
from drone_sizing.motor import OperatingPoint
from drone_sizing.search import Evaluation, SearchResult
from drone_sizing.typical import DronesByPropSize, TypicalParts

GRAMS_PER_KG = 1000.0
MILLINEWTON_METERS_PER_NM = 1000.0
MILLIAMP_HOURS_PER_AMP_HOUR = 1000.0
SECONDS_PER_MINUTE = 60.0
MILLIOHMS_PER_OHM = 1000.0

TABLE_FORMAT = "outline"

# How many stator sizes to suggest for a motor of a given mass.
STATOR_SIZES_SHOWN = 3


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


def watts(power_w: float) -> str:
    return f"{power_w:.2f} W"


def price(price_usd: float | None) -> str:
    return "?" if price_usd is None else f"${price_usd:.2f}"


def milliamp_hours(capacity_ah: float) -> str:
    return f"{capacity_ah * MILLIAMP_HOURS_PER_AMP_HOUR:.0f} mAh"


def millimeters(length_m: float) -> str:
    return f"{length_m / METERS_PER_MILLIMETER:.0f} mm"


def check_value(check_or_value: Check | float, unit: str | None = None) -> str:
    """A check's value or limit in its unit: minutes for seconds, two decimals otherwise."""
    value = check_or_value.value if isinstance(check_or_value, Check) else check_or_value
    unit = check_or_value.unit if isinstance(check_or_value, Check) else unit
    if unit == "s":
        return minutes(value)
    if unit in ("A", "V"):
        return f"{value:.2f} {unit}"
    return f"{value:.2f}"


# Any drone, sized or from the catalog: how it flies.


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


def format_operating_points(result: BuildResult) -> str:
    """One rotor at hover and at full throttle, late in the flight and on a fresh pack."""
    build = result.build
    weight_n = result.total_mass_kg * GRAVITY_M_PER_S2
    battery = build.battery

    def row(label: str, point: OperatingPoint, note: str) -> list[str]:
        thrust_n = build.propeller.thrust_at_speed_n(point.speed_rad_s)
        return [
            label,
            f"{grams_force(thrust_n):.1f} g",
            f"{build.airframe.rotor_count * thrust_n / weight_n:.2f}",
            rpm(point.speed_rad_s),
            f"{point.current_a:.2f} A",
            f"{point.voltage_v:.2f} V",
            f"{point.efficiency:.0%}",
            note,
        ]

    rows = [
        row("hover", result.hover, f"{result.hover_throttle:.0%} throttle on a tired pack"),
        row("full throttle, tired pack", result.tired_full_throttle, f"pack sags from {battery.tired_voltage_v:.2f} V at rest"),
        row("full throttle, fresh pack", result.fresh_full_throttle, f"pack sags from {battery.fresh_voltage_v:.2f} V at rest"),
    ]
    headers = ["each rotor", "thrust", "total thrust / weight", "RPM", "motor current", "motor voltage", "motor efficiency", ""]
    return table(headers, rows, "lrrrrrrl")


def format_power(result: BuildResult) -> str:
    """Where the average battery power goes, loss by loss, and the flight time it leaves."""
    power = result.power
    items = [
        ("lifting the drone: the least any rotor this size could use", power.lifting_w),
        ("props: blade drag and tip losses", power.propeller_loss_w),
        ("motors: heat in the windings", power.winding_loss_w),
        ("motors: friction and iron losses", power.motor_drag_loss_w),
        ("ESCs: switching losses", power.esc_loss_w),
        (f"allowance for maneuvering (hover x {result.requirements.average_power_factor})", power.maneuvering_w),
        *power.electronics_w,
    ]
    rows = [[name, watts(power_w), f"{power_w / power.total_w:.0%}"] for name, power_w in items]
    rows += [
        SEPARATING_LINE,
        ["total average draw", watts(power.total_w), "100%"],
        SEPARATING_LINE,
        [f"usable energy ({result.tech.usable_battery_fraction:.0%} of the pack)", f"{result.usable_energy_j / JOULES_PER_WATT_HOUR:.2f} Wh", ""],
        ["flight time", minutes(result.flight_time_s), ""],
        ["grams lifted per watt", f"{result.total_mass_kg * GRAMS_PER_KG / power.total_w:.1f} g/W", ""],
    ]
    return table(["where the power goes", "power", "share"], rows, "lrr")


def format_battery(result: BuildResult, kind: BatteryKind | None = None) -> str:
    """The pack and the currents it must supply. For a sized pack, the kind it would be made of."""
    pack = result.build.battery
    used_wh = result.average_battery_power_w * result.requirements.flight_time_s / JOULES_PER_WATT_HOUR
    average_a = result.average_battery_power_w / pack.nominal_voltage_v
    peak_a = result.peak_battery_current_a
    lowest_cell_v = result.tired_full_throttle.voltage_v / pack.cell_count
    low = "" if lowest_cell_v >= pack.chemistry.min_cell_voltage_v else "  TOO LOW"

    rows = []
    if kind is not None:
        rows.append(["kind", f"{kind.name}, {kind.specific_energy_wh_per_kg:.0f} Wh/kg"])
    rows += [
        [f"energy used in {minutes(result.requirements.flight_time_s)}", f"{used_wh:.2f} Wh"],
        [f"energy stored, so that {result.tech.usable_battery_fraction:.0%} of it is enough", f"{pack.energy_wh:.2f} Wh"],
        ["capacity", f"{milliamp_hours(pack.capacity_ah)} at {pack.cell_count}S ({pack.nominal_voltage_v:.1f} V)"],
        ["mass", grams(pack.mass_kg)],
        ["average current", f"{average_a:.1f} A ({average_a / pack.capacity_ah:.1f} C)"],
        ["peak current, full throttle on a fresh pack", f"{peak_a:.1f} A ({peak_a / pack.capacity_ah:.1f} C)"],
        [
            "lowest cell voltage, full throttle late in the flight",
            f"{lowest_cell_v:.2f} V (must stay above {pack.chemistry.min_cell_voltage_v:.1f} V){low}",
        ],
    ]
    return table(["battery", ""], rows, "ll")


# The scenario, and the drone the component models predict for it (main.py, compare.py, flight_time.py).


def format_requirements(requirements: Requirements, tech: Technology) -> str:
    """What the drone must do, and the margins it is planned with."""
    payload = "none" if requirements.payload_mass_kg == 0 else grams(requirements.payload_mass_kg)
    rows = [
        ["flight time", minutes(requirements.flight_time_s)],
        ["thrust-to-weight", f"{requirements.thrust_to_weight:.2f} at full throttle, late in the flight"],
        ["payload", payload],
        ["mass margin", f"{requirements.mass_margin_fraction:.0%} on top of the parts"],
        ["average power", f"hover x {requirements.average_power_factor}, to cover maneuvering"],
        ["battery used", f"{tech.usable_battery_fraction:.0%}, the rest is the landing reserve"],
    ]
    return table(["requirement", ""], rows, "ll")


def format_fixed_parts(scenario: Scenario) -> str:
    """The parts that are given rather than sized: what each weighs and draws."""
    rows = [
        [part.name, grams(part.mass_kg), watts(part.full_power_w), f"{part.duty_cycle:.0%}", watts(part.average_power_w)]
        for part in scenario.components
    ]
    rows += [SEPARATING_LINE, ["total", grams(scenario.components_mass_kg), "", "", watts(scenario.electronics_power_w)]]
    return table(["fixed part", "mass", "power when on", "duty cycle", "average power"], rows, "lrrrr")


def format_typical_parts(typical: TypicalParts, scenario: Scenario, tech: Technology) -> str:
    """What the sized parts are assumed to be like."""
    battery = scenario.battery
    cells = battery.chemistry
    motor = typical.motor
    propeller = typical.propeller
    frame = typical.frame
    rows = [
        ["battery", f"{battery.name}: {battery.specific_energy_wh_per_kg:.0f} Wh/kg"],
        [
            "battery voltage",
            f"{cells.tired_cell_voltage_v:.2f} V per cell at rest late in the flight, {cells.full_cell_voltage_v:.2f} V "
            f"fresh, less current x resistance ({cells.cell_resistance_ohm_ah * MILLIOHMS_PER_OHM:.0f} milliohm per Ah "
            f"of capacity, plus {tech.lead_resistance_ohm * MILLIOHMS_PER_OHM:.0f} for the leads)",
        ],
        [
            "props",
            f"typical {propeller.blade_count}-bladed, pitch / diameter {propeller.pitch_ratio} (rule fitted to static "
            f"tests), {1 - propeller.hover_thrust_factor:.0%} less thrust at hover speed",
        ],
        [
            "motors",
            f"typical for their mass m (rule fitted to thrust-stand tests): K_m grows as "
            f"m^{motor.motor_constant.exponent:.2f}, drag as m^{motor.drag_torque.exponent:.2f}",
        ],
        ["ESCs", f"{scenario.board.esc_efficiency:.0%} efficient"],
        [
            "frame",
            f"printed, {frame.mass_kg_per_m * GRAMS_PER_KG / 10:.0f} g per 100 mm of motor-to-motor diagonal, "
            f"{millimeters(frame.prop_clearance_m)} between prop tips",
        ],
    ]
    return table(["sized part", "assumed to be"], rows, "ll")


def motor_summary(design: DroneDesign) -> str:
    """A sized motor in one phrase: the stator size it would be sold as, its mass and its Kv."""
    motor = design.motor
    nearest = design.scaling.motor.stator_sizes_near(motor.mass_kg, 1)
    size = f"{nearest[0].code} size, " if nearest else ""
    return f"{size}{motor.mass_kg * GRAMS_PER_KG:.1f} g, {motor.kv_rpm_per_v:,.0f} Kv"


def battery_summary(pack: BatteryPack) -> str:
    return f"{milliamp_hours(pack.capacity_ah)}, {grams(pack.mass_kg)}"


def format_parts_to_look_for(design: DroneDesign) -> str:
    """The sized parts as you would search for them."""
    airframe = design.choices.airframe
    propeller = design.choices.propeller
    motor = design.motor
    cell_count = design.battery.cell_count
    nearest = design.scaling.motor.stator_sizes_near(motor.mass_kg, STATOR_SIZES_SHOWN)

    rows = [
        [
            "frame, to print",
            f"{millimeters(airframe.frame.diagonal_m)} between diagonal motors, with a budget of {grams(airframe.frame.mass_kg)}",
        ],
        [
            f"props (x{airframe.rotor_count})",
            f"{millimeters(propeller.diameter_m)} ({propeller.diameter_m / METERS_PER_INCH:.1f} in), {grams(propeller.mass_kg)} each, "
            f"C_T {propeller.thrust_coefficient:.3f} and C_P {propeller.power_coefficient:.3f} at hover "
            f"(figure of merit {propeller.figure_of_merit:.2f})",
        ],
        [f"motors (x{airframe.rotor_count})", f"{motor_summary(design)} or more on {cell_count}S"],
    ]
    if nearest:
        sizes = ", ".join(f"{size.code} ({grams(size.typical_mass_kg)})" for size in nearest)
        rows.append(["motor size", f"stator diameter and height in mm. Sizes that weigh about this much: {sizes}"])
    rows += [
        [
            "motor needs",
            f"K_m {motor.motor_constant_nm_per_sqrt_w * MILLINEWTON_METERS_PER_NM:.2f} mNm/sqrt(W) or better, "
            f"rated for {design.fresh_full_throttle.current_a:.1f} A or more",
        ],
        ["battery", f"{cell_count}S, {battery_summary(design.battery)} or lighter"],
    ]
    return table(["part", "what to look for"], rows, "ll")


def format_prop_sizes(drones: DronesByPropSize, best: DroneDesign) -> str:
    """The drone each prop size makes, to show why the best one is the best."""
    rows = []
    for diameter_mm in sorted(drones):
        design = drones[diameter_mm]
        if design is None:
            rows.append([f"{diameter_mm} mm", "", "no drone", "", "", ""])
            continue
        rows.append(
            [
                f"{diameter_mm} mm",
                millimeters(design.choices.airframe.frame.diagonal_m),
                grams(design.built_mass_kg),
                battery_summary(design.battery),
                motor_summary(design),
                "lightest" if design is best else "",
            ]
        )
    return table(["prop", "frame", "total mass", "battery", "motor", ""], rows, "rrrrrl")


def format_battery_kinds(best_by_kind: Sequence[tuple[BatteryKind, DroneDesign | None]], chosen: BatteryKind) -> str:
    """The best drone each kind of battery makes, to show what the choice of battery costs."""
    rows = []
    for kind, design in best_by_kind:
        label = f"{kind.name}, {kind.specific_energy_wh_per_kg:.0f} Wh/kg"
        note = "sized above" if kind is chosen else ""
        if design is None:
            rows.append([label, "", "no drone", "", "", note])
            continue
        rows.append(
            [
                label,
                design.choices.propeller.name,
                grams(design.built_mass_kg),
                battery_summary(design.battery),
                motor_summary(design),
                note,
            ]
        )
    return table(["battery kind", "best prop", "total mass", "battery", "motor", ""], rows, "lrrrrl")


# The catalog search (select_parts.py).


def status(evaluation: Evaluation) -> str:
    if evaluation.passes_worst_case:
        return "robust"
    return "passes" if evaluation.passes else "fails"


def format_search(result: SearchResult, limit: int) -> str:
    """How many builds made it, the best `limit` of them, and why others don't fit."""
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

    ranked = result.ranked
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
                check_value(nominal),
                check_margin(nominal),
                check_value(worst),
                check_margin(worst),
            ]
        )
    headers = ["check", "must be", "catalog numbers", "margin", "worst case", "margin"]
    return table(headers, rows, "llrrrr")


def format_margins(evaluation: Evaluation) -> str:
    """How much safety the planning margins add up to, for the two requirements that matter most.

    The margins act on different things (mass, power, energy, voltage) and feed through
    nonlinearly, so they can't be rolled into one input factor. What can be compared is the
    output: the requirement, the best estimate with no margins, the design value with them, and
    the worst case.
    """
    nominal, worst = evaluation.nominal, evaluation.worst_case
    best = nominal.best_estimate
    requirements = nominal.requirements
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


def format_build(evaluation: Evaluation) -> str:
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
            format_operating_points(nominal),
            "",
            format_power(nominal),
            "",
            format_checks(evaluation),
            "",
            format_margins(evaluation),
            "best estimate: no mass margin, steady hover, the whole pack, thrust on a fresh pack",
            "as designed:   with the mass margin, the maneuvering allowance, the landing reserve, thrust on a tired pack",
            "worst case:    as designed, with every uncertain number pushed the wrong way",
        ]
    )
