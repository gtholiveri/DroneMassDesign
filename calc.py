"""A calculator for one drone: put in what you know, get out what follows.

Give the battery and it tells you the flight time. Give the flight time instead and it works back
to the smallest battery that reaches it. Either way it also gives hover current and throttle,
thrust-to-weight, and peak currents, from the same model as the other scripts.

Examples (a Crazyflie 2.1 Brushless, then the battery it would need for 15 minutes):
    python calc.py --mass-without-battery-g 24.9 --motor-kv 10000 --motor-mass-g 2.4 \
        --motor-resistance-ohm 0.52 --motor-no-load-a 0.4 --motor-no-load-v 4 --ct 0.1007 --cp 0.0538 \
        --prop-diameter-mm 55 --electronics-w 0.35 --battery-mah 350 --battery-mass-g 9.1
    python calc.py --mass-without-battery-g 24.9 --motor-kv 10000 --motor-mass-g 2.4 --prop-diameter-mm 55 \
        --prop-pitch-mm 35 --electronics-w 0.35 --flight-min 15

Anything about the motor or prop that you leave out is estimated from the fitted rules in
catalog/scaling.json and marked as an estimate.
"""

import argparse

from drone_sizing.airframe import Airframe, ControllerBoard, Frame
from drone_sizing.battery import BatteryPack
from drone_sizing.build import Build, BuildResult
from drone_sizing.constants import JOULES_PER_WATT_HOUR, KILOGRAMS_PER_GRAM, METERS_PER_MILLIMETER
from drone_sizing.inputs import Requirements, Technology
from drone_sizing.motor import Motor
from drone_sizing.numerics import minimize, solve_increasing
from drone_sizing.propeller import Propeller
from drone_sizing.report import GRAMS_PER_KG, SECONDS_PER_MINUTE, grams, grams_force, minutes, rpm, table
from parts import FITTED_SCALING, MOTOR_SCALING
from scenario import TECHNOLOGY

AMP_HOURS_PER_MILLIAMP_HOUR = 0.001
NO_LIMIT = 1e9  # for ratings the calculator isn't given

# The battery sizes the backward search looks between.
SMALLEST_PACK_AH = 0.02
LARGEST_PACK_AH = 20.0


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Flight time, hover and thrust numbers for one drone.")

    drone = parser.add_argument_group("the drone")
    drone.add_argument("--rotors", type=int, default=4)
    drone.add_argument("--mass-without-battery-g", type=float, required=True, help="the whole drone except its battery: frame, board, modules, motors, props, wires")
    drone.add_argument("--electronics-w", type=float, default=0.5, help="power drawn by everything that isn't a motor")
    drone.add_argument("--esc-efficiency", type=float, default=0.90)

    motor = parser.add_argument_group("the motor (resistance, no-load current and max current are estimated from mass if left out)")
    motor.add_argument("--motor-kv", type=float, required=True, help="RPM per volt")
    motor.add_argument("--motor-mass-g", type=float, required=True)
    motor.add_argument("--motor-resistance-ohm", type=float)
    motor.add_argument("--motor-no-load-a", type=float, help="no-load (idle) current")
    motor.add_argument("--motor-no-load-v", type=float, default=5.0, help="voltage the no-load current was measured at")
    motor.add_argument("--motor-max-a", type=float, help="rated max current")

    prop = parser.add_argument_group("the prop (give C_T and C_P, or a pitch to estimate them from)")
    prop.add_argument("--prop-diameter-mm", type=float, required=True)
    prop.add_argument("--prop-pitch-mm", type=float)
    prop.add_argument("--blades", type=int, default=2)
    prop.add_argument("--ct", type=float, help="thrust coefficient")
    prop.add_argument("--cp", type=float, help="power coefficient")

    battery = parser.add_argument_group("the battery (give its capacity, or the flight time you want)")
    size = battery.add_mutually_exclusive_group(required=True)
    size.add_argument("--battery-mah", type=float, help="capacity: the calculator gives the flight time")
    size.add_argument("--flight-min", type=float, help="flight time wanted: the calculator gives the battery")
    battery.add_argument("--battery-mass-g", type=float, help="estimated from capacity if left out")
    battery.add_argument("--battery-wh-per-kg", type=float, default=185.0, help="used to estimate battery mass")
    battery.add_argument("--cells", type=int, default=1)
    battery.add_argument("--lihv", action="store_true", help="a high-voltage pack: 3.8 V nominal, 4.35 V full")

    margins = parser.add_argument_group("margins (the defaults give a plain estimate with a landing reserve)")
    margins.add_argument("--usable", type=float, default=TECHNOLOGY.usable_battery_fraction, help="fraction of the pack you'll use")
    margins.add_argument("--power-factor", type=float, default=1.0, help="average power over hover power")
    margins.add_argument("--mass-margin", type=float, default=0.0, help="extra mass as a fraction of the drone")
    return parser.parse_args()


def motor_from(args: argparse.Namespace) -> Motor:
    return MOTOR_SCALING.motor_with_kv(
        name="motor",
        kv_rpm_per_v=args.motor_kv,
        mass_kg=args.motor_mass_g * KILOGRAMS_PER_GRAM,
        resistance_ohm=args.motor_resistance_ohm,
        no_load_current_a=args.motor_no_load_a,
        no_load_test_voltage_v=args.motor_no_load_v if args.motor_no_load_a is not None else None,
        max_current_a=args.motor_max_a,
    )


def propeller_from(args: argparse.Namespace) -> Propeller:
    diameter_m = args.prop_diameter_mm * METERS_PER_MILLIMETER
    # The prop's mass is already inside the mass without battery, so it carries none here.
    if args.ct is not None and args.cp is not None:
        return Propeller("prop", diameter_m, 0.0, args.ct, args.cp)
    if args.prop_pitch_mm is None or FITTED_SCALING is None:
        raise SystemExit("Give --ct and --cp, or --prop-pitch-mm (which needs catalog/scaling.json).")
    return FITTED_SCALING.propeller.propeller(
        "prop", diameter_m, args.prop_pitch_mm * METERS_PER_MILLIMETER, args.blades, mass_kg=0.0
    )


def airframe_from(args: argparse.Namespace) -> Airframe:
    """Everything but the battery and the motors (which the build counts itself), as one lump with no size limit."""
    lump_kg = (args.mass_without_battery_g - args.rotors * args.motor_mass_g) * KILOGRAMS_PER_GRAM
    if lump_kg < 0:
        raise SystemExit("--mass-without-battery-g must include the motors, so it can't be less than their total mass.")
    return Airframe(
        rotor_count=args.rotors,
        frame=Frame(name="drone", diagonal_m=NO_LIMIT, mass_kg=lump_kg, prop_clearance_m=0.0),
        board=ControllerBoard(
            name="electronics",
            mass_kg=0.0,
            power_w=args.electronics_w,
            esc_max_current_a=NO_LIMIT,
            esc_efficiency=args.esc_efficiency,
            supported_cell_counts=(args.cells,),
        ),
        components=(),
    )


def pack_of(capacity_ah: float, args: argparse.Namespace, tech: Technology) -> BatteryPack:
    """A pack of this capacity, weighing what it's listed at or what its energy implies."""
    nominal_cell_voltage_v = 3.8 if args.lihv else tech.nominal_cell_voltage_v
    if args.battery_mass_g is not None:
        mass_kg = args.battery_mass_g * KILOGRAMS_PER_GRAM
    else:
        energy_wh = args.cells * nominal_cell_voltage_v * capacity_ah
        mass_kg = energy_wh / args.battery_wh_per_kg
    return BatteryPack(
        name="battery",
        cell_count=args.cells,
        capacity_ah=capacity_ah,
        mass_kg=mass_kg,
        continuous_discharge_c=NO_LIMIT,
        burst_discharge_c=NO_LIMIT,
        nominal_cell_voltage_v=nominal_cell_voltage_v,
        full_cell_voltage_v=4.35 if args.lihv else None,
    )


def smallest_pack_for(flight_time_s: float, evaluate, args: argparse.Namespace) -> float | None:
    """The smallest capacity that reaches the flight time, or None if no battery does.

    A bigger pack flies longer until its own weight costs more than its energy adds. So flight
    time rises to a peak and then falls: find the peak, and if it's high enough, the answer is on
    the rising side.
    """
    def flight_time(capacity_ah: float) -> float:
        return evaluate(capacity_ah).flight_time_s

    best_capacity_ah = minimize(lambda capacity_ah: -flight_time(capacity_ah), SMALLEST_PACK_AH, LARGEST_PACK_AH)
    if flight_time(best_capacity_ah) < flight_time_s:
        longest_min = flight_time(best_capacity_ah) / SECONDS_PER_MINUTE
        print(f"No battery reaches {flight_time_s / SECONDS_PER_MINUTE:.1f} min. The longest possible is "
              f"{longest_min:.1f} min, with {best_capacity_ah / AMP_HOURS_PER_MILLIAMP_HOUR:.0f} mAh:")
        return best_capacity_ah
    return solve_increasing(flight_time, flight_time_s, SMALLEST_PACK_AH, best_capacity_ah)


def format_result(result: BuildResult, args: argparse.Namespace, tech: Technology) -> str:
    build = result.build
    motor, battery = build.motor, build.battery
    tired, fresh = result.tired_full_throttle, result.fresh_full_throttle
    energy_wh = battery.energy_j(tech.nominal_cell_voltage_v) / JOULES_PER_WATT_HOUR
    no_load = (
        "no-load current estimated"
        if "no-load current" in motor.estimated_fields
        else f"no-load {motor.no_load_current_a:.2f} A at {motor.no_load_test_voltage_v:.1f} V"
    )
    resistance = f"{motor.resistance_ohm:.3f} ohm" + (" (estimated)" if "resistance" in motor.estimated_fields else "")
    max_current = f"max {motor.max_current_a:.1f} A" + (" (estimated)" if "max current" in motor.estimated_fields else "")

    inputs = table(
        ["inputs as used", ""],
        [
            ["motor", f"{motor.kv_rpm_per_v:,.0f} Kv, {resistance}, {no_load}, {max_current}"],
            [
                "prop",
                f"C_T {build.propeller.thrust_coefficient:.4f}, C_P {build.propeller.power_coefficient:.4f}"
                + (" (estimated from pitch)" if build.propeller.estimated else ""),
            ],
            [
                "battery",
                f"{battery.capacity_ah / AMP_HOURS_PER_MILLIAMP_HOUR:.0f} mAh {battery.cell_count}S, "
                f"{energy_wh:.2f} Wh, {grams(battery.mass_kg)}"
                + ("" if args.battery_mass_g is not None else f" (mass from {args.battery_wh_per_kg:.0f} Wh/kg)"),
            ],
        ],
        "ll",
    )
    results = table(
        ["results", ""],
        [
            ["total mass", grams(result.total_mass_kg)],
            ["flight time", f"{minutes(result.flight_time_s)}, using {args.usable:.0%} of the pack"],
            [
                "average power",
                f"{result.average_battery_power_w:.2f} W "
                f"({result.total_mass_kg * GRAMS_PER_KG / result.average_battery_power_w:.1f} g/W)",
            ],
            [
                "hover, per motor",
                f"{grams_force(build.propeller.thrust_at_speed_n(result.hover.speed_rad_s)):.1f} g at "
                f"{rpm(result.hover.speed_rad_s)} RPM, {result.hover.current_a:.2f} A, "
                f"motor efficiency {result.hover.efficiency:.0%}",
            ],
            ["hover throttle", f"{result.hover_throttle:.0%} on a tired pack"],
            [
                "thrust-to-weight",
                f"{result.thrust_to_weight:.2f} on a tired pack (sagging to {tired.voltage_v:.2f} V), "
                f"{result.fresh_thrust_to_weight:.2f} on a fresh one",
            ],
            ["peak current", f"{fresh.current_a:.2f} A per motor, {result.peak_battery_current_a:.1f} A from the battery"],
        ],
        "ll",
    )
    lines = [inputs, results]

    if fresh.current_a > motor.max_current_a:
        lines.append(f"WARNING: full throttle draws {fresh.current_a:.2f} A per motor, over its {motor.max_current_a:.1f} A rating.")
    if tired.voltage_v < battery.cell_count * tech.min_cell_voltage_v:
        lines.append(f"WARNING: full throttle pulls a tired pack down to {tired.voltage_v:.2f} V, below the safe minimum.")
    if result.thrust_to_weight < 1.5:
        lines.append(f"WARNING: thrust-to-weight {result.thrust_to_weight:.2f} on a tired pack leaves little for control.")
    return "\n".join(lines)


def main() -> None:
    args = parse_arguments()
    tech = Technology(**{**TECHNOLOGY.__dict__, "usable_battery_fraction": args.usable})
    requirements = Requirements(
        payload_mass_kg=0.0,
        flight_time_s=(args.flight_min or 1.0) * SECONDS_PER_MINUTE,
        thrust_to_weight=1.0,
        average_power_factor=args.power_factor,
        mass_margin_fraction=args.mass_margin,
    )
    airframe, motor, propeller = airframe_from(args), motor_from(args), propeller_from(args)

    def evaluate(capacity_ah: float) -> BuildResult:
        return Build(airframe, motor, propeller, pack_of(capacity_ah, args, tech)).evaluate(requirements, tech)

    if args.battery_mah is not None:
        capacity_ah = args.battery_mah * AMP_HOURS_PER_MILLIAMP_HOUR
    else:
        capacity_ah = smallest_pack_for(requirements.flight_time_s, evaluate, args)

    print(format_result(evaluate(capacity_ah), args, tech))


if __name__ == "__main__":
    main()
