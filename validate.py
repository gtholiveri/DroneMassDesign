"""Check the model against a real drone: given the Crazyflie 2.1 Brushless's parts, does it predict how it flies?

The prop's C_T and C_P come from Bitcraze's one full-throttle point, so full-throttle thrust and
current match by construction. Hover efficiency and hover time are real predictions: they test
whether the motor and prop models carry over from full throttle down to hover, where the drone
spends its flight.

The motor's drag model has one assumption, the exponent k in Q_0 ~ omega^k. Each column tries one.
"""

from dataclasses import replace

import reference
from drone_sizing.build import Build
from drone_sizing.constants import KILOGRAMS_PER_GRAM
from drone_sizing.inputs import Requirements
from drone_sizing.report import GRAMS_PER_KG, SECONDS_PER_MINUTE, grams_force, table
from scenario import TECHNOLOGY

SPEED_EXPONENTS = (0.0, 0.5, 1.0)

# Pure hover with the real masses: no maneuvering factor, no margin, the whole pack.
HOVER_ONLY = Requirements(
    payload_mass_kg=0.0,
    flight_time_s=reference.FLIGHT_TIME_S,
    thrust_to_weight=1.0,
    average_power_factor=1.0,
    mass_margin_fraction=0.0,
)


def main() -> None:
    rows: dict[str, list[str]] = {
        "Prop C_T": [],
        "Prop C_P": [],
        "Prop figure of merit": [],
        "Thrust, full throttle 4.0 V": [],
        "Current, full throttle 4.0 V": [],
        "Hover current per motor": [],
        "Hover motor efficiency": [],
        "Hover efficiency, battery side": [],
        "Hover time on the whole pack": [],
        "Pack used by 10 min of hover": [],
    }

    for speed_exponent in SPEED_EXPONENTS:
        tech = replace(TECHNOLOGY, no_load_current_speed_exponent=speed_exponent, usable_battery_fraction=1.0)
        propeller = reference.propeller(speed_exponent)
        build = Build(reference.AIRFRAME, reference.MOTOR, propeller, reference.BATTERY)
        result = build.evaluate(HOVER_ONLY, tech)
        peak = reference.MOTOR.full_throttle(reference.PEAK_VOLTAGE_V, propeller, speed_exponent)
        fraction_for_ten_minutes = result.average_battery_power_w * reference.FLIGHT_TIME_S / reference.BATTERY.energy_j

        rows["Prop C_T"].append(f"{propeller.thrust_coefficient:.4f}")
        rows["Prop C_P"].append(f"{propeller.power_coefficient:.4f}")
        rows["Prop figure of merit"].append(f"{propeller.figure_of_merit:.2f}")
        rows["Thrust, full throttle 4.0 V"].append(f"{grams_force(propeller.thrust_at_speed_n(peak.speed_rad_s)):.1f} g")
        rows["Current, full throttle 4.0 V"].append(f"{peak.current_a:.2f} A")
        rows["Hover current per motor"].append(f"{result.hover.current_a:.2f} A")
        rows["Hover motor efficiency"].append(f"{result.hover.efficiency:.0%}")
        rows["Hover efficiency, battery side"].append(
            f"{result.total_mass_kg * GRAMS_PER_KG / result.average_battery_power_w:.1f} g/W"
        )
        rows["Hover time on the whole pack"].append(f"{result.flight_time_s / SECONDS_PER_MINUTE:.1f} min")
        rows["Pack used by 10 min of hover"].append(f"{fraction_for_ten_minutes:.0%}")

    published = {
        "Thrust, full throttle 4.0 V": "30 g",
        "Current, full throttle 4.0 V": "1.80 A",
        "Hover efficiency, battery side": "over 5 g/W",
        "Hover time on the whole pack": "over 10 min",
    }

    print(
        f"Crazyflie 2.1 Brushless, {reference.TAKEOFF_MASS_KG / KILOGRAMS_PER_GRAM:.0f} g, "
        f"{reference.BATTERY.energy_wh:.2f} Wh pack: model vs Bitcraze"
    )
    headers = ["", "Bitcraze"] + [f"k = {k:.1f}" for k in SPEED_EXPONENTS]
    table_rows = [[label, published.get(label, "")] + values for label, values in rows.items()]
    print(table(headers, table_rows, "lr" + "r" * len(SPEED_EXPONENTS)))


if __name__ == "__main__":
    main()
