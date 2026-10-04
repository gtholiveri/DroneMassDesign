"""Does a drone exist that flies this long, and what does it look like?

The general question, answered without any real parts. For each prop size, flight time and kind of
battery, the mass-closure loop builds a drone out of typical parts (models.py) around the fixed
parts in parts.py, with the thrust-to-weight and margins in scenario.py.

A cell reads "none" where no consistent drone exists: past that point every gram of battery costs
more hover power than the energy it brings.

Cell count changes only two things here: the Kv to look for (half the Kv at 2S), and the energy
per kilogram, because a real 2S pack carries a second connector lead and wrap. The model does not
see the other differences: lower currents in the wiring and ESC at 2S, and more room above the
voltage where the board browns out.

main.py lays out the single best drone for the scenario in full.
"""

from dataclasses import replace

from drone_sizing.battery import BatteryKind
from drone_sizing.constants import METERS_PER_MILLIMETER
from drone_sizing.report import GRAMS_PER_KG, battery_summary, table
from drone_sizing.typical import size_drone
from models import BASELINE, TYPICAL
from scenario import BATTERY_KINDS, TECHNOLOGY

DIAMETERS_MM = (55, 65, 76, 89, 102, 127)
FLIGHT_TIMES_MIN = (10, 15, 20, 25, 30)


def design_summary(diameter_mm: float, flight_time_min: float, battery: BatteryKind) -> str:
    """The drone the closure loop finds: total mass, battery, and the motor to look for."""
    requirements = replace(BASELINE.requirements, flight_time_s=flight_time_min * 60)
    scenario = replace(BASELINE, requirements=requirements, battery=battery)
    design = size_drone(scenario, diameter_mm * METERS_PER_MILLIMETER, TYPICAL, TECHNOLOGY)
    if design is None:
        return "none"
    return (
        f"{design.built_mass_kg * GRAMS_PER_KG:.0f} g | {battery_summary(design.battery)}"
        f" | {design.motor.mass_kg * GRAMS_PER_KG:.1f} g {design.motor.kv_rpm_per_v / 1000:.1f}k"
    )


def main() -> None:
    requirements = BASELINE.requirements
    print("Each cell: total mass | battery capacity and mass | motor mass and Kv to look for.")
    print("Typical two-bladed props and typical motors, from fitted rules extracted from scraped test datasets.")
    print(
        f"Carrying {BASELINE.components_mass_kg * GRAMS_PER_KG:.1f} g of board and modules that draw "
        f"{BASELINE.electronics_power_w:.1f} W, at thrust-to-weight {requirements.thrust_to_weight}, with a "
        f"{requirements.mass_margin_fraction:.0%} mass margin, hover power x {requirements.average_power_factor} "
        f"and {TECHNOLOGY.usable_battery_fraction:.0%} of the pack used."
    )
    headers = ["prop"] + [f"{minutes} min" for minutes in FLIGHT_TIMES_MIN]
    for battery in BATTERY_KINDS:
        rows = [
            [f"{diameter_mm} mm"] + [design_summary(diameter_mm, minutes, battery) for minutes in FLIGHT_TIMES_MIN]
            for diameter_mm in DIAMETERS_MM
        ]
        print()
        print(f"{battery.name}: {battery.specific_energy_wh_per_kg:.0f} Wh/kg")
        print(table(headers, rows, "l" * len(headers)))


if __name__ == "__main__":
    main()
