"""The best drone for the scenario, as the component models predict it.

Reads what the drone must do from scenario.py and what it carries from parts.py, builds a drone
out of typical parts (models.py) for each prop size, and lays out the lightest one in full: its
mass, where its power goes, its battery, its rotors and the parts to look for.

The kind of battery is part of the scenario (BATTERY in scenario.py), not something this script
chooses: the kind with the most energy per kilogram would always win, even at sizes where no such
cell is sold. The last table shows the best drone each of the other kinds would make.

No real listings are involved. select_parts.py picks actual parts, compare.py puts variations of
the scenario side by side, and explore.py shows how the answer moves with flight time.
"""

from dataclasses import replace

from drone_sizing.report import (
    GRAMS_PER_KG,
    format_battery,
    format_battery_kinds,
    format_fixed_parts,
    format_mass_breakdown,
    format_operating_points,
    format_parts_to_look_for,
    format_power,
    format_prop_sizes,
    format_requirements,
    format_typical_parts,
)
from drone_sizing.typical import lightest
from models import BASELINE, PROP_SIZES, TYPICAL, best_drone, drones_by_prop_size
from scenario import BATTERY_KINDS, TECHNOLOGY


def print_section(title: str, *blocks: str) -> None:
    print()
    print(title)
    for block in blocks:
        print(block)


def main() -> None:
    drones = drones_by_prop_size(BASELINE)
    best = lightest(drones)
    sizes = f"{PROP_SIZES.smallest_mm} to {PROP_SIZES.largest_mm} mm"

    print_section("SCENARIO (scenario.py)", format_requirements(BASELINE.requirements, TECHNOLOGY))
    print_section("FIXED PARTS (parts.py)", format_fixed_parts(BASELINE))
    print_section("COMPONENT MODELS (models.py, scenario.py)", format_typical_parts(TYPICAL, BASELINE, TECHNOLOGY))

    if best is None:
        print_section(
            "NO DRONE MEETS THIS SCENARIO",
            f"With props from {sizes} and this kind of battery, the mass never closes: every gram of battery "
            "added costs more hover power than the energy it brings.",
            "Shorten the flight, cut the fixed power, or choose a kind of battery with more energy per kilogram.",
        )
    else:
        result = best.result
        print_section(
            "BEST DRONE FOR THIS SCENARIO",
            f"{best.built_mass_kg * GRAMS_PER_KG:.0f} g on {best.choices.propeller.name} props: "
            f"the lightest drone that meets the scenario, of prop sizes from {sizes}.",
        )
        print_section("MASS", format_mass_breakdown(result.mass_breakdown_kg))
        print_section("POWER, averaged over the flight", format_power(result))
        print_section("BATTERY", format_battery(result, BASELINE.battery))
        print_section("ROTORS", format_operating_points(result))
        print_section("PARTS TO LOOK FOR", format_parts_to_look_for(best))
        print_section("OTHER PROP SIZES", format_prop_sizes(drones, best))

    best_by_kind = [
        (kind, best if kind is BASELINE.battery else best_drone(replace(BASELINE, battery=kind)))
        for kind in BATTERY_KINDS
    ]
    print_section(
        "OTHER KINDS OF BATTERY",
        format_battery_kinds(best_by_kind, BASELINE.battery),
        "Each row is the lightest drone that kind of battery makes. Li-ion is sold only as whole cells,",
        "so those rows are buildable only where the capacity comes out near one cell.",
    )


if __name__ == "__main__":
    main()
