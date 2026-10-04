"""Take one drone and see how changing things moves its flight time.

The drone is the one main.py finds for the scenario: its frame, props, motors and battery are then
held fixed. Each variant changes something else (the LED module or how it is lit, the controller
board, how hard the drone is flown, the margins) and the same drone is flown again.

This is the controlled comparison: only the thing named changes. compare.py asks the other
question, how the best drone itself would change.

A variant is the baseline (scenario.py and parts.py) with something replaced. The baseline carries
FIXED_PARTS = (BOARD, UWB, LED), so:
    another LED module      replace(BASELINE, name="...", components=(BOARD, UWB, OTHER_LED))
    the same LED, dimmer    replace(BASELINE, name="...", components=(BOARD, UWB, replace(LED, duty_cycle=0.4)))
    another board           replace(BASELINE, name="...", components=(OTHER_BOARD, UWB, LED))
    flown harder            replace(BASELINE, name="...", requirements=replace(REQUIREMENTS, average_power_factor=1.4))

Edit VARIANTS to ask your own question.
"""

from dataclasses import replace

from drone_sizing.airframe import Component
from drone_sizing.build import Build, BuildResult
from drone_sizing.constants import KILOGRAMS_PER_GRAM
from drone_sizing.inputs import Scenario
from drone_sizing.report import battery_summary, grams, minutes, motor_summary, table, watts
from models import BASELINE, best_drone
from parts import BOARD, EXAMPLE_BRIGHT_LED, EXAMPLE_SMALL_LED, UWB, GRAM
from scenario import TECHNOLOGY



LED_LOW_POWER = Component(name="low power", mass_kg= 5.0 * KILOGRAMS_PER_GRAM, full_power_w=0.5, duty_cycle=1.0)
LED_MID_POWER = Component(name="low power", mass_kg= 5.0 * KILOGRAMS_PER_GRAM, full_power_w=2, duty_cycle=1.0)
LED_HIGH_POWER = Component(name="low power", mass_kg= 5.0 * KILOGRAMS_PER_GRAM, full_power_w=8, duty_cycle=1.0)

VARIANTS = (
    BASELINE,
    replace(BASELINE, name="Lights draw .5W (5 g)", components=(BOARD, UWB, LED_LOW_POWER)),
    replace(BASELINE, name="Lights draw 1W (5 g)", components=(BOARD, UWB, LED_MID_POWER)),
    replace(BASELINE, name="Lights draw 8W  (5 g)", components=(BOARD, UWB, LED_HIGH_POWER)),
)


def fly(drone: Build, scenario: Scenario) -> BuildResult:
    """The same drone, carrying this scenario's fixed parts and flown the way it says."""
    airframe = replace(drone.airframe, components=scenario.components)
    return replace(drone, airframe=airframe).evaluate(scenario.requirements, TECHNOLOGY)


def row(scenario: Scenario, flight: BuildResult, baseline: BuildResult) -> list[str]:
    change_s = flight.flight_time_s - baseline.flight_time_s
    return [
        scenario.name,
        grams(scenario.components_mass_kg),
        watts(scenario.electronics_power_w),
        f"x {scenario.requirements.average_power_factor}",
        grams(flight.total_mass_kg),
        watts(flight.average_battery_power_w),
        minutes(flight.flight_time_s),
        "" if flight is baseline else f"{change_s / 60:+.1f} min ({change_s / baseline.flight_time_s:+.0%})",
        f"{flight.thrust_to_weight:.2f}",
    ]


def main() -> None:
    design = best_drone(BASELINE)
    if design is None:
        raise SystemExit("No drone meets the baseline scenario, so there is none to fly. Run main.py for details.")
    drone = design.build

    print("The same drone flown under each variation. Only what the row names changes.")
    print(
        f"The drone is the best one for the baseline scenario: {design.choices.propeller.name} props, "
        f"{motor_summary(design)} motors, {design.battery.cell_count}S battery of {battery_summary(design.battery)}, "
        f"{grams(design.choices.airframe.frame.mass_kg)} frame."
    )
    print()

    flights = [fly(drone, scenario) for scenario in VARIANTS]
    headers = [
        "variation", "fixed mass", "fixed power", "power factor",
        "total mass", "average draw", "flight time", "vs baseline", "TWR with sag",
    ]  # fmt: skip
    rows = [row(scenario, flight, flights[0]) for scenario, flight in zip(VARIANTS, flights)]
    print(table(headers, rows, "lrrrrrrrr"))
    print("Total mass includes the mass margin, which is a fraction of the parts and so moves with them.")


if __name__ == "__main__":
    main()
