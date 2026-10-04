"""Compare the drones main.py would find for variations of the scenario.

For each variant the component models find the best drone exactly as main.py does, and one table
puts them side by side. Every drone here is different: each is sized for its own variant. To hold
one drone fixed and see how its flight time moves instead, use flight_time.py.

A variant is the baseline (scenario.py and parts.py) with something replaced. The baseline carries
FIXED_PARTS = (BOARD, UWB, LED), so:
    another LED module      replace(BASELINE, name="...", components=(BOARD, UWB, OTHER_LED))
    the same LED, dimmer    replace(BASELINE, name="...", components=(BOARD, UWB, replace(LED, duty_cycle=0.4)))
    another board           replace(BASELINE, name="...", components=(OTHER_BOARD, UWB, LED))
    another flight time     replace(BASELINE, name="...", requirements=replace(REQUIREMENTS, flight_time_s=900))
    another battery         replace(BASELINE, name="...", battery=LI_ION_18650)

Edit VARIANTS to ask your own question.
"""

from dataclasses import replace

from drone_sizing.design import DroneDesign
from drone_sizing.inputs import Scenario
from drone_sizing.report import battery_summary, grams, millimeters, minutes, motor_summary, table, watts
from models import BASELINE, best_drone
from parts import (
    BETAFPV_F4_1S_5A,
    BOARD,
    CRAZYFLIE_BOLT,
    EXAMPLE_BRIGHT_LED,
    EXAMPLE_SMALL_LED,
    LED,
    UWB,
)
from scenario import LI_ION_18650, REQUIREMENTS

VARIANTS = (
    BASELINE,
    replace(BASELINE, name="LED module: example small (1 g, 1 W)", components=(BOARD, UWB, EXAMPLE_SMALL_LED)),
    replace(BASELINE, name="LED module: example bright (5 g, 8 W)", components=(BOARD, UWB, EXAMPLE_BRIGHT_LED)),
    replace(BASELINE, name="LED module: none", components=(BOARD, UWB)),
    replace(BASELINE, name="LED duty cycle: 40%", components=(BOARD, UWB, replace(LED, duty_cycle=0.40))),
    replace(BASELINE, name="LED duty cycle: 20%", components=(BOARD, UWB, replace(LED, duty_cycle=0.20))),
    replace(BASELINE, name=f"board: {BETAFPV_F4_1S_5A.name}", components=(BETAFPV_F4_1S_5A, UWB, LED)),
    replace(BASELINE, name=f"board: {CRAZYFLIE_BOLT.name}", components=(CRAZYFLIE_BOLT, UWB, LED)),
    replace(BASELINE, name="flight: 15 min", requirements=replace(REQUIREMENTS, flight_time_s=15 * 60)),
    replace(BASELINE, name=f"battery: {LI_ION_18650.name}", battery=LI_ION_18650),
)


def row(scenario: Scenario, design: DroneDesign | None, baseline: DroneDesign | None) -> list[str]:
    """One variant: what its fixed parts weigh and draw, and the best drone that leads to."""
    given = [
        scenario.name,
        minutes(scenario.requirements.flight_time_s),
        grams(scenario.components_mass_kg),
        watts(scenario.electronics_power_w),
    ]
    if design is None:
        return given + ["no drone"] + [""] * 6

    if baseline is None or design is baseline:
        change = ""
    else:
        change = f"{design.built_mass_kg / baseline.built_mass_kg - 1:+.0%}"
    return given + [
        grams(design.built_mass_kg),
        change,
        design.choices.propeller.name,
        millimeters(design.choices.airframe.frame.diagonal_m),
        battery_summary(design.battery),
        motor_summary(design),
        watts(design.result.average_battery_power_w),
    ]


def main() -> None:
    drones = [best_drone(scenario) for scenario in VARIANTS]
    baseline = drones[0]
    headers = [
        "scenario", "flight", "fixed mass", "fixed power",
        "total mass", "vs baseline", "prop", "frame", "battery", "motor", "average draw",
    ]  # fmt: skip
    print("The best drone for each variation of the scenario: the lightest that meets it, from the component models.")
    print("Fixed mass and power are the board and modules. Everything to their right is the drone that results.")
    print()
    print(table(headers, [row(scenario, drone, baseline) for scenario, drone in zip(VARIANTS, drones)], "lrrrrrrrrrr"))


if __name__ == "__main__":
    main()
