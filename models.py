"""The component models for this project, and the baseline scenario they size a drone for.

This is the glue between scenario.py (what the drone must do, and its battery) and parts.py (what
it carries, and the rules fitted to test data). The sizing itself lives in drone_sizing/typical.py.

main.py lays the best drone out, compare.py puts the best drones of several scenarios side by side,
flight_time.py flies one drone under several conditions, and explore.py tabulates prop sizes
against flight times.
"""

from drone_sizing import typical
from drone_sizing.constants import KILOGRAMS_PER_GRAM, METERS_PER_MILLIMETER
from drone_sizing.design import DroneDesign
from drone_sizing.inputs import Scenario
from drone_sizing.typical import DronesByPropSize, PropSizes, TypicalParts, TypicalPropeller
from parts import FITTED_SCALING, FIXED_PARTS, MOTOR_SCALING, TYPICAL_FRAME
from scenario import BATTERY, REQUIREMENTS, TECHNOLOGY

if FITTED_SCALING is None:
    raise SystemExit("catalog/scaling.json is missing: run tools/fit_tyto.py and tools/fit_props.py")

TYPICAL = TypicalParts(
    propeller=TypicalPropeller(
        scaling=FITTED_SCALING.propeller,
        blade_count=2,
        # Measured figure of merit is the same from pitch / diameter 0.45 to 0.75, so one pitch stands for all.
        pitch_ratio=0.6,
        # The prop rule is read near each prop's top speed. At hover speed the same props make
        # less thrust for the same power: C_T is a few percent lower and C_P about the same,
        # which made them 4% less efficient in UIUC's tests and 8 to 13% in Cox and Dantsker's.
        # This factor on C_T costs 10%.
        hover_thrust_factor=0.93,
        # Prop mass for sizes we have no listing for: the Bitcraze 55-35, scaled as diameter squared.
        reference_diameter_m=55 * METERS_PER_MILLIMETER,
        reference_mass_kg=0.33 * KILOGRAMS_PER_GRAM,
    ),
    frame=TYPICAL_FRAME,
    motor=MOTOR_SCALING,
)

# Prop sizes to try: every 10 mm across the range, then every 5 mm around the best.
PROP_SIZES = PropSizes(smallest_mm=40, largest_mm=130, coarse_step_mm=10, fine_step_mm=5)

# The scenario as written in scenario.py and parts.py. Variations are made from it with replace().
BASELINE = Scenario(name="baseline", requirements=REQUIREMENTS, battery=BATTERY, components=FIXED_PARTS)


def drones_by_prop_size(scenario: Scenario) -> DronesByPropSize:
    """The drone each prop size makes for this scenario, from the project's component models."""
    return typical.drones_by_prop_size(scenario, TYPICAL, TECHNOLOGY, PROP_SIZES)


def best_drone(scenario: Scenario) -> DroneDesign | None:
    """The lightest drone that meets this scenario, or None if there is none."""
    return typical.best_drone(scenario, TYPICAL, TECHNOLOGY, PROP_SIZES)
