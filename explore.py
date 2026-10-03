"""Does a drone exist that flies this long, and what does it look like?

The general question, answered without any real parts. For each prop size and flight time, the
mass-closure loop sizes a typical prop (from the rule fitted to static tests), the typical motor
for its mass (from the rule fitted to thrust-stand data) and a battery of a given energy per
kilogram, on the smallest printed frame the prop fits. The fixed parts, their power draw and the
margins come from parts.py and scenario.py.

A cell reads "none" where no consistent drone exists: past that point every gram of battery costs
more hover power than the energy it brings.

Cell count changes only two things here: the Kv to look for (half the Kv at 2S), and the energy
per kilogram, because a real 2S pack carries a second connector lead and wrap. The model does not
see the other differences: lower currents in the wiring and ESC at 2S, and more room above the
voltage where the board browns out.

Use the tables to decide what to shop for (prop size, motor mass and Kv, pack size and chemistry),
then put real listings in catalog/ and run select_parts.py.
"""

import math
import statistics
from dataclasses import dataclass, replace

from drone_sizing.airframe import Airframe
from drone_sizing.closure import MassDidNotConverge, close_mass
from drone_sizing.constants import JOULES_PER_WATT_HOUR, KILOGRAMS_PER_GRAM, METERS_PER_MILLIMETER
from drone_sizing.inputs import DesignChoices, ScalingLaws, Technology
from drone_sizing.propeller import Propeller
from drone_sizing.report import GRAMS_PER_KG, MILLIAMP_HOURS_PER_AMP_HOUR, table
from parts import AIRFRAMES, FITTED_SCALING, MOTOR_SCALING, PROP_CLEARANCE_M, printed_frame
from scenario import REQUIREMENTS, TECHNOLOGY, THRUST_TO_WEIGHT_OPTIONS

DIAMETERS_MM = (55, 65, 76, 89, 102, 127)
FLIGHT_TIMES_MIN = (10, 15, 20, 25, 30)
THRUST_TO_WEIGHT = statistics.median(THRUST_TO_WEIGHT_OPTIONS)

# Measured figure of merit is the same from pitch / diameter 0.45 to 0.75, so one pitch stands for all.
PITCH_RATIO = 0.6

# The prop rule is read near each prop's top speed. At hover speed the same props make less
# thrust for the same power: C_T is a few percent lower and C_P about the same, which made them 4%
# less efficient in UIUC's tests and 8 to 13% in Cox and Dantsker's. This factor on C_T costs 10%.
HOVER_THRUST_FACTOR = 0.93

# Prop and frame masses for sizes we don't have listings for, scaled from known points.
REFERENCE_PROP = (55 * METERS_PER_MILLIMETER, 0.33 * KILOGRAMS_PER_GRAM)  # Bitcraze 55-35: mass grows as D^2
FRAME_GRAMS_PER_MM = 6.0 / 100  # TODO: slicer masses; the same guess as parts.py

# Li-ion runs at a lower voltage than LiPo, so the same motor needs a higher Kv.
LI_ION = replace(TECHNOLOGY, nominal_cell_voltage_v=3.6, loaded_cell_voltage_v=3.2, fresh_loaded_cell_voltage_v=3.9)


@dataclass(frozen=True)
class BatteryKind:
    """A kind of battery as real cells deliver it: cells in series, energy per kilogram, voltages."""

    name: str
    cell_count: int
    specific_energy_wh_per_kg: float  # connector or leads included
    tech: Technology


# Energy per kilogram is capacity x nominal voltage / mass, from the listing or datasheet named.
BATTERY_KINDS = (
    BatteryKind("1S LiHV pouch pack, high rate (GNB 550 mAh 13.5 g, 720 mAh 18.5 g)", 1, 150, TECHNOLOGY),
    BatteryKind("1S LiHV pouch pack, lower rate (GNB 850 mAh 17.5 g, 1100 mAh 22 g)", 1, 185, TECHNOLOGY),
    BatteryKind("2S LiHV pouch pack, lower rate (GNB 2S 850 mAh 38 g)", 2, 170, TECHNOLOGY),
    BatteryKind("1S Li-ion 18650 (Molicel P28A: 2,800 mAh, 46 g, plus 3 g of leads)", 1, 205, LI_ION),
    BatteryKind("1S Li-ion 21700 (Samsung 50S: 5,000 mAh, 72.7 g, plus 3 g of leads)", 1, 235, LI_ION),
)


def typical_propeller(diameter_mm: float, pitch_ratio: float = PITCH_RATIO) -> Propeller:
    """A typical two-bladed prop of this size and pitch at hover speed, from the fitted rule."""
    diameter_m = diameter_mm * METERS_PER_MILLIMETER
    reference_diameter_m, reference_mass_kg = REFERENCE_PROP
    propeller = FITTED_SCALING.propeller.propeller(
        name=f"{diameter_mm:.0f} mm",
        diameter_m=diameter_m,
        pitch_m=pitch_ratio * diameter_m,
        blade_count=2,
        mass_kg=reference_mass_kg * (diameter_m / reference_diameter_m) ** 2,
    )
    return replace(propeller, thrust_coefficient=propeller.thrust_coefficient * HOVER_THRUST_FACTOR)


def smallest_airframe_for(propeller: Propeller) -> Airframe:
    """The airframe from parts.py, with the smallest frame this prop fits on.

    Neighboring motors are d * sin(pi / N) apart, and that must cover the prop plus the clearance.
    """
    template = AIRFRAMES[0]
    diagonal_m = (propeller.diameter_m + PROP_CLEARANCE_M) / math.sin(math.pi / template.rotor_count)
    diagonal_mm = diagonal_m / METERS_PER_MILLIMETER
    return replace(template, frame=printed_frame(diagonal_mm, FRAME_GRAMS_PER_MM * diagonal_mm))


def design_summary(propeller: Propeller, flight_time_min: float, battery: BatteryKind) -> str:
    """The drone the closure loop finds: total mass, battery, and the motor to look for."""
    requirements = replace(REQUIREMENTS, thrust_to_weight=THRUST_TO_WEIGHT, flight_time_s=flight_time_min * 60)
    choices = DesignChoices(
        airframe=smallest_airframe_for(propeller), propeller=propeller, cell_count=battery.cell_count
    )
    scaling = ScalingLaws(
        battery_specific_energy_j_per_kg=battery.specific_energy_wh_per_kg * JOULES_PER_WATT_HOUR,
        motor=MOTOR_SCALING,
    )
    try:
        design = close_mass(requirements, choices, battery.tech, scaling).design
    except MassDidNotConverge:
        return "none"
    pack_voltage_v = battery.cell_count * battery.tech.nominal_cell_voltage_v
    capacity_mah = design.battery.energy_wh / pack_voltage_v * MILLIAMP_HOURS_PER_AMP_HOUR
    return (
        f"{design.built_mass_kg * GRAMS_PER_KG:.0f} g | {capacity_mah:.0f} mAh {design.battery.mass_kg * GRAMS_PER_KG:.0f} g"
        f" | {design.motor.mass_kg * GRAMS_PER_KG:.1f} g {design.motor.kv_rpm_per_v / 1000:.1f}k"
    )


def main() -> None:
    if FITTED_SCALING is None:
        raise SystemExit("catalog/scaling.json is missing: run tools/fit_tyto.py and tools/fit_props.py")

    electronics_w = AIRFRAMES[0].electronics_power_w
    fixed_g = (AIRFRAMES[0].fixed_mass_kg - AIRFRAMES[0].frame.mass_kg) * GRAMS_PER_KG
    print("Each cell: total mass | battery capacity and mass | motor mass and Kv to look for.")
    print("Typical two-bladed props and typical motors, from fitted rules extracted from scraped test datasets.")
    print(
        f"Carrying {fixed_g:.1f} g of board and modules that draw {electronics_w:.1f} W, at thrust-to-weight "
        f"{THRUST_TO_WEIGHT}, with a {REQUIREMENTS.mass_margin_fraction:.0%} mass margin, hover power "
        f"x {REQUIREMENTS.average_power_factor} and {TECHNOLOGY.usable_battery_fraction:.0%} of the pack used."
    )
    headers = ["prop"] + [f"{minutes} min" for minutes in FLIGHT_TIMES_MIN]
    for battery in BATTERY_KINDS:
        rows = []
        for diameter_mm in DIAMETERS_MM:
            propeller = typical_propeller(diameter_mm)
            rows.append([propeller.name] + [design_summary(propeller, minutes, battery) for minutes in FLIGHT_TIMES_MIN])
        print()
        print(f"{battery.name}: {battery.specific_energy_wh_per_kg:.0f} Wh/kg")
        print(table(headers, rows, "l" * len(headers)))


if __name__ == "__main__":
    main()
