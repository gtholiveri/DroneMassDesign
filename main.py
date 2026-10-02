"""Size one drone: set up a scenario, run the mass-closure loop, print the result."""

from drone_sizing.closure import MassDidNotConverge, close_mass
from drone_sizing.constants import JOULES_PER_WATT_HOUR, METERS_PER_INCH
from drone_sizing.inputs import DesignChoices, Requirements, Technology
from drone_sizing.propeller import Propeller
from drone_sizing.report import format_mass_history, format_report

#
requirements = Requirements(
    payload_mass_kg=0.1,
    flight_time_s=10 * 60,
    thrust_to_weight=2.0,
    average_power_factor=1.2,
    mass_margin_fraction=0.10,
)

# Chosen prop
propeller = Propeller(
    name="10x4.5 generic",
    diameter_m=10 * METERS_PER_INCH,
    mass_kg=0.014,
    thrust_coefficient=0.11,
    power_coefficient=0.045,
)

choices = DesignChoices(
    rotor_count=4,
    propeller=propeller,
    cell_count=4,
    avionics_mass_kg=0.100,
)

# Rough values for hobby parts, need to research real values
technology = Technology(
    motor_efficiency=0.85,
    esc_efficiency=0.95,
    battery_specific_energy_j_per_kg=150 * JOULES_PER_WATT_HOUR,  # LiPo pack
    usable_battery_fraction=0.8,
    loaded_cell_voltage_v=3.5,  # LiPo late in a flight
    motor_torque_density_nm_per_kg=2.2,
    esc_specific_power_w_per_kg=6000,
    frame_mass_fraction=0.20,
)

def main() -> None:
    try:
        result = close_mass(requirements, choices, technology)
    except MassDidNotConverge as error:
        print(f"Infeasible: {error}")
        print(f"Guesses: {format_mass_history(error.mass_history_kg)}")
        return

    print(format_report(result))


if __name__ == "__main__":
    main()