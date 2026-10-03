"""What we need the drone to do, and what we assume about hardware. Shared by every script."""

from drone_sizing.build import Uncertainty
from drone_sizing.inputs import Requirements, Technology

REQUIREMENTS = Requirements(
    payload_mass_kg=0.0,  # nothing carried: the UWB and LED modules are airframe parts in parts.py
    flight_time_s=20 * 60,
    thrust_to_weight=2.0,  # the scripts replace this with each value below # TODO if this isn't used, get rid of it
    average_power_factor=1.2,
    mass_margin_fraction=0.15,
)

# The scripts run once per thrust-to-weight ratio.
THRUST_TO_WEIGHT_OPTIONS = (1.5, 1.75, 2.0)

TECHNOLOGY = Technology(
    usable_battery_fraction=0.8,
    nominal_cell_voltage_v=3.7,
    fresh_cell_voltage_v=4.2,
    tired_cell_voltage_v=3.7,
    min_cell_voltage_v=3.0,
    # No measured pack resistances yet. About 25 milliohm for a 1 Ah high-rate cell is a typical
    # figure, plus about 15 milliohm for leads and a small connector. TODO: measure a real pack.
    cell_resistance_ohm_ah=0.025,
    lead_resistance_ohm=0.015,
    loaded_cell_voltage_v=3.5,
    fresh_loaded_cell_voltage_v=4.0,
    no_load_current_speed_exponent=0.5,  # matches the Crazyflie best; see validate.py
)

# How far the least certain catalog numbers might be off. select_parts.py re-checks every build
# with all of these at once, to see which builds still work if the numbers are optimistic.
UNCERTAINTY = Uncertainty(
    # Numbers from a datasheet or a thrust table.
    thrust_coefficient=0.10,
    power_coefficient=0.10,
    no_load_current=0.30,
    battery_capacity=0.10,
    battery_resistance=0.50,  # pack resistance is a rule of thumb here, so a wide allowance
    # A prop estimated from the fitted rule (tools/fit_props.py). C_T scatters x/÷ 1.21 and C_P
    # x/÷ 1.31, but they miss together (a wide blade raises both), so the figure of merit only
    # scatters x/÷ 1.18. These two put thrust one standard deviation low and the figure of merit
    # about two low (0.83^1.5 / 1.10 = 0.69).
    estimated_thrust_coefficient=0.17,
    estimated_power_coefficient=0.10,
    # A motor estimated from the fitted rules: one standard deviation of each fit's scatter, as
    # printed by tools/fit_tyto.py (x/÷ 1.44 for K_m, x/÷ 1.95 for drag).
    estimated_motor_constant=0.31,
    estimated_no_load_current=0.95,
)
