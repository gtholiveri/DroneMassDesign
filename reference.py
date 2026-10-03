"""A real drone to check the model against: the Crazyflie 2.1 Brushless, from Bitcraze's published numbers.

Sources: the Crazyflie 2.1 Brushless datasheet (rev 3), Bitcraze's blog post on its 08028 motor,
and Bitcraze's store pages for the 08028 motor, the 55-35 prop and the 350 mAh battery.
"""

from drone_sizing.airframe import Airframe, ControllerBoard, Frame
from drone_sizing.battery import BatteryPack
from drone_sizing.constants import GRAVITY_M_PER_S2, KILOGRAMS_PER_GRAM, METERS_PER_MILLIMETER
from drone_sizing.motor import Motor
from drone_sizing.propeller import Propeller

# What Bitcraze reports for the whole drone.
TAKEOFF_MASS_KG = 34 * KILOGRAMS_PER_GRAM  # with legs
FLIGHT_TIME_S = 10 * 60
HOVER_EFFICIENCY_G_PER_W = 5.0  # "over 5" (the post writes W/g, but means grams per watt)

MOTOR = Motor(
    name="Bitcraze 08028-10000KV",
    kv_rpm_per_v=10000,
    resistance_ohm=0.52,
    no_load_current_a=0.40,
    no_load_test_voltage_v=4.0,
    mass_kg=2.4 * KILOGRAMS_PER_GRAM,
    max_current_a=1.8,  # "peak current"
    shaft_diameter_m=1.0 * METERS_PER_MILLIMETER,
)

# Bitcraze's one published full-throttle point for this motor with the 55-35 prop.
PEAK_VOLTAGE_V = 4.0
PEAK_CURRENT_A = 1.8
PEAK_THRUST_N = 30 * KILOGRAMS_PER_GRAM * GRAVITY_M_PER_S2

PROPELLER_MASS_KG = 0.33 * KILOGRAMS_PER_GRAM


def propeller(speed_exponent: float) -> Propeller:
    """The 55-35 prop, with C_T and C_P backed out of Bitcraze's peak point through the motor model.

    How much of the peak current went to drag depends on the drag model, so the coefficients are
    recomputed for each exponent.
    """
    peak = MOTOR.measured_point(PEAK_VOLTAGE_V, PEAK_CURRENT_A, speed_exponent)
    return Propeller.from_test_point(
        name="Bitcraze 55-35",
        diameter_m=55 * METERS_PER_MILLIMETER,
        mass_kg=PROPELLER_MASS_KG,
        thrust_n=PEAK_THRUST_N,
        speed_rad_s=peak.speed_rad_s,
        shaft_power_w=peak.shaft_power_w,
        bore_diameter_m=1.0 * METERS_PER_MILLIMETER,
    )


BATTERY = BatteryPack(
    name="Bitcraze 350 mAh 1S",
    cell_count=1,
    capacity_ah=0.350,
    mass_kg=9.1 * KILOGRAMS_PER_GRAM,
    continuous_discharge_c=15,
    burst_discharge_c=30,
)

# The circuit board is the frame. Bitcraze doesn't publish the board's mass, so it's whatever the
# takeoff mass leaves after the motors, props and battery (legs included).
BOARD_MASS_KG = TAKEOFF_MASS_KG - 4 * MOTOR.mass_kg - 4 * PROPELLER_MASS_KG - BATTERY.mass_kg

AIRFRAME = Airframe(
    rotor_count=4,
    frame=Frame(
        name="Crazyflie board as frame",
        diagonal_m=100 * METERS_PER_MILLIMETER,
        mass_kg=0.0,
        prop_clearance_m=0.0,
    ),
    board=ControllerBoard(
        name="Crazyflie 2.1 Brushless board",
        mass_kg=BOARD_MASS_KG,
        power_w=0.35,  # guess: MCU, radio and sensors at about 0.1 A
        esc_max_current_a=5.0,  # "1-cell 5A ESCs"
        esc_efficiency=0.90,  # guess
        supported_cell_counts=(1,),
    ),
    components=(),
)
