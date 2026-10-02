"""The three kinds of input to a sizing run, kept separate because they change for different reasons."""

from dataclasses import dataclass

from drone_sizing.propeller import Propeller


@dataclass(frozen=True)
class Requirements:
    """What the mission demands. Change these when the job changes."""

    payload_mass_kg: float
    flight_time_s: float

    # Max total thrust divided by weight. About 2 is the usual minimum for control and wind.
    thrust_to_weight: float

    # Ratio of average battery power to hover power
    # Basically the expected power usage for maneuvering beyond pure hover
    # 1.0 means we expect energy usage to be basically pure hover
    # by vibes, 1.1-1.2 should mean we account for climbing and maneuvering
    average_power_factor: float

    # Extra mass carried in case parts come in heavier than estimated, as a fraction of
    # everything except the payload. Typical early in a design: 0.10 to 0.20. Shrink it as
    # real parts get weighed.
    mass_margin_fraction: float


@dataclass(frozen=True)
class DesignChoices:
    """What you decide about the drone's layout. The outer loop will sweep these."""

    rotor_count: int
    propeller: Propeller

    # Battery cells in series. Sets the voltage, and so the Kv to shop for.
    cell_count: int

    # Flight controller, receiver module, UWB module, LED module: roughly fixed
    avionics_mass_kg: float


@dataclass(frozen=True)
class Technology:
    """What current hardware can do. Calibrate these against real parts."""

    motor_efficiency: float
    esc_efficiency: float

    battery_specific_energy_j_per_kg: float

    # Fraction of stored energy you allow yourself to use (leaves a landing reserve).
    usable_battery_fraction: float

    # Cell voltage under load late in a flight (about 3.5 V for LiPo).
    loaded_cell_voltage_v: float

    # Max torque a motor delivers per kg of motor. To calibrate from a motor's test table:
    # max torque = max power / max speed (in rad/s), then divide by the motor's mass.
    motor_torque_density_nm_per_kg: float

    # Max power an ESC passes to its motor per kg of ESC, including its rating margin.
    esc_specific_power_w_per_kg: float

    # Frame, arms, landing gear and mounts as a fraction of the drone's total mass.
    frame_mass_fraction: float
