"""The inputs to a sizing run, kept separate because they change for different reasons."""

from dataclasses import dataclass

from drone_sizing.airframe import Airframe
from drone_sizing.motor import MotorScaling
from drone_sizing.propeller import Propeller


@dataclass(frozen=True)
class Requirements:
    """What the mission demands. Change these when the job changes."""

    # Mass carried beyond what the drone needs to fly, like a camera or a package. It's specified
    # rather than estimated, so the margin doesn't cover it. Zero for a drone that just flies.
    payload_mass_kg: float
    flight_time_s: float

    # Max total thrust divided by weight, checked on a tired pack late in the flight.
    # About 2 is the usual minimum for control and wind.
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

    def with_payload_and_margin_kg(self, parts_kg: dict[str, float]) -> dict[str, float]:
        """A mass breakdown with the payload and the margin added to the parts."""
        breakdown_kg = {"payload": self.payload_mass_kg, **parts_kg}
        breakdown_kg["margin"] = self.mass_margin_fraction * sum(parts_kg.values())
        return breakdown_kg


@dataclass(frozen=True)
class DesignChoices:
    """What the continuous model takes as given before sizing the motor and battery."""

    airframe: Airframe
    propeller: Propeller

    # Battery cells in series. Sets the voltage, and so the Kv to shop for.
    cell_count: int


@dataclass(frozen=True)
class Technology:
    """Assumptions about how hardware behaves. Both the continuous model and the catalog search use these."""

    # Fraction of stored energy you allow yourself to use (leaves a landing reserve).
    usable_battery_fraction: float

    # Cell voltage used to turn capacity (Ah) into energy (Wh): 3.7 V for LiPo. A pack that lists
    # its own (3.8 V for LiHV) uses that instead.
    nominal_cell_voltage_v: float

    # Resting (no-load) cell voltage at the two ends of a flight. Under load a real pack sits below
    # these by its current times its internal resistance: that drop is the sag.
    fresh_cell_voltage_v: float  # fully charged: 4.2 V for LiPo. A pack that lists its own (4.35 V LiHV) uses that.
    tired_cell_voltage_v: float  # at the landing reserve, about 20% left: roughly 3.7 V

    # The lowest cell voltage allowed under load. Below about 3.0 V the electronics brown out and
    # the cell is damaged, so full throttle on a tired pack must stay above it.
    min_cell_voltage_v: float

    # A pack's internal resistance, for packs that don't list one. A cell's resistance falls in
    # proportion to its capacity (ohms x amp-hours is roughly constant for one cell type), and the
    # leads and connector add a fixed amount.
    cell_resistance_ohm_ah: float
    lead_resistance_ohm: float

    # The continuous model sizes the motor before any pack exists, so it can't compute sag. It
    # uses these loaded cell voltages by rule of thumb instead: about 3.5 V at full throttle late
    # in a flight (least thrust) and about 4.0 V on a fresh pack (most current).
    loaded_cell_voltage_v: float
    fresh_loaded_cell_voltage_v: float

    # How a motor's no-load drag grows with speed (see Motor). 0 means constant drag torque,
    # 1 means drag torque proportional to speed. validate.py compares choices against a real drone.
    no_load_current_speed_exponent: float


@dataclass(frozen=True)
class ScalingLaws:
    """How component mass grows with what the component must do.

    Only the continuous model needs these. The catalog search uses real parts instead.
    """

    battery_specific_energy_j_per_kg: float
    motor: MotorScaling
