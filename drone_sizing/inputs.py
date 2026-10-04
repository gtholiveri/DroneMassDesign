"""The inputs to a sizing run, kept separate because they change for different reasons."""

from dataclasses import dataclass

from drone_sizing.airframe import Airframe, Component, ControllerBoard, controller_board
from drone_sizing.battery import BatteryKind
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
class Technology:
    """Assumptions about hardware that aren't tied to one part. The cells' own numbers live with the battery."""

    # Fraction of stored energy you allow yourself to use (leaves a landing reserve).
    usable_battery_fraction: float

    # What a pack's leads and connector add to its resistance.
    lead_resistance_ohm: float

    # How a motor's no-load drag grows with speed (see Motor). 0 means constant drag torque,
    # 1 means drag torque proportional to speed. validate.py compares choices against a real drone.
    no_load_current_speed_exponent: float


@dataclass(frozen=True)
class DesignChoices:
    """What the continuous model takes as given before sizing the motor and battery."""

    airframe: Airframe
    propeller: Propeller


@dataclass(frozen=True)
class ScalingLaws:
    """How the parts the continuous model sizes are made: a battery of any energy, a motor of any mass."""

    battery: BatteryKind
    motor: MotorScaling


@dataclass(frozen=True)
class Scenario:
    """One sizing problem: what the drone must do, what it carries, and what kind of battery it runs on."""

    name: str
    requirements: Requirements
    battery: BatteryKind
    components: tuple[Component, ...]  # the fixed parts: the controller board, radios, LEDs, sensors

    @property
    def board(self) -> ControllerBoard:
        return controller_board(self.components)

    @property
    def components_mass_kg(self) -> float:
        return sum(part.mass_kg for part in self.components)

    @property
    def electronics_power_w(self) -> float:
        """What the fixed parts draw together, on average over a flight."""
        return sum(part.average_power_w for part in self.components)
