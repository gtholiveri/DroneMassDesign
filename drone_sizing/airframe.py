"""What's fixed before propulsion is chosen: the frame, the controller board, and modules like UWB and LEDs."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Component:
    """A part with a fixed mass and power draw: a radio, an LED deck, a sensor."""

    name: str
    mass_kg: float
    power_w: float = 0.0  # average electrical power it draws from the battery


@dataclass(frozen=True)
class ControllerBoard:
    """The flight controller with the ESCs built in. Its mass is fixed, so ESC mass no longer scales with power."""

    name: str
    mass_kg: float
    power_w: float  # MCU, radio and sensors
    esc_max_current_a: float  # per motor
    esc_efficiency: float  # power out to the motor / power in from the battery
    supported_cell_counts: tuple[int, ...]


@dataclass(frozen=True)
class Frame:
    """The printed frame. Its size is a design input, and its mass comes from the slicer."""

    name: str
    diagonal_m: float  # motor center to motor center, across the middle of the frame
    mass_kg: float
    prop_clearance_m: float  # smallest allowed gap between neighboring prop tips, guards included


@dataclass(frozen=True)
class Airframe:
    """The frame, the board and the fixed modules, plus how many rotors the frame carries."""

    rotor_count: int
    frame: Frame
    board: ControllerBoard
    components: tuple[Component, ...]

    @property
    def max_prop_diameter_m(self) -> float:
        """The largest prop that fits.

        The motors sit on a circle whose diameter is the frame diagonal d, so neighboring motors
        are d * sin(pi / N) apart (d / sqrt(2) for a quad). Two neighboring props fit if that
        spacing covers one prop diameter plus the clearance.
        """
        neighbor_spacing_m = self.frame.diagonal_m * math.sin(math.pi / self.rotor_count)
        return neighbor_spacing_m - self.frame.prop_clearance_m

    def fits_prop(self, diameter_m: float) -> bool:
        """Whether a prop of this diameter fits, allowing for rounding when it fits exactly."""
        return diameter_m <= self.max_prop_diameter_m * (1 + 1e-9)

    @property
    def fixed_mass_breakdown_kg(self) -> dict[str, float]:
        breakdown_kg = {"frame": self.frame.mass_kg, "board": self.board.mass_kg}
        for component in self.components:
            breakdown_kg[component.name] = component.mass_kg
        return breakdown_kg

    @property
    def fixed_mass_kg(self) -> float:
        return sum(self.fixed_mass_breakdown_kg.values())

    @property
    def electronics_power_w(self) -> float:
        """Battery power that doesn't go to the motors: the board plus every module."""
        return self.board.power_w + sum(component.power_w for component in self.components)
