"""What's fixed before propulsion is chosen: the frame, and the components it carries (the controller
board, and modules like UWB and LEDs)."""

import math
from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class Component:
    """A part with a fixed mass and power draw: a controller board, a radio, an LED module, a sensor."""

    name: str
    mass_kg: float
    full_power_w: float = 0.0  # what it draws from the battery when fully on
    duty_cycle: float = 1.0  # the fraction of that it draws on average over a flight

    @property
    def average_power_w(self) -> float:
        return self.full_power_w * self.duty_cycle


@dataclass(frozen=True, kw_only=True)
class ControllerBoard(Component):
    """The flight controller with the ESCs built in: a component that also drives the motors.

    Its full power is what the processor, radio and sensors draw. What the ESCs pass to the
    motors is separate.
    """

    esc_max_current_a: float  # per motor
    esc_efficiency: float  # power out to the motor / power in from the battery
    supported_cell_counts: tuple[int, ...]


def controller_board(components: Sequence[Component]) -> ControllerBoard:
    """The controller board among these components. There must be exactly one."""
    boards = [component for component in components if isinstance(component, ControllerBoard)]
    if len(boards) != 1:
        raise ValueError(f"a drone needs exactly one controller board among its components, not {len(boards)}")
    return boards[0]


@dataclass(frozen=True)
class Frame:
    """The printed frame. Its size is a design input, and its mass comes from the slicer."""

    name: str
    diagonal_m: float  # motor center to motor center, across the middle of the frame
    mass_kg: float
    prop_clearance_m: float  # smallest allowed gap between neighboring prop tips, guards included


@dataclass(frozen=True)
class Airframe:
    """The frame and the components on it, plus how many rotors the frame carries."""

    rotor_count: int
    frame: Frame
    components: tuple[Component, ...]  # one of them is the controller board

    @property
    def board(self) -> ControllerBoard:
        return controller_board(self.components)

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
        breakdown_kg = {"frame": self.frame.mass_kg}
        for component in self.components:
            breakdown_kg[component.name] = component.mass_kg
        return breakdown_kg

    @property
    def fixed_mass_kg(self) -> float:
        return sum(self.fixed_mass_breakdown_kg.values())

    @property
    def electronics_power_w(self) -> float:
        """Average battery power that doesn't go to the motors: what every component draws."""
        return sum(component.average_power_w for component in self.components)
