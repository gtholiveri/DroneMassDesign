"""Drones made of typical parts: the general answer, before any real listing is involved.

A typical prop of any diameter gets the C_T and C_P the fitted rule predicts, a typical frame gets
a mass from its size, and the closure loop sizes a typical motor and battery around them. Trying
every prop size and keeping the lightest drone answers the question the tool exists for: given
what the drone must carry and do, does a drone exist, and what does it look like?
"""

import math
from dataclasses import dataclass, replace

from drone_sizing.airframe import Airframe, Component, Frame
from drone_sizing.closure import MassDidNotConverge, close_mass
from drone_sizing.constants import METERS_PER_MILLIMETER
from drone_sizing.design import DroneDesign
from drone_sizing.inputs import DesignChoices, ScalingLaws, Scenario, Technology
from drone_sizing.motor import MotorScaling
from drone_sizing.propeller import Propeller, PropellerScaling

DronesByPropSize = dict[int, DroneDesign | None]  # prop diameter in mm -> the drone it makes, if any


@dataclass(frozen=True)
class TypicalPropeller:
    """How a typical prop of any diameter is made."""

    scaling: PropellerScaling  # the rule fitted to static tests
    blade_count: int
    pitch_ratio: float  # pitch / diameter

    # The rule is read near each prop's top speed. At hover speed the same prop makes less
    # thrust for the same power, so its C_T is scaled down by this.
    hover_thrust_factor: float

    # Prop mass scales as diameter squared from one prop whose mass is known.
    reference_diameter_m: float
    reference_mass_kg: float

    def propeller(self, diameter_m: float) -> Propeller:
        propeller = self.scaling.propeller(
            name=f"{diameter_m / METERS_PER_MILLIMETER:.0f} mm",
            diameter_m=diameter_m,
            pitch_m=self.pitch_ratio * diameter_m,
            blade_count=self.blade_count,
            mass_kg=self.reference_mass_kg * (diameter_m / self.reference_diameter_m) ** 2,
        )
        return replace(propeller, thrust_coefficient=propeller.thrust_coefficient * self.hover_thrust_factor)


@dataclass(frozen=True)
class TypicalFrame:
    """How a printed frame of any size is made: its mass grows with its motor-to-motor diagonal."""

    rotor_count: int
    mass_kg_per_m: float  # of diagonal, as if the arms dominate
    prop_clearance_m: float  # between neighboring prop tips, guards included

    def of_diagonal(self, diagonal_m: float) -> Frame:
        return Frame(
            name=f"{diagonal_m / METERS_PER_MILLIMETER:.0f} mm frame",
            diagonal_m=diagonal_m,
            mass_kg=self.mass_kg_per_m * diagonal_m,
            prop_clearance_m=self.prop_clearance_m,
        )

    def smallest_for(self, propeller: Propeller) -> Frame:
        """Neighboring motors are d * sin(pi / N) apart, and that must cover the prop plus the clearance."""
        return self.of_diagonal((propeller.diameter_m + self.prop_clearance_m) / math.sin(math.pi / self.rotor_count))

    def airframe(self, propeller: Propeller, components: tuple[Component, ...]) -> Airframe:
        """These fixed parts on the smallest frame the prop fits."""
        return Airframe(rotor_count=self.rotor_count, frame=self.smallest_for(propeller), components=components)


@dataclass(frozen=True)
class TypicalParts:
    """Everything the continuous model builds a drone out of."""

    propeller: TypicalPropeller
    frame: TypicalFrame
    motor: MotorScaling


@dataclass(frozen=True)
class PropSizes:
    """Which prop diameters to try: every coarse step across the range, then fine steps around the best."""

    smallest_mm: int
    largest_mm: int
    coarse_step_mm: int
    fine_step_mm: int

    @property
    def coarse_mm(self) -> range:
        return range(self.smallest_mm, self.largest_mm + 1, self.coarse_step_mm)

    def fine_mm_around(self, diameter_mm: int) -> list[int]:
        candidates = range(diameter_mm - self.coarse_step_mm + self.fine_step_mm, diameter_mm + self.coarse_step_mm, self.fine_step_mm)
        return [mm for mm in candidates if self.smallest_mm <= mm <= self.largest_mm]


def size_drone(scenario: Scenario, diameter_m: float, typical: TypicalParts, tech: Technology) -> DroneDesign | None:
    """The consistent drone on a typical prop of this size, or None if no such drone exists.

    None means every gram of battery added costs more hover power than the energy it brings.
    """
    propeller = typical.propeller.propeller(diameter_m)
    choices = DesignChoices(airframe=typical.frame.airframe(propeller, scenario.components), propeller=propeller)
    scaling = ScalingLaws(battery=scenario.battery, motor=typical.motor)
    try:
        return close_mass(scenario.requirements, choices, tech, scaling)
    except MassDidNotConverge:
        return None


def drones_by_prop_size(
    scenario: Scenario, typical: TypicalParts, tech: Technology, sizes: PropSizes
) -> DronesByPropSize:
    """The drone each prop size makes: coarse steps across the range, then fine ones around the lightest."""

    def size(diameter_mm: int) -> DroneDesign | None:
        return size_drone(scenario, diameter_mm * METERS_PER_MILLIMETER, typical, tech)

    drones = {diameter_mm: size(diameter_mm) for diameter_mm in sizes.coarse_mm}
    consistent = {diameter_mm: drone for diameter_mm, drone in drones.items() if drone is not None}
    if not consistent:
        return drones

    lightest_mm = min(consistent, key=lambda diameter_mm: consistent[diameter_mm].built_mass_kg)
    for diameter_mm in sizes.fine_mm_around(lightest_mm):
        if diameter_mm not in drones:
            drones[diameter_mm] = size(diameter_mm)
    return drones


def lightest(drones: DronesByPropSize) -> DroneDesign | None:
    """The best drone of those found: the lightest one that meets the scenario."""
    consistent = [drone for drone in drones.values() if drone is not None]
    return min(consistent, key=lambda drone: drone.built_mass_kg, default=None)


def best_drone(scenario: Scenario, typical: TypicalParts, tech: Technology, sizes: PropSizes) -> DroneDesign | None:
    """The lightest drone that meets the scenario, over the prop sizes, or None if there is none."""
    return lightest(drones_by_prop_size(scenario, typical, tech, sizes))
