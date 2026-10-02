"""The battery pack: how much energy it must store, and what that weighs."""

from dataclasses import dataclass

from drone_sizing.constants import JOULES_PER_WATT_HOUR
from drone_sizing.inputs import Technology


@dataclass(frozen=True)
class Battery:
    """The pack, sized to supply the mission's average power for the whole flight."""

    energy_j: float
    mass_kg: float

    @property
    def energy_wh(self) -> float:
        return self.energy_j / JOULES_PER_WATT_HOUR

    @classmethod
    def size_for(cls, average_power_w: float, flight_time_s: float, tech: Technology) -> "Battery":
        # Only part of the stored energy is usable, so store extra: E = P * t / f_usable
        energy_j = average_power_w * flight_time_s / tech.usable_battery_fraction

        # Each kilogram of battery holds a fixed amount of energy: m = E / e
        mass_kg = energy_j / tech.battery_specific_energy_j_per_kg

        return cls(energy_j=energy_j, mass_kg=mass_kg)
