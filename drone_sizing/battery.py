"""Batteries: an ideal pack for the continuous model, and real packs from a catalog."""

from dataclasses import dataclass

from drone_sizing.constants import JOULES_PER_WATT_HOUR


@dataclass(frozen=True)
class Battery:
    """An ideal pack for the continuous model, sized to supply the mission's average power for the whole flight."""

    energy_j: float
    mass_kg: float

    @property
    def energy_wh(self) -> float:
        return self.energy_j / JOULES_PER_WATT_HOUR

    @classmethod
    def size_for(
        cls,
        average_power_w: float,
        flight_time_s: float,
        usable_fraction: float,
        specific_energy_j_per_kg: float,
    ) -> "Battery":
        # Only part of the stored energy is usable, so store extra: E = P * t / f_usable
        energy_j = average_power_w * flight_time_s / usable_fraction

        # Each kilogram of battery holds a fixed amount of energy: m = E / e
        mass_kg = energy_j / specific_energy_j_per_kg

        return cls(energy_j=energy_j, mass_kg=mass_kg)


@dataclass(frozen=True)
class BatteryPack:
    """A real pack from a catalog."""

    name: str
    cell_count: int
    capacity_ah: float
    mass_kg: float  # including leads and connector

    # Current limits, as multiples of capacity: max current = C * capacity.
    continuous_discharge_c: float  # sustained, for the average draw
    burst_discharge_c: float  # for a few seconds, for full-throttle bursts

    price_usd: float | None = None

    # This pack's own cell voltages, if they differ from the usual ones: 3.8 V nominal and 4.35 V
    # fully charged for LiHV.
    nominal_cell_voltage_v: float | None = None
    full_cell_voltage_v: float | None = None

    # Measured internal resistance of the whole pack, leads included, if known.
    internal_resistance_ohm: float | None = None

    @property
    def max_continuous_current_a(self) -> float:
        return self.continuous_discharge_c * self.capacity_ah

    @property
    def max_burst_current_a(self) -> float:
        return self.burst_discharge_c * self.capacity_ah

    def resistance_ohm(self, cell_resistance_ohm_ah: float, lead_resistance_ohm: float) -> float:
        """The pack's internal resistance: its listed value, or an estimate from its capacity.

        The estimate: each cell's resistance is cell_resistance_ohm_ah / capacity, cells in series
        add up, and the leads and connector add a fixed amount.
        """
        if self.internal_resistance_ohm is not None:
            return self.internal_resistance_ohm
        return self.cell_count * cell_resistance_ohm_ah / self.capacity_ah + lead_resistance_ohm

    def fresh_voltage_v(self, default_fresh_cell_voltage_v: float) -> float:
        """Resting voltage of the fully charged pack."""
        return self.cell_count * (self.full_cell_voltage_v or default_fresh_cell_voltage_v)

    def nominal_voltage_v(self, default_nominal_cell_voltage_v: float) -> float:
        """The pack's nominal voltage: cells x nominal cell voltage (its own if listed)."""
        return self.cell_count * (self.nominal_cell_voltage_v or default_nominal_cell_voltage_v)

    def energy_j(self, default_nominal_cell_voltage_v: float) -> float:
        """Stored energy: nominal voltage x capacity, in Wh, converted to J."""
        energy_wh = self.nominal_voltage_v(default_nominal_cell_voltage_v) * self.capacity_ah
        return energy_wh * JOULES_PER_WATT_HOUR
