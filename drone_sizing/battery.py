"""Batteries: what a kind of cell does, a pack made of them, and the kind of pack the continuous model
sizes from. Every voltage and resistance a battery has lives here, so a pack from the catalog and a pack
the sizing loop makes are judged the same way."""

import math
from dataclasses import dataclass

from drone_sizing.constants import JOULES_PER_WATT_HOUR


@dataclass(frozen=True)
class CellChemistry:
    """How one kind of cell behaves, whoever makes it."""

    name: str
    nominal_cell_voltage_v: float  # turns capacity (Ah) into energy (Wh)
    full_cell_voltage_v: float  # at rest, fully charged
    tired_cell_voltage_v: float  # at rest, at the landing reserve: about 20% left
    # The lowest allowed under load. Below it the electronics brown out and the cell is damaged.
    min_cell_voltage_v: float
    # One cell's resistance times its capacity. A cell's resistance falls in proportion to its
    # capacity, so this is roughly constant for one kind of cell. It is a rule of thumb: hobby
    # packs don't publish their resistance. TODO: measure a real pack.
    cell_resistance_ohm_ah: float


LIPO = CellChemistry("LiPo", 3.7, 4.2, 3.7, 3.0, 0.025)
LIHV = CellChemistry("LiHV", 3.8, 4.35, 3.7, 3.0, 0.025)
# Li-ion runs lower throughout, so the same motor needs a higher Kv on it. The resistance is the
# Molicel P28A's (20 milliohm at 2.8 Ah); the tired and minimum voltages are assumed, not from a datasheet.
LI_ION = CellChemistry("Li-ion", 3.6, 4.2, 3.4, 2.8, 0.020 * 2.8)

CHEMISTRIES = {"lipo": LIPO, "lihv": LIHV, "liion": LI_ION}


@dataclass(frozen=True)
class BatteryPack:
    """A pack: real from a catalog, or sized by the continuous model."""

    name: str
    cell_count: int  # in series
    capacity_ah: float
    mass_kg: float  # including leads and connector
    chemistry: CellChemistry

    # Current limits, as multiples of capacity: max current = C * capacity. A sized pack has none.
    continuous_discharge_c: float  # sustained, for the average draw
    burst_discharge_c: float  # for a few seconds, for full-throttle bursts

    price_usd: float | None = None
    internal_resistance_ohm: float | None = None  # measured, leads included, if known

    @property
    def nominal_voltage_v(self) -> float:
        return self.cell_count * self.chemistry.nominal_cell_voltage_v

    @property
    def fresh_voltage_v(self) -> float:
        """At rest, fully charged."""
        return self.cell_count * self.chemistry.full_cell_voltage_v

    @property
    def tired_voltage_v(self) -> float:
        """At rest, late in the flight."""
        return self.cell_count * self.chemistry.tired_cell_voltage_v

    @property
    def min_voltage_v(self) -> float:
        """The lowest allowed under load."""
        return self.cell_count * self.chemistry.min_cell_voltage_v

    @property
    def energy_wh(self) -> float:
        return self.nominal_voltage_v * self.capacity_ah

    @property
    def energy_j(self) -> float:
        return self.energy_wh * JOULES_PER_WATT_HOUR

    @property
    def max_continuous_current_a(self) -> float:
        return self.continuous_discharge_c * self.capacity_ah

    @property
    def max_burst_current_a(self) -> float:
        return self.burst_discharge_c * self.capacity_ah

    def resistance_ohm(self, lead_resistance_ohm: float) -> float:
        """The pack's internal resistance: its measured value, or the cells' from their chemistry
        and capacity (cells in series add up) plus the leads and connector."""
        if self.internal_resistance_ohm is not None:
            return self.internal_resistance_ohm
        return self.cell_count * self.chemistry.cell_resistance_ohm_ah / self.capacity_ah + lead_resistance_ohm

    def voltage_under_load_v(
        self, resting_voltage_v: float, lead_resistance_ohm: float, motor_power_w: float, other_current_a: float
    ) -> float:
        """What the pack delivers while the motors draw this much power in total and the rest a fixed current.

        The pack sags by its current times its resistance, and the motors' current is their power
        over the sagged voltage: V = V_rest - R (P / V + I_other). That is a quadratic in V, and the
        higher root is where the pack settles. A pack that can't supply the power at any voltage is
        left at the best it can do, half of what it would otherwise show.
        """
        resistance_ohm = self.resistance_ohm(lead_resistance_ohm)
        unloaded_v = resting_voltage_v - resistance_ohm * other_current_a
        discriminant = unloaded_v**2 - 4 * resistance_ohm * motor_power_w
        return (unloaded_v + math.sqrt(max(discriminant, 0.0))) / 2


@dataclass(frozen=True)
class BatteryKind:
    """A kind of pack the continuous model can size: what it weighs per unit of energy, and its cells."""

    name: str
    cell_count: int  # in series
    specific_energy_wh_per_kg: float  # connector or leads included
    chemistry: CellChemistry

    @property
    def tired_voltage_v(self) -> float:
        return self.cell_count * self.chemistry.tired_cell_voltage_v

    def pack_holding(self, energy_j: float) -> BatteryPack:
        """A pack of this kind with exactly this much energy, and no current limits."""
        energy_wh = energy_j / JOULES_PER_WATT_HOUR
        return BatteryPack(
            name=self.name,
            cell_count=self.cell_count,
            capacity_ah=energy_wh / (self.cell_count * self.chemistry.nominal_cell_voltage_v),
            mass_kg=energy_wh / self.specific_energy_wh_per_kg,
            chemistry=self.chemistry,
            continuous_discharge_c=math.inf,
            burst_discharge_c=math.inf,
        )
