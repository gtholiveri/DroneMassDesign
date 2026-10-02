"""One electronic speed controller (ESC): how heavy it must be, and how much power it draws."""

from dataclasses import dataclass

from drone_sizing.inputs import Technology


@dataclass(frozen=True)
class ESC:
    """One ESC, sized by the most power it must pass to its motor (full throttle)."""

    max_output_power_w: float
    efficiency: float
    mass_kg: float

    @classmethod
    def size_for(cls, max_output_power_w: float, tech: Technology) -> "ESC":
        mass_kg = max_output_power_w / tech.esc_specific_power_w_per_kg
        return cls(
            max_output_power_w=max_output_power_w,
            efficiency=tech.esc_efficiency,
            mass_kg=mass_kg,
        )

    def input_power_w(self, output_power_w: float) -> float:
        """Battery power the ESC draws to deliver this much power to its motor."""
        return output_power_w / self.efficiency
