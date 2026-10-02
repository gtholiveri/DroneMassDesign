"""The frame: arms, plates, landing gear and mounts."""

from dataclasses import dataclass

from drone_sizing.inputs import Technology


@dataclass(frozen=True)
class Frame:
    """The frame, modeled as a fixed fraction of the drone's mass.

    A heavier drone needs a stronger frame. This first model doesn't yet depend on prop size,
    so it won't penalize bigger props; that matters once we sweep prop diameter.
    """

    mass_kg: float

    @classmethod
    def size_for(cls, design_mass_kg: float, tech: Technology) -> "Frame":
        return cls(mass_kg=tech.frame_mass_fraction * design_mass_kg)
