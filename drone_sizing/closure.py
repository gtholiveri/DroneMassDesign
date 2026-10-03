"""The mass-closure loop: find the mass at which the drone you design is the drone you build."""

import math
from dataclasses import dataclass

from drone_sizing.design import DroneDesign
from drone_sizing.inputs import DesignChoices, Requirements, ScalingLaws, Technology

TOLERANCE_KG = 1e-5  # 0.01 g
MAX_ITERATIONS = 500


class MassDidNotConverge(Exception):
    """No consistent design exists for these inputs: the built mass keeps outrunning the guess."""

    def __init__(self, message: str, mass_history_kg: list[float]):
        super().__init__(message)
        self.mass_history_kg = mass_history_kg


@dataclass(frozen=True)
class ClosureResult:
    design: DroneDesign

    # Every design mass the loop tried, in order. The last one is the answer.
    mass_history_kg: list[float]


def close_mass(
    requirements: Requirements,
    choices: DesignChoices,
    tech: Technology,
    scaling: ScalingLaws,
) -> ClosureResult:
    if not choices.airframe.fits_prop(choices.propeller.diameter_m):
        raise ValueError(
            f"{choices.propeller.name} ({choices.propeller.diameter_m:.4f} m) doesn't fit the frame "
            f"(at most {choices.airframe.max_prop_diameter_m:.4f} m)"
        )

    # The drone can't weigh less than its payload and fixed parts, so start there. Starting below
    # the answer means every guess climbs toward the lightest consistent design and never
    # overshoots it. (Above a second, heavier crossing, guesses would run away instead.)
    design_mass_kg = requirements.payload_mass_kg + choices.airframe.fixed_mass_kg
    mass_history_kg = [design_mass_kg]
    previous_mismatch_kg = math.inf

    for _ in range(MAX_ITERATIONS):
        design = DroneDesign.size_for(design_mass_kg, requirements, choices, tech, scaling)
        mismatch_kg = design.mass_mismatch_kg

        # Built = designed for: consistent design found.
        if abs(mismatch_kg) < TOLERANCE_KG:
            return ClosureResult(design=design, mass_history_kg=mass_history_kg)

        # Climbing from below, the mismatch is always positive and should shrink every step.
        # If it grows instead, it never turns around (battery mass grows faster than linearly
        # with total mass), so no consistent design exists.
        if mismatch_kg > previous_mismatch_kg:
            raise MassDidNotConverge(
                f"mismatch grew from {previous_mismatch_kg:.5f} kg to {mismatch_kg:.5f} kg, "
                "so no drone with these inputs can fly for the required time",
                mass_history_kg,
            )

        # Redesign for what we actually built.
        previous_mismatch_kg = mismatch_kg
        design_mass_kg = design.built_mass_kg
        mass_history_kg.append(design_mass_kg)

    raise MassDidNotConverge(f"no consistent mass after {MAX_ITERATIONS} iterations", mass_history_kg)
