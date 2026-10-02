"""One rotor: the thrust its prop must make, and the speed and torque that takes."""

from dataclasses import dataclass

from drone_sizing.constants import GRAVITY_M_PER_S2
from drone_sizing.inputs import DesignChoices, Requirements
from drone_sizing.propeller import Propeller


@dataclass(frozen=True)
class Rotor:
    """One propeller's operating points (hover and full throttle) on a drone of a given mass."""

    propeller: Propeller
    hover_thrust_n: float
    max_thrust_n: float
    hover_speed_rad_s: float
    max_speed_rad_s: float
    hover_torque_nm: float
    max_torque_nm: float

    @classmethod
    def size_for(
        cls,
        design_mass_kg: float,
        requirements: Requirements,
        choices: DesignChoices,
    ) -> "Rotor":
        propeller = choices.propeller

        # In hover the rotors share the drone's weight equally.
        hover_thrust_n = design_mass_kg * GRAVITY_M_PER_S2 / choices.rotor_count
        max_thrust_n = requirements.thrust_to_weight * hover_thrust_n

        # The prop's coefficients set how fast it must spin for each thrust,
        # and how much torque the motor must supply at that speed.
        hover_speed_rad_s = propeller.speed_for_thrust_rad_s(hover_thrust_n)
        max_speed_rad_s = propeller.speed_for_thrust_rad_s(max_thrust_n)

        return cls(
            propeller=propeller,
            hover_thrust_n=hover_thrust_n,
            max_thrust_n=max_thrust_n,
            hover_speed_rad_s=hover_speed_rad_s,
            max_speed_rad_s=max_speed_rad_s,
            hover_torque_nm=propeller.torque_at_speed_nm(hover_speed_rad_s),
            max_torque_nm=propeller.torque_at_speed_nm(max_speed_rad_s),
        )

    @property
    def hover_shaft_power_w(self) -> float:
        return self.hover_torque_nm * self.hover_speed_rad_s

    @property
    def max_shaft_power_w(self) -> float:
        return self.max_torque_nm * self.max_speed_rad_s

    @property
    def mass_kg(self) -> float:
        return self.propeller.mass_kg
