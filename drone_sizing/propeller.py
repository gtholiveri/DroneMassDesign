"""A specific propeller: its size, its mass, and how its thrust and torque depend on speed."""

import math
from dataclasses import dataclass

from drone_sizing.constants import AIR_DENSITY_KG_PER_M3


@dataclass(frozen=True)
class Propeller:
    """One propeller model, described by its static (hover) coefficients from test data.

    With n in revolutions per second:
        T = C_T * rho * n^2 * D^4
        P = C_P * rho * n^3 * D^5
    """

    name: str
    diameter_m: float
    mass_kg: float
    thrust_coefficient: float  # C_T
    power_coefficient: float  # C_P

    @property
    def disk_area_m2(self) -> float:
        return math.pi * self.diameter_m**2 / 4

    @property
    def figure_of_merit(self) -> float:
        """Ideal power / actual power in hover, implied by the coefficients.

        FM = C_T^1.5 / (sqrt(pi / 2) * C_P)
        """
        return self.thrust_coefficient**1.5 / (math.sqrt(math.pi / 2) * self.power_coefficient)

    def speed_for_thrust_rad_s(self, thrust_n: float) -> float:
        """How fast the prop must spin to make this thrust. Solves T = C_T * rho * n^2 * D^4 for n."""
        speed_rev_s = math.sqrt(
            thrust_n / (self.thrust_coefficient * AIR_DENSITY_KG_PER_M3 * self.diameter_m**4)
        )
        return 2 * math.pi * speed_rev_s

    def torque_at_speed_nm(self, speed_rad_s: float) -> float:
        """Torque the motor must supply to spin the prop at this speed.

        Q = P / omega = C_P / (2 * pi) * rho * n^2 * D^5
        """
        speed_rev_s = speed_rad_s / (2 * math.pi)
        return (
            self.power_coefficient
            / (2 * math.pi)
            * AIR_DENSITY_KG_PER_M3
            * speed_rev_s**2
            * self.diameter_m**5
        )
