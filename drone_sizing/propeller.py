"""A specific propeller: its size, its mass, and how its thrust and torque depend on speed."""

import math
from dataclasses import dataclass

from drone_sizing.constants import AIR_DENSITY_KG_PER_M3


@dataclass(frozen=True)
class Propeller:
    """One propeller model, described by its static (hover) coefficients.

    With n in revolutions per second:
        T = C_T * rho * n^2 * D^4
        P = C_P * rho * n^3 * D^5
    """

    name: str
    diameter_m: float
    mass_kg: float
    thrust_coefficient: float  # C_T
    power_coefficient: float  # C_P

    # Hole for the motor shaft. Checked against the motor's shaft when both are known.
    bore_diameter_m: float | None = None
    price_usd: float | None = None  # per prop

    # True if C_T and C_P came from a test on a motor whose numbers were partly estimated.
    estimated: bool = False

    @classmethod
    def from_test_point(
        cls,
        name: str,
        diameter_m: float,
        mass_kg: float,
        thrust_n: float,
        speed_rad_s: float,
        shaft_power_w: float,
        bore_diameter_m: float | None = None,
        price_usd: float | None = None,
    ) -> "Propeller":
        """Back out C_T and C_P from one measured point: thrust, speed and shaft power.

        Solves T = C_T rho n^2 D^4 and P = C_P rho n^3 D^5 for the coefficients. The coefficients
        drift with speed (small props get less efficient when slower), so a point near the speeds
        the prop will actually run at is best. If a vendor table gives volts and amps but not
        speed or shaft power, get them from Motor.measured_point.
        """
        speed_rev_s = speed_rad_s / (2 * math.pi)
        thrust_coefficient = thrust_n / (AIR_DENSITY_KG_PER_M3 * speed_rev_s**2 * diameter_m**4)
        power_coefficient = shaft_power_w / (AIR_DENSITY_KG_PER_M3 * speed_rev_s**3 * diameter_m**5)
        return cls(
            name=name,
            diameter_m=diameter_m,
            mass_kg=mass_kg,
            thrust_coefficient=thrust_coefficient,
            power_coefficient=power_coefficient,
            bore_diameter_m=bore_diameter_m,
            price_usd=price_usd,
        )

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

    def thrust_at_speed_n(self, speed_rad_s: float) -> float:
        """Thrust at this speed: T = C_T * rho * n^2 * D^4."""
        speed_rev_s = speed_rad_s / (2 * math.pi)
        return self.thrust_coefficient * AIR_DENSITY_KG_PER_M3 * speed_rev_s**2 * self.diameter_m**4

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


@dataclass(frozen=True)
class PropellerLaw:
    """coefficient * (pitch / diameter)^pitch_ratio_exponent * diameter^diameter_exponent, diameter in m.

    The diameter term is there because small props are less efficient than big ones of the same
    shape: their blades run at lower Reynolds numbers.
    """

    coefficient: float
    pitch_ratio_exponent: float
    diameter_exponent: float

    def __call__(self, pitch_ratio: float, diameter_m: float) -> float:
        return self.coefficient * pitch_ratio**self.pitch_ratio_exponent * diameter_m**self.diameter_exponent


@dataclass(frozen=True)
class PropellerScaling:
    """C_T and C_P of a typical prop from its pitch and diameter, fitted to thrust-stand tests.

    For props whose listing gives only size, pitch and blade count. Two blades is the baseline;
    other blade counts multiply both coefficients by factors fitted the same way.
    """

    thrust_coefficient: PropellerLaw  # two-bladed C_T
    power_coefficient: PropellerLaw  # two-bladed C_P
    blade_factors: tuple[tuple[int, float, float], ...]  # (blades, C_T factor, C_P factor)

    def propeller(
        self,
        name: str,
        diameter_m: float,
        pitch_m: float,
        blade_count: int,
        mass_kg: float,
        bore_diameter_m: float | None = None,
        price_usd: float | None = None,
    ) -> Propeller:
        """A prop with the coefficients this scaling predicts. It's marked as estimated."""
        if blade_count == 2:
            thrust_factor, power_factor = 1.0, 1.0
        else:
            factors = {blades: (thrust, power) for blades, thrust, power in self.blade_factors}
            if blade_count not in factors:
                raise ValueError(f"{name}: no fitted data for {blade_count}-bladed props")
            thrust_factor, power_factor = factors[blade_count]

        pitch_ratio = pitch_m / diameter_m
        return Propeller(
            name=name,
            diameter_m=diameter_m,
            mass_kg=mass_kg,
            thrust_coefficient=thrust_factor * self.thrust_coefficient(pitch_ratio, diameter_m),
            power_coefficient=power_factor * self.power_coefficient(pitch_ratio, diameter_m),
            bore_diameter_m=bore_diameter_m,
            price_usd=price_usd,
            estimated=True,
        )
