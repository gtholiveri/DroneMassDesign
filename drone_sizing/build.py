"""A drone made of definite parts, and how it flies.

Whether the parts came from a catalog or were just sized, this is the one place their flight is
worked out: hover, flight time, full throttle on a tired and a fresh pack, and every check.
"""

import math
from collections.abc import Callable
from dataclasses import dataclass, replace

from drone_sizing.airframe import Airframe
from drone_sizing.battery import BatteryPack
from drone_sizing.constants import GRAVITY_M_PER_S2
from drone_sizing.inputs import Requirements, Technology
from drone_sizing.motor import Motor, OperatingPoint
from drone_sizing.numerics import minimize, solve_increasing
from drone_sizing.propeller import Propeller

# Lets a check pass when it sits exactly on its limit, despite rounding.
CHECK_TOLERANCE = 1e-9


@dataclass(frozen=True)
class Check:
    """One requirement: the value a build reaches, the limit it must respect, and which side is safe."""

    name: str
    value: float
    limit: float
    is_minimum: bool  # True: the value must be at least the limit. False: at most.
    unit: str  # SI unit of value and limit, for the report

    @property
    def margin(self) -> float:
        """How far inside the limit the value is, as a fraction of the limit. Negative means it fails."""
        if self.is_minimum:
            return self.value / self.limit - 1
        return 1 - self.value / self.limit

    @property
    def passed(self) -> bool:
        return self.margin >= -CHECK_TOLERANCE


@dataclass(frozen=True)
class PowerBudget:
    """Where the average battery power goes: hover power followed from the air back to the battery,
    then the allowance for maneuvering and each electronic part on top. All motors together."""

    lifting_w: float  # the least any rotor this size could use: the ideal power to hold the drone up
    propeller_loss_w: float  # blade drag and tip losses
    winding_loss_w: float  # heat in the motor windings
    motor_drag_loss_w: float  # friction and iron losses in the motors
    esc_loss_w: float  # switching losses
    maneuvering_w: float  # the allowance above hover
    electronics_w: tuple[tuple[str, float], ...]  # each fixed part by name

    @property
    def hover_w(self) -> float:
        """Every motor at hover, measured at the battery."""
        return self.lifting_w + self.propeller_loss_w + self.winding_loss_w + self.motor_drag_loss_w + self.esc_loss_w

    @property
    def total_w(self) -> float:
        return self.hover_w + self.maneuvering_w + sum(power_w for _, power_w in self.electronics_w)


@dataclass(frozen=True)
class Build:
    """One concrete drone: the airframe plus one motor, prop and battery."""

    airframe: Airframe
    motor: Motor  # one of `rotor_count`
    propeller: Propeller  # one of `rotor_count`
    battery: BatteryPack

    @property
    def part_names(self) -> tuple[str, str, str, str]:
        """Frame, motor, prop and battery. A * marks parts whose numbers were partly estimated."""
        motor = self.motor.name + ("*" if self.motor.estimated_fields else "")
        propeller = self.propeller.name + ("*" if self.propeller.estimated else "")
        return self.airframe.frame.name, motor, propeller, self.battery.name

    @property
    def name(self) -> str:
        return " | ".join(self.part_names)

    @property
    def uses_estimates(self) -> bool:
        return bool(self.motor.estimated_fields) or self.propeller.estimated

    @property
    def price_usd(self) -> float | None:
        """What the parts that differ between builds cost. The airframe costs the same for every build."""
        if None in (self.motor.price_usd, self.propeller.price_usd, self.battery.price_usd):
            return None
        rotor_count = self.airframe.rotor_count
        return rotor_count * (self.motor.price_usd + self.propeller.price_usd) + self.battery.price_usd

    def incompatibilities(self) -> list[str]:
        """Reasons these parts can't physically go together. Empty if they can."""
        problems = []

        if not self.airframe.fits_prop(self.propeller.diameter_m):
            problems.append("prop too big for the frame")

        shaft_m = self.motor.shaft_diameter_m
        bore_m = self.propeller.bore_diameter_m
        if shaft_m is not None and bore_m is not None and not math.isclose(shaft_m, bore_m, rel_tol=0.01):
            problems.append("prop bore doesn't match the motor shaft")

        if self.battery.cell_count not in self.airframe.board.supported_cell_counts:
            problems.append(f"board doesn't support {self.battery.cell_count}S")

        max_cells = self.motor.max_cell_count
        if max_cells is not None and self.battery.cell_count > max_cells:
            problems.append(f"motor isn't rated for {self.battery.cell_count}S")

        return problems

    def mass_breakdown_kg(self, requirements: Requirements) -> dict[str, float]:
        """What each group of parts weighs, with the payload and margin the requirements add."""
        rotor_count = self.airframe.rotor_count
        return requirements.with_payload_and_margin_kg(
            {
                **self.airframe.fixed_mass_breakdown_kg,
                "motors": rotor_count * self.motor.mass_kg,
                "props": rotor_count * self.propeller.mass_kg,
                "battery": self.battery.mass_kg,
            }
        )

    def full_throttle_battery_current_a(self, point: OperatingPoint, resting_voltage_v: float) -> float:
        """What the battery supplies with every motor at this full-throttle point.

        At full throttle the ESC passes the battery voltage straight through, so each motor's
        current comes from the battery, plus the ESC's loss. The electronics draw their power on
        top; their current is taken at the resting voltage, which makes it a few percent low.
        """
        motors_a = self.airframe.rotor_count * point.current_a / self.airframe.board.esc_efficiency
        return motors_a + self.airframe.electronics_power_w / resting_voltage_v

    def full_throttle_on_pack(self, resting_voltage_v: float, tech: Technology) -> OperatingPoint:
        """Where the motors settle at full throttle on this pack, sag included.

        Under load the pack delivers its resting voltage minus current times internal resistance.
        Spinning faster needs more voltage and draws more current, which sags the pack further. So
        the motors settle at the speed where the voltage they need, plus the sag their current
        causes, equals the pack's resting voltage. The returned point's voltage is the sagged one.
        """
        speed_exponent = tech.no_load_current_speed_exponent
        resistance_ohm = self.battery.resistance_ohm(tech.lead_resistance_ohm)

        def point_at(speed_rad_s: float) -> OperatingPoint:
            return self.motor.operating_point(
                self.propeller.torque_at_speed_nm(speed_rad_s), speed_rad_s, speed_exponent
            )

        def resting_voltage_needed_v(speed_rad_s: float) -> float:
            point = point_at(speed_rad_s)
            sag_v = self.full_throttle_battery_current_a(point, resting_voltage_v) * resistance_ohm
            return point.voltage_v + sag_v

        no_load_speed_rad_s = self.motor.speed_constant_rad_s_per_v * resting_voltage_v
        return point_at(solve_increasing(resting_voltage_needed_v, resting_voltage_v, 0.0, no_load_speed_rad_s))

    def evaluate(self, requirements: Requirements, tech: Technology) -> "BuildResult":
        rotor_count = self.airframe.rotor_count
        board = self.airframe.board
        speed_exponent = tech.no_load_current_speed_exponent

        mass_breakdown_kg = self.mass_breakdown_kg(requirements)
        weight_n = sum(mass_breakdown_kg.values()) * GRAVITY_M_PER_S2

        # Hover: the rotors share the weight. The prop sets the speed and torque that takes, and
        # the motor model gives the current and voltage it draws to supply them.
        hover_speed_rad_s = self.propeller.speed_for_thrust_rad_s(weight_n / rotor_count)
        hover = self.motor.operating_point(
            self.propeller.torque_at_speed_nm(hover_speed_rad_s), hover_speed_rad_s, speed_exponent
        )

        # Where the power goes. The motor's input power is what its prop takes from the shaft,
        # plus the heat in its windings, plus its friction and iron losses; the ESC adds its own.
        shaft_w = rotor_count * hover.shaft_power_w
        lifting_w = self.propeller.figure_of_merit * shaft_w
        winding_loss_w = rotor_count * hover.current_a**2 * self.motor.resistance_ohm
        motor_input_w = rotor_count * hover.input_power_w
        hover_w = motor_input_w / board.esc_efficiency
        power = PowerBudget(
            lifting_w=lifting_w,
            propeller_loss_w=shaft_w - lifting_w,
            winding_loss_w=winding_loss_w,
            motor_drag_loss_w=motor_input_w - shaft_w - winding_loss_w,
            esc_loss_w=hover_w - motor_input_w,
            maneuvering_w=(requirements.average_power_factor - 1) * hover_w,
            electronics_w=tuple((part.name, part.average_power_w) for part in self.airframe.components),
        )

        # Flight time: usable energy over average battery power.
        flight_time_s = tech.usable_battery_fraction * self.battery.energy_j / power.total_w

        # Least thrust: full throttle late in the flight, on a tired pack that sags under the load.
        tired_full_throttle = self.full_throttle_on_pack(self.battery.tired_voltage_v, tech)
        thrust_to_weight = (
            rotor_count * self.propeller.thrust_at_speed_n(tired_full_throttle.speed_rad_s) / weight_n
        )

        # Most current: full throttle on a fresh pack, which sags less and starts higher.
        fresh_full_throttle = self.full_throttle_on_pack(self.battery.fresh_voltage_v, tech)
        peak_battery_current_a = self.full_throttle_battery_current_a(fresh_full_throttle, self.battery.fresh_voltage_v)
        average_battery_current_a = power.total_w / self.battery.nominal_voltage_v

        checks = [
            Check("thrust-to-weight, tired pack", thrust_to_weight, requirements.thrust_to_weight, True, ""),
            Check("flight time", flight_time_s, requirements.flight_time_s, True, "s"),
            Check(
                "pack voltage at full throttle, tired pack",
                tired_full_throttle.voltage_v,
                self.battery.min_voltage_v,
                True,
                "V",
            ),
            Check("motor current vs motor rating", fresh_full_throttle.current_a, self.motor.max_current_a, False, "A"),
            Check("motor current vs ESC rating", fresh_full_throttle.current_a, board.esc_max_current_a, False, "A"),
            Check("peak battery current vs burst C", peak_battery_current_a, self.battery.max_burst_current_a, False, "A"),
            Check(
                "average battery current vs continuous C",
                average_battery_current_a,
                self.battery.max_continuous_current_a,
                False,
                "A",
            ),
        ]

        return BuildResult(
            build=self,
            requirements=requirements,
            tech=tech,
            mass_breakdown_kg=mass_breakdown_kg,
            hover=hover,
            hover_throttle=hover.voltage_v / tired_full_throttle.voltage_v,
            power=power,
            flight_time_s=flight_time_s,
            tired_full_throttle=tired_full_throttle,
            thrust_to_weight=thrust_to_weight,
            fresh_full_throttle=fresh_full_throttle,
            peak_battery_current_a=peak_battery_current_a,
            checks=checks,
        )

    def evaluate_without_margins(self, requirements: Requirements, tech: Technology) -> "BuildResult":
        """The best estimate of what the drone would actually do, with the planning margins removed.

        No mass margin, a steady hover with no allowance for maneuvering, and the whole pack used.
        Comparing this with `evaluate` shows how much safety the margins add up to.
        """
        return self.evaluate(
            replace(requirements, mass_margin_fraction=0.0, average_power_factor=1.0),
            replace(tech, usable_battery_fraction=1.0),
        )


@dataclass(frozen=True)
class BuildResult:
    """How one build flies under one set of requirements, and which of them it meets."""

    build: Build
    requirements: Requirements
    tech: Technology
    mass_breakdown_kg: dict[str, float]

    hover: OperatingPoint  # one motor
    hover_throttle: float  # motor voltage needed to hover / tired pack voltage
    power: PowerBudget
    flight_time_s: float

    tired_full_throttle: OperatingPoint  # one motor, full throttle late in the flight
    thrust_to_weight: float
    fresh_full_throttle: OperatingPoint  # one motor, full throttle on a fresh pack
    peak_battery_current_a: float

    checks: list[Check]

    @property
    def total_mass_kg(self) -> float:
        return sum(self.mass_breakdown_kg.values())

    @property
    def average_battery_power_w(self) -> float:
        return self.power.total_w

    @property
    def usable_energy_j(self) -> float:
        return self.tech.usable_battery_fraction * self.build.battery.energy_j

    @property
    def fresh_thrust_to_weight(self) -> float:
        """Thrust-to-weight at full throttle on a fresh pack: the most the drone ever has."""
        thrust_n = self.build.propeller.thrust_at_speed_n(self.fresh_full_throttle.speed_rad_s)
        return self.build.airframe.rotor_count * thrust_n / (self.total_mass_kg * GRAVITY_M_PER_S2)

    @property
    def passed(self) -> bool:
        return all(check.passed for check in self.checks)

    @property
    def tightest_check(self) -> Check:
        """The check with the least margin: the one that fails worst, or the next to fail."""
        return min(self.checks, key=lambda check: check.margin)

    @property
    def best_estimate(self) -> "BuildResult":
        """The same build with the planning margins removed."""
        return self.build.evaluate_without_margins(self.requirements, self.tech)


# The pack capacities a search for a flight time looks between.
SMALLEST_PACK_AH = 0.02
LARGEST_PACK_AH = 20.0


def smallest_pack_for(
    flight_time_s: float, build_with_pack_of: Callable[[float], Build], requirements: Requirements, tech: Technology
) -> tuple[float, bool]:
    """The smallest pack capacity that reaches the flight time, and whether any does.

    A bigger pack flies longer until its own weight costs more than its energy adds. So flight
    time rises to a peak and then falls: find the peak, and if it's high enough, the answer is on
    the rising side. If no pack reaches the time, the capacity returned is the one that flies longest.
    """

    def flight_time(capacity_ah: float) -> float:
        return build_with_pack_of(capacity_ah).evaluate(requirements, tech).flight_time_s

    best_capacity_ah = minimize(lambda capacity_ah: -flight_time(capacity_ah), SMALLEST_PACK_AH, LARGEST_PACK_AH)
    if flight_time(best_capacity_ah) < flight_time_s:
        return best_capacity_ah, False
    return solve_increasing(flight_time, flight_time_s, SMALLEST_PACK_AH, best_capacity_ah), True


@dataclass(frozen=True)
class Uncertainty:
    """How far the least certain inputs might be off, in the direction that hurts, as fractions.

    Numbers taken from a datasheet or a thrust table get the first set. Numbers estimated from the
    fitted rules get the wider second set, which comes from the scatter of those fits.
    """

    thrust_coefficient: float  # the prop makes this much less thrust at a given speed
    power_coefficient: float  # and needs this much more power
    no_load_current: float  # the motor's drag is this much higher
    battery_capacity: float  # the pack holds this much less
    battery_resistance: float  # and sags this much more

    estimated_thrust_coefficient: float  # the same, for a prop whose coefficients came from the rule
    estimated_power_coefficient: float
    estimated_motor_constant: float  # an estimated K_m is this much lower (more copper loss)
    estimated_no_load_current: float  # an estimated drag is this much higher

    def worst_case(self, build: Build, tech: Technology) -> Build:
        """The same build with every uncertain number pushed the wrong way at once."""
        battery = build.battery
        return replace(
            build,
            motor=self.worst_case_motor(build.motor),
            propeller=self.worst_case_propeller(build.propeller),
            battery=replace(
                battery,
                capacity_ah=battery.capacity_ah * (1 - self.battery_capacity),
                internal_resistance_ohm=battery.resistance_ohm(tech.lead_resistance_ohm) * (1 + self.battery_resistance),
            ),
        )

    def worst_case_motor(self, motor: Motor) -> Motor:
        if "no-load current" in motor.estimated_fields:
            no_load_current_a = motor.no_load_current_a * (1 + self.estimated_no_load_current)
        else:
            no_load_current_a = motor.no_load_current_a * (1 + self.no_load_current)

        # K_m = K_t / sqrt(R_m), so a lower K_m at the same Kv means a higher resistance.
        resistance_ohm = motor.resistance_ohm
        if "resistance" in motor.estimated_fields:
            resistance_ohm /= (1 - self.estimated_motor_constant) ** 2

        # Keep the no-load test at the same speed, so the drag changes only by the factor above.
        test_voltage_v = (
            motor.no_load_test_speed_rad_s / motor.speed_constant_rad_s_per_v + no_load_current_a * resistance_ohm
        )
        return replace(
            motor,
            no_load_current_a=no_load_current_a,
            resistance_ohm=resistance_ohm,
            no_load_test_voltage_v=test_voltage_v,
        )

    def worst_case_propeller(self, propeller: Propeller) -> Propeller:
        if propeller.estimated:
            less_thrust, more_power = self.estimated_thrust_coefficient, self.estimated_power_coefficient
        else:
            less_thrust, more_power = self.thrust_coefficient, self.power_coefficient
        return replace(
            propeller,
            thrust_coefficient=propeller.thrust_coefficient * (1 - less_thrust),
            power_coefficient=propeller.power_coefficient * (1 + more_power),
        )
