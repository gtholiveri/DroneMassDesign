"""A complete drone design from the continuous model: every component sized for one assumed total mass.

Sizing and judging are kept apart. This module sizes the motor and battery for a design mass; how
the resulting drone flies comes from Build, the same evaluation real catalog parts get.
"""

import math
from dataclasses import dataclass
from functools import cached_property

from drone_sizing.battery import BatteryPack
from drone_sizing.build import Build, BuildResult
from drone_sizing.inputs import DesignChoices, Requirements, ScalingLaws, Technology
from drone_sizing.motor import HEAVIEST_MOTOR_KG, LIGHTEST_MOTOR_KG, Motor, OperatingPoint
from drone_sizing.numerics import minimize, solve_increasing
from drone_sizing.rotor import Rotor


@dataclass(frozen=True)
class DroneDesign:
    """All components sized to fly a drone of `design_mass_kg`.

    `built_mass_kg` is what those components actually weigh. The two match only for a
    consistent design, which is what the closure loop searches for.
    """

    design_mass_kg: float
    requirements: Requirements
    choices: DesignChoices
    tech: Technology
    scaling: ScalingLaws

    rotor: Rotor  # what one rotor must do on a drone of the design mass
    motor: Motor  # a motor that doesn't exist yet, with the properties MotorScaling predicts
    battery: BatteryPack  # a pack of exactly the energy the design needs

    @classmethod
    def size_for(
        cls,
        design_mass_kg: float,
        requirements: Requirements,
        choices: DesignChoices,
        tech: Technology,
        scaling: ScalingLaws,
    ) -> "DroneDesign":
        rotor = Rotor.size_for(design_mass_kg, requirements, choices)

        def design_with_motor_of(log_mass_kg: float) -> "DroneDesign":
            return cls._assemble(design_mass_kg, requirements, choices, tech, scaling, rotor, math.exp(log_mass_kg))

        # The motor must survive full throttle on a fresh pack, where it draws the most current.
        def current_over_rating(log_mass_kg: float) -> float:
            design = design_with_motor_of(log_mass_kg)
            return design.fresh_full_throttle.current_a / design.motor.max_current_a

        # It can't be lighter than the smallest motor the scaling laws were fitted to. If that
        # motor would overheat, go heavier: a heavier motor sheds more heat, so current over
        # rating falls as mass grows, and the lightest motor allowed is where it reaches 1.
        lightest_log_kg = math.log(max(scaling.motor.smallest_mass_kg, LIGHTEST_MOTOR_KG))
        if current_over_rating(lightest_log_kg) > 1.0:
            lightest_log_kg = solve_increasing(
                lambda log_mass_kg: -current_over_rating(log_mass_kg),
                -1.0,
                lightest_log_kg,
                math.log(HEAVIEST_MOTOR_KG),
            )
            if current_over_rating(lightest_log_kg) > 1.0 + 1e-6:
                raise ValueError("no motor up to 100 kg can survive full throttle with this prop")

        # Heavier motors also survive, and trade copper loss against drag, so their efficiency
        # differs. Search up to a motor weighing the whole drone's share and keep the lightest drone.
        heaviest_log_kg = max(lightest_log_kg, math.log(design_mass_kg / choices.airframe.rotor_count))
        best_log_kg = minimize(
            lambda log_mass_kg: design_with_motor_of(log_mass_kg).built_mass_kg, lightest_log_kg, heaviest_log_kg
        )
        return design_with_motor_of(best_log_kg)

    @classmethod
    def _assemble(
        cls,
        design_mass_kg: float,
        requirements: Requirements,
        choices: DesignChoices,
        tech: Technology,
        scaling: ScalingLaws,
        rotor: Rotor,
        motor_mass_kg: float,
    ) -> "DroneDesign":
        """The whole design around a motor of this mass: first the battery, then the motor's winding."""
        airframe = choices.airframe
        rotor_count = airframe.rotor_count
        esc_efficiency = airframe.board.esc_efficiency
        speed_exponent = tech.no_load_current_speed_exponent
        kind = scaling.battery

        # A motor's power at a given speed and torque depends on its size (K_m and drag), not on
        # its winding. So any winding tells us the hover power, and with it the battery.
        unwound = scaling.motor.motor(
            motor_mass_kg, rotor.max_torque_nm, rotor.max_speed_rad_s, kind.tired_voltage_v, speed_exponent
        )
        hover_power_w = unwound.operating_point(rotor.hover_torque_nm, rotor.hover_speed_rad_s, speed_exponent).input_power_w
        max_power_w = unwound.operating_point(rotor.max_torque_nm, rotor.max_speed_rad_s, speed_exponent).input_power_w

        # The battery is sized by the typical case: hover power traced back through the ESC,
        # scaled up for maneuvering, plus the electronics. Only part of the pack is usable, so
        # it stores E = P t / f_usable.
        average_battery_power_w = (
            requirements.average_power_factor * rotor_count * hover_power_w / esc_efficiency
            + airframe.electronics_power_w
        )
        battery = kind.pack_holding(
            average_battery_power_w * requirements.flight_time_s / tech.usable_battery_fraction
        )

        # With the battery known, so is its sag. Wind the motor to just reach the max point at
        # full throttle late in the flight, on the voltage the tired pack delivers under that load.
        full_throttle_voltage_v = battery.voltage_under_load_v(
            kind.tired_voltage_v,
            tech.lead_resistance_ohm,
            rotor_count * max_power_w / esc_efficiency,
            airframe.electronics_power_w / kind.tired_voltage_v,
        )
        motor = scaling.motor.motor(
            motor_mass_kg, rotor.max_torque_nm, rotor.max_speed_rad_s, full_throttle_voltage_v, speed_exponent
        )

        return cls(
            design_mass_kg=design_mass_kg,
            requirements=requirements,
            choices=choices,
            tech=tech,
            scaling=scaling,
            rotor=rotor,
            motor=motor,
            battery=battery,
        )

    @property
    def mass_breakdown_kg(self) -> dict[str, float]:
        """What each group of parts weighs. The single source of truth for the drone's mass."""
        return self.build.mass_breakdown_kg(self.requirements)

    @property
    def built_mass_kg(self) -> float:
        """What the parts actually weigh, added up."""
        return sum(self.mass_breakdown_kg.values())

    @property
    def mass_mismatch_kg(self) -> float:
        """Built mass minus design mass. Positive means the design is undersized."""
        return self.built_mass_kg - self.design_mass_kg

    @cached_property
    def build(self) -> Build:
        """The sized parts as a definite drone."""
        return Build(airframe=self.choices.airframe, motor=self.motor, propeller=self.rotor.propeller, battery=self.battery)

    @cached_property
    def result(self) -> BuildResult:
        """How the sized drone flies, judged exactly as a drone of catalog parts would be.

        For a consistent design (built mass equal to design mass) it meets the requirements it was
        sized for; its flight time and thrust-to-weight come out at the required values.
        """
        return self.build.evaluate(self.requirements, self.tech)

    @property
    def hover(self) -> OperatingPoint:
        return self.result.hover

    @property
    def tired_full_throttle(self) -> OperatingPoint:
        """The motor at full throttle late in the flight, on the sagging pack: the max point it was wound for."""
        return self.result.tired_full_throttle

    @property
    def fresh_full_throttle(self) -> OperatingPoint:
        """The motor at full throttle on a fresh pack, where it draws the most current."""
        return self.result.fresh_full_throttle
