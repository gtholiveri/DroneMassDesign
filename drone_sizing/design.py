"""A complete drone design from the continuous model: every component sized for one assumed total mass."""

import math
from dataclasses import dataclass

from drone_sizing.battery import Battery
from drone_sizing.inputs import DesignChoices, Requirements, ScalingLaws, Technology
from drone_sizing.motor import Motor, OperatingPoint
from drone_sizing.numerics import minimize
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

    # One of each per rotor; the drone has `rotor_count` of each.
    rotor: Rotor
    motor: Motor  # a motor that doesn't exist yet, with the properties MotorScaling predicts
    hover: OperatingPoint  # that motor at hover

    battery: Battery
    average_battery_power_w: float

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

        loaded_voltage_v = choices.cell_count * tech.loaded_cell_voltage_v
        fresh_voltage_v = choices.cell_count * tech.fresh_loaded_cell_voltage_v
        speed_exponent = tech.no_load_current_speed_exponent

        def motor_of_mass(mass_kg: float) -> Motor:
            # Wound to just reach the rotor's max point on a tired pack.
            return scaling.motor.motor(
                mass_kg, rotor.max_torque_nm, rotor.max_speed_rad_s, loaded_voltage_v, speed_exponent
            )

        def design_with(motor: Motor) -> "DroneDesign":
            return cls._assemble(design_mass_kg, requirements, choices, tech, scaling, rotor, motor)

        # The motor must survive full throttle on a fresh pack, which sets its lightest mass (and
        # it can't be lighter than the smallest motor the scaling laws were fitted to). Heavier
        # motors also survive, and trade copper loss against drag, so their efficiency differs.
        # Search up to a motor weighing the whole drone's share and keep the lightest drone.
        lightest_motor_kg = scaling.motor.lightest_mass_kg(
            choices.propeller,
            rotor.max_torque_nm,
            rotor.max_speed_rad_s,
            loaded_voltage_v,
            fresh_voltage_v,
            speed_exponent,
        )
        lightest_motor_kg = max(lightest_motor_kg, scaling.motor.smallest_mass_kg)
        heaviest_motor_kg = max(lightest_motor_kg, design_mass_kg / choices.airframe.rotor_count)
        best_log_mass_kg = minimize(
            lambda log_mass_kg: design_with(motor_of_mass(math.exp(log_mass_kg))).built_mass_kg,
            math.log(lightest_motor_kg),
            math.log(heaviest_motor_kg),
        )
        return design_with(motor_of_mass(math.exp(best_log_mass_kg)))

    @classmethod
    def _assemble(
        cls,
        design_mass_kg: float,
        requirements: Requirements,
        choices: DesignChoices,
        tech: Technology,
        scaling: ScalingLaws,
        rotor: Rotor,
        motor: Motor,
    ) -> "DroneDesign":
        """The rest of the design once the motor is picked."""
        airframe = choices.airframe
        hover = motor.operating_point(
            rotor.hover_torque_nm, rotor.hover_speed_rad_s, tech.no_load_current_speed_exponent
        )

        # The battery is sized by the typical case: hover power traced back through the motor and
        # ESC, scaled up for maneuvering, plus what the electronics draw.
        propulsion_power_w = airframe.rotor_count * hover.input_power_w / airframe.board.esc_efficiency
        average_battery_power_w = (
            requirements.average_power_factor * propulsion_power_w + airframe.electronics_power_w
        )
        battery = Battery.size_for(
            average_battery_power_w,
            requirements.flight_time_s,
            tech.usable_battery_fraction,
            scaling.battery_specific_energy_j_per_kg,
        )

        return cls(
            design_mass_kg=design_mass_kg,
            requirements=requirements,
            choices=choices,
            tech=tech,
            rotor=rotor,
            motor=motor,
            hover=hover,
            battery=battery,
            average_battery_power_w=average_battery_power_w,
        )

    @property
    def fresh_full_throttle(self) -> OperatingPoint:
        """The motor at full throttle on a fresh pack, where it draws the most current."""
        fresh_voltage_v = self.choices.cell_count * self.tech.fresh_loaded_cell_voltage_v
        return self.motor.full_throttle(
            fresh_voltage_v, self.choices.propeller, self.tech.no_load_current_speed_exponent
        )

    @property
    def mass_breakdown_kg(self) -> dict[str, float]:
        """What each group of parts weighs. The single source of truth for the drone's mass."""
        rotor_count = self.choices.airframe.rotor_count
        parts_kg = {
            **self.choices.airframe.fixed_mass_breakdown_kg,
            "motors": rotor_count * self.motor.mass_kg,
            "props": rotor_count * self.rotor.mass_kg,
            "battery": self.battery.mass_kg,
        }
        return self.requirements.with_payload_and_margin_kg(parts_kg)

    @property
    def built_mass_kg(self) -> float:
        """What the parts actually weigh, added up."""
        return sum(self.mass_breakdown_kg.values())

    @property
    def mass_mismatch_kg(self) -> float:
        """Built mass minus design mass. Positive means the design is undersized."""
        return self.built_mass_kg - self.design_mass_kg
