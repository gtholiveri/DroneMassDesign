"""A complete drone design: every component sized for one assumed total mass."""

from dataclasses import dataclass

from drone_sizing.battery import Battery
from drone_sizing.esc import ESC
from drone_sizing.frame import Frame
from drone_sizing.inputs import DesignChoices, Requirements, Technology
from drone_sizing.motor import Motor
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

    # One of each per rotor; the drone has `choices.rotor_count` of each.
    rotor: Rotor
    motor: Motor
    esc: ESC

    battery: Battery
    frame: Frame
    average_battery_power_w: float

    @classmethod
    def size_for(
        cls,
        design_mass_kg: float,
        requirements: Requirements,
        choices: DesignChoices,
        tech: Technology,
    ) -> "DroneDesign":
        rotor = Rotor.size_for(design_mass_kg, requirements, choices)

        # Motor and ESC are sized by the worst case: full throttle.
        loaded_voltage_v = choices.cell_count * tech.loaded_cell_voltage_v
        motor = Motor.size_for(rotor.max_torque_nm, rotor.max_speed_rad_s, loaded_voltage_v, tech)
        esc = ESC.size_for(motor.input_power_w(rotor.max_shaft_power_w), tech)

        # The battery is sized by the typical case: hover power, traced back through the
        # power chain (prop <- motor <- ESC <- battery) and scaled up for maneuvering.
        hover_power_per_rotor_w = esc.input_power_w(motor.input_power_w(rotor.hover_shaft_power_w))
        average_battery_power_w = (
            choices.rotor_count * hover_power_per_rotor_w * requirements.average_power_factor
        )
        battery = Battery.size_for(average_battery_power_w, requirements.flight_time_s, tech)

        frame = Frame.size_for(design_mass_kg, tech)

        return cls(
            design_mass_kg=design_mass_kg,
            requirements=requirements,
            choices=choices,
            rotor=rotor,
            motor=motor,
            esc=esc,
            battery=battery,
            frame=frame,
            average_battery_power_w=average_battery_power_w,
        )

    @property
    def mass_breakdown_kg(self) -> dict[str, float]:
        """What each group of parts weighs. The single source of truth for the drone's mass."""
        rotor_count = self.choices.rotor_count
        breakdown_kg = {
            "payload": self.requirements.payload_mass_kg,
            "avionics": self.choices.avionics_mass_kg,
            "battery": self.battery.mass_kg,
            "motors": rotor_count * self.motor.mass_kg,
            "escs": rotor_count * self.esc.mass_kg,
            "props": rotor_count * self.rotor.mass_kg,
            "frame": self.frame.mass_kg,
        }

        # The payload is specified, not estimated, so the margin covers everything else.
        estimated_mass_kg = sum(breakdown_kg.values()) - breakdown_kg["payload"]
        breakdown_kg["margin"] = self.requirements.mass_margin_fraction * estimated_mass_kg

        return breakdown_kg

    @property
    def built_mass_kg(self) -> float:
        """What the parts actually weigh, added up."""
        return sum(self.mass_breakdown_kg.values())

    @property
    def mass_mismatch_kg(self) -> float:
        """Built mass minus design mass. Positive means the design is undersized."""
        return self.built_mass_kg - self.design_mass_kg
