"""One motor: how heavy it must be, what Kv to shop for, and how much electrical power it draws."""

from dataclasses import dataclass

from drone_sizing.constants import RPM_PER_RAD_PER_S
from drone_sizing.inputs import Technology

# Under load a motor only reaches about 90% of its no-load speed (Kv x volts),
# because some voltage is lost across the winding resistance.
LOADED_SPEED_FRACTION = 0.9


@dataclass(frozen=True)
class Motor:
    """One motor, sized by the torque and speed it must reach at full throttle."""

    max_torque_nm: float
    max_speed_rad_s: float
    required_kv_rpm_per_v: float
    efficiency: float
    mass_kg: float

    @classmethod
    def size_for(
        cls,
        max_torque_nm: float,
        max_speed_rad_s: float,
        loaded_voltage_v: float,
        tech: Technology,
    ) -> "Motor":
        # Torque comes from the size of the rotor, magnets and copper, so mass scales with torque.
        mass_kg = max_torque_nm / tech.motor_torque_density_nm_per_kg

        # Kv must let the motor reach full-throttle speed even on a sagging battery.
        max_speed_rpm = max_speed_rad_s * RPM_PER_RAD_PER_S
        required_kv_rpm_per_v = max_speed_rpm / (LOADED_SPEED_FRACTION * loaded_voltage_v)

        return cls(
            max_torque_nm=max_torque_nm,
            max_speed_rad_s=max_speed_rad_s,
            required_kv_rpm_per_v=required_kv_rpm_per_v,
            efficiency=tech.motor_efficiency,
            mass_kg=mass_kg,
        )

    def input_power_w(self, shaft_power_w: float) -> float:
        """Electrical power the motor draws to deliver this much shaft power."""
        return shaft_power_w / self.efficiency
