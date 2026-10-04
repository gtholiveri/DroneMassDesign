"""Motors: the DC motor model that links volts and amps to speed and torque, and how to size a motor
before a real one has been chosen."""

import math
from collections.abc import Sequence
from dataclasses import dataclass

from drone_sizing.constants import RPM_PER_RAD_PER_S
from drone_sizing.numerics import PowerLaw, solve_increasing
from drone_sizing.propeller import Propeller

# MotorScaling quotes each motor's drag torque at this speed so motors tested at different speeds
# can be compared. Any fixed value works, as long as fitting and sizing use the same one.
REFERENCE_SPEED_RAD_S = 1000.0

# Range of motor masses the sizing search looks over: 0.01 g to 100 kg.
LIGHTEST_MOTOR_KG = 1e-5
HEAVIEST_MOTOR_KG = 100.0


@dataclass(frozen=True)
class OperatingPoint:
    """A motor running steadily: its speed, the torque it delivers, and what it draws."""

    speed_rad_s: float
    torque_nm: float  # delivered to the prop, after friction and iron losses
    current_a: float
    voltage_v: float  # across the motor, after the ESC

    @property
    def shaft_power_w(self) -> float:
        return self.torque_nm * self.speed_rad_s

    @property
    def input_power_w(self) -> float:
        return self.voltage_v * self.current_a

    @property
    def efficiency(self) -> float:
        return self.shaft_power_w / self.input_power_w


@dataclass(frozen=True)
class StatorSize:
    """A motor's form factor as listings name it: the stator's diameter and height in millimeters.

    A "1203" motor has a stator 12 mm across and 3 mm tall. Motors of one size weigh about the same
    whoever makes them, so the size is the quickest way to search for a motor of a given mass.
    """

    diameter_mm: float
    height_mm: float
    typical_mass_kg: float  # the median listed mass of the motors of this size we have seen

    @property
    def code(self) -> str:
        """The size as listings write it: 1203, or 1202.5 for a stator 2.5 mm tall."""
        whole = self.height_mm == int(self.height_mm)
        return f"{self.diameter_mm:02.0f}" + (f"{self.height_mm:02.0f}" if whole else f"{self.height_mm:04.1f}")

    @classmethod
    def from_code(cls, code: str, typical_mass_kg: float) -> "StatorSize":
        return cls(diameter_mm=float(code[:2]), height_mm=float(code[2:]), typical_mass_kg=typical_mass_kg)


def nearest_stator_sizes(sizes: Sequence[StatorSize], mass_kg: float, count: int) -> list[StatorSize]:
    """The sizes whose typical mass is closest to this mass (by ratio), closest first."""
    return sorted(sizes, key=lambda size: abs(math.log(size.typical_mass_kg / mass_kg)))[:count]


@dataclass(frozen=True)
class Motor:
    """One motor, described by the numbers on its datasheet.

    The first-order DC motor model. With k_v the speed constant in rad/s per volt, and
    K_t = 1 / k_v the torque constant in N m per amp:
        voltage:  V = omega / k_v + I * R_m      back-EMF, plus the drop across the windings
        torque:   Q = K_t * I - Q_0(omega)       what the current makes, minus what drag eats

    Q_0 is the drag torque from bearing friction and iron losses: what the no-load current I_0
    pays for. It grows with speed, so the model scales it from the datasheet's no-load test as
        Q_0(omega) = K_t * I_0 * (omega / omega_test)^k
    The exponent k is an assumption (Technology.no_load_current_speed_exponent). k = 0 means drag
    torque stays constant, like pure friction. k = 1 means it grows in proportion to speed, like
    eddy currents in the iron.
    """

    name: str
    kv_rpm_per_v: float
    resistance_ohm: float  # winding resistance R_m ("internal resistance" on most datasheets)
    no_load_current_a: float  # I_0, spinning with no prop...
    no_load_test_voltage_v: float  # ...at this voltage
    mass_kg: float
    max_current_a: float  # the most the windings take ("peak" or "max" current)
    shaft_diameter_m: float | None = None
    price_usd: float | None = None
    max_cell_count: int | None = None  # the most LiPo cells in series it's rated for

    # Names of the numbers above that weren't on the datasheet and were estimated by MotorScaling.
    estimated_fields: tuple[str, ...] = ()

    @property
    def speed_constant_rad_s_per_v(self) -> float:
        """k_v in SI units."""
        return self.kv_rpm_per_v / RPM_PER_RAD_PER_S

    @property
    def torque_constant_nm_per_a(self) -> float:
        """K_t = 1 / k_v: the same constant seen from the mechanical side."""
        return 1 / self.speed_constant_rad_s_per_v

    @property
    def motor_constant_nm_per_sqrt_w(self) -> float:
        """K_m = K_t / sqrt(R_m). Making torque Q costs (Q / K_m)^2 of copper heat, whatever the winding."""
        return self.torque_constant_nm_per_a / math.sqrt(self.resistance_ohm)

    @property
    def no_load_test_speed_rad_s(self) -> float:
        """How fast the motor spun during its no-load test: omega = k_v * (V - I_0 R_m)."""
        return self.speed_constant_rad_s_per_v * (
            self.no_load_test_voltage_v - self.no_load_current_a * self.resistance_ohm
        )

    def drag_torque_nm(self, speed_rad_s: float, speed_exponent: float) -> float:
        """Q_0: torque lost to friction and iron losses at this speed."""
        test_drag_torque_nm = self.torque_constant_nm_per_a * self.no_load_current_a
        return test_drag_torque_nm * (speed_rad_s / self.no_load_test_speed_rad_s) ** speed_exponent

    def operating_point(self, torque_nm: float, speed_rad_s: float, speed_exponent: float) -> OperatingPoint:
        """The current and voltage this motor needs to deliver this torque at this speed."""
        # The current must make the load torque plus the drag torque.
        current_a = (torque_nm + self.drag_torque_nm(speed_rad_s, speed_exponent)) / self.torque_constant_nm_per_a
        voltage_v = speed_rad_s / self.speed_constant_rad_s_per_v + current_a * self.resistance_ohm
        return OperatingPoint(
            speed_rad_s=speed_rad_s, torque_nm=torque_nm, current_a=current_a, voltage_v=voltage_v
        )

    def full_throttle(self, voltage_v: float, propeller: Propeller, speed_exponent: float) -> OperatingPoint:
        """Where this motor and prop settle with the whole voltage applied.

        Spinning faster makes the prop demand more torque, which takes more current and so more
        voltage. The motor settles at the speed where the voltage it needs equals the voltage it
        has. That speed is below the no-load speed k_v * V, so the search looks between 0 and there.
        """

        def voltage_needed_v(speed_rad_s: float) -> float:
            torque_nm = propeller.torque_at_speed_nm(speed_rad_s)
            return self.operating_point(torque_nm, speed_rad_s, speed_exponent).voltage_v

        no_load_speed_rad_s = self.speed_constant_rad_s_per_v * voltage_v
        speed_rad_s = solve_increasing(voltage_needed_v, voltage_v, 0.0, no_load_speed_rad_s)
        return self.operating_point(propeller.torque_at_speed_nm(speed_rad_s), speed_rad_s, speed_exponent)

    def measured_point(
        self,
        voltage_v: float,
        current_a: float,
        speed_exponent: float,
        measured_speed_rad_s: float | None = None,
    ) -> OperatingPoint:
        """Speed and torque implied by a measured voltage and current, like one row of a vendor's thrust table.

        The same two equations, solved the other way: speed from the voltage equation, then torque.
        If the table also lists speed, pass it in: a measured speed beats one inferred from Kv.
        Vendor tables usually measure current on the battery side, which matches the motor current
        only at full throttle, so use full-throttle rows.
        """
        if measured_speed_rad_s is None:
            speed_rad_s = self.speed_constant_rad_s_per_v * (voltage_v - current_a * self.resistance_ohm)
        else:
            speed_rad_s = measured_speed_rad_s
        torque_nm = self.torque_constant_nm_per_a * current_a - self.drag_torque_nm(speed_rad_s, speed_exponent)
        return OperatingPoint(
            speed_rad_s=speed_rad_s, torque_nm=torque_nm, current_a=current_a, voltage_v=voltage_v
        )


@dataclass(frozen=True)
class MotorScaling:
    """How a motor's size-dependent properties grow with its mass, for sizing a motor that hasn't been chosen.

    Each is a power law in motor mass m (kg). None of them depends on the winding (Kv), which is
    why they can be fitted across motors wound for different Kv:
        motor constant   K_m = K_t / sqrt(R_m)   N m / sqrt(W)   sets copper loss, (Q / K_m)^2
        drag torque      Q_0                     N m             friction and iron loss, at REFERENCE_SPEED_RAD_S
        max copper loss  I_max^2 R_m             W               how much heat the windings can take
    """

    motor_constant: PowerLaw
    drag_torque: PowerLaw
    max_copper_loss: PowerLaw

    # The lightest motor the laws were fitted to. Sizing won't pick a lighter one: below this the
    # laws are extrapolation, and such motors may not exist.
    smallest_mass_kg: float = 0.0

    # The sizes motors are sold in, with what each typically weighs: how a sized motor is named.
    stator_sizes: tuple[StatorSize, ...] = ()

    @classmethod
    def fit(cls, motors: Sequence[Motor], speed_exponent: float) -> "MotorScaling":
        """Regress each property against mass across real motors. Needs motors of at least two masses."""
        masses_kg = [motor.mass_kg for motor in motors]
        return cls(
            motor_constant=PowerLaw.fit(masses_kg, [motor.motor_constant_nm_per_sqrt_w for motor in motors]),
            drag_torque=PowerLaw.fit(
                masses_kg, [motor.drag_torque_nm(REFERENCE_SPEED_RAD_S, speed_exponent) for motor in motors]
            ),
            max_copper_loss=PowerLaw.fit(
                masses_kg, [motor.max_current_a**2 * motor.resistance_ohm for motor in motors]
            ),
            smallest_mass_kg=min(masses_kg),
        )

    @classmethod
    def from_motors(cls, motors: Sequence[Motor], speed_exponent: float) -> "MotorScaling":
        """Scaling from the motors whose datasheets list everything (none of their numbers estimated).

        With two or more different masses among them, fit. With one, scale from that motor.
        """
        measured = [motor for motor in motors if not motor.estimated_fields]
        if not measured:
            raise ValueError("need at least one motor with resistance, no-load current and max current listed")
        if len({motor.mass_kg for motor in measured}) >= 2:
            return cls.fit(measured, speed_exponent)
        return cls.from_one_motor(measured[0], speed_exponent)

    @classmethod
    def from_one_motor(cls, motor: Motor, speed_exponent: float) -> "MotorScaling":
        """Scale from one real motor, assuming bigger motors are the same shape, scaled up.

        Scale every length by s, so mass grows as s^3. At the same current density, torque grows
        as s^4 (force grows with volume, times a longer lever arm) and copper loss as s^3, so
        K_m = Q / sqrt(P_Cu) grows as s^2.5, or m^(5/6). Iron and friction losses grow with volume,
        so drag torque grows as m. Heat leaves through the surface, so the copper loss the windings
        can take grows as s^2, or m^(2/3).
        """
        mass_kg = motor.mass_kg
        return cls(
            motor_constant=PowerLaw.through_point(mass_kg, motor.motor_constant_nm_per_sqrt_w, 5 / 6),
            drag_torque=PowerLaw.through_point(
                mass_kg, motor.drag_torque_nm(REFERENCE_SPEED_RAD_S, speed_exponent), 1.0
            ),
            max_copper_loss=PowerLaw.through_point(mass_kg, motor.max_current_a**2 * motor.resistance_ohm, 2 / 3),
        )

    def stator_sizes_near(self, mass_kg: float, count: int) -> list[StatorSize]:
        """The sizes a motor of this mass would be sold as, closest in mass first."""
        return nearest_stator_sizes(self.stator_sizes, mass_kg, count)

    def motor(
        self,
        mass_kg: float,
        max_torque_nm: float,
        max_speed_rad_s: float,
        voltage_v: float,
        speed_exponent: float,
    ) -> Motor:
        """A motor of this mass, wound so that full throttle on this voltage just reaches the max point.

        Mass sets K_m, Q_0 and the heat limit. The winding sets k_v, and with it R_m and I_0.
        Substituting R_m = K_t^2 / K_m^2 and I = (Q + Q_0) / K_t into V = omega / k_v + I R_m
        gives the winding directly:
            k_v = (omega_max + (Q_max + Q_0) / K_m^2) / V
        """
        motor_constant = self.motor_constant(mass_kg)
        reference_drag_torque_nm = self.drag_torque(mass_kg)
        drag_torque_nm = reference_drag_torque_nm * (max_speed_rad_s / REFERENCE_SPEED_RAD_S) ** speed_exponent

        speed_constant = (max_speed_rad_s + (max_torque_nm + drag_torque_nm) / motor_constant**2) / voltage_v
        return self.motor_with_kv(f"sized {mass_kg * 1000:.2f} g", speed_constant * RPM_PER_RAD_PER_S, mass_kg)

    def motor_with_kv(
        self,
        name: str,
        kv_rpm_per_v: float,
        mass_kg: float,
        resistance_ohm: float | None = None,
        no_load_current_a: float | None = None,
        no_load_test_voltage_v: float | None = None,
        max_current_a: float | None = None,
        shaft_diameter_m: float | None = None,
        price_usd: float | None = None,
    ) -> Motor:
        """A motor of this Kv and mass. Datasheet numbers given are kept; missing ones come from this scaling.

        The motor records which numbers were estimated.
        """
        torque_constant = RPM_PER_RAD_PER_S / kv_rpm_per_v
        estimated_fields = []

        # K_m = K_t / sqrt(R_m), so R_m = (K_t / K_m)^2.
        if resistance_ohm is None:
            resistance_ohm = (torque_constant / self.motor_constant(mass_kg)) ** 2
            estimated_fields.append("resistance")

        # Quote an estimated no-load test at the reference speed, where the drag fit applies:
        # I_0 = Q_0 / K_t, at the voltage that spins the motor at that speed with no load.
        if no_load_current_a is None or no_load_test_voltage_v is None:
            no_load_current_a = self.drag_torque(mass_kg) / torque_constant
            no_load_test_voltage_v = REFERENCE_SPEED_RAD_S * torque_constant + no_load_current_a * resistance_ohm
            estimated_fields.append("no-load current")

        # The current at which copper loss reaches the heat limit: I_max^2 R_m = P_Cu,max.
        if max_current_a is None:
            max_current_a = math.sqrt(self.max_copper_loss(mass_kg) / resistance_ohm)
            estimated_fields.append("max current")

        return Motor(
            name=name,
            kv_rpm_per_v=kv_rpm_per_v,
            resistance_ohm=resistance_ohm,
            no_load_current_a=no_load_current_a,
            no_load_test_voltage_v=no_load_test_voltage_v,
            mass_kg=mass_kg,
            max_current_a=max_current_a,
            shaft_diameter_m=shaft_diameter_m,
            price_usd=price_usd,
            estimated_fields=tuple(estimated_fields),
        )

    def kv_window(
        self,
        mass_kg: float,
        max_current_a: float,
        propeller: Propeller,
        max_torque_nm: float,
        max_speed_rad_s: float,
        loaded_voltage_v: float,
        fresh_voltage_v: float,
        speed_exponent: float,
    ) -> tuple[float, float] | None:
        """The Kv range that works for a motor of this mass and current rating, in RPM/V.

        Below the range, full throttle on a tired pack can't reach the max point. Above it, full
        throttle on a fresh pack draws more than the rated current. None if no Kv does both.
        """
        lowest_kv = self.motor(mass_kg, max_torque_nm, max_speed_rad_s, loaded_voltage_v, speed_exponent).kv_rpm_per_v

        def fresh_current_a(kv_rpm_per_v: float) -> float:
            motor = self.motor_with_kv("candidate", kv_rpm_per_v, mass_kg)
            return motor.full_throttle(fresh_voltage_v, propeller, speed_exponent).current_a

        if fresh_current_a(lowest_kv) > max_current_a:
            return None
        # More Kv spins the prop faster on the same voltage, so the current only rises with Kv.
        highest_kv = solve_increasing(fresh_current_a, max_current_a, lowest_kv, 10 * lowest_kv)
        return lowest_kv, highest_kv
