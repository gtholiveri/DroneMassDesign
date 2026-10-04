"""Read candidate parts from CSV files, in the units listings use (g, mm, mAh), and fill the gaps.

Most listings give a motor's Kv, mass, max current or max power, rated cells, stator size and
shaft, but not its winding resistance or no-load current. Those come from rules fitted to
thrust-stand data (catalog/scaling.json, from tools/fit_tyto.py): K_m and drag torque against
motor mass. Each motor records which numbers were estimated.

Prop listings give diameter, pitch and blade count. C_T and C_P come from, in order of preference:
the CSV itself, one full-throttle thrust-table row on a catalog motor, or the rule against
pitch / diameter and blade count fitted to static tests of small props (tools/fit_props.py).

Blank cells mean "not known". Rows whose name starts with # are skipped. Extra columns (stator,
source, notes) are ignored by the model, so keep whatever you like.
"""

import csv
import json
from dataclasses import dataclass, replace
from pathlib import Path

from drone_sizing.battery import CHEMISTRIES, LIPO, BatteryPack
from drone_sizing.constants import GRAVITY_M_PER_S2, KILOGRAMS_PER_GRAM, METERS_PER_MILLIMETER, RPM_PER_RAD_PER_S
from drone_sizing.inputs import Technology
from drone_sizing.motor import Motor, MotorScaling, StatorSize
from drone_sizing.numerics import PowerLaw
from drone_sizing.propeller import Propeller, PropellerLaw, PropellerScaling

AMP_HOURS_PER_MILLIAMP_HOUR = 0.001
OHMS_PER_MILLIOHM = 0.001

MOTOR_DATASHEET_COLUMNS = ("resistance_ohm", "no_load_current_a", "no_load_test_voltage_v", "max_current_a")

Row = dict[str, str | None]


@dataclass(frozen=True)
class FittedScaling:
    """Rules fitted to thrust-stand data, for parts whose listings leave numbers out."""

    motor_constant: PowerLaw  # K_m against motor mass
    drag_torque: PowerLaw  # Q_0 at the reference speed against motor mass
    smallest_motor_mass_kg: float  # the lightest motor in the fit
    stator_sizes: tuple[StatorSize, ...]  # the motor sizes on sale, with what each typically weighs
    propeller: PropellerScaling
    speed_exponent: float  # the drag-speed exponent the fit assumed


def load_fitted_scaling(path: Path, tech: Technology) -> FittedScaling | None:
    """The rules in catalog/scaling.json, or None if it hasn't been made yet."""
    if not path.exists():
        return None
    fitted = json.loads(path.read_text(encoding="utf-8"))
    if fitted["speed_exponent"] != tech.no_load_current_speed_exponent:
        raise ValueError(
            f"{path.name} was fitted with drag-speed exponent {fitted['speed_exponent']}, "
            f"but Technology uses {tech.no_load_current_speed_exponent}: rerun tools/fit_tyto.py"
        )

    def law(entry: dict) -> PowerLaw:
        return PowerLaw(coefficient=entry["coefficient"], exponent=entry["exponent"])

    def propeller_law(entry: dict) -> PropellerLaw:
        return PropellerLaw(
            coefficient=entry["coefficient"],
            pitch_ratio_exponent=entry["pitch_ratio_exponent"],
            diameter_exponent=entry["diameter_exponent"],
        )

    propeller = fitted["propeller"]
    return FittedScaling(
        motor_constant=law(fitted["motor"]["motor_constant"]),
        drag_torque=law(fitted["motor"]["drag_torque"]),
        smallest_motor_mass_kg=fitted["motor"]["mass_range_kg"][0],
        stator_sizes=tuple(
            StatorSize(size["diameter_mm"], size["height_mm"], size["mass_kg"])
            for size in fitted["motor"].get("stator_sizes", [])
        ),
        propeller=PropellerScaling(
            thrust_coefficient=propeller_law(propeller["thrust_coefficient"]),
            power_coefficient=propeller_law(propeller["power_coefficient"]),
            blade_factors=tuple(
                (int(blades), factors["thrust"], factors["power"])
                for blades, factors in propeller["blade_factors"].items()
            ),
        ),
        speed_exponent=fitted["speed_exponent"],
    )


def read_rows(path: Path) -> list[Row]:
    """Each row as {column: text}, with blank cells as None."""
    rows = []
    with open(path, newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            cleaned = {column.strip(): (text.strip() or None) if text else None for column, text in row.items()}
            name = cleaned.get("name")
            if name is None or name.startswith("#"):
                continue
            rows.append(cleaned)
    return rows


def number(row: Row, column: str, scale: float = 1.0) -> float | None:
    """The cell as a number in SI units (scale converts, e.g. grams to kg), or None if blank."""
    text = row.get(column)
    return None if text is None else float(text) * scale


def required(row: Row, column: str, scale: float = 1.0) -> float:
    value = number(row, column, scale)
    if value is None:
        raise ValueError(f"{row['name']}: '{column}' is required")
    return value


def fully_listed_motor(row: Row) -> Motor:
    return Motor(
        name=row["name"],
        kv_rpm_per_v=required(row, "kv_rpm_per_v"),
        resistance_ohm=required(row, "resistance_ohm"),
        no_load_current_a=required(row, "no_load_current_a"),
        no_load_test_voltage_v=required(row, "no_load_test_voltage_v"),
        mass_kg=required(row, "mass_g", KILOGRAMS_PER_GRAM),
        max_current_a=required(row, "max_current_a"),
    )


def load_motor_scaling(path: Path, tech: Technology, fitted: FittedScaling | None) -> MotorScaling:
    """The rules used to fill in motors' missing numbers.

    K_m and drag come from the thrust-stand fit when there is one. The heat limit (max copper
    loss) can't be seen in thrust-stand data, so it always comes from the fully listed motors.
    """
    rows = read_rows(path)
    fully_listed = [fully_listed_motor(row) for row in rows if all(row.get(c) for c in MOTOR_DATASHEET_COLUMNS)]
    scaling = MotorScaling.from_motors(fully_listed, tech.no_load_current_speed_exponent)
    if fitted is None:
        return scaling
    return replace(
        scaling,
        motor_constant=fitted.motor_constant,
        drag_torque=fitted.drag_torque,
        smallest_mass_kg=fitted.smallest_motor_mass_kg,
        stator_sizes=fitted.stator_sizes,
    )


def load_motors(path: Path, tech: Technology, scaling: MotorScaling) -> tuple[Motor, ...]:
    """Columns: name, kv_rpm_per_v, mass_g, then as many as are known of max_current_a,
    max_power_w, max_cells, shaft_mm, price_usd, resistance_ohm, no_load_current_a,
    no_load_test_voltage_v."""
    motors = []
    for row in read_rows(path):
        max_cells = number(row, "max_cells")
        max_current_a = number(row, "max_current_a")
        from_max_power = False

        # Listings often give max power instead of max current: I_max = P_max / V at the rated
        # cells, taking a cell at its LiPo nominal voltage, which is what such listings assume.
        max_power_w = number(row, "max_power_w")
        if max_current_a is None and max_power_w is not None and max_cells is not None:
            max_current_a = max_power_w / (max_cells * LIPO.nominal_cell_voltage_v)
            from_max_power = True

        motor = scaling.motor_with_kv(
            name=row["name"],
            kv_rpm_per_v=required(row, "kv_rpm_per_v"),
            mass_kg=required(row, "mass_g", KILOGRAMS_PER_GRAM),
            resistance_ohm=number(row, "resistance_ohm"),
            no_load_current_a=number(row, "no_load_current_a"),
            no_load_test_voltage_v=number(row, "no_load_test_voltage_v"),
            max_current_a=max_current_a,
            shaft_diameter_m=number(row, "shaft_mm", METERS_PER_MILLIMETER),
            price_usd=number(row, "price_usd"),
        )
        estimated_fields = motor.estimated_fields + (("max current from max power",) if from_max_power else ())
        motors.append(
            replace(
                motor,
                max_cell_count=None if max_cells is None else int(max_cells),
                estimated_fields=estimated_fields,
            )
        )
    return tuple(motors)


def load_propellers(
    path: Path,
    motors: tuple[Motor, ...],
    tech: Technology,
    scaling: PropellerScaling | None,
) -> tuple[Propeller, ...]:
    """Columns: name, diameter_mm, mass_g, then bore_mm, price_usd, and the coefficients from one of:
        thrust_coefficient and power_coefficient, given directly
        test_motor (a catalog motor), test_voltage_v, test_current_a, test_thrust_g (and test_rpm):
            one full-throttle row of a thrust table
        pitch_mm and blades: the fitted rule
    """
    motors_by_name = {motor.name: motor for motor in motors}
    propellers = []

    for row in read_rows(path):
        listing = {
            "name": row["name"],
            "diameter_m": required(row, "diameter_mm", METERS_PER_MILLIMETER),
            "mass_kg": required(row, "mass_g", KILOGRAMS_PER_GRAM),
            "bore_diameter_m": number(row, "bore_mm", METERS_PER_MILLIMETER),
            "price_usd": number(row, "price_usd"),
        }

        thrust_coefficient = number(row, "thrust_coefficient")
        power_coefficient = number(row, "power_coefficient")
        if thrust_coefficient is not None and power_coefficient is not None:
            # A "coefficients_from_model" entry marks coefficients that came from a prop model
            # rather than a measurement, so they get the wider error bars.
            propellers.append(
                Propeller(
                    **listing,
                    thrust_coefficient=thrust_coefficient,
                    power_coefficient=power_coefficient,
                    estimated=row.get("coefficients_from_model") is not None,
                )
            )
        elif row.get("test_motor"):
            propellers.append(from_thrust_table_row(row, listing, motors_by_name, tech))
        elif scaling is not None and row.get("pitch_mm"):
            propellers.append(
                scaling.propeller(
                    **listing,
                    pitch_m=required(row, "pitch_mm", METERS_PER_MILLIMETER),
                    blade_count=int(number(row, "blades") or 2),
                )
            )
        else:
            raise ValueError(f"{row['name']}: give C_T and C_P, a thrust-table row, or a pitch (with scaling.json)")

    return tuple(propellers)


def from_thrust_table_row(row: Row, listing: dict, motors_by_name: dict[str, Motor], tech: Technology) -> Propeller:
    motor = motors_by_name.get(row["test_motor"])
    if motor is None:
        raise ValueError(f"{row['name']}: test_motor {row['test_motor']!r} isn't in the motor catalog")

    test_rpm = number(row, "test_rpm")
    point = motor.measured_point(
        voltage_v=required(row, "test_voltage_v"),
        current_a=required(row, "test_current_a"),
        speed_exponent=tech.no_load_current_speed_exponent,
        measured_speed_rad_s=None if test_rpm is None else test_rpm / RPM_PER_RAD_PER_S,
    )
    propeller = Propeller.from_test_point(
        **listing,
        thrust_n=required(row, "test_thrust_g", KILOGRAMS_PER_GRAM) * GRAVITY_M_PER_S2,
        speed_rad_s=point.speed_rad_s,
        shaft_power_w=point.shaft_power_w,
    )
    # Coefficients from a motor with estimated numbers inherit that uncertainty.
    return replace(propeller, estimated=bool(motor.estimated_fields))


def load_batteries(path: Path) -> tuple[BatteryPack, ...]:
    """Columns: name, cells, capacity_mah, mass_g (with connector), continuous_c, burst_c, price_usd.
    Optional: chemistry (lipo, lihv or liion; LiPo if blank) and resistance_mohm if the pack's
    internal resistance has been measured."""
    return tuple(
        BatteryPack(
            name=row["name"],
            cell_count=int(required(row, "cells")),
            capacity_ah=required(row, "capacity_mah", AMP_HOURS_PER_MILLIAMP_HOUR),
            mass_kg=required(row, "mass_g", KILOGRAMS_PER_GRAM),
            chemistry=chemistry(row),
            continuous_discharge_c=required(row, "continuous_c"),
            burst_discharge_c=required(row, "burst_c"),
            internal_resistance_ohm=number(row, "resistance_mohm", OHMS_PER_MILLIOHM),
            price_usd=number(row, "price_usd"),
        )
        for row in read_rows(path)
    )


def chemistry(row: Row):
    name = row.get("chemistry")
    if name is None:
        return LIPO
    key = name.lower().replace("-", "").replace(" ", "")
    if key not in CHEMISTRIES:
        raise ValueError(f"{row['name']}: chemistry {name!r} isn't one of {', '.join(CHEMISTRIES)}")
    return CHEMISTRIES[key]
