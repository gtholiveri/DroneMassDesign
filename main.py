"""Continuous sizing: for each prop in the catalog, the ideal motor and battery and the drone they make.

Use this to decide what to shop for: Kv, K_m and pack size. select_parts.py picks the actual parts.
Runs once per thrust-to-weight ratio in scenario.py.
"""

from dataclasses import replace

from drone_sizing.airframe import Airframe
from drone_sizing.closure import ClosureResult, MassDidNotConverge, close_mass
from drone_sizing.inputs import DesignChoices, ScalingLaws
from drone_sizing.propeller import Propeller
from drone_sizing.report import format_designs, format_kv_windows, format_report, table
from parts import AIRFRAMES, BATTERIES, MOTOR_SCALING, PROPELLERS
from scenario import REQUIREMENTS, TECHNOLOGY, THRUST_TO_WEIGHT_OPTIONS

CELL_COUNT = 1  # TODO: one the board supports

# Which thrust-to-weight ratio gets the full report (the others get one line per design).
FULL_REPORT_THRUST_TO_WEIGHT = 1.75

# Motor masses and rated currents to show Kv ranges for: a shopping guide for listings that give
# only Kv, mass and max current.
KV_WINDOW_MASSES_KG = [0.002, 0.0025, 0.003, 0.004, 0.005]
KV_WINDOW_CURRENTS_A = [2.0, 3.0, 5.0, 8.0]


def average_battery_specific_energy_j_per_kg() -> float:
    """Energy per kilogram, averaged over the catalog's packs."""
    specific_energies = [
        battery.energy_j(TECHNOLOGY.nominal_cell_voltage_v) / battery.mass_kg for battery in BATTERIES
    ]
    return sum(specific_energies) / len(specific_energies)


def lightest_airframe_for(propeller: Propeller) -> Airframe | None:
    """The lightest airframe the prop fits on, or None if it fits none."""
    fitting = [airframe for airframe in AIRFRAMES if airframe.fits_prop(propeller.diameter_m)]
    return min(fitting, key=lambda airframe: airframe.fixed_mass_kg, default=None)


def main() -> None:
    scaling = ScalingLaws(
        battery_specific_energy_j_per_kg=average_battery_specific_energy_j_per_kg(),
        motor=MOTOR_SCALING,
    )

    print("How motor properties grow with motor mass m (same-shape scaling predicts 0.83, 1.00, 0.67)")
    print(
        table(
            ["property", "grows as"],
            [
                ["K_m", f"m^{MOTOR_SCALING.motor_constant.exponent:.2f}"],
                ["drag torque", f"m^{MOTOR_SCALING.drag_torque.exponent:.2f}"],
                ["max copper loss", f"m^{MOTOR_SCALING.max_copper_loss.exponent:.2f}"],
            ],
            "lr",
        )
    )

    full_report: ClosureResult | None = None
    for thrust_to_weight in THRUST_TO_WEIGHT_OPTIONS:
        requirements = replace(REQUIREMENTS, thrust_to_weight=thrust_to_weight)
        print()
        print(f"Required thrust-to-weight {thrust_to_weight}: one design per prop, on the lightest frame it fits")

        results: list[ClosureResult] = []
        for propeller in PROPELLERS:
            airframe = lightest_airframe_for(propeller)
            if airframe is None:
                print(f"  {propeller.name}: fits none of the frames")
                continue
            choices = DesignChoices(airframe=airframe, propeller=propeller, cell_count=CELL_COUNT)
            try:
                results.append(close_mass(requirements, choices, TECHNOLOGY, scaling))
            except MassDidNotConverge:
                print(f"  {propeller.name} on the {airframe.frame.name}: no design can fly this long")
        if results:
            print(format_designs(results))

        if results and thrust_to_weight == FULL_REPORT_THRUST_TO_WEIGHT:
            full_report = min(results, key=lambda result: result.design.built_mass_kg)

    if full_report is not None:
        print()
        print(f"Lightest design at thrust-to-weight {FULL_REPORT_THRUST_TO_WEIGHT}, in full")
        print(format_report(full_report))
        print()
        print("Kv that works for this design, by motor mass and rated max current")
        print(format_kv_windows(kv_windows(full_report), KV_WINDOW_MASSES_KG, KV_WINDOW_CURRENTS_A))


def kv_windows(result: ClosureResult) -> dict[tuple[float, float], tuple[float, float] | None]:
    """For the design's prop and max point, the Kv range that works for each motor mass and current rating."""
    design = result.design
    cell_count = design.choices.cell_count
    return {
        (mass_kg, max_current_a): MOTOR_SCALING.kv_window(
            mass_kg,
            max_current_a,
            design.choices.propeller,
            design.rotor.max_torque_nm,
            design.rotor.max_speed_rad_s,
            cell_count * TECHNOLOGY.loaded_cell_voltage_v,
            cell_count * TECHNOLOGY.fresh_loaded_cell_voltage_v,
            TECHNOLOGY.no_load_current_speed_exponent,
        )
        for mass_kg in KV_WINDOW_MASSES_KG
        for max_current_a in KV_WINDOW_CURRENTS_A
    }


if __name__ == "__main__":
    main()
