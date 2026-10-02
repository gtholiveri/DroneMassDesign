"""Turn sizing results into readable text. Unit conversion for humans happens here, not in the physics."""

from drone_sizing.closure import ClosureResult
from drone_sizing.constants import GRAVITY_M_PER_S2, RPM_PER_RAD_PER_S

GRAMS_PER_KG = 1000.0
SECONDS_PER_MINUTE = 60.0


def format_mass_history(mass_history_kg: list[float]) -> str:
    """Show the sequence of guesses, shortened in the middle if it's long."""
    masses_g = [f"{mass_kg * GRAMS_PER_KG:.0f}" for mass_kg in mass_history_kg]
    if len(masses_g) > 8:
        masses_g = masses_g[:5] + ["..."] + masses_g[-2:]
    return " -> ".join(masses_g) + " g"


def format_report(result: ClosureResult) -> str:
    design = result.design
    total_mass_kg = design.built_mass_kg

    lines = [
        f"Consistent design: {total_mass_kg * GRAMS_PER_KG:.0f} g "
        f"after {len(result.mass_history_kg)} guesses",
        f"Guesses: {format_mass_history(result.mass_history_kg)}",
        "",
        "Mass breakdown",
    ]
    for part, mass_kg in design.mass_breakdown_kg.items():
        share = mass_kg / total_mass_kg
        lines.append(f"  {part:<10} {mass_kg * GRAMS_PER_KG:7.0f} g  {share:6.1%}")

    rotor = design.rotor
    propeller = rotor.propeller
    motor = design.motor
    hover_thrust_g = rotor.hover_thrust_n / GRAVITY_M_PER_S2 * GRAMS_PER_KG
    max_thrust_g = rotor.max_thrust_n / GRAVITY_M_PER_S2 * GRAMS_PER_KG
    hover_rpm = rotor.hover_speed_rad_s * RPM_PER_RAD_PER_S
    max_rpm = rotor.max_speed_rad_s * RPM_PER_RAD_PER_S
    flight_time_min = design.requirements.flight_time_s / SECONDS_PER_MINUTE

    lines += [
        "",
        f"Rotor ({propeller.name}, figure of merit {propeller.figure_of_merit:.2f})",
        f"  Thrust          {hover_thrust_g:6.0f} g     hover  {max_thrust_g:6.0f} g     max",
        f"  Speed           {hover_rpm:6.0f} RPM   hover  {max_rpm:6.0f} RPM   max",
        f"  Torque          {rotor.hover_torque_nm:6.3f} Nm    hover  {rotor.max_torque_nm:6.3f} Nm    max",
        f"  Shaft power     {rotor.hover_shaft_power_w:6.1f} W     hover  {rotor.max_shaft_power_w:6.1f} W     max",
        f"  Disk loading    {rotor.hover_thrust_n / propeller.disk_area_m2:.1f} N/m^2",
        "",
        "Motor to shop for",
        f"  About {motor.required_kv_rpm_per_v:.0f} Kv on {design.choices.cell_count}S, "
        f"about {motor.mass_kg * GRAMS_PER_KG:.0f} g each",
        "",
        "Power and battery",
        f"  Average battery power   {design.average_battery_power_w:.0f} W",
        f"  Power loading           {total_mass_kg * GRAMS_PER_KG / design.average_battery_power_w:.1f} g/W",
        f"  Battery                 {design.battery.energy_wh:.1f} Wh for {flight_time_min:.0f} min",
    ]
    return "\n".join(lines)
