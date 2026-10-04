"""Take one drone and fly it on each real battery in the catalog.

The drone is the one main.py finds for the scenario: its frame, props and motors are then held
fixed. The model sized its battery to the gram, and no such pack is on sale, so here that battery
is swapped for each catalog pack with the same cell count and chemistry (catalog/batteries.csv)
and the drone is flown again. The first row is the sized battery itself, for reference.

The motors stay wound for the sized battery. A real pack that is heavier, or that sags more under
load, therefore shows up as less thrust-to-weight, and one that can't supply the current fails a check.

To compare more batteries, add rows to catalog/batteries.csv.
"""

import math
from dataclasses import replace

from drone_sizing.battery import BatteryPack
from drone_sizing.build import BuildResult
from drone_sizing.report import grams, milliamp_hours, minutes, motor_summary, price, table, watts
from models import BASELINE, best_drone
from parts import BATTERIES
from scenario import TECHNOLOGY


# The two requirements have columns of their own, so the last column lists only the limits broken.
REQUIREMENT_CHECKS = ("flight time", "thrust-to-weight, tired pack")


def limits_broken(flight: BuildResult) -> str:
    return ", ".join(check.name for check in flight.checks if not check.passed and check.name not in REQUIREMENT_CHECKS)


def share_lost_in_pack(pack: BatteryPack, flight: BuildResult) -> float:
    """The share of the pack's energy that heats its own resistance at the average current: I R / V.

    The flight time does not subtract this. It matters most for cells built for capacity, whose
    resistance is high.
    """
    average_current_a = flight.average_battery_power_w / pack.nominal_voltage_v
    return average_current_a * pack.resistance_ohm(TECHNOLOGY.lead_resistance_ohm) / pack.nominal_voltage_v


def row(pack: BatteryPack, flight: BuildResult) -> list[str]:
    tired = flight.tired_full_throttle
    peak = f"{flight.peak_battery_current_a:.1f} A"
    if math.isfinite(pack.max_burst_current_a):
        peak += f" of {pack.max_burst_current_a:.0f} A"
    return [
        pack.name,
        milliamp_hours(pack.capacity_ah),
        grams(pack.mass_kg),
        f"{pack.energy_wh / pack.mass_kg:.0f} Wh/kg",
        price(pack.price_usd),
        grams(flight.total_mass_kg),
        watts(flight.average_battery_power_w),
        minutes(flight.flight_time_s),
        minutes(flight.best_estimate.flight_time_s),
        f"{share_lost_in_pack(pack, flight):.0%}",
        f"{flight.thrust_to_weight:.2f}",
        f"{tired.voltage_v / pack.cell_count:.2f} V",
        peak,
        limits_broken(flight),
    ]


def main() -> None:
    design = best_drone(BASELINE)
    if design is None:
        raise SystemExit("No drone meets the baseline scenario, so there is none to fly. Run main.py for details.")
    drone = design.build
    sized = replace(design.battery, name="sized by the model")

    packs = [sized] + [
        pack for pack in BATTERIES if pack.cell_count == sized.cell_count and pack.chemistry == sized.chemistry
    ]
    flights = [replace(drone, battery=pack).evaluate(BASELINE.requirements, TECHNOLOGY) for pack in packs]

    print("The same drone flown on each battery. Only the battery changes.")
    print(
        f"The drone is the best one for the baseline scenario: {design.choices.propeller.name} props, "
        f"{motor_summary(design)} motors, {grams(design.choices.airframe.frame.mass_kg)} frame, "
        f"{BASELINE.board.name}."
    )
    print()

    headers = [
        "battery", "capacity", "mass", "energy", "price",
        "total mass", "average draw", "flight time", "best estimate", "lost in the pack", "TWR with sag",
        "lowest cell voltage", "peak current", "limits broken",
    ]  # fmt: skip
    print(table(headers, [row(pack, flight) for pack, flight in zip(packs, flights)], "lrrrrrrrrrrrrl"))
    print("flight time:    as planned, with the mass margin, the maneuvering allowance and the landing reserve")
    print("best estimate:  no mass margin, steady hover, the whole pack")
    print("lost in the pack: heat in the pack's own resistance at the average draw, as a share of its energy. Not subtracted from the flight times")
    print("TWR with sag, lowest cell voltage: at full throttle late in the flight. peak current: on a fresh pack, against the pack's burst rating")


if __name__ == "__main__":
    main()
