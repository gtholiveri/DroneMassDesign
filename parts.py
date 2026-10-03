"""Everything we might put on the drone. Edit this file and the CSVs in catalog/; the scripts read both.

The fixed parts (frames, board, modules) are here. Candidate motors, props and batteries live in
catalog/motors.csv, catalog/propellers.csv and catalog/batteries.csv, in datasheet units (g, mm,
mAh), so they can be kept in a spreadsheet and exported. drone_sizing/catalog.py documents the
columns. Blank cells mean "not known":
    Motors     what listings give: Kv, mass, max current (or max power), max cells, shaft, price.
               Resistance and no-load current are estimated from rules fitted to thrust-stand
               data (catalog/scaling.json); fill them in if a datasheet happens to list them.
    Props      diameter, pitch, blades, mass, bore, price. C_T and C_P come from the fitted rule,
               or from one full-throttle thrust-table row on a catalog motor if you have one.
    Batteries  cells, mAh, mass with connector, continuous and burst C, price.

select_parts.py tries every frame x motor x prop x battery. Entries marked TODO are placeholders.
"""

from pathlib import Path

from drone_sizing.airframe import Airframe, Component, ControllerBoard, Frame
from drone_sizing.catalog import (
    load_batteries,
    load_fitted_scaling,
    load_motor_scaling,
    load_motors,
    load_propellers,
)
from drone_sizing.constants import KILOGRAMS_PER_GRAM, METERS_PER_MILLIMETER
from scenario import TECHNOLOGY

GRAM = KILOGRAMS_PER_GRAM
MILLIMETER = METERS_PER_MILLIMETER
CATALOG = Path(__file__).parent / "catalog"

ROTOR_COUNT = 4

# Prop tips of neighboring rotors must clear each other by this much, guards included.
PROP_CLEARANCE_M = 8 * MILLIMETER  # TODO


def printed_frame(diagonal_mm: float, mass_g: float) -> Frame:
    return Frame(
        name=f"{diagonal_mm:.0f} mm frame",
        diagonal_m=diagonal_mm * MILLIMETER,
        mass_kg=mass_g * GRAM,
        prop_clearance_m=PROP_CLEARANCE_M,
    )


# Frame sizes to try, as motor-to-motor diagonals. TODO: masses from the slicer. Until then they
# grow in proportion to the diagonal from a guessed 6 g at 100 mm, as if the arms dominate.
FRAMES = tuple(printed_frame(diagonal_mm, 6.0 * diagonal_mm / 100) for diagonal_mm in (90, 110, 130))

# Candidate controller boards with the ESCs built in. Mass, ESC rating and cells are from the
# listings. Nobody publishes board power draw or ESC efficiency, so those two are guesses:
# about 0.1 A for the processor, radio and sensors, and 90% for the ESC (0.85 to 0.95 is typical).
BOARD_POWER_W = 0.40  # TODO: measure
ESC_EFFICIENCY = 0.90  # TODO: measure

# Runs ArduPilot (which can fly on UWB beacons) as well as Betaflight. Has a barometer, a spare
# UART and I2C pads. 4.9 g and about $63 (ardupilot.org board page, flywoo.net).
FLYWOO_GOKU_F405_HD = ControllerBoard(
    name="Flywoo GOKU F405 HD 1-2S 12A AIO V2",
    mass_kg=4.9 * GRAM,
    power_w=BOARD_POWER_W,
    esc_max_current_a=12.0,
    esc_efficiency=ESC_EFFICIENCY,
    supported_cell_counts=(1, 2),
)

# The lightest and cheapest kind, but its F411 processor only runs Betaflight: no autonomous flight.
BETAFPV_F4_1S_5A = ControllerBoard(
    name="BETAFPV F4 1S 5A AIO",
    mass_kg=3.0 * GRAM,
    power_w=BOARD_POWER_W,
    esc_max_current_a=5.0,
    esc_efficiency=ESC_EFFICIENCY,
    supported_cell_counts=(1,),
)

# Runs the Crazyflie firmware and takes Crazyflie decks, but costs about $205 and needs separate
# ESCs: 5.4 g for the board, plus a guessed 3 g and 5 A for a small 4-in-1 ESC.
CRAZYFLIE_BOLT = ControllerBoard(
    name="Crazyflie Bolt 1.1 + separate ESC",
    mass_kg=(5.4 + 3.0) * GRAM,
    power_w=BOARD_POWER_W,
    esc_max_current_a=5.0,
    esc_efficiency=ESC_EFFICIENCY,
    supported_cell_counts=(1, 2, 3, 4),
)

BOARD = FLYWOO_GOKU_F405_HD  # TODO: decide; this choice follows from the firmware you'll run

# TODO: confirm which module this is. 1.4 g matches a Qorvo DWM1000, whose datasheet gives 160 mA
# at 3.3 V while receiving and 140 mA while transmitting. A tag that listens all the time (as
# TDoA positioning does) therefore draws about 0.5 W; one that mostly sleeps draws far less.
UWB = Component(
    name="UWB module",
    mass_kg=1.4 * GRAM,
    power_w=0.160 * 3.3,
)

# Estimated from Bitcraze's bottom-mounted Color LED deck: 3.5 g with its diffuser, and one WRGB
# LED at up to about 300 mA per channel, through a DC/DC driver.
LED_POWER_PER_CHANNEL_W = 0.300 * 3.0 / 0.90  # 300 mA at about 3 V, through a ~90% driver: 1.0 W
LED_CHANNELS_LIT = 4  # TODO: 4 for white; 1 for a pure color, 2 for a mix like yellow or cyan
LED_AVERAGE_BRIGHTNESS = 0.80
LED = Component(
    name="LED module",
    mass_kg=3.5 * GRAM,
    power_w=LED_POWER_PER_CHANNEL_W * LED_CHANNELS_LIT * LED_AVERAGE_BRIGHTNESS,
)

AIRFRAMES = tuple(
    Airframe(rotor_count=ROTOR_COUNT, frame=frame, board=BOARD, components=(UWB, LED)) for frame in FRAMES
)

# Rules for numbers listings leave out: motors from the Tyto thrust-stand database
# (tools/fit_tyto.py), props from UIUC's static tests of small props (tools/fit_props.py).
FITTED_SCALING = load_fitted_scaling(CATALOG / "scaling.json", TECHNOLOGY)
MOTOR_SCALING = load_motor_scaling(CATALOG / "motors.csv", TECHNOLOGY, FITTED_SCALING)

MOTORS = load_motors(CATALOG / "motors.csv", TECHNOLOGY, MOTOR_SCALING)
PROPELLERS = load_propellers(
    CATALOG / "propellers.csv",
    MOTORS,
    TECHNOLOGY,
    None if FITTED_SCALING is None else FITTED_SCALING.propeller,
)
BATTERIES = load_batteries(CATALOG / "batteries.csv")
