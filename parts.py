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
    Batteries  cells, mAh, mass with connector, continuous and burst C, chemistry, price.

select_parts.py tries every frame x motor x prop x battery. Entries marked TODO are placeholders.
"""

from pathlib import Path

from drone_sizing.airframe import Airframe, Component, ControllerBoard
from drone_sizing.catalog import (
    load_batteries,
    load_fitted_scaling,
    load_motor_scaling,
    load_motors,
    load_propellers,
)
from drone_sizing.constants import KILOGRAMS_PER_GRAM, METERS_PER_MILLIMETER
from drone_sizing.typical import TypicalFrame
from scenario import TECHNOLOGY

GRAM = KILOGRAMS_PER_GRAM
MILLIMETER = METERS_PER_MILLIMETER
CATALOG = Path(__file__).parent / "catalog"

# The printed frame. TODO: masses from the slicer. Until then a frame's mass grows in proportion to
# its motor-to-motor diagonal from a guessed 6 g at 100 mm, as if the arms dominate. Prop tips of
# neighboring rotors must clear each other by 8 mm, guards included (also a guess).
TYPICAL_FRAME = TypicalFrame(rotor_count=4, mass_kg_per_m=6.0 * GRAM / (100 * MILLIMETER), prop_clearance_m=8 * MILLIMETER)

# Frame sizes for the catalog search to try, as motor-to-motor diagonals.
FRAMES = tuple(TYPICAL_FRAME.of_diagonal(diagonal_mm * MILLIMETER) for diagonal_mm in (90, 110, 130))

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
    full_power_w=BOARD_POWER_W,
    esc_max_current_a=12.0,
    esc_efficiency=ESC_EFFICIENCY,
    supported_cell_counts=(1, 2),
)

# The lightest and cheapest kind, but its F411 processor only runs Betaflight: no autonomous flight.
BETAFPV_F4_1S_5A = ControllerBoard(
    name="BETAFPV F4 1S 5A AIO",
    mass_kg=3.0 * GRAM,
    full_power_w=BOARD_POWER_W,
    esc_max_current_a=5.0,
    esc_efficiency=ESC_EFFICIENCY,
    supported_cell_counts=(1,),
)

# Runs the Crazyflie firmware and takes Crazyflie decks, but costs about $205 and needs separate
# ESCs: 5.4 g for the board, plus a guessed 3 g and 5 A for a small 4-in-1 ESC.
CRAZYFLIE_BOLT = ControllerBoard(
    name="Crazyflie Bolt 1.1 + separate ESC",
    mass_kg=(5.4 + 3.0) * GRAM,
    full_power_w=BOARD_POWER_W,
    esc_max_current_a=5.0,
    esc_efficiency=ESC_EFFICIENCY,
    supported_cell_counts=(1, 2, 3, 4),
)

# The team's pick: the cheapest 1S board in stock that the Skybrush ArduCopter fork has a board
# definition for (CrazyF405). 4.8 g without its power lead, 12 A per motor (15 A for 3 s), runs
# on 2.9 to 8.7 V, about $60 (happymodel.cn, racedayquads.com).
HAPPYMODEL_CRAZYF405HD = ControllerBoard(
    name="Happymodel CrazyF405HD ELRS 1-2S AIO",
    mass_kg=4.8 * GRAM,
    full_power_w=BOARD_POWER_W,
    esc_max_current_a=12.0,
    esc_efficiency=ESC_EFFICIENCY,
    supported_cell_counts=(1, 2),
)

BOARD = HAPPYMODEL_CRAZYF405HD

# TODO: confirm which module this is. 1.4 g matches a Qorvo DWM1000, whose datasheet gives 160 mA
# at 3.3 V while receiving and 140 mA while transmitting. A tag that listens all the time (as
# TDoA positioning does) therefore draws about 0.5 W; one that mostly sleeps draws far less.
UWB = Component(
    name="UWB module",
    mass_kg=1.4 * GRAM,
    full_power_w=0.160 * 3.3,
)

# Estimated from Bitcraze's bottom-mounted Color LED deck: 3.5 g with its diffuser, and one WRGB
# LED at up to about 300 mA per channel, through a DC/DC driver.
LED_POWER_PER_CHANNEL_W = 0.300 * 3.0 / 0.90  # 300 mA at about 3 V, through a ~90% driver: 1.0 W
LED_CHANNELS = 4
LED = Component(
    name="LED module",
    mass_kg=3.5 * GRAM,
    full_power_w=LED_CHANNELS * LED_POWER_PER_CHANNEL_W,  # white at full brightness: every channel on
    # TODO: how it will be lit. 0.80 is white at 80% average brightness. A single pure color at
    # 80% lights one channel of the four, which is 0.20.
    duty_cycle=0.80,
)

# Other LED modules to compare against, for compare.py and flight_time.py. TODO: these two are
# made up to show the pattern. Replace them with real candidates: a module is its mass, what it
# draws when fully on, and the fraction of that it draws on average.
EXAMPLE_SMALL_LED = Component(name="example: small LED", mass_kg=1.0 * GRAM, full_power_w=1.0, duty_cycle=0.80)
EXAMPLE_BRIGHT_LED = Component(name="example: bright LED", mass_kg=5.0 * GRAM, full_power_w=8.0, duty_cycle=0.80)

# What every drone carries, whatever its frame, motors, props and battery.
FIXED_PARTS = (BOARD, UWB, LED)

AIRFRAMES = tuple(Airframe(rotor_count=TYPICAL_FRAME.rotor_count, frame=frame, components=FIXED_PARTS) for frame in FRAMES)

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
