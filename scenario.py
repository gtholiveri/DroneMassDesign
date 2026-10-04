"""What we need the drone to do, and what we assume about hardware. Shared by every script."""

from drone_sizing.battery import LI_ION, LIHV, BatteryKind
from drone_sizing.build import Uncertainty
from drone_sizing.inputs import Requirements, Technology

REQUIREMENTS = Requirements(
    payload_mass_kg=0.0,  # nothing carried: the UWB and LED modules are airframe parts in parts.py
    flight_time_s=20 * 60,
    thrust_to_weight=2.0,  # main.py and explore.py size the drone for this
    average_power_factor=1.2,
    mass_margin_fraction=0.15,
)

# select_parts.py searches the catalog once per ratio, to show what each level of thrust costs.
THRUST_TO_WEIGHT_OPTIONS = (1.5, 1.75, 2.0)

TECHNOLOGY = Technology(
    usable_battery_fraction=0.8,
    lead_resistance_ohm=0.015,  # about 15 milliohm for leads and a small connector. TODO: measure
    no_load_current_speed_exponent=0.5,  # matches the Crazyflie best; see validate.py
)

# Kinds of battery a pack could be built from. Energy per kilogram is capacity x nominal voltage /
# mass, from the listings or datasheet named. What the cells themselves do (voltages, resistance)
# is their chemistry, defined in drone_sizing/battery.py.
#
# A pack's C rating is the most current it can supply, as a multiple of its capacity: a 1 Ah pack
# rated 60C can give 60 A. Cells built for a higher C rating hold less energy per gram, which is
# why the 100C packs (GNB 550 mAh at 13.5 g, 720 mAh at 18.5 g) are worse here than the 60C ones
# (GNB 850 mAh at 17.5 g, 1100 mAh at 22 g). Our drone draws under 10C, so either is plenty.
LIHV_POUCH_100C = BatteryKind("1S LiHV pouch pack, 100C cells", 1, 150, LIHV)
LIHV_POUCH_60C = BatteryKind("1S LiHV pouch pack, 60C cells", 1, 185, LIHV)
# Two cells in series carry a second lead and more wrap: the GNB 2S 850 mAh pack is 38 g.
LIHV_POUCH_60C_2S = BatteryKind("2S LiHV pouch pack, 60C cells", 2, 170, LIHV)

# Cylindrical Li-ion cells, with 3 g added for leads. Molicel P28A: 2,800 mAh in 46 g.
LI_ION_18650 = BatteryKind("1S Li-ion 18650 cell", 1, 205, LI_ION)
# Samsung 50S: 5,000 mAh in 72.7 g.
LI_ION_21700 = BatteryKind("1S Li-ion 21700 cell", 1, 235, LI_ION)

# explore.py makes one table per kind, and main.py compares the best drone of each.
BATTERY_KINDS = (LIHV_POUCH_100C, LIHV_POUCH_60C, LIHV_POUCH_60C_2S, LI_ION_18650, LI_ION_21700)

# The kind main.py sizes the drone around and lays out in full.
BATTERY = LIHV_POUCH_60C

# How far the least certain catalog numbers might be off. select_parts.py re-checks every build
# with all of these at once, to see which builds still work if the numbers are optimistic.
UNCERTAINTY = Uncertainty(
    # Numbers from a datasheet or a thrust table.
    thrust_coefficient=0.10,
    power_coefficient=0.10,
    no_load_current=0.30,
    battery_capacity=0.10,
    battery_resistance=0.50,  # pack resistance is a rule of thumb here, so a wide allowance
    # A prop estimated from the fitted rule (tools/fit_props.py). C_T scatters x/÷ 1.21 and C_P
    # x/÷ 1.31, but they miss together (a wide blade raises both), so the figure of merit only
    # scatters x/÷ 1.18. These two put thrust one standard deviation low and the figure of merit
    # about two low (0.83^1.5 / 1.10 = 0.69).
    estimated_thrust_coefficient=0.17,
    estimated_power_coefficient=0.10,
    # A motor estimated from the fitted rules: one standard deviation of each fit's scatter, as
    # printed by tools/fit_tyto.py (x/÷ 1.44 for K_m, x/÷ 1.95 for drag).
    estimated_motor_constant=0.31,
    estimated_no_load_current=0.95,
)
