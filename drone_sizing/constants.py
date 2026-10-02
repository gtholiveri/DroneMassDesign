"""Physical constants and unit conversions. Everything inside the package is SI."""

import math

GRAVITY_M_PER_S2 = 9.81

# Sea level at 15 °C. Lower this to size for altitude or hot days.
AIR_DENSITY_KG_PER_M3 = 1.225

JOULES_PER_WATT_HOUR = 3600.0
METERS_PER_INCH = 0.0254
RPM_PER_RAD_PER_S = 60 / (2 * math.pi)
