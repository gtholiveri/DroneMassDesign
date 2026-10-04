# Drone Mass Design

A sizing tool for small quadcopters (roughly 40 to 150 g). It answers one question:

> Given the parts a drone must carry and the power they draw, does a drone exist that flies for
> the time we want, and what does it look like?

It then helps turn that answer into parts you can buy. It was built for an indoor swarm drone in
the Crazyflie class that carries a UWB positioning module and an LED module, but nothing in it is
specific to that drone.

## Setup

Developed on Python 3.14. The only dependency is `tabulate`, for the output tables.

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows; on macOS or Linux: source .venv/bin/activate
pip install -r requirements.txt
```

Run every script from the project folder.

## Which script answers which question

| Question | Script |
|---|---|
| What is the best drone for our scenario, laid out in full? | `python main.py` |
| Same drone, different LED, board or flying style: how much does its flight time move? | `python flight_time.py` |
| Same drone, flown on each real battery in the catalog: which cell should we buy? | `python batteries.py` |
| How would the best drone itself change if the LED, the board, the flight time or the battery changed? | `python compare.py` |
| Which prop sizes and flight times are possible at all, for each kind of battery? | `python explore.py` |
| I have one drone in mind: how long does it fly, or what battery does it need? | `python calc.py ...` |
| Which real parts should we buy? | `python select_parts.py` |
| Does the model match a real drone? | `python validate.py` |

## The best drone for the scenario: `main.py`

Takes the scenario (`scenario.py`) and the parts the drone must carry (`parts.py`), builds a drone
out of typical parts for prop sizes from 40 to 130 mm, and lays out the lightest one. No real
listings are involved: the props, motors, frame and battery come from the component models, so the
result says what to look for, not what to buy.

It prints, in order:

- **Scenario, fixed parts, component models**: every requirement, margin and assumption that
  went in.
- **Mass**: each group of parts and the margin, with the total.
- **Power**: the average draw split into the power that lifts the drone, each loss on the way
  from the battery to the air, the maneuvering allowance and each electronic part, with the total.
- **Battery**: energy used and stored, capacity, mass, average and peak current.
- **Rotors**: thrust, RPM, motor current and voltage at hover and at full throttle.
- **Parts to look for**: the frame to print, and the prop, motor and battery to search for.
- **Other prop sizes**: the drone each size would make, to show how sharp the optimum is.
- **Other kinds of battery**: the best drone each kind in `scenario.py` would make.

If no drone can meet the scenario, it says so instead.

The motor is given the way listings name motors: by stator size and Kv. A "1203" has a stator
12 mm across and 3 mm tall. Motors of one size weigh nearly the same whoever makes them, so the
report turns the motor mass the model wants into the sizes that weigh about that much.

The kind of battery is an input (`BATTERY` in `scenario.py`), not something the script chooses.
Left to choose, it would always take the kind with the most energy per kilogram, even at sizes
where no such cell is sold.

## Two ways to compare: `flight_time.py` and `compare.py`

Both take a list of variations of the scenario and print one row per variation. They answer
different questions.

**`flight_time.py` holds the drone fixed.** It takes the drone `main.py` finds, keeps its frame,
props, motors and battery, and flies that same drone under each variation: another LED module or
duty cycle, another controller board, a different average power factor or mass margin. The table
gives the flight time and its change from the baseline. Only the thing the row names changes, so
this is the one to use for "what does this choice cost us".

**`compare.py` re-sizes the drone.** For each variation it finds the best drone from scratch, as
`main.py` would, and shows how the drone itself changes: total mass, prop, frame, battery, motor.
The flight time is the same in every row, because each drone is sized to meet it.

In both, the variations are the `VARIANTS` list at the top of the file. Each is the baseline with
something replaced. The baseline carries the fixed parts `(BOARD, UWB, LED)` from `parts.py`, and
a fixed part is a `Component`: a name, a mass, the power it draws when fully on, and a duty cycle
(the fraction of that power it draws on average).

```python
# Another LED module: describe it, and put it where the LED was.
OTHER_LED = Component(name="other LED", mass_kg=1.0 * GRAM, full_power_w=1.0, duty_cycle=0.80)
replace(BASELINE, name="other LED module", components=(BOARD, UWB, OTHER_LED)),

# The same LED lit less.
replace(BASELINE, name="LED at 40%", components=(BOARD, UWB, replace(LED, duty_cycle=0.40))),

# Another controller board.
replace(BASELINE, name="BETAFPV board", components=(BETAFPV_F4_1S_5A, UWB, LED)),

# Flown harder (flight_time.py), or a shorter flight or another battery (compare.py).
replace(BASELINE, name="flown hard", requirements=replace(REQUIREMENTS, average_power_factor=1.4)),
replace(BASELINE, name="15 min", requirements=replace(REQUIREMENTS, flight_time_s=15 * 60)),
replace(BASELINE, name="Li-ion", battery=LI_ION_18650),
```

## The calculator: `calc.py`

Put in what you know about one drone and it gives flight time, hover current and throttle,
thrust-to-weight and peak currents. No files to edit.

Give the battery, get the flight time:

```bash
python calc.py --mass-without-battery-g 32 --electronics-w 4.13 --motor-kv 10000 --motor-mass-g 3.45 --prop-diameter-mm 65 --prop-pitch-mm 33 --battery-mah 1100 --battery-mass-g 22 --lihv
```

Give the flight time instead, get the smallest battery that reaches it:

```bash
python calc.py --mass-without-battery-g 32 --electronics-w 4.13 --motor-kv 10000 --motor-mass-g 3.45 --prop-diameter-mm 65 --prop-pitch-mm 33 --flight-min 15 --lihv
```

The inputs that matter most:

| Option | Meaning |
|---|---|
| `--mass-without-battery-g` | The whole drone except its battery: frame, board, modules, motors, props, wires |
| `--electronics-w` | Power drawn by everything that isn't a motor: board, UWB module, LEDs |
| `--motor-kv`, `--motor-mass-g` | From the motor listing. Resistance, no-load current and max current are estimated from mass if you leave them out |
| `--prop-diameter-mm` with `--prop-pitch-mm` | From the prop listing. Or give measured `--ct` and `--cp` |
| `--battery-mah` or `--flight-min` | One or the other |
| `--battery-mass-g`, `--lihv`, `--cells` | The pack. Mass is estimated from capacity if left out |
| `--power-factor`, `--mass-margin`, `--usable` | Margins. The defaults give a plain hover estimate using 80% of the pack. Add `--power-factor 1.2 --mass-margin 0.15` to plan the way the other scripts do |

`python calc.py --help` lists the rest.

### Example: what does the LED choice cost?

Two numbers describe an LED module to the calculator: its power goes into `--electronics-w`, and
its mass goes into `--mass-without-battery-g`. For our drone the other electronics draw 0.93 W (0.40 W for the
board and 0.53 W for the UWB module), so `--electronics-w` is 0.93 plus the average LED power.

This is one of our candidate drones (RCinPower 1003 10000 Kv motors with their listed numbers,
65 mm props, a 1100 mAh pack) with the planning margins on. Change `--electronics-w` and
`--mass-without-battery-g` and rerun:

```bash
python calc.py --mass-without-battery-g 32 --electronics-w 4.13 --motor-kv 10000 --motor-mass-g 3.45 --motor-resistance-ohm 0.162 --motor-no-load-a 0.8 --motor-no-load-v 5 --motor-max-a 11.5 --prop-diameter-mm 65 --prop-pitch-mm 33 --battery-mah 1100 --battery-mass-g 22 --lihv --power-factor 1.2 --mass-margin 0.15
```

The 32 g without the battery is the frame (6.6 g), board (4.9 g), UWB module (1.4 g), LED module
(3.5 g), four motors (13.8 g) and four props (1.8 g).

| Average LED power | `--electronics-w` | Flight time |
|---|---|---|
| 0.8 W | 1.73 | 13.9 min |
| 1.6 W | 2.53 | 13.2 min |
| 3.2 W | 4.13 | 11.9 min |

| Mass without battery | Flight time (LED at 3.2 W) |
|---|---|
| 30 g | 12.4 min |
| 32 g | 11.9 min |
| 34 g | 11.5 min |
| 36 g | 11.1 min |

So on this drone each watt of LED costs about 0.8 min and each gram about 0.2 min.

## The general study: `explore.py`

For each prop size and flight time, it sizes a typical prop, the typical motor for its mass and a
battery of a given energy per kilogram, and finds the mass at which the drone it designs is the
drone it builds. No real parts are involved. Each cell reads

```
93 g | 2828 mAh, 50.0 g | 3.0 g 8.3k
```

meaning total mass, battery capacity and mass, and the motor to look for (mass and Kv). A cell
reads `none` where no consistent drone exists: every gram of battery added costs more hover power
than the energy it brings.

There is one table per kind of battery, because energy per kilogram decides more than anything
else. The battery kinds are in `scenario.py`; the prop sizes and flight times are constants at the
top of the file.

## The parts search: `select_parts.py`

Tries every frame, motor, prop and battery in the catalog together, drops the combinations that
don't physically fit (prop too big for the frame, prop bore not matching the motor shaft, cell
count the board or motor can't take) and checks the rest:

- thrust-to-weight at full throttle on a tired pack
- flight time
- pack voltage at full throttle on a tired pack
- motor current against the motor's rating and the ESC's
- battery current against its burst and continuous ratings

How to read the output:

- **TWR with sag** is thrust-to-weight late in the flight, with the pack voltage pulled down by
  the current.
- A `*` after a part means some of its numbers came from a fitted rule, not from its own
  datasheet or test.
- Each build is evaluated three ways. **As designed** includes the margins in `scenario.py`.
  **Worst case** pushes every uncertain number the wrong way at once. **Best estimate** removes
  the margins: steady hover, exact mass, the whole pack.

## Where the inputs live

| File | What to edit there |
|---|---|
| `scenario.py` | Flight time, thrust-to-weight, margins, the kind of battery, how uncertain each kind of number is |
| `parts.py` | The printed frame (mass per length, prop clearance) and the fixed parts: controller board, UWB and LED modules, each with its mass, power when on and duty cycle |
| `models.py` | How a typical prop is made for `main.py`, `compare.py`, `flight_time.py` and `explore.py`: blade count, pitch, the allowance for hover speed, and the prop sizes tried |
| `catalog/motors.csv` | Candidate motors: Kv, mass, and whatever else the listing gives |
| `catalog/propellers.csv` | Candidate props: size, pitch, blades, mass, bore, and measured coefficients if known |
| `catalog/batteries.csv` | Candidate packs: cells, capacity, mass, C ratings, chemistry (lipo, lihv or liion) |
| `catalog/scaling.json` | The fitted rules. Generated; see below |
| `drone_sizing/battery.py` | What each cell chemistry does: nominal, full, tired and minimum voltages, and resistance per Ah |

Blank cells in the CSVs mean "not known" and are filled from the fitted rules. Rows whose name
starts with `#` are skipped. `drone_sizing/catalog.py` documents the columns.

## How the code is laid out

The package `drone_sizing/` is the model. It knows nothing about this project's parts or numbers,
and each layer only uses the ones below it:

| Layer | Modules | What they do |
|---|---|---|
| Physics of one part | `propeller`, `motor`, `battery`, `airframe` | Thrust and torque from speed; volts and amps from torque and speed; a pack's energy, voltage and sag; what the frame carries |
| One definite drone | `build` | How a drone of definite parts flies: hover, flight time, full throttle on a tired and a fresh pack, every check. Every script's numbers come through here |
| Sizing | `rotor`, `design`, `closure`, `typical` | Size a motor and battery for a design mass; find the mass that closes; do it over prop sizes with typical parts |
| Searching | `search`, `catalog` | Try every catalog combination; read the CSVs |
| Presentation | `report` | Tables. Reads results, computes nothing |

The project files at the top level (`scenario.py`, `parts.py`, `models.py`) hold this project's
numbers and hand them to the package. The scripts (`main.py` and the others) only choose what to
run and what to print.

Entries marked `TODO` in `parts.py` and `scenario.py` are placeholders.

## How the model works

**Props.** Thrust and power follow from two coefficients, with $\rho$ air density, $n$ speed in
revolutions per second and $D$ diameter:

$$T = C_T\,\rho\,n^2 D^4 \qquad P = C_P\,\rho\,n^3 D^5$$

**Motors.** The first-order DC motor model, with $k_v$ the speed constant, $R_m$ the winding
resistance, $K_t = 1/k_v$ the torque constant and $Q_0$ the drag torque from friction and iron loss:

$$V = \frac{\omega}{k_v} + I R_m \qquad Q = K_t I - Q_0(\omega)$$

The motor constant $K_m = K_t/\sqrt{R_m}$ sets copper loss and depends on the motor's size, not
its winding. That is what lets a motor be sized by mass alone.

**Battery.** Under load a pack delivers its resting voltage minus current times internal
resistance. Thrust is judged on a tired pack and currents on a fresh one. Every script uses this
one model, whether the pack is a real one from the catalog or one the sizing loop has just made.

**Mass closure.** Start from the fixed parts, size the rotors, motors and battery for that mass,
add up what they weigh, and repeat until the mass designed for is the mass built. If the mismatch
grows instead of shrinking, no such drone exists.

**Flight time.** Usable pack energy over average power, where average power is hover power traced
back through the motor and ESC, scaled up for maneuvering, plus the electronics.

## What is measured and what is a guess

| | Source |
|---|---|
| Prop rule (C_T and C_P from pitch, blades, diameter) | 65 static tests of props from 40 to 140 mm. Predicting a prop left out of the fit, figure of merit is typically within a factor of 1.19 |
| Motor rule (K_m and drag from mass) | Thrust-stand tests and datasheets of 35 motors up to 60 g. The rule itself is loose (a left-out motor's K_m within a factor of 1.5, its drag within 2), but a drone flown on a motor known only by mass and Kv comes out within about 8% on flight time and 9% on thrust |
| Whole-drone check | Crazyflie 2.1 Brushless: `python validate.py` |
| Frame mass, prop clearance | Placeholder |
| Board power, ESC efficiency | Placeholder |
| Pack internal resistance | Rule of thumb; hobby packs don't publish it |
| LED and UWB power | Estimated from a stand-in module and a datasheet |

Known effects the model leaves out, all of which make a real drone slightly worse: motor heating,
frame arms blocking the prop wash, and losses in the leads.

## Rebuilding the fitted rules

`catalog/scaling.json` is generated from the test data in `data/` (see `data/README.md` for what
each folder holds and where it came from). To refit:

```bash
python tools/fit_tyto.py        # the motor rule, from data/tyto/
python tools/fit_props.py       # the prop rule, from data/uiuc/ and data/cox_dantsker_2026/
```

The data itself is already in the repository. The two downloaders only need running to refresh it:

```bash
python tools/scrape_tyto.py     # Tyto Robotics test database (slow)
python tools/fetch_uiuc.py      # UIUC small-propeller static tests
```

## Tests

The unit tests check that the code does what the model says:

```bash
python -m unittest discover -s tests
```

Whether the fitted rules predict real parts is a separate question. This leaves parts out of each
fit, one at a time and a whole family or data source at a time, and predicts them from the rest:

```bash
python tools/cross_validate.py
```

Its findings so far:

- The prop rule predicts unseen props about as well as the ones it was fitted to, so it is not
  overfitted. Across the two data sources it is weaker: fitted to one, it misses the other's figure
  of merit by about 11% on average.
- The motor rule does not carry from the large thrust-stand motors to the micro motors: fitted to
  one group, it misses the other's K_m by a factor of 2 and its drag by a factor of 3. The micro
  motors' own datasheets are what make it usable at this size.
- Motor errors matter less than they look, because motor losses are only part of a drone's power.
  The last table in the output shows that directly.

## Data sources

- Motor and battery listings marked as such in the catalog: ThrustLab component database
  (thrustlab.com), CC BY 4.0.
- J.B. Brandt, R.W. Deters, G.K. Ananda, O.D. Dantsker and M.S. Selig, *UIUC Propeller Database,
  Vols 1-4*, University of Illinois at Urbana-Champaign. Volume 2: Deters, Ananda and Selig,
  "Reynolds Number Effects on the Performance of Small-Scale Propellers", AIAA 2014-2151.
- B. Cox and O.D. Dantsker, "Performance Testing of Small Multi-Rotor UAV Propellers",
  AIAA 2026-2306.
- Tyto Robotics test database (database.tytorobotics.com).
- Bitcraze's published figures for the Crazyflie 2.1 Brushless.
