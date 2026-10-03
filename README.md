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
| Does a drone exist that flies this long, and what does it look like? | `python explore.py` |
| I have one drone in mind: how long does it fly, or what battery does it need? | `python calc.py ...` |
| Which real parts should we buy? | `python select_parts.py` |
| For each real prop, what would the ideal motor and battery be? | `python main.py` |
| Does the model match a real drone? | `python validate.py` |

## The calculator: `calc.py`

Put in what you know about one drone and it gives flight time, hover current and throttle,
thrust-to-weight and peak currents. No files to edit.

Give the battery, get the flight time:

```bash
python calc.py --dry-mass-g 32 --electronics-w 4.13 --motor-kv 10000 --motor-mass-g 3.45 --prop-diameter-mm 65 --prop-pitch-mm 33 --battery-mah 1100 --battery-mass-g 22 --lihv
```

Give the flight time instead, get the smallest battery that reaches it:

```bash
python calc.py --dry-mass-g 32 --electronics-w 4.13 --motor-kv 10000 --motor-mass-g 3.45 --prop-diameter-mm 65 --prop-pitch-mm 33 --flight-min 15 --lihv
```

The inputs that matter most:

| Option | Meaning |
|---|---|
| `--dry-mass-g` | The whole drone without its battery: frame, board, modules, motors, props, wires |
| `--electronics-w` | Power drawn by everything that isn't a motor: board, UWB module, LEDs |
| `--motor-kv`, `--motor-mass-g` | From the motor listing. Resistance, no-load current and max current are estimated from mass if you leave them out |
| `--prop-diameter-mm` with `--prop-pitch-mm` | From the prop listing. Or give measured `--ct` and `--cp` |
| `--battery-mah` or `--flight-min` | One or the other |
| `--battery-mass-g`, `--lihv`, `--cells` | The pack. Mass is estimated from capacity if left out |
| `--power-factor`, `--mass-margin`, `--usable` | Margins. The defaults give a plain hover estimate using 80% of the pack. Add `--power-factor 1.2 --mass-margin 0.15` to plan the way the other scripts do |

`python calc.py --help` lists the rest.

### Example: what does the LED choice cost?

Two numbers describe an LED module to the calculator: its power goes into `--electronics-w`, and
its mass goes into `--dry-mass-g`. For our drone the other electronics draw 0.93 W (0.40 W for the
board and 0.53 W for the UWB module), so `--electronics-w` is 0.93 plus the average LED power.

This is one of our candidate drones (RCinPower 1003 10000 Kv motors with their listed numbers,
65 mm props, a 1100 mAh pack) with the planning margins on. Change `--electronics-w` and
`--dry-mass-g` and rerun:

```bash
python calc.py --dry-mass-g 32 --electronics-w 4.13 --motor-kv 10000 --motor-mass-g 3.45 --motor-resistance-ohm 0.162 --motor-no-load-a 0.8 --motor-no-load-v 5 --motor-max-a 11.5 --prop-diameter-mm 65 --prop-pitch-mm 33 --battery-mah 1100 --battery-mass-g 22 --lihv --power-factor 1.2 --mass-margin 0.15
```

The 32 g of dry mass is the frame (6.6 g), board (4.9 g), UWB module (1.4 g), LED module (3.5 g),
four motors (13.8 g) and four props (1.8 g).

| Average LED power | `--electronics-w` | Flight time |
|---|---|---|
| 0.8 W | 1.73 | 13.9 min |
| 1.6 W | 2.53 | 13.2 min |
| 3.2 W | 4.13 | 11.9 min |

| Dry mass | Flight time (LED at 3.2 W) |
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
93 g | 2828 mAh 50 g | 3.0 g 8.3k
```

meaning total mass, battery capacity and mass, and the motor to look for (mass and Kv). A cell
reads `none` where no consistent drone exists: every gram of battery added costs more hover power
than the energy it brings.

There is one table per kind of battery, because energy per kilogram decides more than anything
else. The battery kinds, prop sizes and flight times are constants at the top of the file.

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
| `scenario.py` | Flight time, thrust-to-weight options, margins, battery voltage assumptions, how uncertain each kind of number is |
| `parts.py` | The fixed parts: frames (diagonal, mass, prop clearance), controller board, UWB and LED modules with their mass and power |
| `catalog/motors.csv` | Candidate motors: Kv, mass, and whatever else the listing gives |
| `catalog/propellers.csv` | Candidate props: size, pitch, blades, mass, bore, and measured coefficients if known |
| `catalog/batteries.csv` | Candidate packs: cells, capacity, mass, C ratings |
| `catalog/scaling.json` | The fitted rules. Generated; see below |

Blank cells in the CSVs mean "not known" and are filled from the fitted rules. Rows whose name
starts with `#` are skipped. `drone_sizing/catalog.py` documents the columns.

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
resistance. Thrust is judged on a tired pack and currents on a fresh one.

**Mass closure.** Start from the fixed parts, size the rotors, motors and battery for that mass,
add up what they weigh, and repeat until the mass designed for is the mass built. If the mismatch
grows instead of shrinking, no such drone exists.

**Flight time.** Usable pack energy over average power, where average power is hover power traced
back through the motor and ESC, scaled up for maneuvering, plus the electronics.

## What is measured and what is a guess

| | Source |
|---|---|
| Prop rule (C_T and C_P from pitch, blades, diameter) | 65 static tests of props from 40 to 140 mm. Figure of merit is predictable only to within a factor of about 1.18 |
| Motor rule (K_m and drag from mass) | Thrust-stand tests and datasheets of motors up to 60 g. From mass and Kv alone it predicts flight time to about 8% and thrust to about 7% |
| Whole-drone check | Crazyflie 2.1 Brushless: `python validate.py` |
| Frame mass, prop clearance | Placeholder |
| Board power, ESC efficiency | Placeholder |
| Pack internal resistance | Rule of thumb; hobby packs don't publish it |
| LED and UWB power | Estimated from a stand-in module and a datasheet |

Known effects the model leaves out, all of which make a real drone slightly worse: motor heating,
frame arms blocking the prop wash, and losses in the leads.

## Rebuilding the fitted rules

`catalog/scaling.json` is generated from test data that is kept out of the repository (`data/` is
git-ignored). To rebuild it:

```bash
python tools/scrape_tyto.py     # Tyto Robotics test database, into data/tyto/ (slow)
python tools/fit_tyto.py        # the motor rule
python tools/fetch_uiuc.py      # UIUC small-propeller static tests, into data/uiuc/
python tools/fit_props.py       # the prop rule
```

`tools/fit_props.py` also reads `data/cox_dantsker_2026/table2.csv` if it exists: Table 2 of the
paper below, typed in by hand, with the columns `name, diameter_mm, pitch_in, blades, max_rpm,
max_thrust_n, max_torque_nm`.

## Tests

```bash
python -m unittest discover -s tests
```

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
