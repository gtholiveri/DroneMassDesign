# Test data

Measurements made by other people, kept here so the fitted rules in `catalog/scaling.json` can be
rebuilt. None of it is ours: please credit the sources below if you reuse it.

| Folder | What it is | Source |
|---|---|---|
| `tyto/` | Thrust-stand tests: throttle, RPM, thrust, torque, voltage and current for 1,518 motor and prop combinations. `index.json` lists each test with the motor, prop and ESC it is linked to and a link back to its page. Uploader names are left out. | Tyto Robotics test database, database.tytorobotics.com, measured and uploaded by its users. Downloaded by `tools/scrape_tyto.py`. |
| `uiuc/` | Static tests of props from 45 mm to 9 inches (`*_static_*.txt`: RPM, C_T, C_P) and their blade geometry (`*_geom.txt`: r/R, chord / R, blade angle). | J.B. Brandt, R.W. Deters, G.K. Ananda, O.D. Dantsker and M.S. Selig, *UIUC Propeller Database, Vols 1-4*, University of Illinois at Urbana-Champaign. Volume 2: Deters, Ananda and Selig, "Reynolds Number Effects on the Performance of Small-Scale Propellers", AIAA 2014-2151. Downloaded by `tools/fetch_uiuc.py`. |
| `cox_dantsker_2026/table2.csv` | Diameter, top speed, and thrust and torque at top speed for 18 current FPV props, typed in from Table 2 of the paper. Pitch and blade count are read from each prop's name. | B. Cox and O.D. Dantsker, "Performance Testing of Small Multi-Rotor UAV Propellers", AIAA SciTech 2026, paper 2026-2306, doi:10.2514/6.2026-2306. |

The paper itself is copyrighted, so its PDF is not in the repository (`*.pdf` is git-ignored). Keep
your own copy in `cox_dantsker_2026/` if you have access to it.
