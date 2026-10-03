"""Download the small-propeller part of the UIUC Propeller Database (Volume 2) for local analysis.

Volume 2 holds static (hover) tests of propellers from 45 mm to 9 inches, measured with a torque
cell at the low Reynolds numbers small props run at, plus each blade's measured chord and twist.
It includes the Crazyflie's propeller. Two kinds of small text file per prop:
    ..._static_NNNN.txt   RPM, C_T, C_P
    ..._geom.txt          r/R, chord / R, blade angle (degrees)

The data stays in data/uiuc/, which git ignores. Cite it as:
    J.B. Brandt, R.W. Deters, G.K. Ananda, O.D. Dantsker, and M.S. Selig, UIUC Propeller Database,
    Vols 1-4, University of Illinois at Urbana-Champaign, Department of Aerospace Engineering.
    Volume 2: Deters, Ananda and Selig, "Reynolds Number Effects on the Performance of Small-Scale
    Propellers", AIAA 2014-2151.

Run from the project folder:  python tools/fetch_uiuc.py
"""

import re
import time
import urllib.request
from pathlib import Path

VOLUME = "https://m-selig.ae.illinois.edu/props/volume-2"
OUTPUT = Path(__file__).resolve().parent.parent / "data" / "uiuc"
PAUSE_S = 0.5
HEADERS = {"User-Agent": "Mozilla/5.0 (student drone-sizing research; one request at a time)"}

# Static tests and blade geometry. Wind-tunnel runs at forward speed aren't needed for hover.
DATA_FILE = re.compile(r'href="data/([^"]+_(?:static_\w+|geom)\.txt)"')


def fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers=HEADERS)
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    page = fetch(f"{VOLUME}/propDB-volume-2.html").decode("utf-8", errors="replace")
    names = sorted(set(DATA_FILE.findall(page)))
    missing = [name for name in names if not (OUTPUT / name).exists()]
    print(f"{len(names)} files listed, {len(missing)} to fetch")

    for name in missing:
        (OUTPUT / name).write_bytes(fetch(f"{VOLUME}/data/{name}"))
        time.sleep(PAUSE_S)
    print(f"done: {len(list(OUTPUT.glob('*.txt')))} files in {OUTPUT}")


if __name__ == "__main__":
    main()
