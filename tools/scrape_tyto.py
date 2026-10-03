"""Download the Tyto Robotics test database (database.tytorobotics.com/tests) for local analysis.

Two steps, both resumable:
  1. The index: every test's title, plus the motor, prop and ESC it's linked to (if any).
  2. Each test's page, which embeds its data table (throttle, RPM, thrust, torque, volts, amps...).

It's polite: at most a few requests at a time, each followed by a pause. The tests the fits can use
are fetched first. The data stays in data/tyto/, which git
ignores. It belongs to Tyto and the people who uploaded it, so it isn't ours to republish.

Run from the project folder:  python tools/scrape_tyto.py
"""

import html
import json
import re
import time
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

SITE = "https://database.tytorobotics.com"
OUTPUT = Path(__file__).resolve().parent.parent / "data" / "tyto"
PAUSE_S = 0.5
WORKERS = 4
HEADERS = {
    "User-Agent": "Mozilla/5.0 (student drone-sizing research; one request at a time)",
    "Accept": "application/json, text/html",
    "X-Requested-With": "XMLHttpRequest",
}

# The data table is a Vue component whose rows sit in a single-quoted, HTML-escaped JSON attribute.
DATA_ATTRIBUTE = re.compile(r"<benchmark-data-table\b.*?:data\s*=\s*'(.*?)'", re.S)


def fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers=HEADERS)
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def index_page(number: int, per_page: int = 100) -> dict:
    params = {
        "draw": 1,
        "per_page": per_page,
        "page": number,
        "filters": json.dumps({"conjunction": "AND", "filters": [[]]}),
        "relations": json.dumps(["creator", "powertrains.motor", "powertrains.propeller", "powertrains.esc"]),
        "aggregates": "[]",
        "order_by": "[]",
    }
    return json.loads(fetch(f"{SITE}/tests/search?{urllib.parse.urlencode(params)}"))


def download_index() -> list[dict]:
    path = OUTPUT / "index.json"
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))

    first = index_page(1)
    tests = first["data"]
    for number in range(2, first["meta"]["last_page"] + 1):
        time.sleep(PAUSE_S)
        tests += index_page(number)["data"]

    path.write_text(json.dumps(tests), encoding="utf-8")
    return tests


def download_test(test: dict) -> None:
    path = OUTPUT / "tests" / f"{test['hash']}.json"
    if path.exists():
        return

    page = fetch(test["link"]).decode("utf-8", errors="replace")
    datasets = [json.loads(html.unescape(match)) for match in DATA_ATTRIBUTE.findall(page)]
    record = {"hash": test["hash"], "title": test["title"], "device": test.get("device"), "datasets": datasets}
    path.write_text(json.dumps(record), encoding="utf-8")


def most_useful_first(test: dict) -> int:
    """Single-motor tests on stands that measure torque, with a linked motor or prop, go first:
    they're the ones tools/fit_tyto.py can use."""
    powertrains = test["powertrains"]
    usable = (
        len(powertrains) == 1
        and test.get("device") != "Series 1520"  # this stand has no torque sensor
        and bool(powertrains[0].get("motor") or powertrains[0].get("propeller"))
    )
    return 0 if usable else 1


def download_with_pause(test: dict) -> tuple[str, str | None]:
    try:
        download_test(test)
        error = None
    except Exception as exception:  # keep going; a rerun retries whatever failed
        error = repr(exception)
    time.sleep(PAUSE_S)
    return test["hash"], error


def main() -> None:
    (OUTPUT / "tests").mkdir(parents=True, exist_ok=True)
    tests = download_index()
    remaining = [test for test in tests if not (OUTPUT / "tests" / f"{test['hash']}.json").exists()]
    remaining.sort(key=most_useful_first)
    print(f"{len(tests)} tests in the index, {len(remaining)} still to download", flush=True)

    # The server takes several seconds per page, so keep a few requests in flight, no more.
    failures = []
    with ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for number, (test_hash, error) in enumerate(pool.map(download_with_pause, remaining), start=1):
            if error:
                failures.append((test_hash, error))
            if number % 50 == 0:
                print(f"  {number}/{len(remaining)}", flush=True)

    print(f"done, {len(failures)} failures")
    for test_hash, error in failures:
        print(f"  {test_hash}: {error}")


if __name__ == "__main__":
    main()
