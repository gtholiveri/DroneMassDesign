"""Pick real parts: try every frame, motor, prop and battery in parts.py, and rank the builds.

Runs once per thrust-to-weight ratio in scenario.py.
"""

from dataclasses import replace

from drone_sizing.build import Build
from drone_sizing.report import format_build, format_search
from drone_sizing.search import search
from parts import AIRFRAMES, BATTERIES, MOTORS, PROPELLERS
from scenario import REQUIREMENTS, TECHNOLOGY, THRUST_TO_WEIGHT_OPTIONS, UNCERTAINTY

ROWS_SHOWN = 10


def main() -> None:
    detailed: set[Build] = set()

    for thrust_to_weight in THRUST_TO_WEIGHT_OPTIONS:
        requirements = replace(REQUIREMENTS, thrust_to_weight=thrust_to_weight)
        result = search(requirements, AIRFRAMES, MOTORS, PROPELLERS, BATTERIES, TECHNOLOGY, UNCERTAINTY)

        print(f"===== Required: thrust-to-weight {thrust_to_weight}, flight time {requirements.flight_time_s / 60:.0f} min")
        print(format_search(result, ROWS_SHOWN))

        # Show the top build in full, unless a lower ratio already showed the same one.
        ranked = result.ranked
        if ranked and ranked[0].build not in detailed:
            detailed.add(ranked[0].build)
            print()
            print("Top build in full")
            print(format_build(ranked[0]))
        print()


if __name__ == "__main__":
    main()
