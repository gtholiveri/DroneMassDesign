"""Pick real parts: try every frame, motor, prop and battery in parts.py, and rank the builds.

Runs once per thrust-to-weight ratio in scenario.py.
"""

from collections.abc import Callable
from dataclasses import replace

from drone_sizing.build import Build
from drone_sizing.report import format_build, format_search
from drone_sizing.search import Evaluation, search
from parts import AIRFRAMES, BATTERIES, MOTORS, PROPELLERS
from scenario import REQUIREMENTS, TECHNOLOGY, THRUST_TO_WEIGHT_OPTIONS, UNCERTAINTY

ROWS_SHOWN = 10


def ranking(evaluations: list[Evaluation]) -> Callable[[Evaluation], tuple]:
    """A sort key, best first.

    Builds that pass come first: those that still pass in the worst case, then cheapest, then
    lightest. Price only counts once every passing build has one; until then a build would rank
    high just for having its price filled in. Builds that fail come last, closest to passing first.
    """
    all_priced = all(e.build.price_usd is not None for e in evaluations if e.passes)

    def key(evaluation: Evaluation) -> tuple:
        mass_kg = evaluation.nominal.total_mass_kg
        if evaluation.passes:
            price_usd = evaluation.build.price_usd if all_priced else 0.0
            return (0, not evaluation.passes_worst_case, price_usd, mass_kg)
        return (1, True, -evaluation.nominal.tightest_check.margin, mass_kg)

    return key


def main() -> None:
    detailed: set[Build] = set()

    for thrust_to_weight in THRUST_TO_WEIGHT_OPTIONS:
        requirements = replace(REQUIREMENTS, thrust_to_weight=thrust_to_weight)
        result = search(requirements, AIRFRAMES, MOTORS, PROPELLERS, BATTERIES, TECHNOLOGY, UNCERTAINTY)
        ranked = sorted(result.evaluations, key=ranking(result.evaluations))

        print(f"===== Required: thrust-to-weight {thrust_to_weight}, flight time {requirements.flight_time_s / 60:.0f} min")
        print(format_search(result, ranked, ROWS_SHOWN))

        # Show the top build in full, unless a lower ratio already showed the same one.
        if ranked and ranked[0].build not in detailed:
            detailed.add(ranked[0].build)
            print()
            print("Top build in full")
            print(format_build(ranked[0], requirements, TECHNOLOGY))
        print()


if __name__ == "__main__":
    main()
