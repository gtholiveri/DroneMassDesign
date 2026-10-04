"""Try every combination of catalog parts on one airframe, judge each one, and rank them."""

import itertools
from collections.abc import Sequence
from dataclasses import dataclass

from drone_sizing.airframe import Airframe
from drone_sizing.battery import BatteryPack
from drone_sizing.build import Build, BuildResult, Uncertainty
from drone_sizing.inputs import Requirements, Technology
from drone_sizing.motor import Motor
from drone_sizing.propeller import Propeller


@dataclass(frozen=True)
class Evaluation:
    """One build, judged on the catalog numbers and again with every uncertain number pushed the wrong way."""

    nominal: BuildResult
    worst_case: BuildResult

    @property
    def build(self) -> Build:
        return self.nominal.build

    @property
    def passes(self) -> bool:
        return self.nominal.passed

    @property
    def passes_worst_case(self) -> bool:
        return self.worst_case.passed


@dataclass(frozen=True)
class SearchResult:
    evaluations: list[Evaluation]  # every build whose parts fit together
    incompatible: list[tuple[Build, list[str]]]  # every build whose parts don't, and why

    @property
    def combination_count(self) -> int:
        return len(self.evaluations) + len(self.incompatible)

    @property
    def ranked(self) -> list[Evaluation]:
        """Every evaluated build, best first.

        Builds that pass come first: those that still pass in the worst case, then cheapest, then
        lightest. Price only counts once every passing build has one; until then a build would rank
        high just for having its price filled in. Builds that fail come last, closest to passing first.
        """
        all_priced = all(e.build.price_usd is not None for e in self.evaluations if e.passes)

        def key(evaluation: Evaluation) -> tuple:
            mass_kg = evaluation.nominal.total_mass_kg
            if evaluation.passes:
                price_usd = evaluation.build.price_usd if all_priced else 0.0
                return (0, not evaluation.passes_worst_case, price_usd, mass_kg)
            return (1, True, -evaluation.nominal.tightest_check.margin, mass_kg)

        return sorted(self.evaluations, key=key)


def search(
    requirements: Requirements,
    airframes: Sequence[Airframe],
    motors: Sequence[Motor],
    propellers: Sequence[Propeller],
    batteries: Sequence[BatteryPack],
    tech: Technology,
    uncertainty: Uncertainty,
) -> SearchResult:
    evaluations = []
    incompatible = []

    # Every airframe with every motor, prop and battery. Tens of thousands of combinations take seconds.
    for airframe, motor, propeller, battery in itertools.product(airframes, motors, propellers, batteries):
        build = Build(airframe=airframe, motor=motor, propeller=propeller, battery=battery)

        problems = build.incompatibilities()
        if problems:
            incompatible.append((build, problems))
            continue

        evaluations.append(
            Evaluation(
                nominal=build.evaluate(requirements, tech),
                worst_case=uncertainty.worst_case(build, tech).evaluate(requirements, tech),
            )
        )

    return SearchResult(evaluations=evaluations, incompatible=incompatible)
