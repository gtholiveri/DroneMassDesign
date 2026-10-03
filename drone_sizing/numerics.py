"""Small numerical tools: a root finder, a 1-D minimizer and power-law fits. Nothing here knows about drones."""

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

# Each step of either search shrinks the interval by a fixed factor (1/2 or 0.618), so 60 steps
# takes any interval far below what the inputs can resolve.
ITERATIONS = 60

GOLDEN_RATIO_CONJUGATE = (math.sqrt(5) - 1) / 2  # 0.618...


def solve_increasing(f: Callable[[float], float], target: float, low: float, high: float) -> float:
    """Find the x in [low, high] where an increasing function f reaches target, by bisection.

    Each step checks the middle of the interval and keeps the half that must contain the answer.
    """
    for _ in range(ITERATIONS):
        middle = (low + high) / 2
        if f(middle) < target:
            low = middle
        else:
            high = middle
    return (low + high) / 2


def minimize(f: Callable[[float], float], low: float, high: float) -> float:
    """Find the x in [low, high] that minimizes f, assuming f has a single dip there.

    Golden-section search: compare f at two interior points and discard the outer part beyond the
    worse one. Placing the points at the golden ratio lets each step reuse one of the last step's
    points, so each step costs one new evaluation of f.
    """
    left = high - GOLDEN_RATIO_CONJUGATE * (high - low)
    right = low + GOLDEN_RATIO_CONJUGATE * (high - low)
    f_left, f_right = f(left), f(right)

    for _ in range(ITERATIONS):
        if f_left < f_right:
            # The minimum can't be beyond `right`.
            high, right, f_right = right, left, f_left
            left = high - GOLDEN_RATIO_CONJUGATE * (high - low)
            f_left = f(left)
        else:
            # The minimum can't be before `left`.
            low, left, f_left = left, right, f_right
            right = low + GOLDEN_RATIO_CONJUGATE * (high - low)
            f_right = f(right)

    return (low + high) / 2


@dataclass(frozen=True)
class PowerLaw:
    """y = coefficient * x^exponent: a straight line on log-log axes."""

    coefficient: float
    exponent: float

    def __call__(self, x: float) -> float:
        return self.coefficient * x**self.exponent

    @classmethod
    def through_point(cls, x: float, y: float, exponent: float) -> "PowerLaw":
        """The power law with this exponent that passes through one known point."""
        return cls(coefficient=y / x**exponent, exponent=exponent)

    @classmethod
    def fit(cls, xs: Sequence[float], ys: Sequence[float]) -> "PowerLaw":
        """Least-squares straight line through the points on log-log axes: ln y = ln a + b ln x."""
        if len(set(xs)) < 2:
            raise ValueError("fitting an exponent needs points at two or more different x values")

        log_xs = [math.log(x) for x in xs]
        log_ys = [math.log(y) for y in ys]
        mean_log_x = sum(log_xs) / len(log_xs)
        mean_log_y = sum(log_ys) / len(log_ys)

        # Slope of the least-squares line: covariance of (ln x, ln y) over variance of ln x.
        covariance = sum((lx - mean_log_x) * (ly - mean_log_y) for lx, ly in zip(log_xs, log_ys))
        variance = sum((lx - mean_log_x) ** 2 for lx in log_xs)
        exponent = covariance / variance

        return cls(coefficient=math.exp(mean_log_y - exponent * mean_log_x), exponent=exponent)
