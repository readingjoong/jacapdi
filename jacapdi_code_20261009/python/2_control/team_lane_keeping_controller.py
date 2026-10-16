from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class StanleyControlDebug:
    theta_e_rad: float
    cte_m: float
    curvature: float
    stanley_rad: float
    feedforward_rad: float
    delta_rad: float


class LaneKeepingController:
    """Stanley controller with curvature feedforward for y = ax^2 + bx + c."""

    def __init__(
        self,
        stanley_gain: float,
        curvature_gain: float,
        softening_speed_mps: float,
        front_axle_offset_m: float,
        heading_gain: float,
    ):
        self.stanley_gain = float(stanley_gain)
        self.curvature_gain = float(curvature_gain)
        self.softening_speed_mps = float(softening_speed_mps)
        self.front_axle_offset_m = float(front_axle_offset_m)
        self.heading_gain = float(heading_gain)

    def compute_delta(self, coeffs: Tuple[float, float, float], speed_mps: float):
        a, b, c = map(float, coeffs)
        x = self.front_axle_offset_m

        dy_dx = 2.0 * a * x + b
        d2y_dx2 = 2.0 * a

        theta_e = np.arctan(dy_dx)
        cte = a * x * x + b * x + c
        curvature = d2y_dx2 / np.power(1.0 + dy_dx * dy_dx, 1.5)

        speed_denom = abs(float(speed_mps)) + self.softening_speed_mps
        stanley = np.arctan((self.stanley_gain * cte) / speed_denom)
        feedforward = self.curvature_gain * curvature
        delta = self.heading_gain * theta_e + stanley + feedforward

        debug = StanleyControlDebug(
            theta_e_rad=float(theta_e),
            cte_m=float(cte),
            curvature=float(curvature),
            stanley_rad=float(stanley),
            feedforward_rad=float(feedforward),
            delta_rad=float(delta),
        )
        return float(delta), debug
