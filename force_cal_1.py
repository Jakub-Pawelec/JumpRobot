from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List
import math

G = 9.81  # m/s^2


@dataclass
class TubePlan:
    strands: int
    draw_length_cm: float
    slack_total_cm: float
    slack_per_leg_cm: float
    stroke_limit_cm: float
    max_height_m: float
    meets_target: bool


def compute_tube_plans(
    mass_kg: float = 0.25,
    target_lift_m: float = 1.0,
    efficiency: float = 0.4,
    box_length_cm: float = 25.0,
    winch_clearance_cm: float = 2.5,
    rod_protrusion_cm: float = 2.5,
    fork_gap_cm: float = 16.0,
    attach_offset_cm: float = 1.0,
    stretch_ratio: float = 3.5,
    base_force_per_strand_kgf: float = 5.69,
    strand_counts: Iterable[int] = (1, 2, 4, 8),
) -> List[TubePlan]:
    if not 1.0 < stretch_ratio <= 5.0:
        raise ValueError("Choose a stretch ratio between 1 and 5 for latex longevity.")
    if not 0 < efficiency <= 1:
        raise ValueError("Efficiency must be between 0 and 1.")
    if box_length_cm <= 0:
        raise ValueError("Box length must be positive.")
    if fork_gap_cm < 0:
        raise ValueError("Fork gap must be non-negative.")

    stroke_limit_m = max((box_length_cm - winch_clearance_cm - rod_protrusion_cm) / 100.0, 0.0)
    stroke_limit_cm = stroke_limit_m * 100.0
    half_gap_m = (fork_gap_cm / 2.0) / 100.0
    attach_offset_m = attach_offset_cm / 100.0

    def leg_length(offset_m: float) -> float:
        return math.hypot(half_gap_m, attach_offset_m + offset_m)

    base_leg_length_m = leg_length(0.0)

    required_energy = mass_kg * G * target_lift_m
    stored_energy_target = required_energy / efficiency
    base_force_N = base_force_per_strand_kgf * G
    plans: List[TubePlan] = []

    for strands in strand_counts:
        draw_force_N = strands * base_force_N
        if draw_force_N <= 0:
            required_draw_m = float("inf")
        else:
            delta_leg_m = stored_energy_target / draw_force_N
            required_leg_length_m = base_leg_length_m + delta_leg_m
            radicand = max(required_leg_length_m ** 2 - half_gap_m ** 2, 0.0)
            required_draw_m = max(math.sqrt(radicand) - attach_offset_m, 0.0)

        meets_target = required_draw_m <= stroke_limit_m and stroke_limit_m > 0
        draw_used_m = min(required_draw_m, stroke_limit_m)
        stretched_leg_m = leg_length(draw_used_m)
        stretched_total_m = 2.0 * stretched_leg_m
        if stretch_ratio:
            slack_total_m = stretched_total_m / stretch_ratio
        else:
            slack_total_m = float("inf")
        slack_per_leg_m = slack_total_m / 2.0

        max_leg_length_m = leg_length(stroke_limit_m)
        max_energy_storable = draw_force_N * (max_leg_length_m - base_leg_length_m)
        max_height_m = efficiency * max_energy_storable / (mass_kg * G) if stroke_limit_m else 0.0

        plans.append(
            TubePlan(
                strands=strands,
                draw_length_cm=required_draw_m * 100.0,
                slack_total_cm=slack_total_m * 100.0,
                slack_per_leg_cm=slack_per_leg_m * 100.0,
                stroke_limit_cm=stroke_limit_cm,
                max_height_m=max_height_m,
                meets_target=meets_target,
            )
        )

    return plans


def print_plan_table(plans: Iterable[TubePlan]) -> None:
    header = (
        "Strands | Draw Needed (cm) | Slack Total (cm) | Slack / Leg (cm) | Stroke Budget (cm) | "
        "Max Height (m) | Meets Target?"
    )
    print(header)
    print("-" * len(header))
    for plan in plans:
        print(
            f"{plan.strands:>7} | "
            f"{plan.draw_length_cm:14.2f} | "
            f"{plan.slack_total_cm:15.2f} | "
            f"{plan.slack_per_leg_cm:14.2f} | "
            f"{plan.stroke_limit_cm:17.2f} | "
            f"{plan.max_height_m:13.2f} | "
            f"{'YES' if plan.meets_target else 'no '}"
        )


def summarize_recommendation(plans: Iterable[TubePlan]) -> None:
    viable = [plan for plan in plans if plan.meets_target]
    if not viable:
        print("\nNo tube count meets the 1 m target with the current geometry.")
        print("Increase efficiency, reduce clearances, or add more strands.")
        return

    pick = min(viable, key=lambda plan: plan.strands)
    print("\nRecommended setup:")
    print(f"- Tubes (strands): {pick.strands}")
    print(f"- Slack length per tube (total loop): {pick.slack_total_cm:.1f} cm")
    print(f"- Slack length per leg: {pick.slack_per_leg_cm:.1f} cm")
    print(f"- Maximum theoretical jump height: {pick.max_height_m:.2f} m")

if __name__ == "__main__":
    plans = compute_tube_plans()
    print_plan_table(plans)
    summarize_recommendation(plans)
