from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence
import math

G = 9.81  # m/s^2


@dataclass
class DrawResult:
    draw_fraction: float
    draw_cm: float
    stretch_ratio_actual: float
    pull_force_N: float
    jump_height_m: float


@dataclass
class TubePlan:
    strands: int
    stretch_ratio: float
    slack_total_cm: float
    slack_per_leg_cm: float
    stroke_limit_cm: float
    max_height_m: float
    peak_force_N: float
    draw_results: List[DrawResult]


def compute_tube_plans(
    mass_kg: float = 0.4,
    efficiency: float = 0.4,
    box_length_cm: float = 25.0,
    winch_clearance_cm: float = 2.5,
    rod_protrusion_cm: float = 2.5,
    fork_gap_cm: float = 16.0,
    attach_offset_cm: float = 1.0,
    stretch_ratios: Sequence[float] = (2.5, 3.0, 3.5, 4.0),
    base_force_per_strand_kgf: float = 5.69,
    strand_counts: Iterable[int] = (1, 2, 3),
    draw_fractions: Sequence[float] = (0.25, 0.5, 0.75, 1.0),
    reference_stretch_ratio: float = 3.5,
    force_exponent: float = 1.4,
    integration_steps: int = 200,
) -> List[TubePlan]:
    if not 0 < efficiency <= 1:
        raise ValueError("Efficiency must be between 0 and 1.")
    if box_length_cm <= 0:
        raise ValueError("Box length must be positive.")
    if fork_gap_cm < 0:
        raise ValueError("Fork gap must be non-negative.")
    if not stretch_ratios:
        raise ValueError("Provide at least one stretch ratio to evaluate.")
    if not draw_fractions:
        raise ValueError("Provide at least one draw fraction to evaluate.")
    if reference_stretch_ratio <= 1.0:
        raise ValueError("Reference stretch ratio must be greater than 1.")
    if integration_steps < 10:
        raise ValueError("Use at least 10 integration steps for stability.")

    stroke_limit_m = max((box_length_cm - winch_clearance_cm - rod_protrusion_cm) / 100.0, 0.0)
    stroke_limit_cm = stroke_limit_m * 100.0
    half_gap_m = (fork_gap_cm / 2.0) / 100.0
    attach_offset_m = attach_offset_cm / 100.0

    def leg_length(offset_m: float) -> float:
        return math.hypot(half_gap_m, attach_offset_m + offset_m)

    base_leg_length_m = leg_length(0.0)

    base_force_N = base_force_per_strand_kgf * G
    plans: List[TubePlan] = []

    def draw_force_at_offset(offset_m: float, slack_leg_m: float, strands: int) -> float:
        if slack_leg_m <= 0:
            return 0.0
        leg = leg_length(offset_m)
        stretch_actual = leg / slack_leg_m
        if stretch_actual <= 1.0:
            return 0.0
        normalized = stretch_actual / reference_stretch_ratio
        per_strand_force = base_force_N * (normalized ** force_exponent)
        return per_strand_force * strands

    for strands in strand_counts:
        for stretch_ratio in stretch_ratios:
            if not 1.0 < stretch_ratio <= 5.0:
                raise ValueError("Each stretch ratio must fall between 1 and 5.")

            stretched_leg_full_m = leg_length(stroke_limit_m)
            slack_leg_m = stretched_leg_full_m / stretch_ratio if stretch_ratio else float("inf")
            slack_total_m = 2.0 * slack_leg_m
            slack_per_leg_m = slack_total_m / 2.0

            draw_results: List[DrawResult] = []
            for fraction in draw_fractions:
                if not 0 < fraction <= 1:
                    raise ValueError("Draw fractions must be in (0, 1].")
                draw_m = stroke_limit_m * fraction
                steps = max(4, int(integration_steps * fraction))
                if steps <= 0:
                    energy = 0.0
                else:
                    dx = draw_m / steps
                    force_prev = draw_force_at_offset(0.0, slack_leg_m, strands)
                    energy = 0.0
                    for step in range(1, steps + 1):
                        offset = step * dx
                        force_curr = draw_force_at_offset(offset, slack_leg_m, strands)
                        energy += 0.5 * (force_prev + force_curr) * dx
                        force_prev = force_curr

                pull_force = draw_force_at_offset(draw_m, slack_leg_m, strands)
                stretch_actual = (leg_length(draw_m) / slack_leg_m) if slack_leg_m else 0.0
                jump_height = efficiency * energy / (mass_kg * G) if energy else 0.0

                draw_results.append(
                    DrawResult(
                        draw_fraction=fraction,
                        draw_cm=draw_m * 100.0,
                        stretch_ratio_actual=stretch_actual,
                        pull_force_N=pull_force,
                        jump_height_m=jump_height,
                    )
                )

            max_height_m = max((result.jump_height_m for result in draw_results), default=0.0)
            peak_force = max((result.pull_force_N for result in draw_results), default=0.0)

            plans.append(
                TubePlan(
                    strands=strands,
                    stretch_ratio=stretch_ratio,
                    slack_total_cm=slack_total_m * 100.0,
                    slack_per_leg_cm=slack_per_leg_m * 100.0,
                    stroke_limit_cm=stroke_limit_cm,
                    max_height_m=max_height_m,
                    peak_force_N=peak_force,
                    draw_results=draw_results,
                )
            )

    return plans


def print_plan_table(plans: Iterable[TubePlan]) -> None:
    header = (
        "Strands | Stretch Ratio | Slack Total (cm) | Slack / Leg (cm) | Stroke Budget (cm) | "
        "Peak Force (N / kgf) | Max Height (m)"
    )
    print(header)
    print("-" * len(header))
    for plan in plans:
        print(
            f"{plan.strands:>7} | "
            f"{plan.stretch_ratio:13.2f} | "
            f"{plan.slack_total_cm:15.2f} | "
            f"{plan.slack_per_leg_cm:14.2f} | "
            f"{plan.stroke_limit_cm:17.2f} | "
            f"{plan.peak_force_N:9.1f} / {plan.peak_force_N / G:6.2f} | "
            f"{plan.max_height_m:13.2f}"
        )


def print_draw_profiles(plans: Iterable[TubePlan]) -> None:
    print("\nDraw profiles:")
    header = (
        "Strands | Stretch Ratio | Draw % | Draw (cm) | Stretch@Draw | Pull Force (N / kgf) | Height (m)"
    )
    print(header)
    print("-" * len(header))
    for plan in plans:
        for result in plan.draw_results:
            print(
                f"{plan.strands:>7} | "
                f"{plan.stretch_ratio:13.2f} | "
                f"{result.draw_fraction * 100:6.1f} | "
                f"{result.draw_cm:10.2f} | "
                f"{result.stretch_ratio_actual:13.2f} | "
                f"{result.pull_force_N:9.1f} / {result.pull_force_N / G:6.2f} | "
                f"{result.jump_height_m:9.2f}"
            )


def summarize_recommendation(plans: Iterable[TubePlan]) -> None:
    if not plans:
        print("\nNo plans available to summarize.")
        return

    pick = max(plans, key=lambda plan: plan.max_height_m)
    full_draw = max(pick.draw_results, key=lambda res: res.draw_fraction, default=None)
    best_draw = max(pick.draw_results, key=lambda res: res.jump_height_m, default=None)
    print("\nRecommended setup:")
    print(f"- Tubes (strands): {pick.strands}")
    print(f"- Stretch ratio: {pick.stretch_ratio:.2f}")
    print(f"- Slack length per tube (total loop): {pick.slack_total_cm:.1f} cm")
    print(f"- Slack length per leg: {pick.slack_per_leg_cm:.1f} cm")
    if full_draw:
        print(
            f"- Pull force at full draw: {full_draw.pull_force_N:.1f} N / "
            f"{full_draw.pull_force_N / G:.1f} kgf"
        )
    if best_draw:
        print(
            f"- Peak height {best_draw.jump_height_m:.2f} m at draw {best_draw.draw_fraction * 100:.0f}%"
        )

if __name__ == "__main__":
    plans = compute_tube_plans()
    print_plan_table(plans)
    print_draw_profiles(plans)
    summarize_recommendation(plans)
