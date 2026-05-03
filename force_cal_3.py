from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from openpyxl import Workbook
except ImportError:  # pragma: no cover - optional dependency
    Workbook = None

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
    rod_protrusion_cm: float = 1.5,
    rod_outside_limit_cm: float = 7.0,
    fork_gap_cm: float = 16.0,
    attach_offset_cm: float = 1.0,
    stretch_ratios: Sequence[float] = (3.0, 3.5, 4.0, 4.5, 5.0),
    base_force_per_strand_kgf: float = 5.69,
    strand_counts: Iterable[int] = (1, 2, 3),
    draw_fractions: Sequence[float] = (0.25, 0.5, 0.75, 1.0),
    reference_stretch_ratio: float = 4.0,
    force_exponent: float = 1.4,
    integration_steps: int = 600,
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

    if rod_outside_limit_cm < rod_protrusion_cm:
        raise ValueError("Outside travel limit must be >= initial protrusion.")

    internal_travel_cm = max(box_length_cm - winch_clearance_cm - rod_protrusion_cm, 0.0)
    external_travel_cm = max(rod_outside_limit_cm - rod_protrusion_cm, 0.0)
    stroke_limit_cm = min(internal_travel_cm, external_travel_cm)
    stroke_limit_m = stroke_limit_cm / 100.0
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
        "Strands | Stretch Ratio | Slack Total (cm) | Slack / Leg (cm) | Draw % | Draw (cm) | "
        "Stretch@Draw | Pull Force (N / kgf) | Height (m)"
    )
    print(header)
    print("-" * len(header))
    for plan in plans:
        print("-" * len(header))
        for result in plan.draw_results:
            print(
                f"{plan.strands:>7} | "
                f"{plan.stretch_ratio:13.2f} | "
                f"{plan.slack_total_cm:15.2f} | "
                f"{plan.slack_per_leg_cm:14.2f} | "
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


def save_plots(plans: Iterable[TubePlan], output_dir: Path | str = ".") -> None:
    plans_list = list(plans)
    if not plans_list:
        print("No plans available for plotting.")
        return

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    by_strands: dict[int, List[TubePlan]] = {}
    for plan in plans_list:
        by_strands.setdefault(plan.strands, []).append(plan)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    for strands, group in sorted(by_strands.items()):
        group_sorted = sorted(group, key=lambda plan: plan.stretch_ratio)
        stretch = [plan.stretch_ratio for plan in group_sorted]
        max_height = [plan.max_height_m for plan in group_sorted]
        peak_force = [plan.peak_force_N for plan in group_sorted]

        axes[0].plot(stretch, max_height, marker="o", linewidth=1.8, label=f"{strands} strand(s)")
        axes[1].plot(stretch, peak_force, marker="o", linewidth=1.8, label=f"{strands} strand(s)")

    axes[0].set_title("Predicted Jump Height vs Stretch Ratio")
    axes[0].set_xlabel("Stretch ratio at full draw")
    axes[0].set_ylabel("Predicted jump height (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].set_title("Peak Pull Force vs Stretch Ratio")
    axes[1].set_xlabel("Stretch ratio at full draw")
    axes[1].set_ylabel("Peak pull force (N)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.suptitle("Tube Analysis Summary")
    fig.tight_layout()
    summary_path = out_dir / "tube_analysis_summary.png"
    fig.savefig(summary_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot: {summary_path.resolve()}")

    recommended = max(plans_list, key=lambda plan: plan.max_height_m)
    draw_percent = [result.draw_fraction * 100.0 for result in recommended.draw_results]
    pull_force = [result.pull_force_N for result in recommended.draw_results]
    jump_height = [result.jump_height_m for result in recommended.draw_results]
    stretch_actual = [result.stretch_ratio_actual for result in recommended.draw_results]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    axes[0].plot(draw_percent, pull_force, marker="o", color="darkorange", linewidth=1.8)
    axes[0].set_title("Recommended Plan: Pull Force vs Draw")
    axes[0].set_xlabel("Draw fraction (%)")
    axes[0].set_ylabel("Pull force (N)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(draw_percent, jump_height, marker="o", color="steelblue", linewidth=1.8)
    for x_val, y_val, stretch in zip(draw_percent, jump_height, stretch_actual):
        axes[1].annotate(f"{stretch:.2f}x", (x_val, y_val), textcoords="offset points", xytext=(0, 6), ha="center", fontsize=8)
    axes[1].set_title("Recommended Plan: Height vs Draw")
    axes[1].set_xlabel("Draw fraction (%)")
    axes[1].set_ylabel("Predicted jump height (m)")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle(
        f"Recommended Tube Plan ({recommended.strands} strand(s), stretch ratio {recommended.stretch_ratio:.2f})"
    )
    fig.tight_layout()
    profile_path = out_dir / "tube_draw_profiles.png"
    fig.savefig(profile_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot: {profile_path.resolve()}")


def export_to_excel(plans: Iterable[TubePlan], path: Path | str = "tube_plan_results.xlsx") -> None:
    output_path = Path(path)
    if not Workbook:
        print(
            "Skipping Excel export: install openpyxl (pip install openpyxl) "
            "to enable spreadsheet output."
        )
        return

    wb = Workbook()
    plans_sheet = wb.active
    plans_sheet.title = "Plans"
    plans_sheet.append(
        [
            "Strands",
            "Stretch Ratio",
            "Slack Total (cm)",
            "Slack / Leg (cm)",
            "Stroke Budget (cm)",
            "Peak Force (N)",
            "Peak Force (kgf)",
            "Max Height (m)",
        ]
    )
    for plan in plans:
        plans_sheet.append(
            [
                plan.strands,
                round(plan.stretch_ratio, 4),
                round(plan.slack_total_cm, 4),
                round(plan.slack_per_leg_cm, 4),
                round(plan.stroke_limit_cm, 4),
                round(plan.peak_force_N, 4),
                round(plan.peak_force_N / G, 4),
                round(plan.max_height_m, 4),
            ]
        )

    profile_sheet = wb.create_sheet("Draw Profiles")
    profile_sheet.append(
        [
            "Strands",
            "Stretch Ratio",
            "Slack Total (cm)",
            "Slack / Leg (cm)",
            "Draw Fraction",
            "Draw (cm)",
            "Stretch @ Draw",
            "Pull Force (N)",
            "Pull Force (kgf)",
            "Height (m)",
        ]
    )
    for plan in plans:
        for result in plan.draw_results:
            profile_sheet.append(
                [
                    plan.strands,
                    round(plan.stretch_ratio, 4),
                    round(plan.slack_total_cm, 4),
                    round(plan.slack_per_leg_cm, 4),
                    round(result.draw_fraction, 4),
                    round(result.draw_cm, 4),
                    round(result.stretch_ratio_actual, 4),
                    round(result.pull_force_N, 4),
                    round(result.pull_force_N / G, 4),
                    round(result.jump_height_m, 4),
                ]
            )

    wb.save(output_path)
    print(f"Excel export written to {output_path.resolve()}")


def analyze_winch_requirements(
    plans: Iterable[TubePlan],
    draw_time_s: float = 5.0,
    drum_radii_mm: Sequence[float] = (5.0, 10.0, 15.0, 20.0, 25.0),
    motor_no_load_rpm: float = 477.0,
    motor_stall_torque_Nm: float = 0.05,
    built_in_ratio: float = 30.0,
) -> None:
    plans_list = list(plans)
    if not plans_list:
        print("\nNo plans available for winch sizing.")
        return

    reference = max(plans_list, key=lambda plan: plan.max_height_m)
    stroke_m = reference.stroke_limit_cm / 100.0
    peak_force = reference.peak_force_N
    if stroke_m <= 0 or draw_time_s <= 0:
        print("\nInvalid stroke or draw time for winch sizing.")
        return

    linear_speed = stroke_m / draw_time_s
    print("\nWinch sizing against reference plan:")
    print(
        f"- Using plan with {reference.strands} strands, stretch ratio {reference.stretch_ratio:.2f},"
        f" stroke {reference.stroke_limit_cm:.1f} cm, peak force {peak_force:.1f} N"
    )
    header = (
        "Drum Ø (mm) | Radius (m) | Circumference (m) | Linear Speed (m/s) | "
        "Drum RPM | Torque Need (N·m) | Total Ratio (speed) | Total Ratio (torque) | Extra Stage"
    )
    print(header)
    print("-" * len(header))
    for diameter_mm in drum_radii_mm:
        radius_m = (diameter_mm / 2.0) / 1000.0
        if radius_m <= 0:
            continue
        circumference = 2.0 * math.pi * radius_m
        drum_rps = linear_speed / circumference
        drum_rpm = drum_rps * 60.0
        torque_required = peak_force * radius_m
        total_ratio_speed = motor_no_load_rpm / drum_rpm if drum_rpm > 0 else float("inf")
        total_ratio_torque = (torque_required / motor_stall_torque_Nm) if motor_stall_torque_Nm > 0 else float("inf")
        extra_stage_speed = total_ratio_speed / built_in_ratio if built_in_ratio > 0 else float("inf")
        extra_stage_torque = total_ratio_torque / built_in_ratio if built_in_ratio > 0 else float("inf")
        extra_stage_needed = max(extra_stage_speed, extra_stage_torque)
        print(
            f"{diameter_mm:11.1f} | "
            f"{radius_m:10.4f} | "
            f"{circumference:16.4f} | "
            f"{linear_speed:17.4f} | "
            f"{drum_rpm:8.1f} | "
            f"{torque_required:16.4f} | "
            f"{total_ratio_speed:17.2f} | "
            f"{total_ratio_torque:18.2f} | "
            f"{extra_stage_needed:11.2f}"
        )


def main(export_excel: bool = False, excel_path: Path | str = "tube_plan_results.xlsx") -> None:
    plans = compute_tube_plans()
    print_plan_table(plans)
    print_draw_profiles(plans)
    summarize_recommendation(plans)
    analyze_winch_requirements(plans)
    save_plots(plans)
    if export_excel:
        export_to_excel(plans, path=excel_path)
    else:
        print("Excel export skipped (use --export-excel to enable).")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compute and report tubing launch plans.")
    parser.add_argument(
        "--export-excel",
        action="store_true",
        default=False,
        help="Write results to an Excel spreadsheet (default: off)",
    )
    parser.add_argument(
        "--excel-path",
        default="tube_plan_results.xlsx",
        help="Destination path for the Excel export",
    )
    args = parser.parse_args()
    main(export_excel=args.export_excel, excel_path=args.excel_path)
