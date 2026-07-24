"""Plot the velocity profile for a complete parking process."""

from __future__ import annotations

import argparse
import csv
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DT = 0.1


def scenario_result_dir(scenario: str) -> Path:
    if scenario == "bus_stop_bay":
        return PROJECT_ROOT / "experiments" / "output_result" / "result_bay"
    if scenario == "bus_stop_bulb":
        return PROJECT_ROOT / "experiments" / "output_result" / "result_bulb"
    return PROJECT_ROOT / "experiments" / "output_result"


def default_solution_path() -> Path:
    """Resolve the solution selected in configurations/scenario.yaml."""
    config_path = PROJECT_ROOT / "configurations" / "scenario.yaml"
    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    scenario = config["scenario"]["type"]
    use_post_opt = bool(config["debug"]["use_post_opt"])
    scenario_id = scenario.replace("_", "")
    filename = f"solution_KS1:WX1:DEU_{scenario_id}-1:2020a.xml"
    return scenario_result_dir(scenario) / f"{scenario}_{use_post_opt}" / filename


def load_velocity_profile(solution_path: Path, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Read time and velocity data from a CSV export or CommonRoad solution XML."""
    if solution_path.suffix.lower() == ".csv":
        with solution_path.open("r", encoding="utf-8", newline="") as csv_file:
            rows = list(csv.DictReader(csv_file))
        if not rows:
            raise ValueError(f"No trajectory rows found in {solution_path}")

        time = np.array([float(row["time_s"]) for row in rows], dtype=float)
        velocities = np.array(
            [float(row["velocity_mps"]) for row in rows], dtype=float
        )
        return time, velocities

    root = ET.parse(solution_path).getroot()
    states = root.findall(".//ksState")
    if not states:
        raise ValueError(f"No ksState entries found in {solution_path}")

    time_steps = np.array([int(state.findtext("time")) for state in states])
    velocities = np.array(
        [float(state.findtext("velocity")) for state in states], dtype=float
    )
    elapsed_time = (time_steps - time_steps[0]) * dt
    return elapsed_time, velocities


def longest_stationary_interval(
    velocities: np.ndarray, threshold: float
) -> tuple[int, int] | None:
    """Return the longest inclusive index interval below the speed threshold."""
    stationary_indices = np.flatnonzero(np.abs(velocities) <= threshold)
    if stationary_indices.size == 0:
        return None

    split_points = np.flatnonzero(np.diff(stationary_indices) > 1) + 1
    runs = np.split(stationary_indices, split_points)
    longest_run = max(runs, key=len)
    return int(longest_run[0]), int(longest_run[-1])


def plot_velocity_profile(
    time: np.ndarray,
    velocity: np.ndarray,
    output_path: Path,
    stationary_threshold: float,
    lane_change_to_stop_interval: tuple[float, float] | None = None,
    stationary_phase_label: str = "Stationary phase",
    stationary_duration_label: str = "Stop",
    stationary_label_y_fraction: float = 0.25,
) -> None:
    """Create and save an academic-style velocity profile line chart."""
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 11,
            "axes.labelsize": 12,
            "axes.titlesize": 13,
            "legend.fontsize": 10,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
        }
    )

    fig, ax = plt.subplots(figsize=(9, 4.8))
    stopping_procedure_end: float | None = None

    if lane_change_to_stop_interval is not None:
        interval_start, interval_end = lane_change_to_stop_interval
        if interval_end > interval_start:
            interval_start = max(float(time[0]), float(interval_start))
            interval_end = min(float(time[-1]), float(interval_end))
            duration_s = interval_end - interval_start
            stopping_procedure_end = interval_end
            ax.axvspan(
                interval_start,
                interval_end,
                color="#F4A261",
                alpha=0.22,
                label="Stopping procedure",
                zorder=0,
            )
            ax.axvline(interval_start, color="#B65F00", linestyle="--", linewidth=0.9)
            ax.axvline(interval_end, color="#B65F00", linestyle="--", linewidth=0.9)
            annotation_x = interval_start + duration_s * 0.6
            ax.annotate(
                f"Stopping procedure\n{duration_s:.2f} s",
                xy=(annotation_x, max(velocity) * 0.78),
                xytext=(annotation_x, max(velocity) * 0.78),
                ha="center",
                va="center",
                color="#7A3A00",
                bbox={
                    "boxstyle": "round,pad=0.25",
                    "fc": "white",
                    "ec": "#B65F00",
                    "alpha": 0.92,
                },
            )

    ax.plot(time, velocity, color="#0066CC", linewidth=1.8, label="Vehicle velocity")
    ax.fill_between(time, velocity, color="#0066CC", alpha=0.08)

    stationary = longest_stationary_interval(velocity, stationary_threshold)
    if stationary is not None:
        start_idx, end_idx = stationary
        start_time = float(time[start_idx])
        end_time = time[end_idx]
        if stopping_procedure_end is not None and start_time < stopping_procedure_end:
            start_time = stopping_procedure_end
        if end_time <= start_time:
            start_time = float(time[start_idx])
        ax.axvspan(
            start_time,
            end_time,
            color="#BDBDBD",
            alpha=0.45,
            label=stationary_phase_label,
        )
        ax.axvline(start_time, color="#666666", linestyle="--", linewidth=0.9)
        ax.axvline(end_time, color="#666666", linestyle="--", linewidth=0.9)
        ax.annotate(
            f"{stationary_duration_label}: {end_time - start_time:.1f} s",
            xy=((start_time + end_time) / 2, 0),
            xytext=(
                (start_time + end_time) / 2,
                max(velocity) * stationary_label_y_fraction,
            ),
            ha="center",
            arrowprops={"arrowstyle": "->", "color": "#555555", "linewidth": 0.9},
        )

    max_idx = int(np.argmax(velocity))
    ax.scatter(
        time[max_idx],
        velocity[max_idx],
        color="#CC0000",
        s=24,
        zorder=3,
    )
    ax.annotate(
        f"Maximum: {velocity[max_idx]:.2f} m/s",
        xy=(time[max_idx], velocity[max_idx]),
        xytext=(time[max_idx] + 4, velocity[max_idx] * 1.045),
        arrowprops={"arrowstyle": "->", "color": "#555555", "linewidth": 0.9},
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_xlim(time[0], time[-1])
    ax.set_ylim(0, max(velocity) * 1.14)
    ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.35)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.legend(loc="upper right", frameon=True, edgecolor="#777777")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot a velocity profile from a CommonRoad solution XML file."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=PROJECT_ROOT
        / "experiments"
        / "output_result"
        / "result_bay"
        / "bus_stop_bay_opt_trajectory.csv",
        help="Trajectory CSV export or CommonRoad solution XML file.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT
        / "experiments"
        / "output_result"
        / "result_bay"
        / "bus_stop_bay_opt_velocity_profile.png",
        help="Output image path; the extension selects the file format.",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=DEFAULT_DT,
        help="Duration of one trajectory time step in seconds.",
    )
    parser.add_argument(
        "--stationary-threshold",
        type=float,
        default=0.05,
        help="Absolute velocity at or below which the vehicle is stationary.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    time, velocity = load_velocity_profile(args.input, args.dt)
    plot_velocity_profile(
        time,
        velocity,
        args.output,
        args.stationary_threshold,
    )
    print(f"Velocity profile saved to {args.output}")


if __name__ == "__main__":
    main()
