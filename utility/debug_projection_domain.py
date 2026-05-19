"""
Debug visualization: find which planning step causes
CurvilinearProjectionDomainLongitudinalError.

Key insight from stack-trace analysis:
  - the error occurs inside ReactivePlanner._check_kinematics
    (convert_to_cartesian_coords), NOT in _compute_initial_states.
  - _check_kinematics is executed in a multiprocessing subprocess, so
    exceptions crash the worker and leave the main process hanging on
    queue_1.get().

Strategy
--------
1. Force single-process mode for every planner instance so exceptions
   propagate normally to the main process.
2. Patch _check_kinematics to catch the domain error, mark the trajectory
   infeasible, and record the failing (s, d) sample.
3. Patch _compute_initial_states to record vehicle position + coord-system
   before each planning call.
4. Run the simulation; when the error (or any exception carrying it) is
   caught, mark the last record and stop.
5. Print a text summary and draw a matplotlib figure.

Usage
-----
    python utility/debug_projection_domain.py
"""

import copy
import os
import sys
import time
import traceback
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib
matplotlib.use("TkAgg")   # change to "Agg" if no display available
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
import numpy as np
import yaml

# ── repo root on path ─────────────────────────────────────────────────────────
path_root = Path(__file__).resolve().parent.parent
if str(path_root) not in sys.path:
    sys.path.insert(0, str(path_root))

# ── import the classes we will patch ─────────────────────────────────────────
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_clcs.clcs import CurvilinearCoordinateSystem

try:
    from commonroad_clcs.pycrccosy import (  # type: ignore[attr-defined]
        CartesianProjectionDomainError,
        CurvilinearProjectionDomainLateralError,
        CurvilinearProjectionDomainLongitudinalError,
    )
    _PROJECTION_DOMAIN_ERRORS = (
        CartesianProjectionDomainError,
        CurvilinearProjectionDomainLateralError,
        CurvilinearProjectionDomainLongitudinalError,
    )
except (ImportError, AttributeError):
    # Treat any ValueError/RuntimeError with the matching message as the trigger
    CartesianProjectionDomainError = (ValueError, RuntimeError)  # type: ignore[assignment,misc]
    CurvilinearProjectionDomainLateralError = (ValueError, RuntimeError)  # type: ignore[assignment,misc]
    CurvilinearProjectionDomainLongitudinalError = (ValueError, RuntimeError)  # type: ignore[assignment,misc]
    _PROJECTION_DOMAIN_ERRORS = (ValueError, RuntimeError)  # type: ignore[assignment,misc]

from source.commonroad_rp.reactive_planner import ReactivePlanner
from source.commonroad_rp.trajectories import FeasibilityStatus
from post_optimization_planner.state_machine import VehicleLeftScenarioError

# ── tee: duplicate stdout/stderr to a log file ───────────────────────────────
class _TeeStream:
    """Write to both the original stream and a log file simultaneously."""
    def __init__(self, original, log_file):
        self._original = original
        self._log = log_file

    def write(self, data):
        self._original.write(data)
        self._original.flush()
        self._log.write(data)
        self._log.flush()

    def flush(self):
        self._original.flush()
        self._log.flush()

    def fileno(self):
        return self._original.fileno()

    # Proxy every other attribute to the original stream
    def __getattr__(self, name):
        return getattr(self._original, name)


# ── global state ─────────────────────────────────────────────────────────────
_plan_records: List[dict] = []
_current_sm_state: str = "UNKNOWN"
_first_domain_error: Optional[dict] = None   # set when the first bad (s,d) is caught


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  PATCH 1 – force single-process mode
# ╚══════════════════════════════════════════════════════════════════════════════
_original_plan = ReactivePlanner.plan

def _patched_plan(self, *args, **kwargs):
    # Make _check_kinematics run in the main process so exceptions propagate.
    if hasattr(self, "config") and hasattr(self.config, "debug"):
        self.config.debug.multiproc = False
    _t_start = time.perf_counter()
    try:
        result = _original_plan(self, *args, **kwargs)
    finally:
        _elapsed_ms = (time.perf_counter() - _t_start) * 1000.0
        if _plan_records:
            _plan_records[-1]["elapsed_ms"] = _elapsed_ms
        step_idx = len(_plan_records) - 1 if _plan_records else -1
        sm_state = _plan_records[-1]["sm_state"] if _plan_records else "UNKNOWN"
        print(
            f"[timing] step {step_idx:>4d}  SM={sm_state:<16s}  "
            f"elapsed={_elapsed_ms:7.1f} ms"
        )
    return result

ReactivePlanner.plan = _patched_plan


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  PATCH 2 – record vehicle state + coord-sys before each planning attempt
# ╚══════════════════════════════════════════════════════════════════════════════
_original_compute_initial_states = ReactivePlanner._compute_initial_states

def _patched_compute_initial_states(self, x_0):
    record = {
        "step":        len(_plan_records),
        "sm_state":    _current_sm_state,
        "position":    copy.deepcopy(x_0.position),
        "velocity":    x_0.velocity,
        "orientation": x_0.orientation,
        "coord_sys":   self._co,
        "error":       None,
        "bad_sd":      None,   # (s, d) that fell outside domain
        "elapsed_ms":  None,
    }
    _plan_records.append(record)

    try:
        return _original_compute_initial_states(self, x_0)
    except CurvilinearProjectionDomainLongitudinalError as exc:
        # vehicle itself is outside domain (initial state projection failed)
        record["error"] = exc
        raise

ReactivePlanner._compute_initial_states = _patched_compute_initial_states


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  PATCH 3 – catch the domain error inside _check_kinematics and record it
# ╚══════════════════════════════════════════════════════════════════════════════
_original_check_kinematics = ReactivePlanner._check_kinematics

def _patched_check_kinematics(self, trajectories, queue_1=None, queue_2=None):
    """
    Run original _check_kinematics but intercept every
    CurvilinearProjectionDomainLongitudinalError raised by
    convert_to_cartesian_coords(s, d) and treat the trajectory as infeasible
    instead of crashing.  Record the first bad (s, d) for the visualiser.
    """
    global _first_domain_error

    # We rebuild the loop ourselves only to intercept per-trajectory errors.
    # Strategy: wrap each individual trajectory check.  The simplest approach
    # is to delegate to the original function but intercept at a coarser level:
    # run the original on one trajectory at a time.

    feasible_out = []
    infeasible_out = []

    for traj in trajectories:
        try:
            result = _original_check_kinematics(self, [traj])
            if result is None:
                # should not happen (we pass no queue), but guard anyway
                continue
            f, inf = result
            feasible_out.extend(f)
            infeasible_out.extend(inf)
        except _PROJECTION_DOMAIN_ERRORS as exc:
            # Mark trajectory infeasible and record the event (Lateral or Longitudinal)
            traj.feasibility_label = FeasibilityStatus.INFEASIBLE_KINEMATIC
            infeasible_out.append(traj)

            if _first_domain_error is None and _plan_records:
                rec = _plan_records[-1]
                rec["error"] = exc
                # Try to extract the s-value from the trajectory's curvilinear data
                bad_sd = _extract_bad_sd(traj, self._co)
                rec["bad_sd"] = bad_sd
                _first_domain_error = rec
                err_type = type(exc).__name__
                print(
                    f"\n[debug] {err_type} at step "
                    f"{rec['step']} (SM={rec['sm_state']}), "
                    f"pos=({rec['position'][0]:.2f}, {rec['position'][1]:.2f})"
                    + (f", bad s={bad_sd[0]:.2f}" if bad_sd else "")
                )

    if queue_1 is not None:
        queue_1.put(feasible_out)
        if self._draw_traj_set and queue_2 is not None:
            queue_2.put(infeasible_out)
        return None

    return feasible_out, infeasible_out

ReactivePlanner._check_kinematics = _patched_check_kinematics


def _extract_bad_sd(traj, co: CurvilinearCoordinateSystem):
    """Try to find which (s, d) in the trajectory is outside the domain."""
    try:
        # Access the raw curvilinear samples stored on the trajectory
        if hasattr(traj, "curvilinear") and traj.curvilinear is not None:
            s_arr = traj.curvilinear.s
            d_arr = traj.curvilinear.d
        elif hasattr(traj, "_long_traj") and traj._long_traj is not None:
            s_arr = traj._long_traj.s
            d_arr = getattr(traj, "_lat_traj", None)
            if d_arr is not None:
                d_arr = d_arr.d
            else:
                d_arr = np.zeros_like(s_arr)
        else:
            return None

        domain_len = co.length()
        for s, d in zip(s_arr, d_arr):
            if s < 0 or s > domain_len:
                return (float(s), float(d))
        # fallback: return the maximum s
        return (float(np.max(s_arr)), 0.0)
    except Exception:
        return None


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  PATCH 4 – keep _current_sm_state updated
# ╚══════════════════════════════════════════════════════════════════════════════
from post_optimization_planner import state_machine as _sm_module

_original_step = _sm_module.BaseStateMachinePlanner.step

def _patched_step(self, state_current, state_list):
    global _current_sm_state
    _current_sm_state = self.get_current_state_name()
    return _original_step(self, state_current, state_list)

_sm_module.BaseStateMachinePlanner.step = _patched_step


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  RUN SIMULATION
# ╚══════════════════════════════════════════════════════════════════════════════
def run_simulation():
    from source.simulation.simulations import simulate_with_planner

    config_path = path_root / "configurations" / "scenario.yaml"
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    bus_stop: str = cfg["scenario"]["type"]
    scenario_dir = path_root / "scenarios" / bus_stop

    print(f"[debug_projection_domain] Scenario: {bus_stop}")
    print("[debug_projection_domain] Multiprocessing disabled – exceptions now propagate.\n")

    try:
        simulate_with_planner(interactive_scenario_path=str(scenario_dir))
    except VehicleLeftScenarioError as exc:
        print(f"[INFO] {exc}")
    except _PROJECTION_DOMAIN_ERRORS as exc:
        if _plan_records:
            _plan_records[-1]["error"] = exc
        print(f"[debug] Caught {type(exc).__name__} (initial state).")
    except Exception as exc:
        tb = traceback.format_exc()
        if any(kw in tb for kw in (
            "CurvilinearProjectionDomainLongitudinalError",
            "CurvilinearProjectionDomainLateralError",
            "Longitudinal coordinate outside",
            "Lateral coordinate outside",
        )):
            print(f"[debug] Projection domain error wrapped in {type(exc).__name__}")
        else:
            print(f"[debug] Unexpected exception: {exc}")
            traceback.print_exc()

    return cfg, bus_stop


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  COLOUR MAP
# ╚══════════════════════════════════════════════════════════════════════════════
_STATE_COLOURS = {
    "DEPARTING":       "#2196F3",
    "HEADING":         "#4CAF50",
    "ARRIVING":        "#FF9800",
    "BEFORE_STOPPING": "#9C27B0",
    "STOPPING":        "#F44336",
    "UNKNOWN":         "#9E9E9E",
}


def _state_colour(name: str) -> str:
    return _STATE_COLOURS.get(name, "#9E9E9E")


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  VISUALISATION
# ╚══════════════════════════════════════════════════════════════════════════════
def load_cr_scenario(bus_stop: str):
    f = path_root / "scenarios" / bus_stop / f"{bus_stop}.cr.xml"
    scenario, _ = CommonRoadFileReader(str(f)).open()
    return scenario


def _draw_projection_domain(ax, co: CurvilinearCoordinateSystem,
                             colour: str, alpha: float = 0.12, label: str = ""):
    """Draw the projection domain polygon and reference path."""
    try:
        domain_poly = co.projection_domain()
        if hasattr(domain_poly, "exterior"):
            xs, ys = np.array(domain_poly.exterior.xy[0]), np.array(domain_poly.exterior.xy[1])
        else:
            pts = np.asarray(domain_poly)
            xs, ys = pts[:, 0], pts[:, 1]
        ax.fill(xs, ys, color=colour, alpha=alpha, zorder=2)
        ax.plot(xs, ys, color=colour, linewidth=0.8, alpha=0.5, zorder=2)
    except Exception:
        pass

    try:
        ref = co.ref_path
        ax.plot(ref[:, 0], ref[:, 1], "--", color=colour, linewidth=1.5,
                alpha=0.8, label=label if label else None, zorder=3)
        # start / end markers
        ax.scatter(ref[0, 0],  ref[0, 1],  marker="|", s=120, color=colour, zorder=4)
        ax.scatter(ref[-1, 0], ref[-1, 1], marker="|", s=120, color=colour, zorder=4)
    except Exception:
        pass


def _cart_from_sd(co: CurvilinearCoordinateSystem, s: float, d: float = 0.0):
    """Convert (s,d) to Cartesian, returning None on any error."""
    try:
        return co.convert_to_cartesian_coords(s, d)
    except Exception:
        return None


def visualise(cfg: dict, bus_stop: str):
    if not _plan_records:
        print("[debug] No planning records captured – nothing to visualise.")
        return

    # find the failing record (first with error set)
    error_idx = next((i for i, r in enumerate(_plan_records) if r["error"] is not None), None)

    scenario = load_cr_scenario(bus_stop)

    fig, axes = plt.subplots(1, 2, figsize=(20, 9),
                             gridspec_kw={"width_ratios": [2.2, 1]})
    ax_map, ax_tbl = axes

    total = len(_plan_records)
    err_str = f"step {error_idx}" if error_idx is not None else "no error caught"
    fig.suptitle(
        f"Projection Domain Debug  |  Scenario: {bus_stop}  |  "
        f"Total planning steps: {total}  |  Error at: {err_str}",
        fontsize=12,
    )

    # ── draw lanelets ─────────────────────────────────────────────────────────
    for ll in scenario.lanelet_network.lanelets:
        lv, rv = ll.left_vertices, ll.right_vertices
        poly = np.vstack([lv, rv[::-1], lv[0]])
        ax_map.fill(poly[:, 0], poly[:, 1], color="#EEEEEE", zorder=0)
        ax_map.plot(lv[:, 0], lv[:, 1], "k-", lw=0.6, zorder=1)
        ax_map.plot(rv[:, 0], rv[:, 1], "k-", lw=0.6, zorder=1)
        cx = ll.center_vertices[len(ll.center_vertices) // 2]
        ax_map.text(cx[0], cx[1], str(ll.lanelet_id),
                    fontsize=7, color="#555", ha="center", va="center", zorder=4)

    # ── draw one domain per unique (sm_state, coord_sys) pair ─────────────────
    seen = {}
    for rec in _plan_records:
        co = rec["coord_sys"]
        if co is None:
            continue
        key = (rec["sm_state"], id(co))
        if key not in seen:
            seen[key] = (rec["sm_state"], co)

    # Draw domains; collect one legend handle per SM state (deduplicated)
    seen_states_legend: set = set()
    legend_handles = []
    for (sm_state, co) in seen.values():
        c = _state_colour(sm_state)
        _draw_projection_domain(ax_map, co, c, alpha=0.10)
        if sm_state not in seen_states_legend:
            seen_states_legend.add(sm_state)
            legend_handles.append(mpatches.Patch(color=c, label=sm_state))

    # ── vehicle trajectory coloured by SM state ────────────────────────────────
    by_state: dict = {}
    for rec in _plan_records:
        by_state.setdefault(rec["sm_state"], []).append(rec["position"])

    for sm_state, positions in by_state.items():
        pts = np.array(positions)
        c = _state_colour(sm_state)
        ax_map.plot(pts[:, 0], pts[:, 1], "-", color=c, lw=1.0, alpha=0.6, zorder=5)
        ax_map.scatter(pts[:, 0], pts[:, 1], color=c, s=22, zorder=6, alpha=0.9)

    # ── highlight error step ───────────────────────────────────────────────────
    if error_idx is not None:
        rec = _plan_records[error_idx]
        ex, ey = rec["position"]

        ax_map.scatter([ex], [ey], color="red", s=200, marker="X", zorder=9)
        # Error annotation: place text to the UPPER-LEFT of the marker
        ax_map.annotate(
            f"Step {error_idx}  [{rec['sm_state']}]\n"
            f"pos=({ex:.2f}, {ey:.2f})\n"
            f"v={rec['velocity']:.2f} m/s",
            xy=(ex, ey), xytext=(ex - 30, ey + 8),
            fontsize=8, color="red", zorder=10,
            arrowprops=dict(arrowstyle="->", color="red", lw=1.2),
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="red", alpha=0.95),
        )

        # show domain length vs vehicle s-coordinate
        co = rec["coord_sys"]
        if co is not None:
            try:
                dom_len = co.length()
                ref = co.ref_path
                # annotate domain end
                ax_map.scatter([ref[-1, 0]], [ref[-1, 1]], color="darkred",
                               s=150, marker="*", zorder=8)
                # Domain end annotation: place text BELOW the end marker
                ax_map.annotate(
                    f"domain end  s={dom_len:.1f} m",
                    xy=(ref[-1, 0], ref[-1, 1]),
                    xytext=(ref[-1, 0] - 15, ref[-1, 1] - 10),
                    fontsize=8, color="darkred",
                    arrowprops=dict(arrowstyle="->", color="darkred", lw=0.9),
                    bbox=dict(boxstyle="round,pad=0.2", fc="lightyellow", ec="darkred", alpha=0.95),
                )
            except Exception:
                pass

            # if we captured a bad (s, d), convert to Cartesian and plot it
            bad = rec.get("bad_sd")
            if bad is not None:
                s_bad, d_bad = bad
                try:
                    # clamp s to domain to find approximate Cartesian location
                    dom_len = co.length()
                    s_clamped = min(s_bad, dom_len - 1e-3)
                    pt = _cart_from_sd(co, s_clamped, d_bad)
                    if pt is not None:
                        ax_map.scatter([pt[0]], [pt[1]], color="orange", s=150,
                                       marker="v", zorder=8)
                        ax_map.annotate(
                            f"last valid point\n(s={s_clamped:.1f} m)",
                            xy=(pt[0], pt[1]),
                            xytext=(pt[0] + 1, pt[1] - 2),
                            fontsize=8, color="darkorange",
                        )
                except Exception:
                    pass

                # draw an arrow extending beyond the domain end
                ax_info_text = (
                    f"\nFirst out-of-domain sample:\n"
                    f"  s = {s_bad:.2f} m\n"
                    f"  d = {d_bad:.3f} m\n"
                )
            else:
                ax_info_text = ""
        else:
            ax_info_text = ""
    else:
        ax_info_text = ""

    if error_idx is not None:
        legend_handles.append(mpatches.Patch(color="red", label="ERROR step"))
        legend_handles.append(
            Line2D([0], [0], marker="*", color="w", markerfacecolor="darkred",
                       markersize=10, label="domain end")
        )
        legend_handles.append(
            Line2D([0], [0], marker="v", color="w", markerfacecolor="orange",
                       markersize=9, label="last valid sample")
        )

    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.set_title("Map view – reference paths, domains and vehicle trajectory")
    # Place legend outside the axes (below), so it never overlaps the map
    ax_map.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.14),
        ncol=min(len(legend_handles), 5),
        fontsize=8,
        framealpha=0.9,
        edgecolor="#aaaaaa",
    )
    ax_map.grid(True, lw=0.4, alpha=0.5)

    # ── info table ─────────────────────────────────────────────────────────────
    ax_tbl.axis("off")

    headers = ["Step", "SM State", "x [m]", "y [m]", "v [m/s]", "t [ms]", "Error"]
    rows = []
    for rec in _plan_records:
        t_str = f"{rec['elapsed_ms']:.0f}" if rec["elapsed_ms"] is not None else "-"
        rows.append([
            rec["step"],
            rec["sm_state"],
            f"{rec['position'][0]:.2f}",
            f"{rec['position'][1]:.2f}",
            f"{rec['velocity']:.2f}",
            t_str,
            "YES ←" if rec["error"] is not None else "",
        ])

    max_rows = 45
    if len(rows) > max_rows:
        if error_idx is not None:
            s_row = max(0, error_idx - max_rows + 6)
            e_row = min(len(rows), error_idx + 6)
        else:
            s_row, e_row = len(rows) - max_rows, len(rows)
        shown = rows[s_row:e_row]
        title_str = f"Steps {s_row}\u2013{e_row - 1}  (showing {len(shown)} / {len(rows)})"
    else:
        shown = rows
        s_row = 0
        title_str = f"All {len(rows)} planning steps"

    # Place the title as figure text above the table panel (avoids overlapping table cells)
    ax_tbl.set_title(title_str, fontsize=9, pad=6)

    tbl = ax_tbl.table(cellText=shown, colLabels=headers,
                       loc="upper center", cellLoc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7.5)
    tbl.scale(1.0, 1.15)

    # highlight error row
    if error_idx is not None:
        rel = error_idx - s_row
        if 0 <= rel < len(shown):
            for col in range(len(headers)):
                tbl[(rel + 1, col)].set_facecolor("#FFCCCC")

    # extra text about bad sample
    if ax_info_text:
        ax_tbl.text(0.05, 0.02, ax_info_text, transform=ax_tbl.transAxes,
                    fontsize=8, va="bottom", color="darkred",
                    bbox=dict(boxstyle="round", fc="lightyellow", ec="orange"))

    plt.tight_layout()
    # Leave room at the bottom for the legend that sits outside ax_map
    plt.subplots_adjust(bottom=0.18)
    out = path_root / "experiments" / "output_result" / "debug_projection_domain.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out), dpi=150, bbox_inches="tight")
    print(f"\n[debug] Figure saved to {out}")
    plt.show()


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  TEXT SUMMARY
# ╚══════════════════════════════════════════════════════════════════════════════
def print_summary():
    error_idx = next((i for i, r in enumerate(_plan_records) if r["error"] is not None), None)
    print("\n" + "=" * 65)
    times_ms = [r["elapsed_ms"] for r in _plan_records if r["elapsed_ms"] is not None]
    print(f"  Total planning steps recorded : {len(_plan_records)}")
    if times_ms:
        print(f"  Planning time  min / mean / max : "
              f"{min(times_ms):.1f} / {sum(times_ms)/len(times_ms):.1f} / {max(times_ms):.1f} ms")
        slowest = max(range(len(_plan_records)),
                      key=lambda i: _plan_records[i]["elapsed_ms"] or 0)
        print(f"  Slowest step  : {slowest}  "
              f"({_plan_records[slowest]['elapsed_ms']:.1f} ms, "
              f"SM={_plan_records[slowest]['sm_state']})")
    if error_idx is not None:
        rec = _plan_records[error_idx]
        print(f"  *** ERROR at step {error_idx} ***")
        print(f"  SM state    : {rec['sm_state']}")
        print(f"  Position    : x={rec['position'][0]:.4f}  y={rec['position'][1]:.4f}")
        print(f"  Velocity    : {rec['velocity']:.4f} m/s")
        print(f"  Orientation : {rec['orientation']:.5f} rad")

        co = rec["coord_sys"]
        if co is not None:
            try:
                ref = co.ref_path
                print(f"  Ref path x  : [{ref[:, 0].min():.2f}, {ref[:, 0].max():.2f}]")
                print(f"  Ref path y  : [{ref[:, 1].min():.2f}, {ref[:, 1].max():.2f}]")
            except Exception:
                pass
            try:
                print(f"  Domain len  : {co.length():.2f} m")
            except Exception:
                pass
            try:
                inside = co.cartesian_point_inside_projection_domain(
                    rec["position"][0], rec["position"][1]
                )
                print(f"  Ego inside domain? {inside}")
            except Exception:
                pass

        bad = rec.get("bad_sd")
        if bad:
            print(f"  Bad sample  : s={bad[0]:.4f}  d={bad[1]:.4f}")
            try:
                print(f"  Exceeds domain by: {bad[0] - co.length():.4f} m")
            except Exception:
                pass
    else:
        print("  No CurvilinearProjectionDomainLongitudinalError recorded.")
        if _plan_records:
            print("  (Simulation may have completed without error, "
                  "or error was not in _check_kinematics / _compute_initial_states)")
    print("=" * 65)


# ── entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # ── set up logging to file ────────────────────────────────────────────────
    _log_dir = path_root / "experiments" / "output_result"
    _log_dir.mkdir(parents=True, exist_ok=True)
    _log_ts = time.strftime("%Y%m%d_%H%M%S")
    _log_path = _log_dir / f"debug_projection_domain_{_log_ts}.log"
    _log_file = open(_log_path, "w", encoding="utf-8", buffering=1)
    sys.stdout = _TeeStream(sys.__stdout__, _log_file)
    sys.stderr = _TeeStream(sys.__stderr__, _log_file)
    print(f"[log] Output is being recorded to: {_log_path}")
    # ─────────────────────────────────────────────────────────────────────────
    try:
        cfg, bus_stop = run_simulation()
        print_summary()
        visualise(cfg, bus_stop)
    finally:
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        _log_file.close()
        print(f"[log] Log saved to: {_log_path}")
