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
    python utility/bus_stop_bulb_opt.py
"""

import copy
import csv
import builtins
import io
import os
import sys
import time
import traceback
from contextlib import contextmanager
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib
_backend = os.environ.get("MPLBACKEND")
if _backend is None:
    _backend = "TkAgg" if os.environ.get("DEBUG_PROJECTION_INTERACTIVE") == "1" else "Agg"
matplotlib.use(_backend)
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
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, TrafficSignIDGermany
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.state import ExtendedPMState, InitialState
from commonroad.visualization.draw_params import DynamicObstacleParams, MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
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
from post_optimization_planner.State import _extend_path
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
_first_domain_error: Optional[dict] = None    # set when the first bad (s,d) is caught
_active_plan_record: Optional[dict] = None    # the record being filled during plan()
_pre_plan_states: list = []                   # _compute_initial_states calls from outside plan()
_plan_call_count: int = 0                     # total plan() invocations (for sanity checks)
_lane_change_to_stop_timer: dict = {
    "lane_change_state": "ARRIVING",
    "armed": False,
    "reference_y": None,
    "reference_orientation": None,
    "scan_index": 0,
    "lane_change_step": None,
    "lane_change_time_s": None,
    "stop_step": None,
    "stop_time_s": None,
    "duration_s": None,
}


_DEBUG_MAX_REQUIRED_DECEL_RATIO = 0.8
_TARGET_SCENARIO = "bus_stop_bulb"


def _sampling_attr(config, name: str, default=None):
    return getattr(getattr(config, "sampling", None), name, default)


def _planning_attr(config, name: str, default=None):
    return getattr(getattr(config, "planning", None), name, default)


def _state_list_time_s(state_list: list, dt: float = 0.1) -> Optional[float]:
    """Return executed simulation time from the continuous state_list timeline."""
    if not state_list:
        return None
    try:
        return float(getattr(state_list[-1], "time_step", len(state_list) - 1)) * dt
    except (TypeError, ValueError):
        return None


def _record_lane_change_to_stop_timing(before: str, after: str, state_list: list) -> None:
    """Measure elapsed simulation time from lane-change start to complete standstill."""
    sim_time_s = _state_list_time_s(state_list)
    sim_step = len(state_list) - 1 if state_list else None
    lane_change_state = _lane_change_to_stop_timer["lane_change_state"]

    if (
        before != lane_change_state
        and after == lane_change_state
        and not _lane_change_to_stop_timer["armed"]
    ):
        reference_state = state_list[-1] if state_list else None
        _lane_change_to_stop_timer["armed"] = True
        _lane_change_to_stop_timer["reference_y"] = (
            float(reference_state.position[1]) if reference_state is not None else None
        )
        _lane_change_to_stop_timer["reference_orientation"] = (
            float(getattr(reference_state, "orientation", 0.0)) if reference_state is not None else None
        )
        _lane_change_to_stop_timer["scan_index"] = len(state_list) if state_list else 0
        if sim_time_s is not None:
            print(
                f"[stop-timing] lane-change timing armed in {after} at "
                f"sim_step={sim_step}, t={sim_time_s:.2f}s"
            )

    if (
        _lane_change_to_stop_timer["armed"]
        and _lane_change_to_stop_timer["lane_change_time_s"] is None
        and state_list
    ):
        reference_y = _lane_change_to_stop_timer["reference_y"]
        reference_orientation = _lane_change_to_stop_timer["reference_orientation"]
        scan_index = int(_lane_change_to_stop_timer["scan_index"] or 0)
        for idx in range(max(0, scan_index), len(state_list)):
            state = state_list[idx]
            lateral_shift = (
                abs(float(state.position[1]) - reference_y)
                if reference_y is not None else 0.0
            )
            orientation_change = (
                abs(float(getattr(state, "orientation", 0.0)) - reference_orientation)
                if reference_orientation is not None else 0.0
            )
            steering = abs(float(getattr(state, "steering_angle", 0.0)))
            if lateral_shift >= 0.05 or orientation_change >= 0.01 or steering >= 0.01:
                lane_change_time_s = _state_list_time_s(state_list[:idx + 1])
                _lane_change_to_stop_timer["lane_change_step"] = idx
                _lane_change_to_stop_timer["lane_change_time_s"] = lane_change_time_s
                if lane_change_time_s is not None:
                    print(
                        f"[stop-timing] lane change started at "
                        f"sim_step={idx}, t={lane_change_time_s:.2f}s "
                        f"(dy={lateral_shift:.2f}m, dpsi={orientation_change:.3f}rad, "
                        f"steering={steering:.3f}rad)"
                    )
                break
        _lane_change_to_stop_timer["scan_index"] = len(state_list)

    if (
        _lane_change_to_stop_timer["lane_change_time_s"] is not None
        and _lane_change_to_stop_timer["stop_time_s"] is None
        and state_list
        and abs(float(getattr(state_list[-1], "velocity", 0.0))) < 1e-5
    ):
        _lane_change_to_stop_timer["stop_step"] = sim_step
        _lane_change_to_stop_timer["stop_time_s"] = sim_time_s
        start = _lane_change_to_stop_timer["lane_change_time_s"]
        if start is not None and sim_time_s is not None:
            duration_s = sim_time_s - start
            _lane_change_to_stop_timer["duration_s"] = duration_s
            print(
                f"[stop-timing] lane change start -> complete stop duration: "
                f"{duration_s:.2f}s "
                f"(sim_step {_lane_change_to_stop_timer['lane_change_step']} -> {sim_step})"
            )


def _orientation_error_to_reference(co: CurvilinearCoordinateSystem, state) -> Optional[float]:
    """Return heading error w.r.t. the reference path at the ego projection."""
    if co is None or state is None:
        return None
    try:
        s, _ = co.convert_to_curvilinear_coords(state.position[0], state.position[1])
        s_idx = int(np.argmax(co.ref_pos > s) - 1)
        s_idx = max(0, min(s_idx, len(co.ref_pos) - 2))
        theta_ref = np.unwrap(co.ref_theta)
        denom = co.ref_pos[s_idx + 1] - co.ref_pos[s_idx]
        if abs(denom) < 1e-9:
            ref_theta = theta_ref[s_idx]
        else:
            lam = (s - co.ref_pos[s_idx]) / denom
            ref_theta = theta_ref[s_idx] + lam * (theta_ref[s_idx + 1] - theta_ref[s_idx])
        return float(np.arctan2(np.sin(state.orientation - ref_theta),
                                np.cos(state.orientation - ref_theta)))
    except Exception:
        return None


def _distance_to_stop_goal(planner) -> Optional[float]:
    try:
        if planner.x_0_cl is None or planner._desired_lon_position is None:
            return None
        return float(planner._desired_lon_position - planner.x_0_cl[0][0])
    except Exception:
        return None


def _record_planner_context(record: dict, planner) -> None:
    """Capture planner/config values that explain no-trajectory fallbacks."""
    config = getattr(planner, "config", None)
    if config is None:
        return

    horizon = float(getattr(planner, "horizon", _planning_attr(config, "dt", 0.1) *
                            _planning_attr(config, "time_steps_computation", 0)))
    x_0 = getattr(planner, "x_0", None)
    current_v = float(getattr(x_0, "velocity", 0.0)) if x_0 is not None else None
    desired_v = getattr(planner, "_desired_speed", None)
    desired_v = float(desired_v) if desired_v is not None else None
    a_req_velocity = None
    if current_v is not None and desired_v is not None and horizon > 1e-9:
        a_req_velocity = (desired_v - current_v) / horizon

    distance_to_goal = _distance_to_stop_goal(planner)
    a_req_stop_distance = None
    if current_v is not None and distance_to_goal is not None and distance_to_goal > 1e-6:
        a_req_stop_distance = -(current_v ** 2) / (2.0 * distance_to_goal)

    record.setdefault("planner_returned_none", False)
    record.setdefault("fallback_result", None)
    record.setdefault("fallback_reason_hint", None)

    record.update({
        "longitudinal_mode": _sampling_attr(config, "longitudinal_mode"),
        "desired_speed": desired_v,
        "desired_lon_position": getattr(planner, "_desired_lon_position", None),
        "low_vel_mode": getattr(planner, "_low_vel_mode", None),
        "horizon": horizon,
        "dt": _planning_attr(config, "dt"),
        "time_steps_computation": _planning_attr(config, "time_steps_computation"),
        "replanning_frequency": _planning_attr(config, "replanning_frequency"),
        "v_min": _sampling_attr(config, "v_min"),
        "v_max": _sampling_attr(config, "v_max"),
        "d_min": _sampling_attr(config, "d_min"),
        "d_max": _sampling_attr(config, "d_max"),
        "s_min": _sampling_attr(config, "s_min"),
        "s_max": _sampling_attr(config, "s_max"),
        "a_max": getattr(getattr(config, "vehicle", None), "a_max", None),
        "x0_cl": copy.deepcopy(getattr(planner, "x_0_cl", None)),
        "orientation_error_ref": _orientation_error_to_reference(getattr(planner, "_co", None), x_0),
        "distance_to_stop_goal": distance_to_goal,
        "required_decel_to_desired_v": a_req_velocity,
        "required_decel_to_stop_goal": a_req_stop_distance,
        "total_samples": getattr(planner, "total_count_samples", None),
        "infeasible_kinematics": getattr(planner, "infeasible_count_kinematics", None),
        "infeasible_collision": getattr(planner, "infeasible_count_collision", None),
        "infeasible_reasons": copy.deepcopy(getattr(planner, "infeasible_reason_dict", None)),
    })


def _update_fallback_hint(record: dict) -> None:
    a_max = record.get("a_max")
    hints = []
    req_v = record.get("required_decel_to_desired_v")
    if a_max and req_v is not None and req_v < -_DEBUG_MAX_REQUIRED_DECEL_RATIO * float(a_max):
        hints.append(f"desired velocity requires {abs(req_v):.2f} m/s^2 decel")

    req_stop = record.get("required_decel_to_stop_goal")
    if a_max and req_stop is not None and req_stop < -_DEBUG_MAX_REQUIRED_DECEL_RATIO * float(a_max):
        hints.append(f"stop target requires {abs(req_stop):.2f} m/s^2 decel")

    theta = record.get("orientation_error_ref")
    if theta is not None and abs(theta) > 0.35:
        hints.append(f"orientation error {theta:.2f} rad")

    if record.get("x0_cl") is None:
        hints.append("initial state outside coordinate system")

    total = record.get("total_samples")
    kin = record.get("infeasible_kinematics")
    coll = record.get("infeasible_collision")
    if total and kin == total and not coll:
        reasons = record.get("infeasible_reasons") or {}
        dominant = sorted(
            ((name, count) for name, count in reasons.items() if count),
            key=lambda item: item[1],
            reverse=True,
        )
        if dominant:
            hints.append(f"all samples kinematically infeasible ({dominant[0][0]})")
        else:
            hints.append("all samples kinematically infeasible")

    record["fallback_reason_hint"] = "; ".join(hints) if hints else "no obvious threshold violation"


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  PATCH 1 – force single-process mode + one record per plan() call
# ╚══════════════════════════════════════════════════════════════════════════════
_original_plan = ReactivePlanner.plan

def _patched_plan(self, *args, **kwargs):
    global _active_plan_record, _plan_call_count

    # Make _check_kinematics run in the main process so exceptions propagate.
    if hasattr(self, "config") and hasattr(self.config, "debug"):
        self.config.debug.multiproc = False

    # ── create exactly one record for this plan() call ────────────────────────
    record: dict = {
        "step":                         len(_plan_records),
        "sm_state":                     _current_sm_state,
        "position":                     None,   # filled by _compute_initial_states
        "velocity":                     None,
        "orientation":                  None,
        "coord_sys":                    None,
        "error":                        None,
        "bad_sd":                       None,
        "elapsed_ms":                   None,
        "compute_initial_states_calls": 0,
        "planner_returned_none":        False,
        "fallback_result":              None,
        "fallback_reason_hint":         None,
        "virtual_substate":             None,
    }
    # Pre-populate from the most recent pre-plan state (set_x_0 call) as a
    # fallback in case _compute_initial_states is not called inside plan().
    if _pre_plan_states:
        pre = _pre_plan_states[-1]
        record["position"]    = pre["position"]
        record["velocity"]    = pre["velocity"]
        record["orientation"] = pre["orientation"]
        record["coord_sys"]   = pre["coord_sys"]

    _plan_records.append(record)
    _active_plan_record = record
    _plan_call_count += 1
    _record_planner_context(record, self)

    _t_start = time.perf_counter()
    try:
        result = _original_plan(self, *args, **kwargs)
        if result is None:
            record["planner_returned_none"] = True
            _update_fallback_hint(record)
        _record_planner_context(record, self)
    except Exception as exc:
        record["error"] = exc
        _record_planner_context(record, self)
        _update_fallback_hint(record)
        raise
    finally:
        _elapsed_ms = (time.perf_counter() - _t_start) * 1000.0
        record["elapsed_ms"] = _elapsed_ms
        _active_plan_record = None

        step_idx = record["step"]
        sm_state = record["sm_state"]
        cis_calls = record["compute_initial_states_calls"]
        status = " fallback" if record.get("planner_returned_none") else ""
        extra = f"  [init_calls={cis_calls}]" if cis_calls != 1 else ""
        print(
            f"[timing] step {step_idx:>4d}  SM={sm_state:<16s}  "
            f"elapsed={_elapsed_ms:7.1f} ms{extra}{status}"
        )

        # ── periodic sanity check ─────────────────────────────────────────────
        if _plan_call_count % 50 == 0:
            normal = [r for r in _plan_records if "source" not in r]
            bad_time = [r for r in normal if r["elapsed_ms"] is None]
            bad_cis  = [r for r in normal if r["compute_initial_states_calls"] < 1]
            if len(normal) != _plan_call_count:
                print(f"[sanity] WARNING: {len(normal)} normal records "
                      f"but {_plan_call_count} plan calls")
            elif bad_time:
                print(f"[sanity] WARNING: {len(bad_time)} record(s) with elapsed_ms=None")
            elif bad_cis:
                print(f"[sanity] WARNING: {len(bad_cis)} record(s) with "
                      f"compute_initial_states_calls=0")
            else:
                print(f"[sanity] OK at plan call {_plan_call_count}: "
                      f"all {len(normal)} records valid.")

    return result

ReactivePlanner.plan = _patched_plan


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  PATCH 2 – update the active plan record; do NOT append a new record
# ╚══════════════════════════════════════════════════════════════════════════════
_original_compute_initial_states = ReactivePlanner._compute_initial_states

def _patched_compute_initial_states(self, x_0):
    global _active_plan_record, _pre_plan_states

    if _active_plan_record is not None:
        # Inside plan(): update the single active record with the latest state.
        _active_plan_record["compute_initial_states_calls"] += 1
        _active_plan_record["position"]    = copy.deepcopy(x_0.position)
        _active_plan_record["velocity"]    = x_0.velocity
        _active_plan_record["orientation"] = x_0.orientation
        _active_plan_record["coord_sys"]   = self._co
    else:
        # Outside plan() (e.g. called from set_x_0) – store as pre-plan
        # diagnostic; do NOT touch _plan_records.
        _pre_plan_states.append({
            "source":      "outside_plan",
            "sm_state":    _current_sm_state,
            "position":    copy.deepcopy(x_0.position),
            "velocity":    x_0.velocity,
            "orientation": x_0.orientation,
            "coord_sys":   self._co,
        })

    try:
        result = _original_compute_initial_states(self, x_0)
        if _active_plan_record is not None:
            _active_plan_record["x0_cl"] = copy.deepcopy(result)
            _active_plan_record["orientation_error_ref"] = _orientation_error_to_reference(self._co, x_0)
            if result is None:
                _active_plan_record["fallback_reason_hint"] = "initial state outside coordinate system"
        return result
    except CurvilinearProjectionDomainLongitudinalError as exc:
        if _active_plan_record is not None:
            _active_plan_record["error"] = exc
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
    before = self.get_current_state_name()
    _current_sm_state = before
    next_state = _original_step(self, state_current, state_list)
    after = self.get_current_state_name()
    _record_lane_change_to_stop_timing(before, after, state_list)
    return next_state

_sm_module.BaseStateMachinePlanner.step = _patched_step


def _diagnose_transition_blockers(state_name: str, next_state, config, goal_x: float) -> List[str]:
    blockers: List[str] = []
    if next_state is None:
        return ["next_state is None"]

    distance_to_goal = abs(float(goal_x) - float(next_state.position[0]))
    desired_v = float(_sampling_attr(config, "desire_velocity", 0.0) or 0.0)

    if state_name == "DEPARTING":
        if next_state.velocity < desired_v:
            blockers.append(f"velocity {next_state.velocity:.2f} < desired {desired_v:.2f}")
        orientation = float(getattr(next_state, "orientation", 0.0))
        acceleration = float(getattr(next_state, "acceleration", 0.0))
        if orientation >= 1e-5:
            blockers.append(f"orientation {orientation:.6f} >= 1e-5")
        if acceleration >= 1e-5:
            blockers.append(f"acceleration {acceleration:.6f} >= 1e-5")
    elif state_name == "HEADING":
        threshold = float(_planning_attr(config, "distance_heading_to_next_to_arriving", 0.0) or 0.0)
        if distance_to_goal >= threshold or next_state.position[0] >= goal_x:
            blockers.append(f"distance {distance_to_goal:.2f} >= threshold {threshold:.2f}")
    elif state_name == "ARRIVING":
        if getattr(config, "planning", None) is None:
            return blockers
        threshold = getattr(config.planning, "distance_arriving_to_stopping", None)
        if threshold is None:
            threshold = getattr(config.planning, "distance_arriving_to_next_to_before_stopping", None)
        if threshold is not None and distance_to_goal >= float(threshold):
            blockers.append(f"distance {distance_to_goal:.2f} >= threshold {float(threshold):.2f}")
    elif state_name == "BEFORE_STOPPING_ALIGN":
        threshold = float(_planning_attr(config, "distance_arriving_to_next_to_before_stopping", 0.0) or 0.0)
        if distance_to_goal >= threshold:
            blockers.append(f"distance {distance_to_goal:.2f} >= merge threshold {threshold:.2f}")
    elif state_name == "BEFORE_STOPPING_MERGE":
        threshold = float(_planning_attr(config, "distance_arriving_to_stopping", 0.0) or 0.0)
        if distance_to_goal >= threshold:
            blockers.append(f"distance {distance_to_goal:.2f} >= final threshold {threshold:.2f}")
    elif state_name in ("BEFORE_STOPPING_FINAL", "BEFORE_STOPPING"):
        threshold = float(_planning_attr(config, "distance_before_stopping_to_stopping", 0.0) or 0.0)
        if distance_to_goal >= threshold:
            blockers.append(f"distance {distance_to_goal:.2f} >= stopping threshold {threshold:.2f}")

    return blockers


_original_check_state_transition = _sm_module.BaseStateMachinePlanner._check_state_transition

def _patched_check_state_transition(self, next_state, config) -> None:
    before = self.get_current_state_name()
    blockers = _diagnose_transition_blockers(before, next_state, config, self.goal_x)
    _original_check_state_transition(self, next_state, config)
    after = self.get_current_state_name()

    if _plan_records:
        rec = _plan_records[-1]
        rec["transition_from"] = before
        rec["transition_to"] = after if after != before else None
        rec["transition_blockers"] = [] if after != before else blockers

    if after != before:
        print(f"[transition] {before} -> {after}")
    elif blockers and (before == "DEPARTING" or before.startswith("BEFORE_STOPPING")):
        print(f"[transition] {before} held: {'; '.join(blockers)}")

_sm_module.BaseStateMachinePlanner._check_state_transition = _patched_check_state_transition


_original_plan_and_optimize = _sm_module.BaseStateMachinePlanner._plan_and_optimize

def _patched_plan_and_optimize(self, planner, config, state_list, is_stopping: bool = False):
    global _current_sm_state
    start_idx = len(_plan_records)
    cycle_start = time.perf_counter()
    next_state = None
    trajectory = None
    previous_sm_state = _current_sm_state
    _current_sm_state = getattr(self, "_active_planning_state_name", None) or self.get_current_state_name()
    try:
        next_state, trajectory = _original_plan_and_optimize(
            self, planner, config, state_list, is_stopping=is_stopping
        )
        if (
            trajectory is not None
            and float(getattr(planner.x_0, "velocity", 0.0)) <= 1e-3
            and abs(float(getattr(planner.x_0, "orientation", 0.0))) > 1e-3
        ):
            print("[restart-debug] first replanned states:")
            for idx, state in enumerate(
                trajectory[0].state_list[:config.planning.replanning_frequency + 1]
            ):
                print(
                    f"  i={idx} position=({state.position[0]:.6f}, "
                    f"{state.position[1]:.6f}) "
                    f"orientation={float(state.orientation):.6f} "
                    f"velocity={float(state.velocity):.6f} "
                    f"yaw_rate={float(getattr(state, 'yaw_rate', 0.0)):.6f}"
                )
        return next_state, trajectory
    finally:
        if len(_plan_records) > start_idx:
            cycle_records = _plan_records[start_idx:]
            rec = cycle_records[-1]
            rec["cycle_elapsed_ms"] = (time.perf_counter() - cycle_start) * 1000.0
            if trajectory is not None:
                for earlier_record in cycle_records[:-1]:
                    if earlier_record.get("planner_returned_none"):
                        earlier_record["planner_returned_none"] = False
                        earlier_record["recovered_by_dense_replan"] = True
            elif getattr(self, "_fallback_braking_active", False):
                for earlier_record in cycle_records[:-1]:
                    if earlier_record.get("planner_returned_none"):
                        earlier_record["planner_returned_none"] = False
                        earlier_record["superseded_by_braking_fallback"] = True
            _record_planner_context(rec, planner)
            if rec.get("planner_returned_none"):
                if getattr(self, "_fallback_braking_active", False):
                    rec["fallback_result"] = "controlled braking to standstill"
                elif trajectory is None:
                    rec["fallback_result"] = "failed: keeping current state"
                else:
                    rec["fallback_result"] = "standstill trajectory"
                _update_fallback_hint(rec)
                if next_state is not None:
                    rec["fallback_next_position"] = copy.deepcopy(getattr(next_state, "position", None))
                    rec["fallback_next_velocity"] = getattr(next_state, "velocity", None)
        _current_sm_state = previous_sm_state

_sm_module.BaseStateMachinePlanner._plan_and_optimize = _patched_plan_and_optimize


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  PATCH 6 – bulb-only departure behaviour
# ╚══════════════════════════════════════════════════════════════════════════════
_original_bulb_allow_state_transition = _sm_module.BusStopBulbPlanner._allow_state_transition
_BULB_DEPARTURE_MERGE_LENGTH = 42.0
_BULB_DEPARTURE_DESIRED_SPEED = 3.5
_BULB_MISSION_COMPLETE_DISTANCE = 90.0


def _bulb_lane_coordinate_system(self, lanelet_id: int, extra_m: float = 120.0):
    vertices = np.asarray(
        self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id).center_vertices,
        dtype=float,
    )
    return _sm_module.create_coordinate_system(_extend_path(vertices, extra_m=extra_m))


def _bulb_interpolate_centerline_y(self, lanelet_id: int, x_values: np.ndarray) -> np.ndarray:
    vertices = np.asarray(
        self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id).center_vertices,
        dtype=float,
    )
    order = np.argsort(vertices[:, 0])
    xs = vertices[order, 0]
    ys = vertices[order, 1]
    return np.interp(x_values, xs, ys, left=ys[0], right=ys[-1])


def _bulb_departure_coordinate_system(self, state_current):
    if self._departing_entry_pose is None:
        self._departing_entry_pose = (
            float(state_current.position[0]),
            float(state_current.position[1]),
            max(0.0, float(state_current.velocity)),
        )

    entry_x, entry_y, entry_v = self._departing_entry_pose
    merge_start_x = entry_x + max(8.0, 1.5 * entry_v + 8.0)
    merge_end_x = merge_start_x + _BULB_DEPARTURE_MERGE_LENGTH
    end_x = max(float(self.goal_x) + _BULB_MISSION_COMPLETE_DISTANCE + 45.0, merge_end_x + 80.0)
    start_x = min(entry_x - 25.0, float(state_current.position[0]) - 20.0)
    n_pts = max(220, int((end_x - start_x) * 6))
    x_values = np.linspace(start_x, end_x, n_pts)

    target_y_values = _bulb_interpolate_centerline_y(self, lanelet_id=1, x_values=x_values)
    denom = max(merge_end_x - merge_start_x, 1e-6)
    progress = np.clip((x_values - merge_start_x) / denom, 0.0, 1.0)
    progress = progress ** 3 * (10.0 + progress * (-15.0 + 6.0 * progress))
    y_values = entry_y * (1.0 - progress) + target_y_values * progress
    return _sm_module.create_coordinate_system(np.column_stack((x_values, y_values)))


def _bulb_lateral_offset_to_lane(self, state, lanelet_id: int) -> Optional[float]:
    try:
        coord_sys = _bulb_lane_coordinate_system(self, lanelet_id)
        _, d = coord_sys.convert_to_curvilinear_coords(
            float(state.position[0]),
            float(state.position[1]),
        )
        return float(d)
    except Exception:
        return None


def _patched_bulb_execute_departing(self, state_current, state_list):
    if getattr(self, "_completed_stop_service", False):
        coord_sys = _bulb_departure_coordinate_system(self, state_current)
    else:
        coord_sys = _bulb_lane_coordinate_system(self, lanelet_id=2)
    planner, config = self._create_planner(self.current_state, coord_sys, state_current)
    if getattr(self, "_completed_stop_service", False):
        config.sampling.d_min = -0.45
        config.sampling.d_max = 0.45
        config.sampling.desire_velocity = min(
            _BULB_DEPARTURE_DESIRED_SPEED,
            float(getattr(config.sampling, "v_max", _BULB_DEPARTURE_DESIRED_SPEED)),
        )
        try:
            planner.set_permitted_lanelet_ids([1, 2])
        except Exception as exc:
            print(f"[bus_stop_bulb_opt] WARNING: could not restrict permitted lanelets: {exc}")

    planner._low_vel_mode = False
    planner._desired_speed = config.sampling.desire_velocity
    planner.set_desired_velocity(
        current_speed=state_current.velocity,
        desired_velocity=config.sampling.desire_velocity,
    )

    next_state, _ = self._plan_and_optimize(planner, config, state_list)
    self._check_state_transition(next_state, config)
    return next_state


def _patched_bulb_allow_state_transition(self, from_state, to_state, next_state, config) -> bool:
    if isinstance(from_state, _sm_module.DepartingState) and isinstance(to_state, _sm_module.HeadingState):
        lateral_offset = _bulb_lateral_offset_to_lane(self, next_state, lanelet_id=1)
        if lateral_offset is None:
            print("[transition] DEPARTING held: cannot evaluate main-lane lateral offset")
            return False
        if abs(lateral_offset) > 0.6:
            print(
                f"[transition] DEPARTING held: lateral offset to main lane "
                f"{lateral_offset:.2f} m > 0.60 m"
            )
            return False

    return _original_bulb_allow_state_transition(self, from_state, to_state, next_state, config)


def _patched_bulb_is_mission_complete(self, state_current) -> bool:
    if not self._completed_stop_service or not isinstance(self.current_state, _sm_module.HeadingState):
        return False

    distance_after_stop = float(state_current.position[0]) - float(self.goal_x)
    lateral_offset = _bulb_lateral_offset_to_lane(self, state_current, lanelet_id=1)
    if lateral_offset is None:
        return False

    return distance_after_stop > _BULB_MISSION_COMPLETE_DISTANCE and abs(lateral_offset) < 0.8


_sm_module.BusStopBulbPlanner._execute_departing = _patched_bulb_execute_departing
_sm_module.BusStopBulbPlanner._allow_state_transition = _patched_bulb_allow_state_transition
_sm_module.BusStopBulbPlanner._is_mission_complete = _patched_bulb_is_mission_complete


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  RUN SIMULATION
# ╚══════════════════════════════════════════════════════════════════════════════
def _export_driven_trajectory_csv(planning_problem_set, ego_vehicles, output_path: Path):
    """Export the complete executed trajectory at the simulation time resolution."""
    if not ego_vehicles:
        raise ValueError("No ego vehicle trajectory is available for export")

    planning_problem_id, ego_vehicle = next(iter(ego_vehicles.items()))
    states = list(ego_vehicle.driven_trajectory.trajectory.state_list)
    planning_problem = planning_problem_set.planning_problem_dict[planning_problem_id]
    initial_state = planning_problem.initial_state

    if not states or states[0].time_step != initial_state.time_step:
        states.insert(0, initial_state)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "time_step",
                "time_s",
                "velocity_mps",
                "acceleration_mps2",
                "x_m",
                "y_m",
            ],
        )
        writer.writeheader()
        first_time_step = int(states[0].time_step)
        for state in states:
            writer.writerow(
                {
                    "time_step": int(state.time_step),
                    "time_s": (int(state.time_step) - first_time_step) * 0.1,
                    "velocity_mps": float(state.velocity),
                    "acceleration_mps2": float(getattr(state, "acceleration", 0.0)),
                    "x_m": float(state.position[0]),
                    "y_m": float(state.position[1]),
                }
            )

    print(
        f"[bus_stop_bulb_opt] Full trajectory CSV saved to: {output_path} "
        f"({len(states)} states)"
    )


def _generate_velocity_profile(
    trajectory_path: Path,
    output_path: Path,
    stationary_threshold: float = 0.05,
) -> None:
    """Generate the complete planning-process velocity profile."""
    try:
        from utility.plot_velocity_profile import (
            load_velocity_profile,
            plot_velocity_profile,
        )

        time_values, velocity_values = load_velocity_profile(
            trajectory_path,
            dt=0.1,
        )
        plot_velocity_profile(
            time_values,
            velocity_values,
            output_path,
            stationary_threshold,
        )
        print(f"[bus_stop_bulb_opt] Velocity profile saved to: {output_path}")
    except Exception as exc:
        print(f"[bus_stop_bulb_opt] WARNING: Velocity profile generation failed: {exc}")


def _ego_obstacle_from_driven_trajectory(scenario, ego_vehicles):
    if not ego_vehicles:
        raise ValueError("No ego vehicle trajectory is available for video rendering")

    ego_vehicle = next(iter(ego_vehicles.values()))
    prediction = ego_vehicle.driven_trajectory
    trajectory = prediction.trajectory
    prediction_shape = Rectangle(length=12.95, width=2.55)
    prediction = TrajectoryPrediction(trajectory, prediction_shape)
    initial = trajectory.state_list[0]
    if not isinstance(initial, InitialState):
        initial = InitialState(
            position=initial.position,
            orientation=initial.orientation,
            velocity=initial.velocity,
            acceleration=getattr(initial, "acceleration", None),
            yaw_rate=getattr(initial, "yaw_rate", None),
            slip_angle=getattr(initial, "slip_angle", None),
            time_step=initial.time_step,
        )

    return DynamicObstacle(
        obstacle_id=scenario.generate_object_id(),
        obstacle_type=ObstacleType.BUS,
        obstacle_shape=prediction_shape,
        initial_state=initial,
        prediction=prediction,
    )


def _make_bulb_passengers() -> Tuple[DynamicObstacle, DynamicObstacle]:
    t_start, t_end = 264, 276
    shape = Circle(0.5)

    pos1 = np.array([1.0, -9.0])
    p1 = DynamicObstacle(
        obstacle_id=4001,
        obstacle_type=ObstacleType.PEDESTRIAN,
        obstacle_shape=shape,
        initial_state=InitialState(time_step=t_start, position=pos1, velocity=0.2, orientation=0.0),
    )
    p1.prediction = TrajectoryPrediction(
        Trajectory(
            t_start,
            [
                ExtendedPMState(
                    time_step=t,
                    position=pos1 + np.array([0.0, 0.01 * (t - t_start)]),
                    velocity=0.2,
                    orientation=0.0,
                    acceleration=0.0,
                )
                for t in range(t_start, t_end + 1)
            ],
        ),
        shape,
    )

    pos2 = np.array([-1.0, -9.0])
    p2 = DynamicObstacle(
        obstacle_id=4002,
        obstacle_type=ObstacleType.PEDESTRIAN,
        obstacle_shape=shape,
        initial_state=InitialState(time_step=t_start, position=pos2, velocity=0.2, orientation=0.0),
    )
    p2.prediction = TrajectoryPrediction(
        Trajectory(
            t_start,
            [
                ExtendedPMState(
                    time_step=t,
                    position=pos2 + np.array([0.0, 0.01 * (t - t_start)]),
                    velocity=0.2,
                    orientation=0.0,
                    acceleration=0.0,
                )
                for t in range(t_start, t_end + 1)
            ],
        ),
        shape,
    )

    return p1, p2


def _create_bulb_video(simulated_scenario, planning_problem_set, ego_vehicles, output_dir: Path) -> Path:
    planning_problem = next(iter(planning_problem_set.planning_problem_dict.values()))
    ego_obstacle = _ego_obstacle_from_driven_trajectory(simulated_scenario, ego_vehicles)
    time_end = int(ego_obstacle.prediction.final_time_step)

    draw_params = MPDrawParams()
    draw_params.time_begin = 0
    draw_params.time_end = time_end
    draw_params.axis_visible = False
    draw_params.dynamic_obstacle.show_label = False
    draw_params.dynamic_obstacle.draw_icon = True
    draw_params.dynamic_obstacle.draw_shape = True
    draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    draw_params.dynamic_obstacle.occupancy.shape.edgecolor = "#9C4100"
    draw_params.dynamic_obstacle.trajectory.draw_trajectory = True

    ego_params = DynamicObstacleParams()
    ego_params.time_begin = 0
    ego_params.time_end = time_end
    ego_params.draw_icon = True
    ego_params.draw_shape = True
    ego_params.show_label = False
    ego_params.occupancy.draw_occupancies = True
    ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#C55A11"
    ego_params.vehicle_shape.occupancy.shape.opacity = 1
    ego_params.vehicle_shape.occupancy.shape.zorder = 200
    ego_params.trajectory.draw_trajectory = True

    car_params = DynamicObstacleParams()
    car_params.time_begin = 0
    car_params.time_end = time_end
    car_params.use_type_color = False
    car_params.occupancy.draw_occupancies = True
    car_params.vehicle_shape.occupancy.shape.facecolor = "#43A047"
    car_params.vehicle_shape.occupancy.shape.edgecolor = "#000000"
    car_params.vehicle_shape.occupancy.shape.opacity = 1
    car_params.vehicle_shape.occupancy.shape.zorder = 100
    car_params.draw_icon = True
    car_params.show_label = False

    others = list(simulated_scenario.dynamic_obstacles)

    bike_polys = [simulated_scenario.lanelet_network.find_lanelet_by_id(3).polygon]
    bike_dp = MPDrawParams()
    bike_dp.time_begin = 0
    bike_dp.time_end = time_end
    bike_dp.occupancy.shape.zorder = 1
    bike_dp.shape.facecolor = "#98c6ea"
    bike_dp.shape.edgecolor = "#98c6ea"

    bus_stop_sign = TrafficSign(
        400,
        [TrafficSignElement(TrafficSignIDGermany.BUS_STOP)],
        {7},
        np.array([10.0, -4.0]),
    )
    sign_dp = MPDrawParams()
    sign_dp.time_begin = 0
    sign_dp.time_end = time_end
    sign_dp.traffic_sign.draw_traffic_signs = True
    sign_dp.traffic_sign.show_label = False
    sign_dp.traffic_sign.zorder = 1

    pedestrian_params = DynamicObstacleParams()
    pedestrian_params.time_begin = 0
    pedestrian_params.time_end = time_end
    pedestrian_params.draw_icon = True
    pedestrian_params.use_type_color = True
    pedestrian_params.trajectory.draw_trajectory = False
    pedestrian_params.occupancy.draw_occupancies = False
    pedestrian_params.vehicle_shape.occupancy.shape.zorder = 1000

    passengers = list(_make_bulb_passengers())

    rnd = MPRenderer()
    rnd.draw_params.axis_visible = False
    rnd.f.subplots_adjust(left=0, right=1, bottom=0, top=1)
    rnd.plot_limits = [-155.0, 155.0, -10.0, 4.0]

    objects = (
        [simulated_scenario.lanelet_network, planning_problem, ego_obstacle]
        + others
        + bike_polys
        + [bus_stop_sign]
        + passengers
    )
    params = (
        [draw_params, draw_params, ego_params]
        + [car_params] * len(others)
        + [bike_dp] * len(bike_polys)
        + [sign_dp]
        + [pedestrian_params] * len(passengers)
    )

    output_path = output_dir / "sumo_result_bus_stop_bulb_opt.gif"
    rnd.create_video(objects, str(output_path), draw_params=params, fig_size=[15, 8], dpi=120, progress=False)
    plt.close(rnd.f)
    return output_path


@contextmanager
def _scenario_config_override(config_path: Path, cfg: dict):
    """Serve the target scenario config to helpers that read scenario.yaml."""
    original_open = builtins.open
    config_text = yaml.safe_dump(cfg)
    resolved_config_path = config_path.resolve()

    def patched_open(file, mode="r", *args, **kwargs):
        try:
            file_path = Path(file).resolve()
        except TypeError:
            return original_open(file, mode, *args, **kwargs)

        read_only = "r" in mode and not any(flag in mode for flag in ("w", "a", "+"))
        if file_path == resolved_config_path and read_only:
            if "b" in mode:
                encoding = kwargs.get("encoding") or "utf-8"
                return io.BytesIO(config_text.encode(encoding))
            return io.StringIO(config_text)
        return original_open(file, mode, *args, **kwargs)

    builtins.open = patched_open
    try:
        yield
    finally:
        builtins.open = original_open


def run_simulation():
    from source.simulation.simulations import simulate_with_planner

    config_path = path_root / "configurations" / "scenario.yaml"
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    configured_bus_stop = cfg.get("scenario", {}).get("type")
    cfg = copy.deepcopy(cfg)
    cfg.setdefault("scenario", {})["type"] = _TARGET_SCENARIO

    bus_stop: str = _TARGET_SCENARIO
    scenario_dir = path_root / "scenarios" / bus_stop
    output_dir = path_root / "experiments" / "output_result" / "result_bulb"

    print(f"[bus_stop_bulb_opt] Scenario: {bus_stop}")
    if configured_bus_stop != bus_stop:
        print(
            f"[bus_stop_bulb_opt] Overriding scenario.yaml type "
            f"{configured_bus_stop!r} for this run only."
        )
    if cfg.get("debug", {}).get("use_post_opt"):
        print("[bus_stop_bulb_opt] debug.use_post_opt is ignored by the state machine.")
    print("[bus_stop_bulb_opt] Planner mode: CommonRoad reactive planner only.")
    print("[bus_stop_bulb_opt] Multiprocessing disabled – exceptions now propagate.\n")

    try:
        with _scenario_config_override(config_path, cfg):
            simulated_scenario, planning_problem_set, ego_vehicles = simulate_with_planner(
                interactive_scenario_path=str(scenario_dir),
                return_on_planner_completion=True,
            )
        output_dir.mkdir(parents=True, exist_ok=True)
        trajectory_path = output_dir / "bus_stop_bulb_opt_trajectory.csv"
        _export_driven_trajectory_csv(
            planning_problem_set,
            ego_vehicles,
            trajectory_path,
        )
        _generate_velocity_profile(
            trajectory_path,
            output_dir / "bus_stop_bulb_opt_velocity_profile.png",
        )
        gif_path = _create_bulb_video(
            simulated_scenario,
            planning_problem_set,
            ego_vehicles,
            output_dir,
        )
        print(f"[bus_stop_bulb_opt] SUMO trajectory GIF saved to: {gif_path}")
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
    "BEFORE_STOPPING_ALIGN": "#5E35B1",
    "BEFORE_STOPPING_MERGE": "#7E57C2",
    "BEFORE_STOPPING_FINAL": "#AB47BC",
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
    scenario, planning_problem_set = CommonRoadFileReader(str(f)).open()
    return scenario, planning_problem_set


def _draw_projection_domain(ax, co: CurvilinearCoordinateSystem,
                             colour: str, alpha: float = 0.12, label: str = "",
                             fill: bool = False, draw_boundary: bool = True,
                             ref_alpha: float = 0.8, zorder: int = 3):
    """Draw the projection domain polygon and reference path."""
    if fill or draw_boundary:
        try:
            domain_poly = co.projection_domain()
            if hasattr(domain_poly, "exterior"):
                xs, ys = np.array(domain_poly.exterior.xy[0]), np.array(domain_poly.exterior.xy[1])
            else:
                pts = np.asarray(domain_poly)
                xs, ys = pts[:, 0], pts[:, 1]
            if fill:
                ax.fill(xs, ys, color=colour, alpha=alpha, zorder=zorder)
            if draw_boundary:
                ax.plot(xs, ys, color=colour, linewidth=0.9, alpha=max(alpha, 0.45), zorder=zorder + 1)
        except Exception:
            pass

    try:
        ref = co.ref_path
        ax.plot(ref[:, 0], ref[:, 1], "--", color=colour, linewidth=1.5,
                alpha=ref_alpha, label=label if label else None, zorder=zorder + 2)
        # start / end markers
        ax.scatter(ref[0, 0],  ref[0, 1],  marker="|", s=90, color=colour, alpha=ref_alpha, zorder=zorder + 3)
        ax.scatter(ref[-1, 0], ref[-1, 1], marker="|", s=90, color=colour, alpha=ref_alpha, zorder=zorder + 3)
    except Exception:
        pass


def _legend_state_name(name: str) -> str:
    return name


def _cart_from_sd(co: CurvilinearCoordinateSystem, s: float, d: float = 0.0):
    """Convert (s,d) to Cartesian, returning None on any error."""
    try:
        return co.convert_to_cartesian_coords(s, d)
    except Exception:
        return None


def _planning_time_ms(rec: dict) -> Optional[float]:
    value = rec.get("cycle_elapsed_ms")
    if value is None:
        value = rec.get("elapsed_ms")
    return float(value) if value is not None else None


def _format_time_ms(value: Optional[float]) -> str:
    if value is None:
        return "n/a"
    if value >= 1000.0:
        return f"{value / 1000.0:.2f} s"
    return f"{value:.0f} ms"


def _compact_state_name(name: str) -> str:
    aliases = {
        "HEADING": "HEAD",
        "ARRIVING": "ARR",
        "BEFORE_STOPPING_ALIGN": "ALIGN",
        "BEFORE_STOPPING_MERGE": "MERGE",
        "BEFORE_STOPPING_FINAL": "FINAL",
        "BEFORE_STOPPING": "PRESTOP",
        "STOPPING": "STOP",
        "DEPARTING": "DEPART",
    }
    return aliases.get(name, name)


def _trajectory_segments(records: List[dict]) -> List[dict]:
    segments: List[dict] = []
    current: Optional[dict] = None
    for rec in records:
        if rec.get("position") is None:
            continue
        sm_state = rec["sm_state"]
        if current is None or current["sm_state"] != sm_state:
            current = {"sm_state": sm_state, "records": [], "positions": []}
            segments.append(current)
        current["records"].append(rec)
        current["positions"].append(rec["position"])
    return segments


def _segment_avg_time_ms(segment: dict) -> Optional[float]:
    times = []
    for rec in segment["records"]:
        time_ms = _planning_time_ms(rec)
        if time_ms is not None:
            times.append(time_ms)
    return sum(times) / len(times) if times else None


def _rects_overlap(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> bool:
    return not (a[1] < b[0] or b[1] < a[0] or a[3] < b[2] or b[3] < a[2])


def _add_segment_time_labels(ax, segments: List[dict]):
    if not segments:
        return

    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()
    x_span = max(x_max - x_min, 1.0)
    y_span = max(y_max - y_min, 1.0)
    occupied: List[Tuple[float, float, float, float]] = []

    offsets = [
        (0.00, 0.18),
        (0.00, -0.18),
        (0.06, 0.28),
        (-0.06, -0.28),
        (-0.06, 0.28),
        (0.06, -0.28),
        (0.12, 0.18),
        (-0.12, -0.18),
        (-0.12, 0.18),
        (0.12, -0.18),
    ]

    for idx, segment in enumerate(segments):
        positions = np.asarray(segment["positions"])
        if positions.size == 0:
            continue
        anchor = positions[len(positions) // 2]
        label = f"{_compact_state_name(segment['sm_state'])}\navg {_format_time_ms(_segment_avg_time_ms(segment))}"
        max_line_len = max(len(line) for line in label.splitlines())
        label_w = max(15.0, max_line_len * 0.014 * x_span)
        label_h = 0.22 * y_span

        chosen = None
        for ox, oy in offsets[idx % len(offsets):] + offsets[:idx % len(offsets)]:
            tx = float(anchor[0] + ox * x_span)
            ty = float(anchor[1] + oy * y_span)
            tx = min(max(tx, x_min + 0.04 * x_span), x_max - 0.04 * x_span)
            ty = min(max(ty, y_min + 0.13 * y_span), y_max - 0.11 * y_span)
            rect = (
                tx - label_w / 2.0,
                tx + label_w / 2.0,
                ty - label_h / 2.0,
                ty + label_h / 2.0,
            )
            if not any(_rects_overlap(rect, used) for used in occupied):
                chosen = (tx, ty, rect)
                break

        if chosen is None:
            tx = float(anchor[0])
            ty = min(max(float(anchor[1] + 0.20 * y_span), y_min + 0.13 * y_span), y_max - 0.11 * y_span)
            chosen = (
                tx,
                ty,
                (tx - label_w / 2.0, tx + label_w / 2.0, ty - label_h / 2.0, ty + label_h / 2.0),
            )

        tx, ty, rect = chosen
        occupied.append(rect)
        ax.text(
            tx,
            ty,
            label,
            fontsize=7.2,
            color="#1f1f1f",
            ha="center",
            va="center",
            zorder=12,
            bbox=dict(boxstyle="round,pad=0.22", fc="white", ec="#666666", lw=0.55, alpha=0.92),
        )


def visualise(cfg: dict, bus_stop: str):
    if not _plan_records:
        print("[debug] No planning records captured – nothing to visualise.")
        return

    # find the failing record (first with error set)
    error_idx = next((i for i, r in enumerate(_plan_records) if r["error"] is not None), None)
    first_fallback_idx = next((i for i, r in enumerate(_plan_records) if r.get("planner_returned_none")), None)

    scenario, planning_problem_set = load_cr_scenario(bus_stop)
    planning_problem = next(iter(planning_problem_set.planning_problem_dict.values()))

    fig, ax_map = plt.subplots(figsize=(13.5, 2.85))

    total = len(_plan_records)
    err_str = f"step {error_idx}" if error_idx is not None else "no domain error"
    fb_str = f"step {first_fallback_idx}" if first_fallback_idx is not None else "none"
    title_times = []
    for rec in _plan_records:
        time_ms = _planning_time_ms(rec)
        if time_ms is not None:
            title_times.append(time_ms)
    avg_plan_str = _format_time_ms(sum(title_times) / len(title_times) if title_times else None)
    fig.suptitle(
        f"Projection Domain Debug  |  Scenario: {bus_stop}  |  "
        f"CommonRoad RP only  |  Steps: {total}  |  Avg plan: {avg_plan_str}  |  "
        f"Error: {err_str}  |  First fallback: {fb_str}",
        fontsize=10,
        y=0.985,
    )

    # ── draw lanelets ─────────────────────────────────────────────────────────
    for ll in scenario.lanelet_network.lanelets:
        lv, rv = ll.left_vertices, ll.right_vertices
        poly = np.vstack([lv, rv[::-1], lv[0]])
        ax_map.fill(poly[:, 0], poly[:, 1], color="#F4F4F4", zorder=0)
        ax_map.plot(lv[:, 0], lv[:, 1], color="#777777", lw=0.45, zorder=1)
        ax_map.plot(rv[:, 0], rv[:, 1], color="#777777", lw=0.45, zorder=1)
        cx = ll.center_vertices[len(ll.center_vertices) // 2]
        ax_map.text(cx[0], cx[1], str(ll.lanelet_id),
                    fontsize=6, color="#777", ha="center", va="center", zorder=4)

    goal_shape = planning_problem.goal.state_list[0].position
    goal_center = goal_shape.center
    goal_rect = mpatches.Rectangle(
        (goal_center[0] - goal_shape.length / 2.0, goal_center[1] - goal_shape.width / 2.0),
        goal_shape.length,
        goal_shape.width,
        facecolor="#00A676",
        edgecolor="#007A5A",
        linewidth=1.2,
        alpha=0.14,
        zorder=2,
    )
    ax_map.add_patch(goal_rect)

    lanelet_points = []
    for ll in scenario.lanelet_network.lanelets:
        lanelet_points.extend(ll.left_vertices)
        lanelet_points.extend(ll.right_vertices)
    lanelet_points = np.asarray(lanelet_points)

    # ── draw one reference path per unique (sm_state, coord_sys) pair ─────────
    seen = {}
    for rec in _plan_records:
        co = rec["coord_sys"]
        if co is None:
            continue
        key = (rec["sm_state"], id(co))
        if key not in seen:
            seen[key] = (rec["sm_state"], co)

    focus_indices = [idx for idx in (error_idx, first_fallback_idx) if idx is not None]
    focus_coord_ids = {
        id(_plan_records[idx]["coord_sys"])
        for idx in focus_indices
        if _plan_records[idx].get("coord_sys") is not None
    }

    # Draw reference paths for context. Projection-domain fills are intentionally
    # disabled here; their polygons are much larger than the road geometry and
    # make the actual manoeuvre hard to inspect.
    seen_states_legend: set = set()
    legend_handles = []
    for (sm_state, co) in seen.values():
        c = _state_colour(sm_state)
        legend_state = _legend_state_name(sm_state)
        is_focus_domain = id(co) in focus_coord_ids
        _draw_projection_domain(
            ax_map, co, c,
            alpha=0.0,
            fill=False,
            draw_boundary=False,
            ref_alpha=0.65 if is_focus_domain else 0.12,
            zorder=2 if is_focus_domain else 1,
        )
        if legend_state not in seen_states_legend:
            seen_states_legend.add(legend_state)
            legend_handles.append(mpatches.Patch(color=c, label=legend_state))

    # ── vehicle trajectory coloured by SM state ────────────────────────────────
    trajectory_segments = _trajectory_segments(_plan_records)
    for segment in trajectory_segments:
        sm_state = segment["sm_state"]
        positions = segment["positions"]
        if not positions:
            continue
        pts = np.array(positions)
        c = _state_colour(sm_state)
        ax_map.plot(pts[:, 0], pts[:, 1], "-", color=c, lw=1.4, alpha=0.78, zorder=5)
        ax_map.scatter(pts[:, 0], pts[:, 1], color=c, s=18, zorder=6, alpha=0.88)

    traj_points = np.array([
        rec["position"] for rec in _plan_records if rec.get("position") is not None
    ])

    if first_fallback_idx is not None:
        rec_fb = _plan_records[first_fallback_idx]
        if rec_fb.get("position") is not None:
            fx, fy = rec_fb["position"]
            ax_map.scatter([fx], [fy], color="#FFC107", edgecolors="#5D4500",
                           linewidths=0.8, s=145, marker="D", zorder=8)
            ax_map.annotate(
                f"First fallback\nstep {first_fallback_idx} [{rec_fb['sm_state']}]\n"
                f"{rec_fb.get('fallback_reason_hint') or ''}",
                xy=(fx, fy), xytext=(fx + 8, fy + 6),
                fontsize=8, color="#5D4500", zorder=10,
                arrowprops=dict(arrowstyle="->", color="#5D4500", lw=1.0),
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#FFC107", alpha=0.95),
            )

    if traj_points.size:
        start = traj_points[0]
        end = traj_points[-1]
        ax_map.scatter([start[0]], [start[1]], color="#2E7D32", s=70, marker="o", zorder=9)
        ax_map.scatter([end[0]], [end[1]], color="#0D47A1", s=90, marker="s", zorder=9)
        ax_map.annotate(
            "end",
            xy=(end[0], end[1]),
            xytext=(end[0] + 5, end[1] + 2),
            fontsize=8,
            color="#0D47A1",
            arrowprops=dict(arrowstyle="->", color="#0D47A1", lw=0.9),
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#0D47A1", alpha=0.9),
        )

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
    if first_fallback_idx is not None:
        legend_handles.append(
            Line2D([0], [0], marker="D", color="w", markerfacecolor="#FFC107",
                   markeredgecolor="#5D4500", markersize=8, label="first fallback")
        )
    legend_handles.append(mpatches.Patch(facecolor="#00A676", edgecolor="#007A5A", alpha=0.25, label="goal"))

    ax_map.set_aspect("equal")
    visible_points = lanelet_points
    if traj_points.size:
        visible_points = np.vstack([visible_points, traj_points])
    min_xy = visible_points.min(axis=0)
    max_xy = visible_points.max(axis=0)
    pad_x = max(4.0, 0.025 * (max_xy[0] - min_xy[0]))
    pad_y = max(2.0, 0.12 * (max_xy[1] - min_xy[1]))
    ax_map.set_xlim(min_xy[0] - pad_x, max_xy[0] + pad_x)
    ax_map.set_ylim(min_xy[1] - pad_y, max_xy[1] + pad_y)
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    _add_segment_time_labels(ax_map, trajectory_segments)
    # Place legend just outside the axes, keeping the map clear for poster use.
    ax_map.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.30),
        ncol=min(len(legend_handles), 7),
        fontsize=7,
        framealpha=0.9,
        edgecolor="#aaaaaa",
        borderpad=0.25,
        handlelength=1.2,
        columnspacing=0.8,
    )
    ax_map.grid(True, lw=0.4, alpha=0.5)

    plt.subplots_adjust(left=0.045, right=0.995, bottom=0.43, top=0.80)
    out = path_root / "experiments" / "output_result" / "result_bulb" / "bus_stop_bulb_opt.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out), dpi=150, bbox_inches="tight")
    print(f"\n[debug] Figure saved to {out}")
    if matplotlib.get_backend().lower() != "agg":
        plt.show()


# ╔══════════════════════════════════════════════════════════════════════════════
# ║  TEXT SUMMARY
# ╚══════════════════════════════════════════════════════════════════════════════
def print_summary():
    error_idx = next((i for i, r in enumerate(_plan_records) if r["error"] is not None), None)
    fallback_indices = [i for i, r in enumerate(_plan_records) if r.get("planner_returned_none")]
    cycle_records = [r for r in _plan_records if r.get("cycle_elapsed_ms") is not None]
    print("\n" + "=" * 65)
    times_ms = [r["cycle_elapsed_ms"] for r in cycle_records]
    print(f"  Total planning steps recorded : {len(cycle_records)}")
    print(f"  Planner mode                  : CommonRoad RP only")
    print(f"  Fallback count                : {len(fallback_indices)}")
    duration_s = _lane_change_to_stop_timer.get("duration_s")
    if duration_s is not None:
        print(
            f"  Lane-change start -> complete stop duration : {duration_s:.2f} s "
            f"(sim_step {_lane_change_to_stop_timer['lane_change_step']} "
            f"-> {_lane_change_to_stop_timer['stop_step']})"
        )
    else:
        print("  Lane-change start -> complete stop duration : not completed")
    if fallback_indices:
        print(f"  First fallback step           : {fallback_indices[0]}")
        reason_counts = {}
        for idx in fallback_indices:
            reason = _plan_records[idx].get("fallback_reason_hint") or "unknown"
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
        print("  Fallback reason hints:")
        for reason, count in sorted(reason_counts.items(), key=lambda item: item[1], reverse=True)[:5]:
            print(f"    - {count:>3d}x {reason}")
    transition_blockers = {}
    for rec in _plan_records:
        for blocker in rec.get("transition_blockers") or []:
            transition_blockers[blocker] = transition_blockers.get(blocker, 0) + 1
    if transition_blockers:
        print("  State transition blockers:")
        for blocker, count in sorted(transition_blockers.items(), key=lambda item: item[1], reverse=True)[:6]:
            print(f"    - {count:>3d}x {blocker}")
    if times_ms:
        print(f"  Planning time  min / mean / max : "
              f"{min(times_ms):.1f} / {sum(times_ms)/len(times_ms):.1f} / {max(times_ms):.1f} ms")
        slowest_record = max(cycle_records, key=lambda record: record["cycle_elapsed_ms"])
        slowest = _plan_records.index(slowest_record)
        print(f"  Slowest step  : {slowest}  "
              f"({_plan_records[slowest]['cycle_elapsed_ms']:.1f} ms, "
              f"SM={_plan_records[slowest]['sm_state']})")
    if error_idx is not None:
        rec = _plan_records[error_idx]
        print(f"  *** ERROR at step {error_idx} ***")
        print(f"  SM state    : {rec['sm_state']}")
        if rec.get("position") is not None:
            print(f"  Position    : x={rec['position'][0]:.4f}  y={rec['position'][1]:.4f}")
        if rec.get("velocity") is not None:
            print(f"  Velocity    : {rec['velocity']:.4f} m/s")
        if rec.get("orientation") is not None:
            print(f"  Orientation : {rec['orientation']:.5f} rad")
        if rec.get("orientation_error_ref") is not None:
            print(f"  Ref theta err: {rec['orientation_error_ref']:.5f} rad")
        if rec.get("required_decel_to_desired_v") is not None:
            print(f"  Req decel v : {rec['required_decel_to_desired_v']:.4f} m/s^2")
        if rec.get("required_decel_to_stop_goal") is not None:
            print(f"  Req decel s : {rec['required_decel_to_stop_goal']:.4f} m/s^2")

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
                if rec.get("position") is None:
                    raise ValueError("position unavailable")
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

    if fallback_indices:
        print("  Fallback diagnostics:")
        for idx in fallback_indices:
            rec = _plan_records[idx]
            pos = rec.get("position")
            pos_str = f"({pos[0]:.2f}, {pos[1]:.2f})" if pos is not None else "-"
            reasons = rec.get("infeasible_reasons") or {}
            reason_str = ", ".join(f"{k}={v}" for k, v in reasons.items() if v)
            if not reason_str:
                reason_str = "-"
            print(f"    Step {idx} [{rec.get('sm_state')}] pos={pos_str} v={rec.get('velocity')}")
            print(f"      result={rec.get('fallback_result')} hint={rec.get('fallback_reason_hint')}")
            print(f"      mode={rec.get('longitudinal_mode')} desired_v={rec.get('desired_speed')} "
                  f"theta_ref_err={rec.get('orientation_error_ref')}")
            print(f"      req_decel_v={rec.get('required_decel_to_desired_v')} "
                  f"req_decel_stop={rec.get('required_decel_to_stop_goal')}")
            print(f"      samples={rec.get('total_samples')} kin/coll="
                  f"{rec.get('infeasible_kinematics')} / {rec.get('infeasible_collision')}")
            print(f"      infeasible reasons: {reason_str}")
    print("=" * 65)


# ── entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # ── set up logging to file ────────────────────────────────────────────────
    _log_dir = path_root / "experiments" / "output_result" / "result_bulb"
    _log_dir.mkdir(parents=True, exist_ok=True)
    _log_ts = time.strftime("%Y%m%d_%H%M%S")
    _log_path = _log_dir / f"bus_stop_bulb_opt_{_log_ts}.log"
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
