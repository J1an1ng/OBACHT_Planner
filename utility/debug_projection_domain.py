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
_first_domain_error: Optional[dict] = None    # set when the first bad (s,d) is caught
_active_plan_record: Optional[dict] = None    # the record being filled during plan()
_pre_plan_states: list = []                   # _compute_initial_states calls from outside plan()
_plan_call_count: int = 0                     # total plan() invocations (for sanity checks)


_DEBUG_MAX_REQUIRED_DECEL_RATIO = 0.8


def _sampling_attr(config, name: str, default=None):
    return getattr(getattr(config, "sampling", None), name, default)


def _planning_attr(config, name: str, default=None):
    return getattr(getattr(config, "planning", None), name, default)


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
    _current_sm_state = self.get_current_state_name()
    return _original_step(self, state_current, state_list)

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
    elif state_name.startswith("BEFORE_STOPPING"):
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
    elif blockers and before in ("DEPARTING", "BEFORE_STOPPING"):
        print(f"[transition] {before} held: {'; '.join(blockers)}")

_sm_module.BaseStateMachinePlanner._check_state_transition = _patched_check_state_transition


_original_plan_and_optimize = _sm_module.BaseStateMachinePlanner._plan_and_optimize

def _patched_plan_and_optimize(self, planner, config, state_list, is_stopping: bool = False):
    global _current_sm_state
    start_idx = len(_plan_records)
    next_state = None
    trajectory = None
    previous_sm_state = _current_sm_state
    _current_sm_state = getattr(self, "_active_planning_state_name", None) or self.get_current_state_name()
    try:
        next_state, trajectory = _original_plan_and_optimize(
            self, planner, config, state_list, is_stopping=is_stopping
        )
        return next_state, trajectory
    finally:
        if len(_plan_records) > start_idx:
            rec = _plan_records[-1]
            _record_planner_context(rec, planner)
            if rec.get("planner_returned_none"):
                if trajectory is None:
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
    if cfg.get("debug", {}).get("use_post_opt"):
        print("[debug_projection_domain] debug.use_post_opt is ignored by the state machine.")
    print("[debug_projection_domain] Planner mode: CommonRoad reactive planner only.")
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
    if name.startswith("BEFORE_STOPPING_"):
        return "#7E57C2"
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
    if name.startswith("BEFORE_STOPPING_"):
        return "BEFORE_STOPPING"
    return name


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
    first_fallback_idx = next((i for i, r in enumerate(_plan_records) if r.get("planner_returned_none")), None)

    scenario, planning_problem_set = load_cr_scenario(bus_stop)
    planning_problem = next(iter(planning_problem_set.planning_problem_dict.values()))

    fig, axes = plt.subplots(1, 2, figsize=(18, 7.5),
                             gridspec_kw={"width_ratios": [3.2, 0.9]})
    ax_map, ax_tbl = axes

    total = len(_plan_records)
    err_str = f"step {error_idx}" if error_idx is not None else "no domain error"
    fb_str = f"step {first_fallback_idx}" if first_fallback_idx is not None else "none"
    fig.suptitle(
        f"Projection Domain Debug  |  Scenario: {bus_stop}  |  "
        f"CommonRoad RP only  |  Steps: {total}  |  Error: {err_str}  |  First fallback: {fb_str}",
        fontsize=12,
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
    by_state: dict = {}
    for rec in _plan_records:
        if rec.get("position") is not None:
            by_state.setdefault(rec["sm_state"], []).append(rec["position"])

    for sm_state, positions in by_state.items():
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
    pad_x = max(8.0, 0.06 * (max_xy[0] - min_xy[0]))
    pad_y = max(4.0, 0.20 * (max_xy[1] - min_xy[1]))
    ax_map.set_xlim(min_xy[0] - pad_x, max_xy[0] + pad_x)
    ax_map.set_ylim(min_xy[1] - pad_y, max_xy[1] + pad_y)
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.set_title("Vehicle trajectory by state")
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

    # ── compact diagnostics ────────────────────────────────────────────────────
    ax_tbl.axis("off")
    ax_tbl.set_title("Diagnostics", fontsize=10, pad=8)

    fallback_indices = [i for i, r in enumerate(_plan_records) if r.get("planner_returned_none")]
    state_counts = {}
    for rec in _plan_records:
        state_counts[rec["sm_state"]] = state_counts.get(rec["sm_state"], 0) + 1
    lines = [
        f"steps: {len(_plan_records)}",
        f"fallbacks: {len(fallback_indices)}",
        f"first fallback: {first_fallback_idx if first_fallback_idx is not None else 'none'}",
        f"domain error: {error_idx if error_idx is not None else 'none'}",
        "",
        "state counts:",
    ]
    for state, count in sorted(state_counts.items(), key=lambda item: item[0]):
        lines.append(f"  {state}: {count}")
    if first_fallback_idx is not None:
        rec = _plan_records[first_fallback_idx]
        lines.extend([
            "",
            "first fallback:",
            f"  state: {rec['sm_state']}",
            f"  pos: ({rec['position'][0]:.2f}, {rec['position'][1]:.2f})",
            f"  v: {rec['velocity']:.2f} m/s",
            f"  hint: {rec.get('fallback_reason_hint') or 'unknown'}",
        ])
    if ax_info_text:
        lines.append(ax_info_text)
    ax_tbl.text(
        0.02, 0.98, "\n".join(lines),
        transform=ax_tbl.transAxes,
        fontsize=8.5,
        va="top",
        ha="left",
        family="monospace",
        bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="#cccccc", alpha=0.96),
    )

    plt.tight_layout()
    # Leave room at the bottom for the legend that sits outside ax_map
    plt.subplots_adjust(bottom=0.18)
    out = path_root / "experiments" / "output_result" / "debug_projection_domain.png"
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
    print("\n" + "=" * 65)
    times_ms = [r["elapsed_ms"] for r in _plan_records if r["elapsed_ms"] is not None]
    print(f"  Total planning steps recorded : {len(_plan_records)}")
    print(f"  Planner mode                  : CommonRoad RP only")
    print(f"  Fallback count                : {len(fallback_indices)}")
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
        slowest = max(range(len(_plan_records)),
                      key=lambda i: _plan_records[i]["elapsed_ms"] or 0)
        print(f"  Slowest step  : {slowest}  "
              f"({_plan_records[slowest]['elapsed_ms']:.1f} ms, "
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
