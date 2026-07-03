"""utility/evaluate_solution_costs.py
================================================
Utility helpers for evaluating solution costs and visualising CommonRoad
simulation results.  The module exposes two public functions:

* :func:`evaluate_solution_costs` – parse *scenario.yaml*, evaluate costs,
  and optionally write them to CSV.
* :func:`simulate_and_render` – load the same configuration, run the SUMO‑
  backed simulation with a solution, and render the resulting animation as a
  GIF.

Both helpers follow the same conventions:

* The YAML is searched at ``<repo_root>/configurations/scenario.yaml`` unless
  an explicit *cfg_path* is provided.
* All artefacts are written to the scenario-specific folder under
  ``<repo_root>/experiments/output_result``.
* Filenames encode the scenario type and whether post‑optimisation is active.
"""

from __future__ import annotations

from pathlib import Path
from dataclasses import asdict, is_dataclass
from collections.abc import Mapping
from enum import Enum
from typing import Final, List

import yaml
import pandas as pd
from IPython.display import Image, display

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader, Solution
from commonroad.visualization.draw_params import DynamicObstacleParams
from commonroad.visualization.mp_renderer import MPDrawParams, MPRenderer
from commonroad.scenario.obstacle import ObstacleType
from commonroad_dc.costs.evaluation import CostFunctionEvaluator

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_REPO_ROOT: Final[Path] = Path(__file__).resolve().parent.parent
_CFG_DEFAULT: Final[Path] = _REPO_ROOT / "configurations" / "scenario.yaml"
_OUT_DIR: Final[Path] = _REPO_ROOT / "experiments" / "output_result"
_SCENARIO_DIR: Final[Path] = _REPO_ROOT / "scenarios"


def _scenario_output_dir(scenario_type: str) -> Path:
    if scenario_type == "bus_stop_bay":
        return _OUT_DIR / "result_bay"
    if scenario_type == "bus_stop_bulb":
        return _OUT_DIR / "result_bulb"
    return _OUT_DIR

# ---------------------------------------------------------------------------
# Public API – 1) cost evaluation
# ---------------------------------------------------------------------------

def evaluate_solution_costs(
    cfg_path: str | Path | None = None,
    csv_stem: str = "solution_costs",
    verbose: bool = False,
):
    """Evaluate costs and (optionally) export them as a CSV.

    The CSV filename is ``<stem>_<scenario_type>_<postopt|nopostopt>.csv``.
    """

    cfg_path = Path(cfg_path) if cfg_path is not None else _CFG_DEFAULT
    cfg = yaml.safe_load(cfg_path.read_text())

    scenario_type = cfg["scenario"]["type"]
    solution_tag = scenario_type.replace("_", "")
    out_dir = _scenario_output_dir(scenario_type)

    solution_file = out_dir / f"solution_KS1:WX1:DEU_{solution_tag}-1:2020a.xml"
    scenario_file = _SCENARIO_DIR / scenario_type / f"{scenario_type}.xml"

    if verbose:
        print(f"[INFO] Scenario   : {scenario_file}")
        print(f"[INFO] Solution   : {solution_file}")

    scenario, pp_set = CommonRoadFileReader(scenario_file).open()
    solution = CommonRoadSolutionReader.open(str(solution_file))

    evaluator = CostFunctionEvaluator.init_from_solution(solution)
    sol_result = evaluator.evaluate_solution(scenario, pp_set, solution)

    if cfg["debug"].get("save_Solution_Costs_as_csv", False):
        rows = _flatten_solution_result(sol_result)
        opt_flag = "postopt" if cfg["debug"].get("use_post_opt", False) else "original"
        csv_name = f"{csv_stem}_{scenario_type}_{opt_flag}.csv"
        out_path = out_dir / csv_name
        out_dir.mkdir(parents=True, exist_ok=True)
        pd.DataFrame(rows).to_csv(out_path, index=False)
        if verbose:
            print(f"[INFO] CSV saved : {out_path}")

    return sol_result


# ---------------------------------------------------------------------------
# Public API – 2) simulation + GIF rendering
# ---------------------------------------------------------------------------

def simulate_and_render(
    cfg_path: str | Path | None = None,
    gif_stem: str = "sumo_result",
    display_inline: bool = True,
    verbose: bool = False,
):
    """Run the reactive‑planner simulation and render a GIF.

    Parameters
    ----------
    cfg_path
        Path to YAML; *None* falls back to the project default.
    gif_stem
        Base name for the GIF. Final name will be
        ``<stem>_<scenario_type>.gif``.
    display_inline
        When *True* and running in a notebook, display the GIF.
    verbose
        Print key paths when *True*.
    """

    # --- Load config and common paths ---------------------------------
    cfg_path = Path(cfg_path) if cfg_path is not None else _CFG_DEFAULT
    cfg = yaml.safe_load(cfg_path.read_text())
    scenario_type = cfg["scenario"]["type"]
    solution_tag = scenario_type.replace("_", "")
    out_dir = _scenario_output_dir(scenario_type)

    solution_file = out_dir / f"solution_KS1:WX1:DEU_{solution_tag}-1:2020a.xml"
    scenario_dir = _SCENARIO_DIR / scenario_type
    scenario_file = scenario_dir / f"{scenario_type}.cr.xml"

    if verbose:
        print(f"[INFO] Scenario   : {scenario_file}")
        print(f"[INFO] Solution   : {solution_file}")

    # --- Read scenario & solution -------------------------------------
    scenario, pp_set = CommonRoadFileReader(str(scenario_file)).open()
    solution: Solution = CommonRoadSolutionReader.open(str(solution_file))
    planning_problem = next(iter(pp_set.planning_problem_dict.values()))

    # --- Simulate with the reactive planner ---------------------------
    from source.simulation.simulations import simulate_with_solution  # local import

    reactive_scenario, pp_set_sim, _ = simulate_with_solution(
        interactive_scenario_path=str(scenario_dir), solution=solution
    )

    # --- Build ego obstacle ------------------------------------------
    ego_obstacle = next(iter(solution.create_dynamic_obstacle().values()))
    ego_obstacle._obstacle_id = 10
    ego_obstacle.obstacle_shape.length = 12.95
    ego_obstacle.obstacle_shape.width = 2.55
    ego_obstacle._obstacle_type = ObstacleType.BUS
    reactive_scenario.add_objects(ego_obstacle)

    if scenario_type == "bus_stop_bulb":
        reactive_scenario.obstacle_by_id(3004)._obstacle_type = ObstacleType.BICYCLE
        reactive_scenario.obstacle_by_id(3001)._obstacle_type = ObstacleType.BICYCLE

    # --- Drawing parameters ------------------------------------------
    ego_params = DynamicObstacleParams()
    ego_params.draw_icon = True
    ego_params.vehicle_shape.occupancy.draw_occupancies = True
    ego_params.vehicle_shape.occupancy.shape.facecolor = "#3A6EA5"
    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#1F487E"
    ego_params.vehicle_shape.direction.zorder = 55
    ego_params.draw_shape = True
    ego_params.trajectory.draw_trajectory = True

    draw_params = MPDrawParams()
    draw_params.time_end = ego_obstacle.prediction.final_time_step
    draw_params.dynamic_obstacle.show_label = True
    draw_params.dynamic_obstacle.draw_icon = True
    draw_params.dynamic_obstacle.draw_shape = True
    draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    draw_params.dynamic_obstacle.occupancy.shape.edgecolor = "#9C4100"
    draw_params.dynamic_obstacle.trajectory.draw_trajectory = True

    # --- Render -------------------------------------------------------
    out_dir.mkdir(parents=True, exist_ok=True)
    gif_name = out_dir / f"{gif_stem}_{scenario_type}.gif"

    renderer = MPRenderer()
    renderer.create_video(
        [reactive_scenario, planning_problem, ego_obstacle],
        str(gif_name),
        draw_params=[draw_params, draw_params, ego_params],
    )

    if verbose:
        print(f"[INFO] GIF saved  : {gif_name}")

    if display_inline:
        display(Image(filename=str(gif_name)))

    return gif_name


# ---------------------------------------------------------------------------
# Internal helper – flatten SolutionResult
# ---------------------------------------------------------------------------

def _flatten_solution_result(sol_res):
    """Return ``list[dict]`` suitable for ``pandas.DataFrame``."""

    # Locate attribute that maps pp‑id → PlanningProblemCostResult
    for attr in ("pp_results", "partial_costs", "costs_per_planning_problem"):
        if hasattr(sol_res, attr):
            mapping = getattr(sol_res, attr)
            if isinstance(mapping, Mapping) and mapping:
                break
    else:
        raise TypeError(f"No cost map found in {type(sol_res)}.")

    total_global = getattr(sol_res, "total_costs", None)
    rows: List[dict] = []

    for pp_id, pp_cost in mapping.items():
        pp_dict = asdict(pp_cost) if is_dataclass(pp_cost) else pp_cost.__dict__

        weighted: dict[str, float] = {}
        for enum_key, raw_val in pp_dict["partial_costs"].items():
            weight = pp_dict["weights"][enum_key]
            key = enum_key.value if isinstance(enum_key, Enum) else str(enum_key)
            weighted[key] = raw_val * weight

        rows.append({
            "planning_problem_id": pp_id,
            **weighted,
            "total_costs": total_global,
        })

    return rows


if __name__ == "__main__":
    # Example standalone run for quick testing

    simulate_and_render(verbose=False)
