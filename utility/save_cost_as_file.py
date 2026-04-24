import yaml
import pandas as pd
from pathlib import Path
from dataclasses import asdict, is_dataclass
from collections.abc import Mapping
from enum import Enum

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad_dc.costs.evaluation import CostFunctionEvaluator


def evaluate_solution_costs(
    cfg_path: str | Path | None = None,
    csv_stem: str = "solution_costs",
    verbose: bool = True,
):
    """Load *scenario.yaml*, evaluate solution costs, optionally export them as a CSV
    file whose name encodes the scenario type and whether post‑optimisation was used.

    Parameters
    ----------
    cfg_path
        Path to the YAML configuration. If *None*, defaults to
        ``<repo_root>/configurations/scenario.yaml``.
    csv_stem
        Base name for the CSV (without extension). Final name becomes
        ``<stem>_<scenario_type>_<postopt|nopostopt>.csv``.
    verbose
        When *True*, print the key paths and the final CSV path.
    """

    # ------------------------------------------------------------------
    # 1) Parse the YAML configuration
    repo_root = Path(__file__).resolve().parent.parent
    cfg_path = (
        Path(cfg_path)
        if cfg_path is not None
        else repo_root / "configurations" / "scenario.yaml"
    )
    cfg = yaml.safe_load(cfg_path.read_text())

    # ------------------------------------------------------------------
    # 2) Build absolute paths for the scenario and solution files
    scenario_type = cfg["scenario"]["type"]
    solutions_dir = repo_root / "experiments" / "output_result"
    solution_tag = scenario_type.replace("_", "")

    solution_file = (
        solutions_dir / f"solution_KS1:WX1:DEU_{solution_tag}-1:2020a.xml"
    )
    scenarios_path = repo_root / "scenarios" / scenario_type / f"{scenario_type}.xml"

    if verbose:
        print(f"[INFO] Scenario file: {scenarios_path}")
        print(f"[INFO] Solution file: {solution_file}")

    # ------------------------------------------------------------------
    # 3) Load the scenario and the solution
    scenario, planning_problem_set = CommonRoadFileReader(scenarios_path).open()
    solution = CommonRoadSolutionReader.open(str(solution_file))

    # ------------------------------------------------------------------
    # 4) Evaluate costs using CommonRoad‑DC
    evaluator = CostFunctionEvaluator.init_from_solution(solution)
    cost_result = evaluator.evaluate_solution(
        scenario, planning_problem_set, solution
    )

    # ------------------------------------------------------------------
    # 5) Flatten results and export to CSV when enabled
    if cfg["debug"].get("save_Solution_Costs_as_csv", False):
        rows = _flatten_solution_result(cost_result)

        # Compose dynamic file name
        opt_flag = "postopt" if cfg["debug"].get("use_post_opt", False) else "original"
        csv_name = f"{csv_stem}_{scenario_type}_{opt_flag}.csv"
        out_path = solutions_dir / csv_name

        pd.DataFrame(rows).to_csv(out_path, index=False)
        if verbose:
            print(f"[INFO] CSV written to {out_path}")

    return cost_result


# ----------------------------------------------------------------------
# Helper: SolutionResult → list[dict]
# ----------------------------------------------------------------------

def _flatten_solution_result(sol_res):
    """Convert *SolutionResult* into a row‑based structure suitable for
    ``pandas.DataFrame``.

    * Column names derive from :class:`PartialCostFunction` values (``'T'``,
      ``'A'`` …).
    * Each value is the **weighted** cost, i.e. ``cost * weight``.
    """

    # Find the attribute that holds ``{pp_id: PlanningProblemCostResult}``
    for attr in ("pp_results", "partial_costs", "costs_per_planning_problem"):
        if hasattr(sol_res, attr):
            mapping = getattr(sol_res, attr)
            if isinstance(mapping, Mapping) and mapping:
                break
    else:
        raise TypeError(
            f"No planning‑problem cost map found in {type(sol_res)}."
        )

    total_global = getattr(sol_res, "total_costs", None)
    rows: list[dict] = []

    for pp_id, pp_cost in mapping.items():
        pp_dict = asdict(pp_cost) if is_dataclass(pp_cost) else pp_cost.__dict__

        weighted: dict[str, float] = {}
        for enum_key, raw_val in pp_dict["partial_costs"].items():
            weight = pp_dict["weights"][enum_key]
            key_str = enum_key.value if isinstance(enum_key, Enum) else str(enum_key)
            weighted[key_str] = raw_val * weight

        rows.append({
            "planning_problem_id": pp_id,
            **weighted,
            "total_costs": total_global,
        })

    return rows


if __name__ == "__main__":
    evaluate_solution_costs(verbose=True)
