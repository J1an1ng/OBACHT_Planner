import os
from pathlib import Path
import yaml
from source.planner.planner import motion_planner_interactive
import post_optimization_planner.state_machine as sm


path_root = Path(__file__).resolve().parent.parent
config_path = path_root / "configurations" / "scenario.yaml"
with open(config_path, "r") as f:
    cfg = yaml.safe_load(f)

bus_stop = cfg["scenario"]["type"]
opt = cfg["debug"]["use_post_opt"]
#
scenario_dir = os.path.join(path_root, "scenarios")
solution_dir = os.path.join(path_root, f"experiments/output_result/{bus_stop}_{opt}")


solution = motion_planner_interactive(
    os.path.join(scenario_dir, bus_stop), solution_dir, bus_stop
)

