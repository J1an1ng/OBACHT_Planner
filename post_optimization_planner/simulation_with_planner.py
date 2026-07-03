import os
import sys
from pathlib import Path
import yaml


path_root = Path(__file__).resolve().parent.parent
if str(path_root) not in sys.path:
    sys.path.insert(0, str(path_root))

from source.planner.planner import motion_planner_interactive
import post_optimization_planner.state_machine as sm


config_path = path_root / "configurations" / "scenario.yaml"
with open(config_path, "r") as f:
    cfg = yaml.safe_load(f)

bus_stop = cfg["scenario"]["type"]
#
scenario_dir = os.path.join(path_root, "scenarios")
if bus_stop == "bus_stop_bay":
    output_dir = os.path.join(path_root, "experiments", "output_result", "result_bay")
elif bus_stop == "bus_stop_bulb":
    output_dir = os.path.join(path_root, "experiments", "output_result", "result_bulb")
else:
    output_dir = os.path.join(path_root, "experiments", "output_result")
solution_dir = os.path.join(output_dir, f"{bus_stop}_commonroad_rp")


solution = motion_planner_interactive(
    os.path.join(scenario_dir, bus_stop), solution_dir, bus_stop
)
