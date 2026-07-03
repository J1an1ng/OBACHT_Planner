import os
from pathlib import Path
import  yaml

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader,CommonRoadSolutionWriter
from commonroad_dc.costs.evaluation import CostFunctionEvaluator




path_root = Path(__file__).resolve().parent.parent
config_path = path_root / "configurations" / "scenario.yaml"
with open(config_path, "r") as f:
    cfg = yaml.safe_load(f)

bus_stop = cfg["scenario"]["type"]
if bus_stop == "bus_stop_bay":
    solutions_dir = path_root / "experiments" / "output_result" / "result_bay"
elif bus_stop == "bus_stop_bulb":
    solutions_dir = path_root / "experiments" / "output_result" / "result_bulb"
else:
    solutions_dir = path_root / "experiments" / "output_result"
bus_stop_no_underscore = bus_stop.replace("_", "")
solution_file = (
    solutions_dir / f"solution_KS1:WX1:DEU_{bus_stop_no_underscore}-1:2020a.xml"
)
solution = CommonRoadSolutionReader.open(str(solution_file))

scenarios_path = path_root/"scenarios"/bus_stop/f"{bus_stop}.xml"
csw = CommonRoadSolutionWriter(solution)
# csw.write_to_file(overwrite=True)
# load the CommonRoad scenario that has been created in the CommonRoad tutorial

scenario, planning_problem_set = CommonRoadFileReader(scenarios_path).open()

ce = CostFunctionEvaluator.init_from_solution(solution)

cost_result = ce.evaluate_solution(scenario, planning_problem_set, solution)

print(cost_result)
print(cost_result.pp_results[1].partial_costs)
