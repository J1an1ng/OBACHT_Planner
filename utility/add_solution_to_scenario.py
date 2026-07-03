import os
import sys
from datetime import datetime
from pathlib import Path

import yaml

# CommonRoad imports
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad.scenario.scenario import Tag
from commonroad.scenario.state import KSState

# Add project path to system path
path_notebook = os.getcwd()
sys.path.append(os.path.join(path_notebook, "../"))

# Define project paths
path_root = Path(__file__).resolve().parent.parent
config_path = path_root / "configurations" / "scenario.yaml"
scenarios_root = path_root / "scenarios"

# Load configurations
with config_path.open("r") as f:
    cfg = yaml.safe_load(f)
bus_stop = cfg["scenario"]["type"]
if bus_stop == "bus_stop_bay":
    solutions_dir = path_root / "experiments" / "output_result" / "result_bay"
elif bus_stop == "bus_stop_bulb":
    solutions_dir = path_root / "experiments" / "output_result" / "result_bulb"
else:
    solutions_dir = path_root / "experiments" / "output_result"

# Load solution file
bus_stop_no_underscore = bus_stop.replace("_", "")
solution_file = (
    solutions_dir / f"solution_KS1:WX1:DEU_{bus_stop_no_underscore}-1:2020a.xml"
)
solution = CommonRoadSolutionReader.open(str(solution_file))

# Load scenario
scenario_path = scenarios_root / bus_stop / f"{bus_stop}.cr.xml"
scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# Create and modify ego vehicle
ego_obstacle = list(solution.create_dynamic_obstacle().values())[0]

# Fix trajectory time steps
new_state_list = []
for idx, state in enumerate(ego_obstacle.prediction.trajectory.state_list):
    # Create new state with corrected time_step (starting from 1)
    new_state = KSState(
        time_step=idx + 1,
        position=state.position,
        steering_angle=state.steering_angle,
        velocity=state.velocity,
        orientation=state.orientation,
    )
    new_state_list.append(new_state)

# Replace original state list
ego_obstacle.prediction.trajectory._state_list = new_state_list

# Update ego vehicle properties
ego_obstacle._obstacle_id = 10
ego_obstacle._obstacle_shape.length = 12
ego_obstacle._obstacle_shape.width = 2.555
print(ego_obstacle.initial_shape_lanelet_ids)

# Add ego vehicle to scenario
scenario.add_objects(ego_obstacle)
scenario.obstacle_by_id(10)

# Define scenario metadata
author = "Youran Wang"
affiliation = "Technical University of Munich, Germany"
source = ""
tags = {Tag.CRITICAL, Tag.INTERSTATE}

# Write modified scenario to file
fw = CommonRoadFileWriter(
    scenario, planning_problem_set, author, affiliation, source, tags
)
filename = f"{bus_stop}.xml"
out_path = scenarios_root / bus_stop / f"{bus_stop}.xml"
fw.write_to_file(str(out_path) , OverwriteExistingFile.ALWAYS)
print("edit_time: {}".format(datetime.now()))
