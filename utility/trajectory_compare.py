
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
from utility.visualization_2 import plot_state_dual_config

# Add project path to system path
path_notebook = os.getcwd()
sys.path.append(os.path.join(path_notebook, "../"))

# Define project paths using strings
path_root = str(Path(__file__).resolve().parent.parent)
config_path = os.path.join(path_root, "configurations", "scenario.yaml")
scenarios_root = os.path.join(path_root, "scenarios")

# Load configurations
with open(config_path, "r") as f:
    cfg = yaml.safe_load(f)

bus_stop = cfg["scenario"]["type"]
opt = cfg["debug"]["use_post_opt"]
if bus_stop == "bus_stop_bay":
    solutions_dir = os.path.join(path_root, "experiments", "output_result", "result_bay")
elif bus_stop == "bus_stop_bulb":
    solutions_dir = os.path.join(path_root, "experiments", "output_result", "result_bulb")
else:
    solutions_dir = os.path.join(path_root, "experiments", "output_result")

# Load solution file
bus_stop_no_underscore = bus_stop.replace("_", "")


solution_dir_True = os.path.join(solutions_dir, f"{bus_stop}_True")
solution_dir_False = os.path.join(solutions_dir, f"{bus_stop}_False")

solution_file_True = os.path.join(solution_dir_True, f"solution_KS1:WX1:DEU_{bus_stop_no_underscore}-1:2020a.xml")
solution_file_False = os.path.join(solution_dir_False, f"solution_KS1:WX1:DEU_{bus_stop_no_underscore}-1:2020a.xml")


if not os.path.exists(solution_file_True):
    print(f"Error: Solution file not found: {solution_file_True}")
    if os.path.exists(solution_dir_True):
        print(f"Directory contents: {os.listdir(solution_dir_True)}")
    else:
        print("Directory does not exist")
    sys.exit(1)

if not os.path.exists(solution_file_False):
    print(f"Error: Solution file not found: {solution_file_False}")
    if os.path.exists(solution_dir_False):
        print(f"Directory contents: {os.listdir(solution_dir_False)}")
    else:
        print("Directory does not exist")
    sys.exit(1)


try:
    solution_True = CommonRoadSolutionReader.open(solution_file_True)
    solution_False = CommonRoadSolutionReader.open(solution_file_False)

    print(f"Successfully loaded solutions:")
    print(f"True solution: {len(solution_True.planning_problem_solutions)} planning problems")
    print(f"False solution: {len(solution_False.planning_problem_solutions)} planning problems")

except Exception as e:
    print(f"Error loading solution files: {e}")
    sys.exit(1)


if len(solution_True.planning_problem_solutions) == 0:
    print("Error: No planning problem solutions found in True solution")
    sys.exit(1)

if len(solution_False.planning_problem_solutions) == 0:
    print("Error: No planning problem solutions found in False solution")
    sys.exit(1)

# Load trajectories
trajectory_True = solution_True.planning_problem_solutions[0].trajectory
trajectory_False = solution_False.planning_problem_solutions[0].trajectory

print(f"Trajectory True: {len(trajectory_True.state_list)} states")
print(f"Trajectory False: {len(trajectory_False.state_list)} states")



try:
    #define transition point
    state_transitions = {

        'post_opt': {
            46: ('Heading to next station→Arriving', '#FF9800'),  # Heading to Arriving
            264: ('Arriving→Boarding and Alighting', '#9E9E9E'),  # Arriving to Boarding
            276: ('Boarding and Alighting→Departure', '#2196F3'),  # Boarding to Departure
            410: ('Departure→Heading to next station', '#4CAF50')  # Departure to Heading
        },
        'original': {
            46: ('H→A', '#FF9800'),
            274: ('A→B', '#9E9E9E'),
            286: ('B→D', '#2196F3'),
            431: ('D→H', '#4CAF50')
        }
    }


    plot_state_dual_config(
        trajectory_True,
        trajectory_False,
        "comparison_result",
        labels=["the post-optimized planner", "CommonRoad Reactive Planner"],
        show_state_markers=True,
        state_transitions=state_transitions,
        bus_stop_type=bus_stop
    )
    print("Comparison plots with state transition markers generated successfully!")

except Exception as e:
    print(f"Error generating plots: {e}")
    import traceback

    traceback.print_exc()
