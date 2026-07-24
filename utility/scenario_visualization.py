from pathlib import Path
from commonroad.prediction.prediction import Occupancy
from commonroad.visualization.draw_params import OccupancyParams
import yaml
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad.visualization.draw_params import DynamicObstacleParams
from commonroad.visualization.mp_renderer import MPDrawParams, MPRenderer
from IPython.display import Image, display
from commonroad.scenario.obstacle import Obstacle,ObstacleType
from source.simulation.simulations import simulate_with_solution
# standard imports
from utility.edit_scenario import create_passenger_prediction
import os
import sys
# third party
import numpy as np
from matplotlib import pyplot as plt
path_notebook = os.getcwd()
# commonroad-io
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Rectangle, Circle
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, TrafficSignIDGermany
from commonroad.scenario.obstacle import  ObstacleType, DynamicObstacle
from commonroad.scenario.state import InitialState
from commonroad.visualization.draw_params import ShapeParams
from commonroad.scenario.state import InitialState, TraceState
from commonroad.scenario.trajectory import Trajectory

from commonroad.scenario.state import InitialState, TraceState

from commonroad.prediction.prediction import TrajectoryPrediction
# 1. Base paths
path_root = Path(__file__).resolve().parent.parent
config_path = path_root / "configurations" / "scenario.yaml"

scenarios_root = path_root / "scenarios"

# 2. Load config
with config_path.open("r") as f:
    cfg = yaml.safe_load(f)
bus_stop = cfg["scenario"]["type"]
if bus_stop == "bus_stop_bay":
    viz_dir = path_root / "experiments" / "output_result" / "result_bay"
elif bus_stop == "bus_stop_bulb":
    viz_dir = path_root / "experiments" / "output_result" / "result_bulb"
else:
    viz_dir = path_root / "experiments" / "output_result"

solutions_dir = viz_dir / f"{bus_stop}_True"
# 3. Load solution file
bus_stop_no_underscore = bus_stop.replace("_", "")
solution_file = (
    solutions_dir / f"solution_KS1:WX1:DEU_{bus_stop_no_underscore}-1:2020a.xml"
)
solution = CommonRoadSolutionReader.open(str(solution_file))

# 4. Build scenario paths
cr_file = scenarios_root / bus_stop / f"{bus_stop}.cr.xml"
scenario, planning_problem_set = CommonRoadFileReader(str(cr_file)).open()
planning_problem = next(iter(planning_problem_set.planning_problem_dict.values()))

# 5. Run simulation with solution
reactive_scenario, planning_problem_set, ego_vehicles = simulate_with_solution(
    interactive_scenario_path=str(scenarios_root / bus_stop), solution=solution
)
p_shape = Circle(0.5)
b_shape = Rectangle(10,2,orientation=0.0)
init_state_p1 = InitialState(time_step=0, position=np.array([12, -5]), acceleration=0, velocity=0)
p1 = DynamicObstacle(obstacle_id=4001, obstacle_type=ObstacleType.PEDESTRIAN, obstacle_shape=p_shape, initial_state=init_state_p1)
init_state_p2 = InitialState(time_step=0, position=np.array([10, -5]), acceleration=0, velocity=0)
p2 = DynamicObstacle(obstacle_id=4002, obstacle_type=ObstacleType.PEDESTRIAN, obstacle_shape=p_shape, initial_state=init_state_p2)
create_passenger_prediction(reactive_scenario, p1, 10, 0, 1)
create_passenger_prediction(reactive_scenario, p2, 10, 0, 1)
init_state_b1 = InitialState(time_step=0, position=np.array([0, -11]), acceleration=0, velocity=0,orientation=0)
bike = DynamicObstacle(obstacle_id=5001, obstacle_type=ObstacleType.BICYCLE,initial_state=init_state_b1,obstacle_shape=b_shape)
reactive_scenario.add_objects(bike)
reactive_scenario.add_objects(p1)
reactive_scenario.add_objects(p2)




ego_obstacle = next(iter(solution.create_dynamic_obstacle().values()))
ego_obstacle._obstacle_id = 10
ego_obstacle.obstacle_shape.length = 12.95
ego_obstacle.obstacle_shape.width = 2.55
ego_obstacle._obstacle_type = ObstacleType.BUS

# Bus parameters in TUM orange (ego vehicle)
ego_params = DynamicObstacleParams()
ego_params.draw_icon = True
ego_params.use_type_color = False  # Do not use the default type color
ego_params.vehicle_shape.occupancy.draw_occupancies = True
ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"  # TUM Orange
ego_params.vehicle_shape.occupancy.shape.edgecolor = "#C55A11"  # Darker orange outline
ego_params.vehicle_shape.direction.zorder = 55
ego_params.draw_shape = True
ego_params.time_begin = 0
# ego_params.time_end = 0

# Car parameters
car_params = DynamicObstacleParams()
car_params.use_type_color = False  # Do not use the default type color
car_params.time_begin = 0
# car_params.time_end = 0
car_params.vehicle_shape.occupancy.draw_occupancies = True
car_params.vehicle_shape.occupancy.shape.facecolor = "#43A047"  # Brighter green (Green-600)
car_params.vehicle_shape.occupancy.shape.edgecolor  = "#000000"  # Black outline
car_params.draw_icon = True
car_params.vehicle_shape.occupancy.shape.zorder = 100
car_params.vehicle_shape.occupancy.shape.opacity = 1

# Bicycle parameters using the default color
bicycle_params = DynamicObstacleParams()
bicycle_params.use_type_color = True  # Use the default type color
bicycle_params.time_begin = 0
# bicycle_params.time_end = 0
bicycle_params.vehicle_shape.occupancy.draw_occupancies = True

bicycle_params.draw_icon = True
bicycle_params.vehicle_shape.occupancy.shape.zorder = 100
bicycle_params.vehicle_shape.occupancy.shape.opacity = 1



# 7. Set up renderer params
draw_params = MPDrawParams()
draw_params.dynamic_obstacle.use_type_color = False  # Use the configured color for other obstacles
draw_params.time_begin = 0

draw_params.draw_icon = True
draw_params.traffic_sign.draw_traffic_signs = True

# Set ego-trajectory occupancy shape parameters (TUM orange trajectory)
occ_params = OccupancyParams()
occ_params.shape.facecolor = "#E37222"  # TUM Orange
occ_params.shape.edgecolor = "#C55A11"  # Dark-orange outline
occ_params.shape.opacity = 0.2
occ_params.shape.zorder = 20




# Configure high-resolution rendering
plt.figure(figsize=(16, 9))

rnd = MPRenderer(draw_params)
# reactive_scenario.draw(rnd)

# Draw bicycle lanes
bicycle_lane_ids = [310,313,303,314,315]
# bicycle_lane_ids = [3]
rnd.draw_params.shape.facecolor = "#98c6ea"
for id in bicycle_lane_ids:
    scenario.lanelet_network.find_lanelet_by_id(id).polygon.draw(rnd)

# Draw the ego-vehicle trajectory and vehicle (TUM orange bus)
if ego_obstacle is not None:
    # draw occupancies of ego vehicle trajectory
    [
        occ.draw(rnd, draw_params=occ_params)
        for occ in ego_obstacle.prediction.occupancy_set
    ]
    # visualize ego vehicle at specified time step
    ego_obstacle.draw(rnd, draw_params=ego_params)

# Configure and draw other vehicles
# Cars (red)
reactive_scenario.obstacle_by_id(3002)._obstacle_type = ObstacleType.CAR
reactive_scenario.obstacle_by_id(3001)._obstacle_type = ObstacleType.CAR
reactive_scenario.obstacle_by_id(3002).draw(rnd, car_params)
reactive_scenario.obstacle_by_id(3001).draw(rnd, car_params)

# # Bicycle (default color)
reactive_scenario.obstacle_by_id(5001)._obstacle_type = ObstacleType.BICYCLE
reactive_scenario.obstacle_by_id(5001).draw(rnd, bicycle_params)

# Add the bus-stop sign
bus_stop_sign_element = TrafficSignElement(TrafficSignIDGermany.BUS_STOP)
bus_stop_sign = TrafficSign(400, [bus_stop_sign_element], {7}, np.array([6, -4.5]))
bus_stop_sign.draw(rnd)

# Draw the ego vehicle again to keep it on top
ego_obstacle.draw(rnd, ego_params)

planning_problem.draw(rnd)
rnd.render()

obj_list = [reactive_scenario, planning_problem, ego_obstacle]
param_list = [draw_params, draw_params, ego_params]

# 8. Ensure visualization directory exists
viz_dir.mkdir(parents=True, exist_ok=True)

# 9. Render GIF
gif_name = viz_dir / f"sumo_result_{bus_stop}.gif"
rnd.create_video(obj_list, str(gif_name), draw_params=param_list)

# 10. Display
display(Image(filename=str(gif_name)))
