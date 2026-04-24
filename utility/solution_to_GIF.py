from pathlib import Path
import numpy as np
import yaml
from typing import List
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad.visualization.draw_params import DynamicObstacleParams
from commonroad.visualization.mp_renderer import MPDrawParams, MPRenderer
from IPython.display import Image, display,Video
from commonroad.scenario.obstacle import Obstacle,ObstacleType
from source.simulation.simulations import simulate_with_solution
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, TrafficSignIDGermany
from commonroad.geometry.shape import Circle
from commonroad.scenario.state import InitialState, ExtendedPMState,TraceState
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import  ObstacleType, DynamicObstacle




# 1. Base paths
path_root = Path(__file__).resolve().parent.parent
config_path = path_root / "configurations" / "scenario.yaml"

scenarios_root = path_root / "scenarios"
viz_dir = path_root / "experiments" / "output_result"

# 2. Load config
with config_path.open("r") as f:
    cfg = yaml.safe_load(f)
bus_stop = cfg["scenario"]["type"]

solutions_dir = path_root / "experiments" / "output_result" / f"{bus_stop}_True"
# 3. Load solution file
bus_stop_no_underscore = bus_stop.replace("_", "")
solution_file = (
    solutions_dir /f"solution_KS1:WX1:DEU_{bus_stop_no_underscore}-1:2020a.xml"
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


#6 passenger
#passenger:
t_start, t_end = 264, 276
p_shape = Circle(0.5)

# passengers
pos1 = np.array([1.0, -9.0])
init_state_p1 = InitialState(time_step=t_start, position=pos1, velocity=5, orientation=0.0)
p1 = DynamicObstacle(obstacle_id=4001, obstacle_type=ObstacleType.PEDESTRIAN, obstacle_shape=p_shape, initial_state=init_state_p1)
states_p1: List[TraceState] = [
    ExtendedPMState(time_step=t,
                    position=pos1 + np.array([ 0.0,0.01 * (t - t_start)]),
                    velocity=0.2, orientation=0.0, acceleration=0.0)
    for t in range(t_start, t_end + 1)
]
traj_p1 = Trajectory(t_start, states_p1)
p1.prediction = TrajectoryPrediction(traj_p1, p_shape)

# passengers
pos2_0 = np.array([-1.0, -9.0])
init_state_p2 = InitialState(time_step=t_start, position=pos2_0, velocity=5, orientation=0.0)
p2 = DynamicObstacle(obstacle_id=4002, obstacle_type=ObstacleType.PEDESTRIAN, obstacle_shape=p_shape, initial_state=init_state_p2)
states_p2: List[TraceState] = [
    ExtendedPMState(time_step=t,
                    position=pos2_0 + np.array([0.0,0.01*(t - t_start)]),
                    velocity=1, orientation=0.0, acceleration=0.0)
    for t in range(t_start, t_end + 1)
]
traj_p2 = Trajectory(t_start, states_p2)
p2.prediction = TrajectoryPrediction(traj_p2, p_shape)


ped_params = DynamicObstacleParams()
ped_params.draw_icon = True
ped_params.use_type_color = True
ped_params.trajectory.draw_trajectory = False
ped_params.occupancy.draw_occupancies = False
ped_params.vehicle_shape.occupancy.shape.zorder = 1000



# 7. Add ego vehicle to reactive_scenario
ego_obstacle = next(iter(solution.create_dynamic_obstacle().values()))
ego_obstacle._obstacle_id = 10
ego_obstacle.obstacle_shape.length = 12.95
ego_obstacle.obstacle_shape.width = 2.55

ego_obstacle._obstacle_type = ObstacleType.BUS
reactive_scenario.add_objects(ego_obstacle)
# if bus_stop == "bus_stop_bulb":
#    reactive_scenario.obstacle_by_id(3004)._obstacle_type = ObstacleType.BICYCLE
#    reactive_scenario.obstacle_by_id(3001)._obstacle_type = ObstacleType.BICYCLE

ego_params = DynamicObstacleParams()
ego_params.draw_icon = True
# ego_params.vehicle_shape.occupancy.draw_occupancies = True
ego_params.occupancy.draw_occupancies = True
ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
ego_params.vehicle_shape.occupancy.shape.edgecolor = "#C55A11"
ego_params.vehicle_shape.direction.zorder = 55
ego_params.draw_shape = True
ego_params.trajectory.draw_trajectory = True

# 8. Set up renderer params
draw_params = MPDrawParams()
draw_params.time_end = ego_obstacle.prediction.final_time_step
draw_params.dynamic_obstacle.show_label = False
draw_params.dynamic_obstacle.draw_icon = True
draw_params.dynamic_obstacle.draw_shape = True
draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
draw_params.dynamic_obstacle.occupancy.shape.edgecolor = "#9C4100"
draw_params.dynamic_obstacle.trajectory.draw_trajectory = True

# car parameter
car_params = DynamicObstacleParams()
car_params.use_type_color = False
car_params.time_begin = 0
car_params.occupancy.draw_occupancies=True
car_params.vehicle_shape.occupancy.shape.facecolor = "#43A047"  #  (Green-600)
car_params.vehicle_shape.occupancy.shape.edgecolor  = "#000000"  # black
car_params.draw_icon = True
car_params.zorder= 200
car_params.show_label = False
car_params.vehicle_shape.occupancy.shape.zorder = 100
car_params.vehicle_shape.occupancy.shape.opacity = 1

#bike parameter
# bike_obs = reactive_scenario.obstacle_by_id(3003)
if bus_stop == "bus_stop_bulb":
   bike_ids = [3001,3005]
   bike_obs = []
   for oid in bike_ids:
      o = reactive_scenario.obstacle_by_id(oid)
      if o is None:
        print(f"[warn] obstacle {oid} not found")
        continue
      o._obstacle_type = ObstacleType.BICYCLE
      bike_obs.append(o)
if bus_stop == "bus_stop_bay":
   bike_ids = [3006]
   bike_obs = []
   for oid in bike_ids:
      o = reactive_scenario.obstacle_by_id(oid)
      if o is None:
        print(f"[warn] obstacle {oid} not found")
        continue
      o._obstacle_type = ObstacleType.BICYCLE
      bike_obs.append(o)

bicycle_params = DynamicObstacleParams()
bicycle_params.use_type_color = True  # default color
bicycle_params.time_begin = 0
bicycle_params.vehicle_shape.occupancy.draw_occupancies = True
bicycle_params.draw_icon = True
bicycle_params.vehicle_shape.occupancy.shape.zorder = 100
bicycle_params.vehicle_shape.occupancy.shape.opacity = 1



#car
others = [o for o in reactive_scenario.dynamic_obstacles
          if o.obstacle_id not in {ego_obstacle.obstacle_id, *bike_ids}]

#bike lane
if bus_stop == "bus_stop_bay":
  bicycle_lane_ids = [310, 313, 303, 314, 315]
  bike_polys = [scenario.lanelet_network.find_lanelet_by_id(i).polygon for i in bicycle_lane_ids]
elif bus_stop == "bus_stop_bulb":
  bicycle_lane_ids = [3]
  bike_polys = [scenario.lanelet_network.find_lanelet_by_id(i).polygon for i in bicycle_lane_ids]


bike_dp = MPDrawParams()
bike_dp.time_begin = 0
bike_dp.occupancy.shape.zorder =1
bike_dp.time_end   = draw_params.time_end
bike_dp.shape.facecolor = "#98c6ea"
bike_dp.shape.edgecolor = "#98c6ea"

#bus_stop
if bus_stop == "bus_stop_bay":
   bus_stop_sign_element = TrafficSignElement(TrafficSignIDGermany.BUS_STOP)
   bus_stop_sign = TrafficSign(
    400,
    [bus_stop_sign_element],
    {7},
    np.array([10.0, -7.5])
)
elif bus_stop == "bus_stop_bulb":
   bus_stop_sign_element = TrafficSignElement(TrafficSignIDGermany.BUS_STOP)
   bus_stop_sign = TrafficSign(
        400,
        [bus_stop_sign_element],
        {7},
        np.array([10.0, -4])
    )



sign_dp = MPDrawParams()
sign_dp.time_begin = 0
sign_dp.time_end   = draw_params.time_end
sign_dp.traffic_sign.draw_traffic_signs = True
sign_dp.traffic_sign.show_label = False       #
sign_dp.traffic_sign.zorder = 1

rnd = MPRenderer()

rnd.draw_params.axis_visible = False
rnd.f.subplots_adjust(left=0, right=1, bottom=0, top=1)
obj_list = [reactive_scenario, planning_problem, ego_obstacle]+others+bike_polys+[bus_stop_sign]+bike_obs
param_list = [ draw_params ,draw_params, ego_params]+[car_params]*len(others)+[bike_dp] * len(bike_polys)+[sign_dp]+[bicycle_params]*len(bike_obs)

obj_list   += [p1, p2]
param_list += [ped_params, ped_params]
# 8. Ensure visualization directory exists
viz_dir.mkdir(parents=True, exist_ok=True)


# 9. Render MP4
mp4_name = viz_dir / f"sumo_result_{bus_stop}.mp4"
rnd.create_video(obj_list, str(mp4_name), draw_params=param_list)

# 10. Display  ← use Video not Image

display(Video(filename=str(mp4_name), embed=True))

## 9. Render GIF
# gif_name = viz_dir / f"sumo_result_{bus_stop}.gif"
# rnd.create_video(obj_list, str(gif_name), draw_params=param_list)
#
# # 10. Display
# display(Image(filename=str(gif_name)))
