import time, copy, numpy as np
from source.commonroad_rp.utility.config import ReactivePlannerConfiguration
from source.commonroad_rp.reactive_planner import ReactivePlanner
from commonroad.common.file_reader import CommonRoadFileReader
from source.commonroad_rp.utility.utils_coordinate_system import create_coordinate_system
from post_optimization_planner.State import _extend_path
from source.commonroad_rp.state import ReactivePlannerState
import warnings; warnings.filterwarnings("ignore")

scenario, pp_set = CommonRoadFileReader("scenarios/bus_stop_bay/bus_stop_bay.xml").open()
planning_problem = list(pp_set.planning_problem_dict.values())[0]
cfg = ReactivePlannerConfiguration.load("configurations/bus_stop_bay/heading_to_next.yaml")
cfg.update(scenario, planning_problem=planning_problem)
print(f"multiproc={cfg.debug.multiproc}, num_workers={cfg.debug.num_workers}")

vertices = scenario.lanelet_network.find_lanelet_by_id(2).center_vertices
coord_sys = create_coordinate_system(_extend_path(vertices))
init = planning_problem.initial_state
x0 = ReactivePlannerState(position=init.position, velocity=init.velocity,
                           orientation=init.orientation, acceleration=0.0,
                           yaw_rate=0.0, steering_angle=0.0, time_step=0)

p = ReactivePlanner(cfg)
p.x_0 = copy.deepcopy(x0)
p.record_state_and_input(p.x_0)
p.set_reference_path(coordinate_system=coord_sys)
p.reset(cfg, collision_checker=p.collision_checker, coordinate_system=p.coordinate_system)
p.set_desired_velocity(current_speed=x0.velocity, desired_velocity=cfg.sampling.desire_velocity)
p.plan()  # warm-up

times = []
for i in range(8):
    p.x_0 = copy.deepcopy(x0)
    p.x_0_cl = None
    t0 = time.perf_counter()
    p.plan()
    elapsed = (time.perf_counter() - t0) * 1000
    times.append(elapsed)
    print(f"  step {i+1}: {elapsed:.1f} ms")

print(f"\nmean={np.mean(times):.1f} ms  median={np.median(times):.1f} ms  min={np.min(times):.1f} ms  max={np.max(times):.1f} ms")
