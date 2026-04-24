import copy
import importlib.resources as pkg_resources
import logging
import shelve
import warnings
from collections import defaultdict
from dataclasses import dataclass
from functools import lru_cache, partial
from pathlib import Path
from typing import Optional, Set, Union

import numpy as np
from commonroad.common.solution import PlanningProblemSolution, vehicle_parameters
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import ObstacleType, Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.feasibility.solution_checker import (
    _simulate_trajectory_if_input_vector,
)

import source.crmonitor as crmonitor
from source.crmonitor.common.helper import create_other_vehicles_param, load_yaml
from source.crmonitor.common.road_network import RoadNetwork
from source.crmonitor.common.vehicle import (
    ControlledVehicle,
    CurvilinearStateManager,
    DynamicObstaclePTVehicle,
    DynamicObstacleVehicle,
    PredicateCache,
    Vehicle,
)


@lru_cache(maxsize=None)
def get_world_config():
    with pkg_resources.path(crmonitor, "config.yaml") as config_path:
        config = load_yaml(config_path)
    return config


@lru_cache(maxsize=None)
def get_world_config_pt():
    with pkg_resources.path(crmonitor, "config_pt.yaml") as config_path:
        config = load_yaml(config_path)
    return config


@dataclass
class World:
    vehicles: Set[Vehicle]
    road_network: RoadNetwork
    scenario: Optional[Scenario] = None
    cache: Union[None, shelve.Shelf, dict] = None

    @classmethod
    def create_from_solution(
        cls,
        scenario: Scenario,
        planning_problem: PlanningProblem,
        planning_problem_solution: PlanningProblemSolution,
    ):
        """Create a rule evaluator to check a planning problem solution."""
        pp_id = planning_problem.planning_problem_id
        _, trajectory = _simulate_trajectory_if_input_vector(
            PlanningProblemSet([planning_problem]),
            planning_problem_solution,
            scenario.dt,
        )
        # We have to remove the initial time step
        trajectory = Trajectory(
            trajectory.state_list[1].time_step, trajectory.state_list[1:]
        )
        shape = Rectangle(
            length=vehicle_parameters[planning_problem_solution.vehicle_type].l,
            width=vehicle_parameters[planning_problem_solution.vehicle_type].w,
        )
        prediction = TrajectoryPrediction(trajectory, shape=shape)
        obstacle = DynamicObstacle(
            # FIXME
            obstacle_id=pp_id + 1000,
            obstacle_type=ObstacleType.CAR,
            obstacle_shape=shape,
            initial_state=planning_problem.initial_state,
            prediction=prediction,
        )
        scenario = copy.deepcopy(scenario)
        scenario.add_objects(obstacle)
        scenario.assign_obstacles_to_lanelets()
        world = World.create_from_scenario(scenario)
        ego_vehicle = world.vehicle_by_id(obstacle.obstacle_id)
        return world, ego_vehicle

    def _warn_persistent_cache(self):
        if len(self.controlled_vehicle_ids) > 0 and isinstance(
            self.cache, shelve.Shelf
        ):
            warnings.warn(
                "Using controlled vehicles with persistent caching may result in inconsistent caches and is therfore discouraged!"
            )

    def __post_init__(self):
        self._warn_persistent_cache()

    def add_vehicle(self, vehicle: Vehicle):
        self.vehicles.add(vehicle)
        self._warn_persistent_cache()

    @classmethod
    def create_from_scenario(
        cls, scenario: Scenario, config=None, road_network=None, cache_dir=None
    ):
        if config is None:
            config = get_world_config()
        if road_network is None:
            params = config.get("road_network_param")
            road_network = RoadNetwork(scenario.lanelet_network, params)
        else:
            road_network = road_network
        others_params = create_other_vehicles_param(config.get("other_vehicles_param"))
        vehicles = set()
        if cache_dir is not None:
            cache_file = Path(cache_dir) / f"{scenario.scenario_id}"
            cache = shelve.open(str(cache_file), writeback=True)
        else:
            cache = {}
        for obs in filter(
            lambda o: o.obstacle_type
            in [
                ObstacleType.CAR,
                ObstacleType.BUS,
                ObstacleType.TRUCK,
                ObstacleType.MOTORCYCLE,
                ObstacleType.TAXI,
            ],
            scenario.dynamic_obstacles,
        ):
            # Skip obstacles that go out of the road
            if any(
                map(
                    lambda a: len(a) == 0,
                    obs.prediction.shape_lanelet_assignment.values(),
                )
            ):
                continue
            cls.augment_state_acceleration_jerk(scenario.dt, obs)
            curvi_cache, predicate_dict = cache.setdefault(
                str(obs.obstacle_id), (dict(), defaultdict(partial(defaultdict, dict)))
            )
            if obs.obstacle_type == ObstacleType.BUS:
                vehicles.add(
                    DynamicObstaclePTVehicle(
                        obs,
                        CurvilinearStateManager(road_network, curvi_cache),
                        others_params,
                        PredicateCache(predicate_dict),
                    )
                )
            else:
                vehicles.add(
                    DynamicObstacleVehicle(
                        obs,
                        CurvilinearStateManager(road_network, curvi_cache),
                        others_params,
                        PredicateCache(predicate_dict),
                    )
                )
        return cls(vehicles, road_network, scenario, cache)

    @property
    def controlled_vehicle_ids(self) -> Set[int]:
        return {
            vehicle.id
            for vehicle in self.vehicles
            if isinstance(vehicle, ControlledVehicle)
        }

    def vehicle_ids_for_time_step(self, time_step: int):
        return [v.id for v in self.vehicles if v.is_valid(time_step)]

    @staticmethod
    def augment_state_acceleration_jerk(dt, obs):
        accelerations = (
            np.diff(
                [
                    s.velocity
                    for s in [obs.initial_state] + obs.prediction.trajectory.state_list
                ]
            )
            / dt
        ).tolist()
        obs.initial_state.acceleration = accelerations[0]
        if len(accelerations) >= 2:
            jerk = (np.diff(accelerations) / dt).tolist()
            accelerations += accelerations[-1:]
            jerk += [jerk[-1], 0]
            obs.initial_state.jerk = jerk[0]
        else:
            jerk = [None] * 2
        for a, j, state in zip(
            accelerations[1:], jerk[1:], obs.prediction.trajectory.state_list
        ):
            state.acceleration = a
            if j is not None:
                state.jerk = j

    def vehicle_by_id(self, id) -> Optional[Vehicle]:
        for veh in self.vehicles:
            if veh.id == id:
                break
        else:
            logging.warning(f"Vehicle with ID {id} not found!")
            veh = None
        return veh

    @property
    def dt(self):
        if self.scenario is not None:
            return self.scenario.dt
        else:
            return 0.1

    def __del__(self):
        if isinstance(self.cache, shelve.Shelf):
            logging.info("Cache close!")
            self.cache.close()
