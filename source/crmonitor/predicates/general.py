import logging
import math
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

import matplotlib.colors
import numpy as np
from commonroad.common.util import subtract_orientations
from commonroad.scenario.intersection import IntersectionIncomingElement
from commonroad.scenario.lanelet import LaneletType, LineMarking
from commonroad.scenario.traffic_light import TrafficLightState
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignIDGermany
from matplotlib import pyplot as plt

from source.crmonitor.common.helper import (
    cartesian_to_curvilinear,
    get_curvilinear_coordinate_system,
)
from source.crmonitor.common.road_network import Lane
from source.crmonitor.common.vehicle import DynamicObstaclePTVehicle, PTVehicle, Vehicle
from source.crmonitor.common.world import World
from source.crmonitor.predicates.base import BasePredicateEvaluator
from source.crmonitor.predicates.position import PredInFrontOf, PredInSameLane, PredSingleLane
from source.crmonitor.predicates.utils import cal_road_width, distance_to_left_bounds

logger = logging.getLogger(__name__)


class GeneralPredicates(str, Enum):
    CutIn = "cut_in"
    InterstateBroadEnough = "interstate_broad_enough"
    InCongestion = "in_congestion"
    InSlowMovingTraffic = "in_slow_moving_traffic"
    InQueueOfVehicles = "in_queue_of_vehicles"
    MakesUTurn = "makes_u_turn"
    SlInFront = "sl_in_front"
    RightTurn = "on_right_turn"
    TlRed = "tl_red"
    InIntersection = "on_intersection"
    HazardLightsOn = "hazard_lights_on"
    TurnSignalOn = "turn_signal_on"
    DoorOpen = "door_open"
    Kneeling = "kneeling"
    RampExtended = "ramp_extended"
    StandingPassenger = "standing_passenger"
    StopRequired = "stop_required"
    # Arrival = "arrival"
    # Departure = "departure"
    # HeadingToTheNextBusStop = "heading_to_the_next_bus_stop"


class PredHazardLightsOn(BasePredicateEvaluator):
    predicate_name = GeneralPredicates.HazardLightsOn
    arity = 1

    def __init__(self, config):
        super().__init__(config)

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        if not isinstance(vehicle, DynamicObstaclePTVehicle) or isinstance(
            vehicle, PTVehicle
        ):
            return -1
        if not (vehicle.bus_signal and vehicle.bus_signal[time_step]):
            return -1
        if vehicle.bus_signal[time_step]["hazard_lights_on"]:
            return 1
        return -1


class PredTurnSignalOn(BasePredicateEvaluator):
    predicate_name = GeneralPredicates.TurnSignalOn
    arity = 1

    def __init__(self, config):
        super().__init__(config)

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        if not isinstance(vehicle, DynamicObstaclePTVehicle) or isinstance(
            vehicle, PTVehicle
        ):
            return -1
        if not (vehicle.bus_signal and vehicle.bus_signal[time_step]):
            return -1
        if vehicle.bus_signal[time_step]["turn_signal_on"]:
            return 1
        return -1


class PredDoorOpen(BasePredicateEvaluator):
    predicate_name = GeneralPredicates.DoorOpen
    arity = 1

    def __init__(self, config):
        super().__init__(config)

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        if not isinstance(vehicle, DynamicObstaclePTVehicle) or isinstance(
            vehicle, PTVehicle
        ):
            return -1
        if not (vehicle.bus_signal and vehicle.bus_signal[time_step]):
            return -1
        if vehicle.bus_signal[time_step]["door_open"]:
            return 1.0
        return -1.0


class PredKneeling(BasePredicateEvaluator):
    predicate_name = GeneralPredicates.Kneeling
    arity = 1

    def __init__(self, config):
        super().__init__(config)

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        if not isinstance(vehicle, DynamicObstaclePTVehicle) or isinstance(
            vehicle, PTVehicle
        ):
            return -math.inf
        if vehicle.bus_signal[time_step]["kneeling"]:
            return 1
        return -1


class PredRampExtended(BasePredicateEvaluator):
    predicate_name = GeneralPredicates.RampExtended
    arity = 1

    def __init__(self, config):
        super().__init__(config)

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        if not isinstance(vehicle, DynamicObstaclePTVehicle) or isinstance(
            vehicle, PTVehicle
        ):
            return -1
        if not (vehicle.bus_signal and vehicle.bus_signal[time_step]):
            return -1
        if vehicle.bus_signal[time_step]["ramp_extended"]:
            return 1
        return -1


class PredStandingPassenger(BasePredicateEvaluator):
    predicate_name = GeneralPredicates.StandingPassenger
    arity = 1

    def __init__(self, config):
        super().__init__(config)

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        if not isinstance(vehicle, DynamicObstaclePTVehicle) or isinstance(
            vehicle, PTVehicle
        ):
            return -1
        if not (vehicle.bus_signal and vehicle.bus_signal[time_step]):
            return -1
        if vehicle.bus_signal[time_step]["standing_passenger"]:
            return 1
        return -1


class PredStopRequired(BasePredicateEvaluator):
    predicate_name = GeneralPredicates.StopRequired
    arity = 1

    def __init__(self, config):
        super().__init__(config)

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        if not isinstance(vehicle, DynamicObstaclePTVehicle) or isinstance(
            vehicle, PTVehicle
        ):
            return -1
        if not (vehicle.bus_signal and vehicle.bus_signal[time_step]):
            return -1
        if vehicle.bus_signal[time_step]["stop_required"]:
            return 1
        return -1


# class PredBusPlannerState(BasePredicateEvaluator):
#     def __init__(self, config):
#         super().__init__(config)
#
#     def find_bus_stop_signs(self, world: World) -> List[TrafficSign]:
#         traffic_signs = world.road_network.lanelet_network.traffic_signs
#         bus_stop_signs = []
#         for sign in traffic_signs:
#             for sign_element in sign.traffic_sign_elements:
#                 if sign_element.traffic_sign_element_id == TrafficSignIDGermany.BUS_STOP:
#                     bus_stop_signs.append(sign)
#         return bus_stop_signs
#
#     def evaluate_robustness(
#             self, world: World, time_step, vehicle_ids: List[int]
#     ) -> float:
#         raise NotImplementedError
#
# class PredArrival(PredBusPlannerState):
#     predicate_name = GeneralPredicates.Arrival
#     arity = 1
#
#     def __init__(self, config):
#         super().__init__(config)
#
#     def evaluate_robustness(
#             self, world: World, time_step, vehicle_ids: List[int]
#     ) -> float:
#         vehicle = world.vehicle_by_id(vehicle_ids[0])
#         bus_stop_signs = self.find_bus_stop_signs(world)
#         robs = []
#         for sign in bus_stop_signs:
#             sign_x = sign.position[0]
#             x = vehicle.states_cr[time_step].position[0]
#             if sign_x - x < 30:
#                 robs.append(sign_x - x)
#             else:
#                 robs.append(-math.inf)
#         return self._scale_lon_dist(max(robs))
#
# class PredDeparture(PredBusPlannerState):
#     predicate_name = GeneralPredicates.Departure
#     arity = 1
#
#     def __init__(self, config):
#         super().__init__(config)
#
#     def evaluate_robustness(
#         self, world: World, time_step, vehicle_ids: List[int]
#     ) -> float:
#         vehicle = world.vehicle_by_id(vehicle_ids[0])
#         bus_stop_signs = self.find_bus_stop_signs(world)
#         robs = []
#         for sign in bus_stop_signs:
#             sign_x = sign.position[0]
#             x = vehicle.states_cr[time_step].position[0]
#             if x - sign_x < 30:
#                 robs.append(x-sign_x)
#             else:
#                 robs.append(-math.inf)
#         return self._scale_lon_dist(max(robs))
#
# class PredHeadingToTheNextBusStop(BasePredicateEvaluator):
#     predicate_name = GeneralPredicates.HeadingToTheNextBusStop
#     arity = 1
#
#     def __init__(self, config):
#         self._pred_departure = PredDeparture(config)
#         self._pred_arrival = PredArrival(config)
#         super().__init__(config)
#
#     def evaluate_robustness(
#         self, world: World, time_step, vehicle_ids: List[int]
#     ) -> float:
#         rob_departure = self._pred_departure.evaluate_robustness(world, time_step, vehicle_ids)
#         rob_arrival = self._pred_arrival.evaluate_robustness(world, time_step, vehicle_ids)
#         if rob_departure >= 0 or rob_arrival >= 0:
#             return -1.0
#         else:
#             return 1.0


class PredCutIn(BasePredicateEvaluator):
    predicate_name = GeneralPredicates.CutIn
    arity = 2

    def __init__(self, config):
        super().__init__(config)
        self._same_lane_evaluator = PredInSameLane(config)
        self._single_lane_evaluator = PredSingleLane(config)

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        cutting_vehicle = world.vehicle_by_id(vehicle_ids[0])
        cutted_vehicle = world.vehicle_by_id(vehicle_ids[1])

        single_lane = self._single_lane_evaluator.evaluate_boolean(
            world, time_step, [vehicle_ids[0]]
        )
        if single_lane:
            return False
        same_lane = self._same_lane_evaluator.evaluate_boolean(
            world, time_step, vehicle_ids
        )
        if not same_lane:
            return False
        cutting_lane = cutting_vehicle.get_lane(time_step)
        cutted_lat = cutted_vehicle.get_lat_state(time_step, cutting_lane)
        cutting_lat = cutting_vehicle.get_lat_state(time_step)
        d_p = cutted_lat.d
        d_k = cutting_lat.d
        orient_k = cutting_lat.theta

        result = (d_k < d_p and orient_k > self.eps) or (
            d_k > d_p and orient_k < -self.eps
        )
        return result

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        cutting_vehicle = world.vehicle_by_id(vehicle_ids[0])
        cutted_vehicle = world.vehicle_by_id(vehicle_ids[1])

        single_lane = self._single_lane_evaluator.evaluate_robustness_with_cache(
            world,
            time_step,
            [
                vehicle_ids[0],
            ],
        )
        same_lane = self._same_lane_evaluator.evaluate_robustness_with_cache(
            world, time_step, vehicle_ids
        )

        cutting_lane = cutting_vehicle.get_lane(time_step)
        cutted_lat = cutted_vehicle.get_lat_state(time_step, cutting_lane)
        cutting_lat = cutting_vehicle.get_lat_state(time_step)
        r_l_dist = cutted_lat.d - cutting_lat.d
        r_l_orient = subtract_orientations(cutting_lat.theta, self.eps)
        l_r_dist = cutting_lat.d - cutted_lat.d
        l_r_orient = subtract_orientations(-self.eps, cutting_lat.theta)

        r_l_dist = self._scale_lat_dist(r_l_dist)
        l_r_dist = self._scale_lat_dist(l_r_dist)
        r_l_orient = self._scale_angle(r_l_orient)
        l_r_orient = self._scale_angle(l_r_orient)

        rob = min(
            -single_lane,
            same_lane,
            max(min(r_l_dist, r_l_orient), min(l_r_dist, l_r_orient)),
        )
        return rob

    @staticmethod
    def _get_color_map():
        return plt.get_cmap("bwr")

    def visualize(
        self,
        vehicle_ids: List[int],
        add_vehicle_draw_params: Callable[[int, any], None],
        world: World,
        time_step: int,
        predicate_names2vehicle_ids2values: Dict[str, Dict[Tuple[int, ...], float]],
    ):
        self._gather_predicate_values_to_plot(
            vehicle_ids, world, time_step, predicate_names2vehicle_ids2values
        )

        latest_value = self.evaluate_robustness_with_cache(
            world, time_step, vehicle_ids
        )
        latest_value_normalized = (latest_value + 1) / 2
        violation_color = self._get_color_map()(latest_value_normalized)
        violation_color_hex = matplotlib.colors.rgb2hex(violation_color)

        vehicle = vehicle_ids[0]
        draw_params = {
            "dynamic_obstacle": {
                "vehicle_shape": {
                    "occupancy": {
                        "shape": {"rectangle": {"facecolor": violation_color_hex}}
                    }
                }
            }
        }
        add_vehicle_draw_params(vehicle, draw_params)

        draw_functions1 = self._same_lane_evaluator.visualize(
            vehicle_ids,
            add_vehicle_draw_params,
            world,
            time_step,
            predicate_names2vehicle_ids2values,
        )
        draw_functions2 = self._single_lane_evaluator.visualize(
            [vehicle],
            add_vehicle_draw_params,
            world,
            time_step,
            predicate_names2vehicle_ids2values,
        )

        return () + draw_functions1 + draw_functions2

    @staticmethod
    def plot_predicate_visualization_legend(ax):
        points = np.linspace(0, 1, 256)
        points = np.vstack((points, points))
        ax.imshow(points, cmap=PredCutIn._get_color_map(), extent=[-1, 1, 0, 1])
        ax.get_yaxis().set_ticks([])
        ax.set_ylabel("vehicle color")


class PredInterstateBroadEnough(BasePredicateEvaluator):
    """
    Evaluates if an interstate is broad enough to build a standard emergency lane.
    """

    predicate_name = GeneralPredicates.InterstateBroadEnough
    arity = 1

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        lanelet_ids_occ = vehicle.lanelet_assignment[time_step]
        s = vehicle.get_lon_state(time_step).s
        for l_id in lanelet_ids_occ:
            lanelet = world.road_network.lanelet_network.find_lanelet_by_id(l_id)
            if (
                cal_road_width(lanelet, world.road_network, s)
                <= self.config["min_interstate_width"]
            ):
                return False
        return True

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        lanelet_ids_occ = vehicle.lanelet_assignment[time_step]
        s = vehicle.get_lon_state(time_step).s
        comparison_list = []
        for l_id in lanelet_ids_occ:
            lanelet = world.road_network.lanelet_network.find_lanelet_by_id(l_id)
            comparison_list.append(
                self._scale_lat_dist(
                    cal_road_width(lanelet, world.road_network, s)
                    - self.config["min_interstate_width"]
                    - 1.0e-17
                )
            )
        return min(comparison_list)


class PredInCongestion(BasePredicateEvaluator):
    """
    Evaluates if a vehicle is in a congestion.
    """

    predicate_name = GeneralPredicates.InCongestion
    arity = 1

    def __init__(self, config):
        super().__init__(config)
        self._in_front_of_evaluator = PredInFrontOf(config)
        self._same_lane_evaluator = PredInSameLane(config)

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        other_vehicles = [
            world.vehicle_by_id(v_id)
            for v_id in world.vehicle_ids_for_time_step(time_step)
            if v_id != vehicle.id
        ]
        num_vehicles = 0
        for veh_o in other_vehicles:
            if veh_o.get_lon_state(time_step) is None:
                continue
            if (
                self._in_front_of_evaluator.evaluate_boolean(
                    world, time_step, [vehicle_ids[0], veh_o.id]
                )
                and self._same_lane_evaluator.evaluate_boolean(
                    world, time_step, [vehicle_ids[0], veh_o.id]
                )
                and veh_o.get_lon_state(time_step).v
                <= self.config["max_congestion_velocity"]
            ):
                num_vehicles += 1
        if num_vehicles >= self.config["num_veh_congestion"]:
            return True
        else:
            return False

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        other_vehicles = [
            world.vehicle_by_id(v_id)
            for v_id in world.vehicle_ids_for_time_step(time_step)
            if v_id != vehicle.id
        ]

        rob_cong_veh_list = [self._scale_speed(-np.inf)]
        for veh_o in other_vehicles:
            if veh_o.get_lon_state(time_step) is None:
                rob_cong_veh_list.append(self._scale_speed(-np.inf))
            rob_cong_veh_list.append(
                min(
                    self._in_front_of_evaluator.evaluate_robustness(
                        world, time_step, [vehicle_ids[0], veh_o.id]
                    ),
                    self._same_lane_evaluator.evaluate_robustness(
                        world, time_step, [vehicle_ids[0], veh_o.id]
                    ),
                    self._scale_speed(
                        self.config["max_congestion_velocity"]
                        - veh_o.get_lon_state(time_step).v
                        - 1.0e-17
                    ),
                )
            )
        # values are already normalized
        if (
            sum(rob > 0 for rob in rob_cong_veh_list)
            >= self.config["num_veh_congestion"]
        ):
            return min(rob for rob in rob_cong_veh_list if rob > 0)
        else:
            return max(rob for rob in rob_cong_veh_list if rob < 0)


class PredInSlowMovingTraffic(BasePredicateEvaluator):
    """
    Evaluates if a vehicle is part of slow moving traffic.
    """

    predicate_name = GeneralPredicates.InSlowMovingTraffic
    arity = 1

    def __init__(self, config):
        super().__init__(config)
        self._in_front_of_evaluator = PredInFrontOf(config)
        self._same_lane_evaluator = PredInSameLane(config)

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        other_vehicles = [
            world.vehicle_by_id(v_id)
            for v_id in world.vehicle_ids_for_time_step(time_step)
            if v_id != vehicle.id
        ]
        num_vehicles = 0
        for veh_o in other_vehicles:
            if veh_o.get_lon_state(time_step) is None:
                continue
            if (
                self._in_front_of_evaluator.evaluate_boolean(
                    world, time_step, [vehicle_ids[0], veh_o.id]
                )
                and self._same_lane_evaluator.evaluate_boolean(
                    world, time_step, [vehicle_ids[0], veh_o.id]
                )
                and veh_o.get_lon_state(time_step).v
                <= self.config["max_slow_moving_traffic_velocity"]
            ):
                num_vehicles += 1
        if num_vehicles >= self.config["num_veh_slow_moving_traffic"]:
            return True
        else:
            return False

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        other_vehicles = [
            world.vehicle_by_id(v_id)
            for v_id in world.vehicle_ids_for_time_step(time_step)
            if v_id != vehicle.id
        ]

        rob_cong_veh_list = [self._scale_speed(-np.inf)]
        for veh_o in other_vehicles:
            if veh_o.get_lon_state(time_step) is None:
                rob_cong_veh_list.append(self._scale_speed(-np.inf))
            rob_cong_veh_list.append(
                min(
                    self._in_front_of_evaluator.evaluate_robustness(
                        world, time_step, [vehicle_ids[0], veh_o.id]
                    ),
                    self._same_lane_evaluator.evaluate_robustness(
                        world, time_step, [vehicle_ids[0], veh_o.id]
                    ),
                    self._scale_speed(
                        self.config["max_slow_moving_traffic_velocity"]
                        - veh_o.get_lon_state(time_step).v
                        - 1.0e-17
                    ),
                )
            )
        # values are already normalized
        if (
            sum(rob > 0 for rob in rob_cong_veh_list)
            >= self.config["num_veh_slow_moving_traffic"]
        ):
            return min(rob for rob in rob_cong_veh_list if rob > 0)
        else:
            return max(rob for rob in rob_cong_veh_list if rob < 0)


class PredInQueueOfVehicles(BasePredicateEvaluator):
    """
    Evaluates if a vehicle is part of a queue of vehicles
    """

    predicate_name = GeneralPredicates.InQueueOfVehicles
    arity = 1

    def __init__(self, config):
        super().__init__(config)
        self._in_front_of_evaluator = PredInFrontOf(config)
        self._same_lane_evaluator = PredInSameLane(config)

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        other_vehicles = [
            world.vehicle_by_id(v_id)
            for v_id in world.vehicle_ids_for_time_step(time_step)
            if v_id != vehicle.id
        ]
        num_vehicles = 0
        for veh_o in other_vehicles:
            if veh_o.get_lon_state(time_step) is None:
                continue
            if (
                self._in_front_of_evaluator.evaluate_boolean(
                    world, time_step, [vehicle_ids[0], veh_o.id]
                )
                and self._same_lane_evaluator.evaluate_boolean(
                    world, time_step, [vehicle_ids[0], veh_o.id]
                )
                and veh_o.get_lon_state(time_step).v
                <= self.config["max_queue_of_vehicles_velocity"]
            ):
                num_vehicles += 1
        if num_vehicles >= self.config["num_veh_queue_of_vehicles"]:
            return True
        else:
            return False

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        other_vehicles = [
            world.vehicle_by_id(v_id)
            for v_id in world.vehicle_ids_for_time_step(time_step)
            if v_id != vehicle.id
        ]

        rob_cong_veh_list = [self._scale_speed(-np.inf)]
        for veh_o in other_vehicles:
            if veh_o.get_lon_state(time_step) is None:
                rob_cong_veh_list.append(self._scale_speed(-np.inf))
            rob_cong_veh_list.append(
                min(
                    self._in_front_of_evaluator.evaluate_robustness(
                        world, time_step, [vehicle_ids[0], veh_o.id]
                    ),
                    self._same_lane_evaluator.evaluate_robustness(
                        world, time_step, [vehicle_ids[0], veh_o.id]
                    ),
                    self._scale_speed(
                        self.config["max_queue_of_vehicles_velocity"]
                        - veh_o.get_lon_state(time_step).v
                        - 1.0e-17
                    ),
                )
            )
        # values are already normalized
        if (
            sum(rob > 0 for rob in rob_cong_veh_list)
            >= self.config["num_veh_queue_of_vehicles"]
        ):
            return min(rob for rob in rob_cong_veh_list if rob > 0)
        else:
            return max(rob for rob in rob_cong_veh_list if rob < 0)


class PredMakesUTurn(BasePredicateEvaluator):
    """
    Predicate which evaluates if vehicle makes U-turn
    """

    predicate_name = GeneralPredicates.MakesUTurn
    arity = 1

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        lanes = world.road_network.find_lanes_by_lanelets(
            vehicle.lanelet_assignment[time_step]
        )
        for la in lanes:
            if self.config["u_turn"] <= abs(
                vehicle.get_lat_state(time_step, la).theta
                - la.orientation(vehicle.get_lon_state(time_step, la).s)
            ):
                return True
        return False

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        robustness_values = []
        vehicle = world.vehicle_by_id(vehicle_ids[0])
        lanes = world.road_network.find_lanes_by_lanelets(
            vehicle.lanelet_assignment[time_step]
        )
        for la in lanes:
            robustness_values.append(
                self._scale_angle(
                    abs(
                        vehicle.get_lat_state(time_step, la).theta
                        - la.orientation(vehicle.get_lon_state(time_step, la).s)
                    )
                    - self.config["u_turn"]
                    - 1.0e-17
                )
            )
        return max(robustness_values)


class PredStopLineInFront(BasePredicateEvaluator):
    """Evaluate if a stop line is in front of the vehicle, with respect to a drivable
    path."""

    predicate_name = GeneralPredicates.SlInFront
    arity = 1

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        return self.evaluate_robustness(world, time_step, vehicle_ids) >= 0.0

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        ego = world.vehicle_by_id(vehicle_ids[0])
        lanes = ego.ref_path_lanes(time_step)
        # Find all lanelets in the map that have a stop line
        lanelets_with_stop_line = [
            l
            for l in world.road_network.lanelet_network.lanelets
            if l.stop_line is not None
            and l.stop_line.line_marking is LineMarking.BROAD_SOLID
        ]
        robustness = -np.inf
        # For all possible paths of the vehicle
        for lane in lanes:
            # Get the set of lanelets in the current path, that have a stop line
            relevant_lanelet_ids_with_stop_line = [
                l
                for l in lanelets_with_stop_line
                if l.lanelet_id in lane.contained_lanelets
            ]
            # If there is none, continue
            if len(relevant_lanelet_ids_with_stop_line) == 0:
                continue

            ccs = get_curvilinear_coordinate_system(lane.lanelet.center_vertices)
            curvi_occ = ccs(ego.occupancy_at_time_step(time_step).vertices)
            stop_line_pts = np.array(
                [
                    [l.stop_line.start, l.stop_line.end]
                    for l in relevant_lanelet_ids_with_stop_line
                ]
            )
            curvi_stop_line = ccs(stop_line_pts.reshape((-1, 2))).reshape((-1, 2, 2))
            # Order stop line points from right to left
            curvi_stop_line = np.where(
                curvi_stop_line[:, 0, 1] < curvi_stop_line[:, 0, 1],
                curvi_stop_line,
                curvi_stop_line[:, ::-1],
            )

            occ_stop_line_ccs = cartesian_to_curvilinear(
                curvi_stop_line, curvi_occ, limit_start_end=False
            )
            stop_line_distance = np.nanmin(
                occ_stop_line_ccs[..., 1], axis=-1, initial=np.inf
            )
            stop_line_robustness = np.fmin(
                self.config["d_sl"] - np.abs(stop_line_distance), stop_line_distance
            )
            robustness = max(robustness, stop_line_robustness)
        return self._scale_lon_dist(float(robustness))


class PredInIntersection(BasePredicateEvaluator):
    """Evaluate if a vehicle occupancy is intersecting with an intersection lanelet."""

    predicate_name = GeneralPredicates.InIntersection
    arity = 1

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        ego = world.vehicle_by_id(vehicle_ids[0])
        lanelets = ego.lanelet_assignment[time_step]
        intersection_lanelets = [
            lanelet
            for lanelet in lanelets
            if LaneletType.INTERSECTION
            in world.road_network.lanelet_network.find_lanelet_by_id(
                lanelet
            ).lanelet_type
        ]
        return len(intersection_lanelets) > 0

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        ego = world.vehicle_by_id(vehicle_ids[0])
        lanes = ego.lanes_at_state(time_step)
        if not self.evaluate_boolean(world, time_step, vehicle_ids):
            all_intersection_lanelets = [
                l.lanelet_id
                for l in world.road_network.lanelet_network.lanelets
                if LaneletType.INTERSECTION in l.lanelet_type
            ]
            dist = np.inf
            for lane in lanes:
                intersection_lanelets = lane.contained_lanelets.intersection(
                    all_intersection_lanelets
                )
                if len(intersection_lanelets) == 0:
                    continue
                front_s = ego.front_s(time_step, lane) or -np.inf
                rear_s = ego.rear_s(time_step, lane) or np.inf
                start_s = np.array(
                    [
                        lane.clcs.convert_to_curvilinear_coords(
                            *world.road_network.lanelet_network.find_lanelet_by_id(
                                l
                            ).center_vertices[0]
                        )[0]
                        for l in intersection_lanelets
                    ]
                )
                end_s = np.array(
                    [
                        lane.clcs.convert_to_curvilinear_coords(
                            *world.road_network.lanelet_network.find_lanelet_by_id(
                                l
                            ).center_vertices[-1]
                        )[0]
                        for l in intersection_lanelets
                    ]
                )
                dist_succ = start_s - front_s
                dist_succ = np.min(dist_succ[dist_succ > 0], initial=np.inf)
                dist_pred = rear_s - end_s
                dist_pred = np.min(dist_pred[dist_pred > 0], initial=np.inf)
                dist = min(dist, dist_succ, dist_pred)
            return -self._scale_lon_dist(dist)
        else:
            all_intersection_lanelets = [
                l.lanelet_id
                for l in world.road_network.lanelet_network.lanelets
                if LaneletType.INTERSECTION not in l.lanelet_type
            ]
            dist = np.inf
            for lane in lanes:
                intersection_lanelets = lane.contained_lanelets.intersection(
                    all_intersection_lanelets
                )
                if len(intersection_lanelets) == 0:
                    continue
                front_s = ego.front_s(time_step, lane) or np.inf
                rear_s = ego.rear_s(time_step, lane) or -np.inf
                start_s = np.array(
                    [
                        lane.clcs.convert_to_curvilinear_coords(
                            *world.road_network.lanelet_network.find_lanelet_by_id(
                                l
                            ).center_vertices[0]
                        )[0]
                        for l in intersection_lanelets
                    ]
                )
                end_s = np.array(
                    [
                        lane.clcs.convert_to_curvilinear_coords(
                            *world.road_network.lanelet_network.find_lanelet_by_id(
                                l
                            ).center_vertices[-1]
                        )[0]
                        for l in intersection_lanelets
                    ]
                )
                dist_succ = start_s - rear_s
                dist_succ = np.min(dist_succ[dist_succ > 0], initial=np.inf)
                dist_pred = front_s - end_s
                dist_pred = np.min(dist_pred[dist_pred > 0], initial=np.inf)
                dist = min(dist, dist_succ, dist_pred)
            return self._scale_lon_dist(dist)


class PredTrafficLightRed(BasePredicateEvaluator):
    """Evaluate if an upcoming traffic light is red."""

    predicate_name = GeneralPredicates.TlRed
    arity = 1

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        tl_states = self._get_tl_states(time_step, vehicle_ids, world)
        return TrafficLightState.RED in tl_states

    @staticmethod
    def _get_tl_states(time_step, vehicle_ids, world):
        ego = world.vehicle_by_id(vehicle_ids[0])
        lanelet_ids = ego.lanelet_assignment[time_step]
        lanelets = [
            world.scenario.lanelet_network.find_lanelet_by_id(i) for i in lanelet_ids
        ]
        tl_states = []
        for lanelet in lanelets:
            if len(lanelet.traffic_lights) == 0:
                continue
            # TODO: Only works for one traffic light per lanelet!
            assert (
                len(lanelet.traffic_lights) == 1
            ), "TODO: Only works for one traffic light per lanelet!"
            tl = world.scenario.lanelet_network.find_traffic_light_by_id(
                list(lanelet.traffic_lights)[0]
            )
            tl_states.append(tl.get_state_at_time_step(time_step))
        return tl_states

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        tl_states = self._get_tl_states(time_step, vehicle_ids, world)
        if TrafficLightState.RED in tl_states:
            robustness = 1.0
        elif (
            TrafficLightState.YELLOW in tl_states
            or TrafficLightState.RED_YELLOW in tl_states
        ):
            robustness = -0.5
        else:
            robustness = -1.0
        return robustness


class PredOnRightTurn(BasePredicateEvaluator):
    """Evaluate if a vehicle is within a right turning lanelet."""

    predicate_name = GeneralPredicates.RightTurn
    arity = 1

    def evaluate_boolean(self, world: World, time_step, vehicle_ids: List[int]) -> bool:
        return self.evaluate_robustness(world, time_step, vehicle_ids) >= 0.0

    def _get_incoming(self, ego_id, world) -> Optional[IntersectionIncomingElement]:
        ego = world.vehicle_by_id(ego_id)
        # Try to find out if we previously have been on any incoming lanelets to
        # determine which are the corresponding
        # right-turning elements
        inc = {
            frozenset(i.incoming_lanelets): i
            for i in world.scenario.lanelet_network.intersections[0].incomings
        }
        tsteps = sorted(ego.lanelet_assignment.keys())
        incoming = None
        for t in tsteps:
            ls = ego.lanelet_assignment[t]
            for i in inc:
                if len(i.intersection(ls)) > 0:
                    incoming = inc[i]
                    break
            if incoming is not None:
                break
        return incoming

    def _get_right_turning_lanes(
        self, world: World, incoming: IntersectionIncomingElement
    ):
        incoming_lanelet_ids = incoming.incoming_lanelets
        right_turning_lanelet_ids = incoming.successors_right
        right_turning_lanes = []
        right_turning_lanelets = []
        for l in world.road_network.lanes:
            right_turning_lanlet = l.contained_lanelets.intersection(
                right_turning_lanelet_ids
            )
            if (
                len(l.contained_lanelets.intersection(incoming_lanelet_ids)) > 0
                and len(right_turning_lanlet) > 0
            ):
                right_turning_lanes.append(l)
                assert len(right_turning_lanlet) == 1
                right_turning_lanelets.append(right_turning_lanlet.pop())
        return right_turning_lanes, right_turning_lanelets, incoming

    def _robustness_on_lane(
        self,
        world: World,
        vehicle: Vehicle,
        time_step: int,
        lane: Lane,
        right_turning_lanelet,
        incoming,
    ):
        lanelet = world.road_network.lanelet_network.find_lanelet_by_id(
            right_turning_lanelet
        )
        try:
            lon_state, _ = lane.clcs.convert_to_curvilinear_coords(
                *vehicle.states_cr[time_step].position,
            )
        except ValueError:
            return self._scale_lon_dist(-np.inf)

        front_s = lon_state + 0.5 * vehicle.shape.length
        rear_s = lon_state - 0.5 * vehicle.shape.length
        start_s, _ = lane.clcs.convert_to_curvilinear_coords(
            *lanelet.center_vertices[0]
        )
        end_s, _ = lane.clcs.convert_to_curvilinear_coords(*lanelet.center_vertices[-1])
        if start_s <= front_s and rear_s <= end_s:
            d_left = distance_to_left_bounds(
                vehicle, incoming.successors_right, world, time_step
            )
            rob = -np.max(d_left, initial=-np.inf)
            rob = self._scale_lat_dist(rob)
        elif front_s < start_s:
            rob = self._scale_lon_dist(front_s - start_s)
        elif end_s < rear_s:
            rob = self._scale_lon_dist(end_s - rear_s)
        else:
            assert False
        return rob

    def evaluate_robustness(
        self, world: World, time_step, vehicle_ids: List[int]
    ) -> float:
        incoming = self._get_incoming(vehicle_ids[0], world)
        if incoming is None:
            return self._scale_lon_dist(-np.inf)
        ego = world.vehicle_by_id(vehicle_ids[0])
        (
            right_turning_lanes,
            right_turning_lanelets,
            incoming,
        ) = self._get_right_turning_lanes(world, incoming)
        rob = []
        for lane, lanelet in zip(right_turning_lanes, right_turning_lanelets):
            rob.append(
                self._robustness_on_lane(world, ego, time_step, lane, lanelet, incoming)
            )
        return min(rob, default=self._scale_lon_dist(-np.inf))
