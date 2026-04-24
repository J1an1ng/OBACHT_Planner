import copy
import numpy as np
import yaml
import configurations
import importlib.resources as pkg_resources
from abc import ABC, abstractmethod
from typing import Dict, Tuple, Optional
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.trajectory import State
from source.commonroad_rp.state import append_state_to_list, append_states_to_list_ver
from source.commonroad_rp.utility.config import ReactivePlannerConfiguration
from source.commonroad_rp.utility.utils_coordinate_system import CoordinateSystem, create_coordinate_system,create_initial_ref_path
from post_optimization_planner.post_optimization import PostOptimizerPlanner as ReactivePlanner
from source.commonroad_rp.utility.logger import initialize_logger
import logging



from copy import deepcopy
class BaseStateMachinePlanner(ABC):
    """
    Abstract base class for state machine planners.
    Defines common interface and shared functionality for all planning scenarios.
    """

    def __init__(
            self,
            scenario,
            planning_problem: PlanningProblem,
            scenario_type: str,
            use_post_opt: bool = False,
            initial_state: str = "HEADING",
    ):
        # Core parameters
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.scenario_type = scenario_type
        self.use_post_opt = use_post_opt
        self.current_state = initial_state
        self.stopping_counter = 0
        self.stopping_next_state = None

        # Configuration setup
        self.config_root = pkg_resources.files(configurations) / scenario_type

        # Initialize scenario-specific components
        self._initialize_coordinate_systems()
        self._initialize_goal_positions()
        self._initialize_state_handlers()

        # Validate initialization
        self._validate_initialization()
        self.cr_State = None
    @abstractmethod
    def _initialize_coordinate_systems(self):
        """Initialize coordinate systems specific to the scenario type"""
        pass

    @abstractmethod
    def _initialize_goal_positions(self):
        """Initialize goal positions specific to the scenario type"""
        pass

    @abstractmethod
    def _get_valid_states(self) -> set:
        """Return set of valid states for this scenario type"""
        pass

    def _initialize_state_handlers(self):
        """Initialize state handler mapping (common for all scenarios)"""
        self.handlers = {
            "DEPARTING": self._execute_departing,
            "HEADING": self._execute_heading,
            "ARRIVING": self._execute_arriving,
            "BEFORE_STOPPING": self._execute_before_stopping,
            "STOPPING": self._execute_stopping,
        }

    def _validate_initialization(self):
        """Validate that initialization was successful"""
        if not hasattr(self, 'coord_systems'):
            raise RuntimeError("Coordinate systems not initialized")
        if not hasattr(self, 'goal_x'):
            raise RuntimeError("Goal positions not initialized")

        # Check if current state is valid for this scenario
        valid_states = self._get_valid_states()
        if self.current_state not in valid_states:
            raise ValueError(f"Invalid initial state '{self.current_state}' for {self.scenario_type}")

    def step(self, state_current: State, state_list: list) -> State:
        """Execute one step of state machine planning"""
        # Handle special logic for STOPPING state
        self.cr_State = state_current
        if self._execute_stopping_counter(state_list):
            return self.stopping_next_state

        # Call corresponding state handler
        if self.current_state not in self.handlers:
            raise ValueError(f"Unknown state: {self.current_state}")

        # Check if state is valid for current scenario
        if self.current_state not in self._get_valid_states():
            raise ValueError(f"State '{self.current_state}' not valid for {self.scenario_type}")

        return self.handlers[self.current_state](state_current, state_list)

    def _execute_stopping_counter(self, state_list: list) -> bool:
        """Handle counter logic for STOPPING state (common implementation)"""
        if self.current_state == "STOPPING" and self.stopping_counter > 0:
            self.stopping_counter += 1
            if self.stopping_counter <= 15:
                append_state_to_list(state_list, self.stopping_next_state)
                return True
            else:
                # Reset stopping state and transition to departing
                self.stopping_counter = 0
                self.stopping_next_state = None
                self.current_state = "DEPARTING"
        return False

    def _create_planner(self, yaml_name: str, coord_sys: CoordinateSystem, state_current: State) -> Tuple[
        ReactivePlanner, object]:
        """Create and configure planner (common implementation)"""
        # Load configuration
        config_path = str(self.config_root / yaml_name)
        config = ReactivePlannerConfiguration.load(config_path)
        config.update(self.scenario, planning_problem=self.planning_problem)

        # Initialize planner
        planner = ReactivePlanner(config)
        planner.x_0 = copy.deepcopy(state_current)
        planner.record_state_and_input(planner.x_0)
        # Ensure necessary state attributes exist
        for attr in ("steering_angle", "yaw_rate", "slip_angle"):
            if not hasattr(planner.x_0, attr):
                setattr(planner.x_0, attr, 0)

        # Set reference path and reset planner
        planner.set_reference_path(coordinate_system=coord_sys)
        planner.reset(
            config,
            collision_checker=planner.collision_checker,
            coordinate_system=planner.coordinate_system,
        )

        return planner, config

    def _plan_and_optimize(self, planner: ReactivePlanner, config, state_list: list, is_stopping: bool = False) -> \
    Tuple[State, object]:
        """Execute planning with optional post-processing optimization (common implementation)"""
        # Execute planni

        trajectory = planner.plan()

        # Post-processing optimization
        if self.use_post_opt and hasattr(planner, "best_sample"):
            if is_stopping and hasattr(planner, "post_optimize_for_stopping"):
                optimized = planner.post_optimize_for_stopping(planner.best_sample)
            else:
                optimized = planner.post_optimize(planner.best_sample)

            if optimized is not None:
                trajectory = optimized


        # Extract next state and update state list
        next_state = trajectory[0].state_list[config.sumo.sumo_planning_steps]
        append_states_to_list_ver(
            state_list,
            trajectory[0].state_list,
            config.sumo.sumo_planning_steps
        )

        return next_state, trajectory

    # Default implementations for states (can be overridden)
    def _execute_heading(self, state_current: State, state_list: list) -> State:
        """Execute HEADING state (default implementation)"""
        planner, config = self._create_planner(
            "heading_to_next.yaml",
            self.coord_systems["HEADING"],
            state_current
        )

        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)

        # Check if transition to ARRIVING state is needed
        distance_threshold = config.planning.distance_heading_to_next_to_arriving
        if abs(self.goal_x - next_state.position[0]) < distance_threshold:
            self.current_state = "ARRIVING"

        return next_state

    def _execute_arriving(self, state_current: State, state_list: list) -> State:
        """Execute ARRIVING state (default implementation)"""
        planner, config = self._create_planner(
            "arriving.yaml",
            self.coord_systems["ARRIVING"],
            state_current
        )

        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)

        # Determine next state based on scenario type
        self._check_arriving_transition(next_state, config)

        return next_state

    @abstractmethod
    def _execute_departing(self, state_current: State, state_list: list) -> State:
        """Execute DEPARTING state (scenario-specific implementation required)"""
        pass

    @abstractmethod
    def _execute_stopping(self, state_current: State, state_list: list) -> State:
        """Execute STOPPING state (scenario-specific implementation required)"""
        pass

    @abstractmethod
    def _check_arriving_transition(self, next_state: State, config) -> None:
        """Check transition conditions from ARRIVING state (scenario-specific)"""
        pass

    def _execute_before_stopping(self, state_current: State, state_list: list) -> State:
        """Execute BEFORE_STOPPING state (default: not implemented)"""
        raise NotImplementedError(f"BEFORE_STOPPING state not implemented for {self.scenario_type}")

    def reset_state(self, new_state: str = "HEADING"):
        """Reset state machine to specified state"""
        if new_state not in self._get_valid_states():
            raise ValueError(f"Invalid state '{new_state}' for {self.scenario_type}")

        self.current_state = new_state
        self.stopping_counter = 0
        self.stopping_next_state = None

    def get_current_state(self) -> str:
        """Get current state"""
        return self.current_state


class BusStopBulbPlanner(BaseStateMachinePlanner):
    """
    State machine planner for bus stop bulb scenarios.
    State flow: DEPARTING → HEADING → ARRIVING → STOPPING
    """

    def _get_valid_states(self) -> set:
        """Return valid states for bulb scenario"""
        return {"DEPARTING", "HEADING", "ARRIVING", "STOPPING"}

    def _initialize_coordinate_systems(self):
        """Initialize coordinate systems for bulb scenario"""
        lane1_coords = create_coordinate_system(
            self.scenario.lanelet_network.find_lanelet_by_id(1).center_vertices
        )
        lane2_coords = create_coordinate_system(
            self.scenario.lanelet_network.find_lanelet_by_id(2).center_vertices
        )

        self.coord_systems = {
            "DEPARTING": lane2_coords,
            "HEADING": lane1_coords,
            "ARRIVING": lane2_coords,
            "STOPPING": lane2_coords,
        }

    def _initialize_goal_positions(self):
        """Initialize goal positions for bulb scenario"""
        goal_state = self.planning_problem.goal.state_list[0]
        self.goal_x = goal_state.position.center[0]

        # Calculate longitudinal goal position
        left_vertex = self.scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[0][0]
        self.lon_goal = self.goal_x - left_vertex

        # Calculate goal boundary for stopping condition
        self.goal_boundary = self.goal_x + 0.5 * goal_state.position.length

    def _execute_departing(self, state_current: State, state_list: list) -> State:
        """Execute DEPARTING state for bulb scenario"""

        planner, config = self._create_planner(
            "departure.yaml",
            self.coord_systems["DEPARTING"],
            state_current
        )
        planner._low_vel_mode = False
        # Set desired velocity
        planner._desired_speed = config.sampling.desire_velocity
        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        return next_state

    def _execute_stopping(self, state_current: State, state_list: list) -> State:
        """Execute STOPPING state for bulb scenario"""
        # Check if stopping condition is already met
        if self._check_stopping_condition(state_current, state_list):
            return state_current

        planner, config = self._create_planner(
            "stopping.yaml",
            self.coord_systems["STOPPING"],
            state_current
        )

        planner._desired_speed = 0
        planner.set_desired_lon_position(lon_position=self.lon_goal)

        next_state, _ = self._plan_and_optimize(planner, config, state_list, is_stopping=True)
        return next_state

    def _check_arriving_transition(self, next_state: State, config) -> None:
        """Check transition conditions from ARRIVING state for bulb scenario"""
        threshold = config.planning.distance_arriving_to_stopping
        if abs(self.goal_x - next_state.position[0]) < threshold:
            self.current_state = "STOPPING"

    def _check_stopping_condition(self, state_current: State, state_list: list) -> bool:
        """Check stopping condition for bulb scenario"""
        ego_x = state_current.position[0]
        ego_v = state_current.velocity

        condition = ego_x < self.goal_boundary and ego_v < 1e-5

        if condition:
            append_state_to_list(state_list, state_current)
            self.stopping_counter = 1
            self.stopping_next_state = state_current
            return True

        return False


class BusStopBayPlanner(BaseStateMachinePlanner):
    """
    State machine planner for bus stop bay scenarios.
    State flow: DEPARTING → HEADING → ARRIVING → BEFORE_STOPPING → STOPPING
    """

    def _get_valid_states(self) -> set:
        """Return valid states for bay scenario"""
        return {"DEPARTING", "HEADING", "ARRIVING", "BEFORE_STOPPING", "STOPPING"}

    def _initialize_coordinate_systems(self):
        """Initialize coordinate systems for bay scenario"""
        lane1_coords = create_coordinate_system(
            self.scenario.lanelet_network.find_lanelet_by_id(1).center_vertices
        )
        lane2_coords = create_coordinate_system(
            self.scenario.lanelet_network.find_lanelet_by_id(2).center_vertices
        )
        goal_state = self.planning_problem.goal.state_list[0]
        self.goal_y = goal_state.position.center[1]
        # Create middle path coordinate system (y=-7.5)
        left_vertex = self.scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[0][0]
        right_vertex = self.scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[-1][0]
        num = (right_vertex -left_vertex )*10
        x_values = np.linspace(left_vertex, right_vertex, int(num) )
        y_values = np.full_like(x_values, -7.5)
        goal_region_coords = create_coordinate_system(np.column_stack((x_values, y_values)))


        self.coord_systems = {
            "DEPARTING": None,  # May be dynamically adjusted
            "HEADING": lane2_coords,
            "ARRIVING": lane1_coords,
            "BEFORE_STOPPING": goal_region_coords,
            "STOPPING": goal_region_coords,
        }

    def _initialize_goal_positions(self):
        """Initialize goal positions for bay scenario"""
        goal_state = self.planning_problem.goal.state_list[0]
        self.goal_x = goal_state.position.center[0]

        # Calculate longitudinal goal position
        left_vertex = self.scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[0][0]
        self.lon_goal = self.goal_x - left_vertex

        # Calculate goal boundary for bay scenario
        self.goal_left = self.goal_x + 0.5 * goal_state.position.length

    def _execute_departing(self, state_current: State, state_list: list) -> State:
        """Execute DEPARTING state for bay scenario"""
        # Dynamic coordinate system selection based on velocity
        coord_sys = self._get_departing_coord_system(state_current)

        planner, config = self._create_planner("departure.yaml",coord_sys, state_current)
        # initialize_logger(config)
        # logger = logging.getLogger("RP_LOGGER")


        # Set desired velocity
        if planner.x_0.velocity<config.planning.low_vel_mode_threshold:
            planner._low_vel_mode = True
        else :
            config.sampling.d_min =-3.75
            config.sampling.d_max = 0
        planner._desired_speed = config.sampling.desire_velocity

        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        return next_state

    def _execute_before_stopping(self, state_current: State, state_list: list) -> State:
        """Execute BEFORE_STOPPING state for bay scenario"""
        planner, config = self._create_planner(
            "before_stopping.yaml",
            self.coord_systems["BEFORE_STOPPING"],
            state_current
        )

        planner._desired_speed = 0
        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)

        # Check transition to STOPPING state
        threshold = config.planning.distance_before_stopping_to_stopping
        if abs(self.goal_x - next_state.position[0]) < threshold:
            self.current_state = "STOPPING"

        return next_state

    def _execute_stopping(self, state_current: State, state_list: list) -> State:
        """Execute STOPPING state for bay scenario"""
        # Check if stopping condition is already met
        if self._check_stopping_condition(state_current, state_list):
            return state_current

        planner, config = self._create_planner(
            "stopping.yaml",
            self.coord_systems["STOPPING"],
            state_current
        )
        planner._low_vel_mode = True
        planner._desired_speed = 0
        planner.set_desired_lon_position(lon_position=self.lon_goal)  # Fixed position for bay

        next_state, _ = self._plan_and_optimize(planner, config, state_list, is_stopping=True)
        return next_state

    def _check_arriving_transition(self, next_state: State, config) -> None:
        """Check transition conditions from ARRIVING state for bay scenario"""
        threshold = config.planning.distance_arriving_to_next_to_before_stopping
        if abs(self.goal_x - next_state.position[0]) < threshold:
            self.current_state = "BEFORE_STOPPING"

    def _check_stopping_condition(self, state_current: State, state_list: list) -> bool:
        """Check stopping condition for bay scenario"""
        ego_x = state_current.position[0]
        ego_v = state_current.velocity

        condition = ego_x < self.goal_left and ego_v < 1e-5

        if condition:
            append_state_to_list(state_list, state_current)
            self.stopping_counter = 1
            self.stopping_next_state = state_current
            return True

        return False

    def _get_departing_coord_system(self, state_current: State) -> CoordinateSystem:
        """Get appropriate coordinate system for DEPARTING state"""
        if state_current.velocity < 2:

            x_values = np.linspace(-150, 150, 3000)
            y_values = np.full_like(x_values, state_current.position[1])
            reference_path = np.column_stack((x_values, y_values))
            return create_coordinate_system(
                reference_path
            )
        else:
             return create_coordinate_system(
                self.scenario.lanelet_network.find_lanelet_by_id(1).center_vertices
            )


# Factory function for creating appropriate planner
def create_state_machine_planner(
        scenario,
        planning_problem: PlanningProblem,
        scenario_type: str,
        use_post_opt: bool = False,
        initial_state: str = "HEADING"
) -> BaseStateMachinePlanner:
    """
    Factory function to create appropriate state machine planner based on scenario type.

    Args:
        scenario_type: "bus_stop_bulb" or "bus_stop_bay"
        use_post_opt: Whether to enable post-processing optimization
        initial_state: Initial state for the state machine

    Returns:
        Appropriate planner instance
    """
    if scenario_type == "bus_stop_bulb":
        return BusStopBulbPlanner(
            scenario, planning_problem, scenario_type, use_post_opt, initial_state
        )
    elif scenario_type == "bus_stop_bay":
        return BusStopBayPlanner(
            scenario, planning_problem, scenario_type, use_post_opt, initial_state
        )
    else:
        raise ValueError(f"Unsupported scenario type: {scenario_type}")


# Global planner cache
_planners: Dict[str, BaseStateMachinePlanner] = {}


def simulate_state_machine(
        current_scenario,
        state_current_ego,
        planning_problem,
        state_list,
        yaml_file_path:str,
) -> State:
    """
    Unified state machine function using factory pattern.

    Args:
        scenario_type: "bus_stop_bulb" or "bus_stop_bay"
        post_opt: Whether to enable post-processing optimization

    Returns:
        Next state from the planner
    """
    # Use cache to avoid repeatedly creating planners
    try:
        with open(yaml_file_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
            scenario_type = config.get('scenario', {}).get('type')
            post_opt = config.get('debug', {}).get('use_post_opt')
    except FileNotFoundError:
        print(f"file: {yaml_file_path} not found")

    except yaml.YAMLError as e:
        print(f"YAML error: {e}")


    key = f"{scenario_type}_{'opt' if post_opt else 'noopt'}"
    if key not in _planners:
        _planners[key] = create_state_machine_planner(
            current_scenario,
            planning_problem,
            scenario_type,
            use_post_opt=post_opt,
            initial_state="HEADING"
        )

    return _planners[key].step(state_current_ego, state_list)


def clear_planner_cache():
    """Clear planner cache"""
    global _planners
    _planners.clear()
