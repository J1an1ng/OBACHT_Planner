import copy
import logging
import numpy as np
import yaml

logger = logging.getLogger("RP_LOGGER")
import configurations
import importlib.resources as pkg_resources
from abc import ABC, abstractmethod
from typing import Dict, Tuple, Optional, Set
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.trajectory import State
from source.commonroad_rp.state import append_state_to_list, append_states_to_list_ver
from source.commonroad_rp.utility.config import ReactivePlannerConfiguration
from source.commonroad_rp.utility.utils_coordinate_system import CoordinateSystem, create_coordinate_system
from post_optimization_planner.post_optimization import PostOptimizerPlanner as ReactivePlanner
import time

# Import state classes
from post_optimization_planner.State import (
    PlannerState, DepartingState, HeadingState, ArrivingState,
    BeforeStoppingState, StoppingState, create_initial_state,
)

from utility.cost_calculate import TrajectoryCostTracker_1


class VehicleLeftScenarioError(Exception):
    """Raised when the vehicle has driven outside the scenario boundary."""
    pass


class BaseStateMachinePlanner(ABC):
    """
    Abstract base class for state machine planners using state classes.
    Defines common interface and shared functionality for all planning scenarios.
    """

    def __init__(
            self,
            scenario,
            planning_problem: PlanningProblem,
            scenario_type: str,
            use_post_opt: bool = False,
            initial_state: PlannerState = None,
    ):
        # Core parameters
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.scenario_type = scenario_type
        self.use_post_opt = use_post_opt

        # Initialize state with state class instead of string
        if initial_state is None:
            self.current_state = create_initial_state(scenario_type, "HEADING")
        else:
            self.current_state = initial_state

        self.stopping_counter = 0
        self.stopping_next_state = None

        # Add state entry tracking for logging
        self.entered_states: Set[str] = set()

        # Configuration setup
        self.config_root = pkg_resources.files(configurations) / scenario_type

        # Initialize scenario-specific components
        self._initialize_goal_positions()
        self._initialize_state_handlers()

        # Validate initialization
        self._validate_initialization()
        self.cr_State = None

        # Log initial state entry
        self._log_state_entry(self.current_state)


        #new cost calculate:

        self._last_planner = None
        self.cost_tracker = TrajectoryCostTracker_1()
        #new for timer
        self.timing_logs = []



    @abstractmethod
    def _initialize_goal_positions(self):
        """Initialize goal positions specific to the scenario type"""
        pass

    def _initialize_state_handlers(self):
        """Initialize state handler mapping using state classes"""
        self.handlers = {
            DepartingState: self._execute_departing,
            HeadingState: self._execute_heading,
            ArrivingState: self._execute_arriving,
            BeforeStoppingState: self._execute_before_stopping,
            StoppingState: self._execute_stopping,
        }

    def _validate_initialization(self):
        """Validate that initialization was successful"""
        if not hasattr(self, 'goal_x'):
            raise RuntimeError("Goal positions not initialized")

        # Check if current state is valid for this scenario
        if not self.current_state.is_valid_for_scenario():
            raise ValueError(f"Invalid initial state '{self.current_state.get_state_name()}' for {self.scenario_type}")

    def _log_state_entry(self, state: PlannerState):
        """Log state entry information only on first entry"""
        state_name = state.get_state_name()

        if state_name not in self.entered_states:
            self.entered_states.add(state_name)

            print(f"\n=== ENTERING STATE: {state_name} ===")
            print(f"Scenario Type: {self.scenario_type}")
            print(f"Config File: {state.get_config_filename()}")

            # Add state-specific information


            if hasattr(self, 'cr_State') and self.cr_State is not None:
                print(f"Current Vehicle Position: ({self.cr_State.position[0]:.2f}, {self.cr_State.position[1]:.2f})")
                print(f"Current Vehicle Velocity: {self.cr_State.velocity:.2f} m/s")
                print(f"Current Vehicle Orientation: {self.cr_State.orientation:.3f} rad")

            # State-specific information
            if isinstance(state, HeadingState):
                print("State Description: Vehicle is heading towards the goal position")
                if hasattr(self, 'goal_x') and hasattr(self, 'cr_State') and self.cr_State is not None:
                    distance_to_goal = abs(self.goal_x - self.cr_State.position[0])
                    print(f"Distance to Goal: {distance_to_goal:.2f} m")

            elif isinstance(state, ArrivingState):
                print("State Description: Vehicle is arriving at the target location")

            elif isinstance(state, DepartingState):
                print("State Description: Vehicle is departing from current position")

            elif isinstance(state, BeforeStoppingState):
                print("State Description: Vehicle is preparing to stop (bay scenario only)")

            elif isinstance(state, StoppingState):
                print("State Description: Vehicle is executing stopping maneuver")
                if hasattr(self, 'lon_goal'):
                    print(f"Longitudinal Goal Position: {self.lon_goal:.2f}")

            print(f"Post-optimization Enabled: {self.use_post_opt}")
            print("=" * 40)

    def step(self, state_current: State, state_list: list) -> State:
        """Execute one step of state machine planning"""
        self.cr_State = state_current

        # Handle special logic for STOPPING state
        if self._execute_stopping_counter(state_list):
            return self.stopping_next_state

        # Get handler for current state class
        handler = self.handlers.get(type(self.current_state))
        if handler is None:
            raise ValueError(f"Unknown state type: {type(self.current_state)}")

        return handler(state_current, state_list)

    def _execute_stopping_counter(self, state_list: list) -> bool:
        """Handle counter logic for STOPPING state"""
        if isinstance(self.current_state, StoppingState) and self.stopping_counter > 0:
            self.stopping_counter += 1
            if self.stopping_counter <= 15:
                append_state_to_list(state_list, self.stopping_next_state)
                return True
            else:
                # Reset stopping state and transition to departing
                self.stopping_counter = 0
                self.stopping_next_state = None
                self.current_state = DepartingState(self.scenario_type)
                # Log the new state entry
                self._log_state_entry(self.current_state)
        return False

    def _create_planner(self, state: PlannerState, coord_sys: CoordinateSystem,
                        state_current: State) -> Tuple[ReactivePlanner, object]:
        """Create and configure planner using state class"""
        # Get config filename from state
        yaml_name = state.get_config_filename()
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

    def _plan_and_optimize(self, planner: ReactivePlanner, config, state_list: list,
                           is_stopping: bool = False) -> Tuple[State, object]:
        """Execute planning with optional post-processing optimization"""
        # Execute planning


        #for time
        t0 = time.perf_counter()


        trajectory = planner.plan()
        base_plan_time = planner.planning_times[-1] if planner.planning_times else (time.perf_counter() - t0)

        # Fallback when planner returns None
        if trajectory is None:
            logger.warning("Planner returned no trajectory; attempting standstill fallback")
            standstill = planner._compute_standstill_trajectory()
            if standstill is not None:
                trajectory = planner._create_output(standstill)
            if trajectory is None or trajectory[0] is None:
                if planner.x_0_cl is None:
                    pos = planner.x_0.position
                    raise VehicleLeftScenarioError(
                        f"车辆已到达场景边界 (位置: ({pos[0]:.2f}, {pos[1]:.2f}))"
                    )
                print(
                    f"[ERROR] 规划失败: 无法找到有效轨迹且 standstill fallback 也失败，"
                    f"位置 ({planner.x_0.position[0]:.2f}, {planner.x_0.position[1]:.2f})，保持当前状态。"
                )
                logger.warning("Standstill fallback also failed; keeping current state")
                return planner.x_0, None

        # Post-processing optimization
        #for post time
        t1 = time.perf_counter()
        post_time = 0.0

        if self.use_post_opt and hasattr(planner, "best_sample"):
            if is_stopping and hasattr(planner, "post_optimize_for_stopping"):
                optimized = planner.post_optimize_for_stopping(planner.best_sample)
            else:
                optimized = planner.post_optimize(planner.best_sample)
            post_time = time.perf_counter() - t1
            if optimized is not None and optimized[0] is not None:
                trajectory = optimized

        # Extract next state and update state list
        next_state = trajectory[0].state_list[config.planning.replanning_frequency]

        append_states_to_list_ver(
            state_list,
            trajectory[0].state_list,
            config.planning.replanning_frequency
        )
        # 在 _plan_and_optimize 方法中，在 return 之前添加

        self.timing_logs.append({
            "state": self.get_current_state_name(),
            "plan_time": base_plan_time,  # CRRP 单次
            "post_opt_time": post_time,  # 优化器单次
            "total_time": base_plan_time + post_time,  # Post-Optimizer Planner 单次
            "Treplan": config.planning.replanning_frequency,
            "T": config.planning.time_steps_computation * config.planning.dt
        })
        self._last_planner = planner  # 保存planner引用
        # 追踪成本
        if planner.best_sample is not None:
            self.cost_tracker.track_planning_step(
                planner,
                self.get_current_state_name(),
                replanning_frequency=config.planning.replanning_frequency
            )
        return next_state, trajectory

    def _check_state_transition(self, next_state: State, config) -> None:
        """Check if state transition is needed using state class method"""
        new_state = self.current_state.check_transition(next_state, config, self.goal_x)
        if new_state is not None:
            self.current_state = new_state
            # Log the new state entry
            self._log_state_entry(self.current_state)

    # Default implementations for states
    def _execute_heading(self, state_current: State, state_list: list) -> State:
        """Execute HEADING state using state class"""
        coord_sys = self.current_state.get_coordinate_system(self.scenario, self.planning_problem)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)
        if abs(planner.x_0.velocity -config.sampling.desire_velocity) < 2 and planner.x_0.position[0]>10:
            config.planning.time_steps_computation = 10
            config.planning.replanning_frequency = 2
        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )


        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        self._check_state_transition(next_state, config)

        return next_state

    def _execute_arriving(self, state_current: State, state_list: list) -> State:
        """Execute ARRIVING state using state class"""
        coord_sys = self.current_state.get_coordinate_system(self.scenario, self.planning_problem)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)

        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        self._check_state_transition(next_state, config)

        return next_state

    @abstractmethod
    def _execute_departing(self, state_current: State, state_list: list) -> State:
        """Execute DEPARTING state (scenario-specific implementation required)"""
        pass

    @abstractmethod
    def _execute_stopping(self, state_current: State, state_list: list) -> State:
        """Execute STOPPING state (scenario-specific implementation required)"""
        pass

    def _execute_before_stopping(self, state_current: State, state_list: list) -> State:
        """Execute BEFORE_STOPPING state (default: not implemented)"""
        raise NotImplementedError(f"BEFORE_STOPPING state not implemented for {self.scenario_type}")

    def reset_state(self, new_state_name: str = "HEADING"):
        """Reset state machine to specified state"""
        self.current_state = create_initial_state(self.scenario_type, new_state_name)
        self.stopping_counter = 0
        self.stopping_next_state = None
        # Clear entered states history so logging works again after reset
        self.entered_states.clear()
        # Log the new initial state
        self._log_state_entry(self.current_state)

    def get_current_state(self) -> PlannerState:
        """Get current state object"""
        return self.current_state

    def get_current_state_name(self) -> str:
        """Get current state name as string"""
        return self.current_state.get_state_name()

    def clear_state_entry_history(self):
        """Clear the state entry history, allowing all states to be logged again"""
        self.entered_states.clear()


class BusStopBulbPlanner(BaseStateMachinePlanner):
    """
    State machine planner for bus stop bulb scenarios.
    State flow: DEPARTING → HEADING → ARRIVING → STOPPING
    """

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
        coord_sys = self.current_state.get_coordinate_system(self.scenario, self.planning_problem)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)

        planner._low_vel_mode = False
        planner._desired_speed = config.sampling.desire_velocity
        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        #check transition
        self._check_state_transition(next_state, config)
        return next_state

    def _execute_stopping(self, state_current: State, state_list: list) -> State:
        """Execute STOPPING state for bulb scenario"""
        # Check if stopping condition is already met
        if self._check_stopping_condition(state_current, state_list):
            return state_current

        coord_sys = self.current_state.get_coordinate_system(self.scenario, self.planning_problem)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)

        planner._desired_speed = 0
        planner.set_desired_lon_position(lon_position=self.lon_goal)

        next_state, _ = self._plan_and_optimize(planner, config, state_list, is_stopping=True)
        return next_state

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

    def _initialize_goal_positions(self):
        """Initialize goal positions for bay scenario"""
        goal_state = self.planning_problem.goal.state_list[0]
        self.goal_x = goal_state.position.center[0]
        self.goal_y = goal_state.position.center[1]

        # Calculate longitudinal goal position
        left_vertex = self.scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[0][0]
        self.lon_goal = self.goal_x - left_vertex

        # Calculate goal boundary for bay scenario
        self.goal_left = self.goal_x + 0.5 * goal_state.position.length

    def _execute_departing(self, state_current: State, state_list: list) -> State:
        """Execute DEPARTING state for bay scenario"""
        # Handle dynamic coordinate system for bay scenario
        if isinstance(self.current_state, DepartingState):
            coord_sys = self._get_departing_coord_system(state_current)
        else:
            coord_sys = self.current_state.get_coordinate_system(self.scenario, self.planning_problem)

        planner, config = self._create_planner(self.current_state, coord_sys, state_current)

        # Set desired velocity
        if planner.x_0.velocity < config.planning.low_vel_mode_threshold:
            planner._low_vel_mode = True
        else:
            config.sampling.d_min = -3.75
            config.sampling.d_max = 0

        planner._desired_speed = config.sampling.desire_velocity
        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        # check transition
        self._check_state_transition(next_state, config)
        return next_state

    def _execute_before_stopping(self, state_current: State, state_list: list) -> State:
        """Execute BEFORE_STOPPING state for bay scenario"""
        coord_sys = self.current_state.get_coordinate_system(self.scenario, self.planning_problem)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)

        planner._desired_speed = 0
        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )

        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        self._check_state_transition(next_state, config)

        return next_state

    def _execute_stopping(self, state_current: State, state_list: list) -> State:
        """Execute STOPPING state for bay scenario"""
        # Check if stopping condition is already met
        if self._check_stopping_condition(state_current, state_list):
            return state_current

        coord_sys = self.current_state.get_coordinate_system(self.scenario, self.planning_problem)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)

        planner._low_vel_mode = True
        planner._desired_speed = 0
        planner.set_desired_lon_position(lon_position=self.lon_goal)

        next_state, _ = self._plan_and_optimize(planner, config, state_list, is_stopping=True)
        return next_state

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
        """Get appropriate coordinate system for DEPARTING state in bay scenario"""
        if state_current.velocity < 2:
            x_values = np.linspace(-150, 150, 3000)
            y_values = np.full_like(x_values, state_current.position[1])
            reference_path = np.column_stack((x_values, y_values))
            return create_coordinate_system(reference_path)
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
        initial_state_name: str = "HEADING"
) -> BaseStateMachinePlanner:
    """
    Factory function to create appropriate state machine planner based on scenario type.

    Args:
        scenario_type: "bus_stop_bulb" or "bus_stop_bay"
        use_post_opt: Whether to enable post-processing optimization
        initial_state_name: Initial state name for the state machine

    Returns:
        Appropriate planner instance
    """
    # Create initial state instance
    initial_state = create_initial_state(scenario_type, initial_state_name)

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


def simulate_state_machine(
        current_scenario,
        state_current_ego,
        planning_problem,
        state_list,
        yaml_file_path: str,
) -> State:
    """
    Store planner instances using function attributes to avoid global variables

    Args:
        current_scenario: Current CommonRoad scenario
        state_current_ego: Current ego vehicle state
        planning_problem: Planning problem to solve
        state_list: List to store trajectory states
        yaml_file_path: Path to configuration YAML file

    Returns:
        Next state from the planner
    """
    try:
        with open(yaml_file_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
            scenario_type = config.get('scenario', {}).get('type')
            post_opt = config.get('debug', {}).get('use_post_opt')
    except FileNotFoundError:
        print(f"file: {yaml_file_path} not found")
        return None
    except yaml.YAMLError as e:
        print(f"YAML error: {e}")
        return None

    # Generate unique key for planner caching
    key = f"{scenario_type}_{'opt' if post_opt else 'noopt'}"

    # Use function attribute to store planners
    if not hasattr(simulate_state_machine, '_planners'):
        simulate_state_machine._planners = {}

    if key not in simulate_state_machine._planners:
        simulate_state_machine._planners[key] = create_state_machine_planner(
            current_scenario,
            planning_problem,
            scenario_type,
            use_post_opt=post_opt,
            initial_state_name="HEADING",
        )


    return simulate_state_machine._planners[key].step(state_current_ego, state_list)


def clear_planner_cache():
    """Clear planner cache stored in function attributes"""
    if hasattr(simulate_state_machine, '_planners'):
        simulate_state_machine._planners.clear()


def reset_planner_state(new_state_name: str = "HEADING"):
    """Reset all cached planners to specified state"""
    if hasattr(simulate_state_machine, '_planners'):
        for planner in simulate_state_machine._planners.values():
            planner.reset_state(new_state_name)


def clear_all_state_entry_history():
    """Clear state entry history for all cached planners"""
    if hasattr(simulate_state_machine, '_planners'):
        for planner in simulate_state_machine._planners.values():
            planner.clear_state_entry_history()