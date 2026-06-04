import copy
import logging
import numpy as np
import yaml

logger = logging.getLogger("RP_LOGGER")
import configurations
import importlib.resources as pkg_resources
from abc import ABC, abstractmethod
from typing import Any, Dict, Tuple, Optional, Set, Type
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.trajectory import State
from source.commonroad_rp.state import append_state_to_list, append_states_to_list_ver
from source.commonroad_rp.utility.config import ReactivePlannerConfiguration
from source.commonroad_rp.utility.utils_coordinate_system import CoordinateSystem, create_coordinate_system
from source.commonroad_rp.reactive_planner import ReactivePlanner as CommonRoadReactivePlanner
import time

# Import state classes
from post_optimization_planner.State import (
    PlannerState, DepartingState, HeadingState, ArrivingState,
    BeforeStoppingState, StoppingState, create_initial_state,
)

from utility.cost_calculate import TrajectoryCostTracker_1


def _clip01(value: float) -> float:
    return np.clip(value, 0.0, 1.0)


def _select_planner_class(use_post_opt: bool) -> Type[Any]:
    if use_post_opt:
        logger.warning("post-optimization requested but disabled; using CommonRoad RP only")
    return CommonRoadReactivePlanner


class VehicleLeftScenarioError(Exception):
    """Raised when the vehicle has driven outside the scenario boundary."""
    pass


class StateMachineFinished(VehicleLeftScenarioError):
    """Raised when the state machine has completed the configured single-goal task."""
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
        self.use_post_opt = False
        self.planner_cls: Type[Any] = _select_planner_class(use_post_opt)
        self.fallback_logs: list[dict[str, Any]] = []
        self._active_planning_state_name: Optional[str] = None
        self._before_stopping_entry_distance: Optional[float] = None
        self._before_stopping_entry_pose: Optional[Tuple[float, float, float]] = None
        self._departing_entry_pose: Optional[Tuple[float, float, float]] = None
        self._completed_stop_service = False
        self.goal_x: float = 0.0
        self.goal_y: float = 0.0
        self.lon_goal: float = 0.0
        self.goal_boundary: float = 0.0
        self.goal_left: float = 0.0

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

            print("Planner backend: CommonRoad RP only")
            print("=" * 40)

    def step(self, state_current: State, state_list: list) -> State:
        """Execute one step of state machine planning"""
        self.cr_State = state_current

        if self._is_mission_complete(state_current):
            pos = state_current.position
            raise StateMachineFinished(
                f"状态机已完成一次停靠-发车任务 (位置: ({pos[0]:.2f}, {pos[1]:.2f}))"
            )

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
                self._completed_stop_service = True
                self._departing_entry_pose = None
                self.current_state = DepartingState(self.scenario_type)
                # Log the new state entry
                self._log_state_entry(self.current_state)
        return False

    def _create_planner(self, state: PlannerState, coord_sys: CoordinateSystem,
                        state_current: State) -> Tuple[CommonRoadReactivePlanner, object]:
        """Create and configure planner using state class"""
        # Get config filename from state
        yaml_name = state.get_config_filename()
        self._active_planning_state_name = state.get_state_name()
        config_path = str(self.config_root / yaml_name)
        config: Any = ReactivePlannerConfiguration.load(config_path)
        config.update(self.scenario, planning_problem=self.planning_problem)

        # Initialize planner
        planner = self.planner_cls(config)
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

    def _plan_and_optimize(self, planner: CommonRoadReactivePlanner, config: Any, state_list: list,
                           is_stopping: bool = False) -> Tuple[State, object]:
        """Execute one CommonRoad reactive-planner cycle."""
        # Execute planning


        #for time
        t0 = time.perf_counter()


        trajectory = planner.plan()
        base_plan_time = planner.planning_times[-1] if planner.planning_times else (time.perf_counter() - t0)

        # Fallback when planner returns None
        if trajectory is None:
            self._record_fallback_diagnostics(planner, config, "planner returned no trajectory")
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
            self.fallback_logs[-1]["result"] = "standstill trajectory"

        # Extract next state and update state list
        next_state = trajectory[0].state_list[config.planning.replanning_frequency]

        append_states_to_list_ver(
            state_list,
            trajectory[0].state_list,
            config.planning.replanning_frequency
        )

        self.timing_logs.append({
            "state": self.get_current_state_name(),
            "plan_time": base_plan_time,  # CRRP 单次
            "total_time": base_plan_time,
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

    def _record_fallback_diagnostics(self, planner: CommonRoadReactivePlanner, config: Any, reason: str) -> None:
        horizon = getattr(planner, "horizon", config.planning.dt * config.planning.time_steps_computation)
        current_v = float(getattr(planner.x_0, "velocity", 0.0))
        desired_v = getattr(planner, "_desired_speed", None)
        required_decel_to_desired_v = None
        if desired_v is not None and horizon > 1e-9:
            required_decel_to_desired_v = (float(desired_v) - current_v) / horizon

        distance_to_stop_goal = None
        required_decel_to_stop_goal = None
        if getattr(planner, "_desired_lon_position", None) is not None and planner.x_0_cl is not None:
            distance_to_stop_goal = float(planner._desired_lon_position - planner.x_0_cl[0][0])
            if distance_to_stop_goal > 1e-6:
                required_decel_to_stop_goal = -(current_v ** 2) / (2.0 * distance_to_stop_goal)

        entry = {
            "state": self._active_planning_state_name or self.get_current_state_name(),
            "reason": reason,
            "result": "failed",
            "position": copy.deepcopy(getattr(planner.x_0, "position", None)),
            "velocity": current_v,
            "desired_speed": desired_v,
            "longitudinal_mode": getattr(config.sampling, "longitudinal_mode", None),
            "horizon": horizon,
            "required_decel_to_desired_v": required_decel_to_desired_v,
            "distance_to_stop_goal": distance_to_stop_goal,
            "required_decel_to_stop_goal": required_decel_to_stop_goal,
            "total_samples": getattr(planner, "total_count_samples", None),
            "infeasible_kinematics": getattr(planner, "infeasible_count_kinematics", None),
            "infeasible_collision": getattr(planner, "infeasible_count_collision", None),
            "infeasible_reasons": copy.deepcopy(getattr(planner, "infeasible_reason_dict", None)),
            "x0_cl_is_none": planner.x_0_cl is None,
        }
        self.fallback_logs.append(entry)

    def _check_state_transition(self, next_state: State, config: Any) -> None:
        """Check if state transition is needed using state class method"""
        new_state = self.current_state.check_transition(next_state, config, self.goal_x)
        if new_state is not None and self._allow_state_transition(self.current_state, new_state, next_state, config):
            self.current_state = new_state
            # Log the new state entry
            self._log_state_entry(self.current_state)

    def _allow_state_transition(
            self,
            from_state: PlannerState,
            to_state: PlannerState,
            next_state: State,
            config: Any,
    ) -> bool:
        return True

    def _is_mission_complete(self, state_current: State) -> bool:
        return False

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
        self._before_stopping_entry_distance = None
        self._before_stopping_entry_pose = None
        self._departing_entry_pose = None
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

    def _interpolate_centerline_y(self, lanelet_id: int, x_values: np.ndarray) -> np.ndarray:
        vertices = np.asarray(
            self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id).center_vertices,
            dtype=float,
        )
        order = np.argsort(vertices[:, 0])
        xs = vertices[order, 0]
        ys = vertices[order, 1]
        return np.interp(x_values, xs, ys, left=ys[0], right=ys[-1])

    def _build_lateral_shift_path(
            self,
            state_current: State,
            target_y: float,
            merge_start_x: float,
            merge_end_x: float,
            end_x: float,
            target_lanelet_id: Optional[int] = None,
            start_x: Optional[float] = None,
            start_y: Optional[float] = None,
    ) -> np.ndarray:
        start_x = float(state_current.position[0]) if start_x is None else float(start_x)
        start_y = float(state_current.position[1]) if start_y is None else float(start_y)
        start_x = min(start_x - 20.0, merge_start_x - 5.0)
        end_x = max(end_x, merge_end_x + 30.0)
        n_pts = max(80, int((end_x - start_x) * 8))
        x_values = np.linspace(start_x, end_x, n_pts)

        if target_lanelet_id is None:
            target_y_values = np.full_like(x_values, float(target_y))
        else:
            target_y_values = self._interpolate_centerline_y(target_lanelet_id, x_values)

        denom = max(merge_end_x - merge_start_x, 1e-6)
        progress = _clip01((x_values - merge_start_x) / denom)
        progress = progress * progress * (3.0 - 2.0 * progress)
        y_values = start_y * (1.0 - progress) + target_y_values * progress
        return np.column_stack((x_values, y_values))

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
        if self._completed_stop_service:
            config.sampling.d_min = -0.75
            config.sampling.d_max = 0.75

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
        coord_sys = self._get_before_stopping_coord_system(state_current)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)
        virtual_state_name, progress, distance_to_goal = self._smooth_before_stopping_config(config, state_current)
        self._active_planning_state_name = virtual_state_name
        self._regularize_transition_initial_state(planner, progress)

        planner._desired_speed = 0
        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=config.sampling.desire_velocity,
        )
        if progress > 0.2:
            planner.set_v_sampling_parameters(
                0.0,
                max(float(state_current.velocity), float(config.sampling.desire_velocity)) + 0.5,
            )

        fallback_count_before = len(self.fallback_logs)
        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        if len(self.fallback_logs) > fallback_count_before:
            self.fallback_logs[-1]["before_stopping_progress"] = progress
            self.fallback_logs[-1]["distance_to_goal_x"] = distance_to_goal
        self._check_state_transition(next_state, config)

        return next_state

    def _smooth_before_stopping_config(self, config: Any, state_current: State) -> Tuple[str, float, float]:
        distance_to_goal = abs(float(self.goal_x) - float(state_current.position[0]))
        transition_threshold = float(getattr(config.planning, "distance_before_stopping_to_stopping", 3.0))
        horizon = max(
            float(config.planning.dt),
            float(config.planning.dt) * float(config.planning.time_steps_computation),
        )
        current_v = max(0.0, float(state_current.velocity))
        a_max = float(getattr(config.vehicle, "a_max", 2.0))

        if self._before_stopping_entry_distance is None or distance_to_goal > self._before_stopping_entry_distance:
            self._before_stopping_entry_distance = max(distance_to_goal, transition_threshold + 1.0)

        comfort_decel = max(0.4, min(0.8 * a_max, 1.4))
        braking_distance = current_v ** 2 / max(2.0 * comfort_decel, 1e-6)
        smoothing_span = max(
            8.0,
            current_v * horizon,
            braking_distance + current_v * 1.5,
            self._before_stopping_entry_distance - transition_threshold,
        )
        progress = _clip01((transition_threshold + smoothing_span - distance_to_goal) / smoothing_span)

        n_substates = 4
        sub_idx = min(n_substates, max(1, int(np.floor(progress * n_substates)) + 1))
        virtual_state_name = f"BEFORE_STOPPING_{sub_idx}"

        min_pre_stop_speed = 0.35
        base_desired = max(min_pre_stop_speed, float(config.sampling.desire_velocity))
        target_v = base_desired * (1.0 - progress) + min_pre_stop_speed * progress
        max_drop = comfort_decel * horizon
        target_v = max(min_pre_stop_speed, current_v - max_drop, target_v)
        config.sampling.desire_velocity = float(min(base_desired, target_v))

        if hasattr(config.sampling, "d_min") and hasattr(config.sampling, "d_max"):
            width_scale = 1.0 + 0.25 * progress
            config.sampling.d_min = float(config.sampling.d_min) * width_scale
            config.sampling.d_max = float(config.sampling.d_max) * width_scale

        print(
            f"[transition] {virtual_state_name}: distance_to_goal={distance_to_goal:.2f} m, "
            f"progress={progress:.2f}, desired_v={config.sampling.desire_velocity:.2f} m/s"
        )

        return virtual_state_name, progress, distance_to_goal

    def _regularize_transition_initial_state(self, planner: CommonRoadReactivePlanner, progress: float) -> None:
        max_steering = 0.035 - 0.015 * _clip01(progress)
        steering = float(getattr(planner.x_0, "steering_angle", 0.0))
        if abs(steering) > max_steering:
            planner.x_0.steering_angle = float(np.sign(steering) * max_steering)
        yaw_rate = float(getattr(planner.x_0, "yaw_rate", 0.0))
        if abs(yaw_rate) > 0.04:
            planner.x_0.yaw_rate = float(np.sign(yaw_rate) * 0.04)

    def _allow_state_transition(
            self,
            from_state: PlannerState,
            to_state: PlannerState,
            next_state: State,
            config: Any,
    ) -> bool:
        if isinstance(from_state, DepartingState) and isinstance(to_state, HeadingState):
            try:
                heading_coord_sys = HeadingState(self.scenario_type).get_coordinate_system(
                    self.scenario, self.planning_problem
                )
                _, d = heading_coord_sys.convert_to_curvilinear_coords(
                    next_state.position[0], next_state.position[1]
                )
                if abs(d) > 0.6:
                    print(f"[transition] DEPARTING held: lateral offset to heading lane {d:.2f} m > 0.60 m")
                    return False
            except Exception as exc:
                print(f"[transition] DEPARTING held: cannot evaluate heading lane offset ({exc})")
                return False

        if isinstance(from_state, BeforeStoppingState) and isinstance(to_state, StoppingState):
            steering = abs(float(getattr(next_state, "steering_angle", 0.0)))
            velocity = float(getattr(next_state, "velocity", 0.0))
            lateral_goal_error = abs(float(next_state.position[1]) - float(self.goal_y))
            if velocity > 0.8:
                print(f"[transition] BEFORE_STOPPING held: velocity {velocity:.2f} m/s > 0.80 m/s")
                return False
            if steering > 0.12:
                print(f"[transition] BEFORE_STOPPING held: steering {steering:.3f} rad > 0.120 rad")
                return False
            if lateral_goal_error > 1.2:
                print(
                    f"[transition] BEFORE_STOPPING held: lateral goal error "
                    f"{lateral_goal_error:.2f} m > 1.20 m"
                )
                return False

        return True

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
        ego_y = state_current.position[1]
        ego_v = state_current.velocity

        goal_state = self.planning_problem.goal.state_list[0]
        half_length = 0.5 * float(getattr(goal_state.position, "length", 24.0))
        half_width = 0.5 * float(getattr(goal_state.position, "width", 6.0))
        longitudinal_ok = abs(float(ego_x) - float(self.goal_x)) <= half_length
        lateral_ok = abs(float(ego_y) - float(self.goal_y)) <= half_width
        condition = longitudinal_ok and lateral_ok and ego_v < 1e-5

        if condition:
            append_state_to_list(state_list, state_current)
            self.stopping_counter = 1
            self.stopping_next_state = state_current
            return True

        return False

    def _get_before_stopping_coord_system(self, state_current: State) -> CoordinateSystem:
        """Use a smooth, state-relative reference from the current lane to the stop lane."""
        if self._before_stopping_entry_pose is None:
            self._before_stopping_entry_pose = (
                float(state_current.position[0]),
                float(state_current.position[1]),
                max(0.0, float(state_current.velocity)),
            )
        entry_x, entry_y, entry_v = self._before_stopping_entry_pose
        bay_entry_x = float(self.goal_x) - 44.0
        merge_start_x = max(entry_x + max(4.0, entry_v * 0.8), bay_entry_x)
        merge_end_x = max(merge_start_x + 22.0, float(self.goal_x) - 8.0)
        end_x = self.goal_x + 60.0
        reference_path = self._build_lateral_shift_path(
            state_current=state_current,
            target_y=self.goal_y,
            merge_start_x=merge_start_x,
            merge_end_x=merge_end_x,
            end_x=end_x,
            start_x=entry_x,
            start_y=entry_y,
        )
        return create_coordinate_system(reference_path)

    def _get_departing_coord_system(self, state_current: State) -> CoordinateSystem:
        """Get appropriate coordinate system for DEPARTING state in bay scenario"""
        if not self._completed_stop_service:
            if state_current.velocity < 2:
                x_values = np.linspace(-150, 150, 3000)
                y_values = np.full_like(x_values, state_current.position[1])
                reference_path = np.column_stack((x_values, y_values))
                return create_coordinate_system(reference_path)
            return create_coordinate_system(
                self.scenario.lanelet_network.find_lanelet_by_id(1).center_vertices
            )

        if self._departing_entry_pose is None:
            self._departing_entry_pose = (
                float(state_current.position[0]),
                float(state_current.position[1]),
                max(0.0, float(state_current.velocity)),
            )
        entry_x, entry_y, entry_v = self._departing_entry_pose
        main_lanelet_id = 2
        merge_start_x = entry_x + max(8.0, entry_v * 1.2)
        merge_end_x = merge_start_x + 70.0
        end_x = merge_end_x + 100.0
        target_y = float(self._interpolate_centerline_y(main_lanelet_id, np.array([merge_end_x]))[0])
        reference_path = self._build_lateral_shift_path(
            state_current=state_current,
            target_y=target_y,
            merge_start_x=merge_start_x,
            merge_end_x=merge_end_x,
            end_x=end_x,
            target_lanelet_id=main_lanelet_id,
            start_x=entry_x,
            start_y=entry_y,
        )
        return create_coordinate_system(reference_path)

    def _is_mission_complete(self, state_current: State) -> bool:
        if not self._completed_stop_service or not isinstance(self.current_state, HeadingState):
            return False
        distance_after_stop = float(state_current.position[0]) - float(self.goal_x)
        return distance_after_stop > 35.0


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
        use_post_opt: Ignored; planning always uses CommonRoad RP only.
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
            requested_post_opt = bool(config.get('debug', {}).get('use_post_opt'))
    except FileNotFoundError:
        print(f"file: {yaml_file_path} not found")
        return None
    except yaml.YAMLError as e:
        print(f"YAML error: {e}")
        return None

    if requested_post_opt:
        logger.warning("debug.use_post_opt is ignored; using CommonRoad RP only")

    # Generate unique key for planner caching
    key = f"{scenario_type}_commonroad_rp"

    # Use function attribute to store planners
    if not hasattr(simulate_state_machine, '_planners'):
        simulate_state_machine._planners = {}

    if key not in simulate_state_machine._planners:
        simulate_state_machine._planners[key] = create_state_machine_planner(
            current_scenario,
            planning_problem,
            scenario_type,
            use_post_opt=False,
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
