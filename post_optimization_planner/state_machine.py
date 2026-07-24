import copy
import logging
import numpy as np
import yaml

logger = logging.getLogger("RP_LOGGER")
import configurations
import importlib.resources as pkg_resources
from abc import ABC, abstractmethod
from typing import Any, Dict, Tuple, Optional, Set, Type
import commonroad_dc.pycrcc as pycrcc
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
    BeforeStoppingState, BeforeStoppingAlignState, BeforeStoppingMergeState,
    BeforeStoppingFinalState, StoppingState, EmergencyBrakeState,
    create_initial_state,
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
        self._before_stopping_entry_pose: Optional[Tuple[float, float, float]] = None
        self._before_stopping_entry_heading: Optional[float] = None
        self._departing_entry_pose: Optional[Tuple[float, float, float]] = None
        self._completed_stop_service = False
        self.state_before_emergency: Optional[PlannerState] = None
        # True means regular planning failed and the state machine is generating
        # controlled braking states directly.
        self._fallback_braking_active = False
        # Parameters stay fixed for one complete fallback-braking episode.
        self._fallback_braking_params: Optional[Dict[str, float]] = None
        self._emergency_standstill_cycles = 0
        self._max_emergency_standstill_cycles = 30
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
            BeforeStoppingAlignState: self._execute_before_stopping,
            BeforeStoppingMergeState: self._execute_before_stopping,
            BeforeStoppingFinalState: self._execute_before_stopping,
            StoppingState: self._execute_stopping,
            EmergencyBrakeState: self._execute_emergency_brake,
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
            config_filename = state.get_config_filename()
            if config_filename is None:
                print("Config File: N/A (controlled braking fallback)")
            else:
                print(f"Config File: {config_filename}")

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

            elif isinstance(state, BeforeStoppingAlignState):
                print("State Description: Vehicle is aligning pose before bay merge")

            elif isinstance(state, BeforeStoppingMergeState):
                print("State Description: Vehicle is merging into the bus bay")

            elif isinstance(state, BeforeStoppingFinalState):
                print("State Description: Vehicle is preparing for final stop")

            elif isinstance(state, BeforeStoppingState):
                print("State Description: Vehicle is preparing to stop (bay scenario only)")

            elif isinstance(state, StoppingState):
                print("State Description: Vehicle is executing stopping maneuver")
                if hasattr(self, 'lon_goal'):
                    print(f"Longitudinal Goal Position: {self.lon_goal:.2f}")

            elif isinstance(state, EmergencyBrakeState):
                print("State Description: Safety fallback controlled braking")
                if self.state_before_emergency is not None:
                    print(
                        "State Before Emergency: "
                        f"{self.state_before_emergency.get_state_name()}"
                    )

            print("Planner backend: CommonRoad RP only")
            print("=" * 40)

    def step(self, state_current: State, state_list: list) -> State:
        """Execute one step of state machine planning"""
        self.cr_State = state_current

        if self._is_mission_complete(state_current):
            pos = state_current.position
            raise StateMachineFinished(
                f"Bus has finished one stop-and-go task (position: ({pos[0]:.2f}, {pos[1]:.2f}))"
            )

        # After fallback braking starts, regular planning is skipped until the
        # vehicle has settled and the emergency horizon is clear.
        if self._fallback_braking_active:
            if self._fallback_braking_complete(state_current):
                restart_state = self._select_restart_state_after_emergency(state_current)
                if self._emergency_recovery_allowed(state_current, restart_state):
                    self._fallback_braking_active = False
                    self._fallback_braking_params = None
                    self._emergency_standstill_cycles = 0
                    self._restart_after_emergency(restart_state)
                    # The settled state becomes x_0 for the restarted planner.
                    print(
                        f"[emergency-brake] Restart x0: "
                        f"position=({state_current.position[0]:.3f}, "
                        f"{state_current.position[1]:.3f}), "
                        f"orientation={float(state_current.orientation):.6f} rad, "
                        f"velocity={float(state_current.velocity):.3f} m/s, "
                        f"steering={float(getattr(state_current, 'steering_angle', 0.0)):.6f} rad; "
                        f"restarting {self.get_current_state_name()} planning"
                    )
                else:
                    return self._execute_emergency_standstill(state_current, state_list)
            else:
                # Keep braking for one replanning segment and return immediately.
                return self._execute_fallback_braking(state_current, state_list)

        # Handle special logic for STOPPING state
        if self._execute_stopping_counter(state_list):
            return self.stopping_next_state

        # Get handler for current state class
        handler = self.handlers.get(type(self.current_state))
        if handler is None:
            raise ValueError(f"Unknown state type: {type(self.current_state)}")

        return handler(state_current, state_list)

    def _execute_emergency_brake(self, state_current: State, state_list: list) -> State:
        """Keep ego stopped while waiting for imminent_collision_risk to clear."""
        return self._execute_emergency_standstill(state_current, state_list)

    def _execute_stopping_counter(self, state_list: list) -> bool:
        """Handle counter logic for STOPPING state"""
        if isinstance(self.current_state, StoppingState) and self.stopping_counter > 0:
            self.stopping_counter += 1
            if self.stopping_counter <= 10:
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
        if yaml_name is None:
            raise RuntimeError(
                f"State '{state.get_state_name()}' is not planner-backed and has no YAML config"
            )
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

    def _activate_fallback_braking(self, config: Any, state_current: State) -> None:
        """Freeze fallback parameters at the trigger state and enter braking mode."""
        # Read the configured acceleration limit and avoid a pathological zero cap.
        a_max = max(0.1, float(config.vehicle.a_max))

        # Keep vehicle parameters and control limits fixed for this fallback
        # episode; motion integration still starts from each current state.
        self._fallback_braking_params = {
            "dt": float(config.planning.dt),
            "replanning_frequency": int(config.planning.replanning_frequency),
            # Emergency deceleration never exceeds the vehicle acceleration cap.
            "max_deceleration": min(2.0, a_max),
            # v_delta_max limits steering-angle change per dt.
            "max_steering_rate": max(0.0, float(config.vehicle.v_delta_max)),
            # The bicycle model uses wheelbase to map steering angle to yaw rate.
            "wheelbase": max(1e-6, float(config.vehicle.wheelbase)),
        }

        # From the next step, the state machine generates controlled braking
        # states directly until the vehicle has fully settled.
        self._fallback_braking_active = True
        self._emergency_standstill_cycles = 0

    def _enter_emergency_brake(self, config: Any, state_current: State, reason: str) -> None:
        """Enter EMERGENCY_BRAKE after imminent_collision_risk becomes true."""
        if not isinstance(self.current_state, EmergencyBrakeState):
            self.state_before_emergency = create_initial_state(
                self.scenario_type,
                self.current_state.get_state_name(),
            )
            self._clear_motion_phase_anchors()
            self.current_state = EmergencyBrakeState(self.scenario_type)
            self._log_state_entry(self.current_state)

        self._activate_fallback_braking(config, state_current)
        if self.fallback_logs:
            self.fallback_logs[-1]["imminent_collision_risk"] = True
            self.fallback_logs[-1]["state_before_emergency"] = (
                self.state_before_emergency.get_state_name()
                if self.state_before_emergency is not None
                else None
            )
            self.fallback_logs[-1]["emergency_reason"] = reason

    def _restart_after_emergency(self, restart_state: PlannerState) -> None:
        """Restart in the logical state selected from the stopped ego state."""
        self.current_state = create_initial_state(
            self.scenario_type,
            restart_state.get_state_name(),
        )
        self.state_before_emergency = None
        self._log_state_entry(self.current_state)

    def _select_restart_state_after_emergency(self, state_current: State) -> PlannerState:
        """Select the normal state whose transition guards match the stopped ego state."""
        previous_state = self.state_before_emergency
        if previous_state is None:
            previous_state = create_initial_state(self.scenario_type, "HEADING")

        state_names = self._restart_state_sequence()
        previous_name = previous_state.get_state_name()
        start_index = state_names.index(previous_name) if previous_name in state_names else 0
        selected_index = start_index

        for state_name in state_names[start_index:]:
            from_state = create_initial_state(self.scenario_type, state_name)
            if from_state.get_config_filename() is None:
                continue

            config = self._load_config_for_state(from_state)
            to_state = from_state.check_transition(state_current, config, self.goal_x)
            if to_state is None:
                continue
            to_name = to_state.get_state_name()
            if to_name not in state_names:
                continue
            to_index = state_names.index(to_name)
            if to_index < selected_index:
                continue
            if not self._allow_state_transition(from_state, to_state, state_current, config):
                continue
            selected_index = to_index

        selected_state = create_initial_state(self.scenario_type, state_names[selected_index])
        if self.fallback_logs:
            self.fallback_logs[-1]["restart_state"] = selected_state.get_state_name()
        return selected_state

    def _restart_state_sequence(self) -> Tuple[str, ...]:
        """Return normal state order used to classify a stopped emergency-recovery state."""
        if self._completed_stop_service:
            return ("DEPARTING", "HEADING")
        if self.scenario_type == "bus_stop_bulb":
            return ("DEPARTING", "HEADING", "ARRIVING", "STOPPING")
        if self.scenario_type == "bus_stop_bay":
            return (
                "DEPARTING",
                "HEADING",
                "ARRIVING",
                "BEFORE_STOPPING_MERGE",
                "BEFORE_STOPPING_ALIGN",
                "BEFORE_STOPPING_FINAL",
                "STOPPING",
            )
        raise ValueError(f"Unknown scenario type: {self.scenario_type}")

    def _load_config_for_state(self, state: PlannerState) -> Any:
        yaml_name = state.get_config_filename()
        if yaml_name is None:
            raise RuntimeError(
                f"State '{state.get_state_name()}' is not planner-backed and has no YAML config"
            )
        config = ReactivePlannerConfiguration.load(str(self.config_root / yaml_name))
        config.update(self.scenario, planning_problem=self.planning_problem)
        return config

    def _clear_motion_phase_anchors(self) -> None:
        """Drop physical reference anchors so recovery replans from the stopped ego state."""
        self._before_stopping_entry_pose = None
        self._before_stopping_entry_heading = None
        self._departing_entry_pose = None

    @staticmethod
    def _fallback_braking_complete(state: State) -> bool:
        """Return True only when the vehicle and steering have settled."""
        return (
            # Longitudinal velocity has settled, allowing small numerical noise.
            abs(float(getattr(state, "velocity", 0.0))) <= 1e-3
            # Acceleration must also be back at zero after braking.
            and abs(float(getattr(state, "acceleration", 0.0))) <= 1e-3
            # Steering must be centered before replanning resumes.
            and abs(float(getattr(state, "steering_angle", 0.0))) <= 1e-3
        )

    def _execute_emergency_standstill(self, state_current: State, state_list: list) -> State:
        """Advance time with a zero-velocity ego state while emergency risk persists."""
        self._emergency_standstill_cycles += 1
        if self._emergency_standstill_cycles > self._max_emergency_standstill_cycles:
            pos = state_current.position
            raise VehicleLeftScenarioError(
                "Emergency braking recovery did not become safe after "
                f"{self._max_emergency_standstill_cycles} standstill cycles "
                f"at position ({pos[0]:.2f}, {pos[1]:.2f}); aborting to avoid a dead loop."
            )

        replanning_frequency = 1
        if self._fallback_braking_params is not None:
            replanning_frequency = int(self._fallback_braking_params["replanning_frequency"])

        states = [copy.deepcopy(state_current)]
        current = states[0]
        for _ in range(replanning_frequency):
            next_state = copy.deepcopy(current)
            next_state.velocity = 0.0
            next_state.acceleration = 0.0
            next_state.steering_angle = 0.0
            next_state.yaw_rate = 0.0
            if hasattr(next_state, "slip_angle"):
                next_state.slip_angle = 0.0
            next_state.time_step = int(getattr(current, "time_step", 0)) + 1
            states.append(next_state)
            current = next_state

        append_states_to_list_ver(state_list, states, replanning_frequency)
        print(
            "[emergency-brake] Holding standstill; "
            "imminent_collision_risk remains true"
        )
        return states[-1]

    def _execute_fallback_braking(self, state_current: State, state_list: list) -> State:
        """Generate one dynamically consistent emergency-braking segment."""
        if self._fallback_braking_params is None:
            raise RuntimeError("Fallback braking is active without braking parameters")

        self._emergency_standstill_cycles = 0

        # Use the frozen parameters from fallback activation so the entire
        # braking episode uses one vehicle model and one set of control limits.
        params = self._fallback_braking_params
        dt = params["dt"]
        max_deceleration = params["max_deceleration"]
        wheelbase = params["wheelbase"]
        # Convert steering-rate limit into a per-dt steering change.
        max_steering_step = params["max_steering_rate"] * dt
        replanning_frequency = int(params["replanning_frequency"])

        # The first item is the real trigger state; following states are
        # integrated continuously from their predecessor.
        states = [copy.deepcopy(state_current)]
        current = states[0]

        # One call generates one full replanning segment.
        for _ in range(replanning_frequency):
            # Clamp tiny numerical errors so velocity cannot become negative.
            velocity = max(0.0, float(getattr(current, "velocity", 0.0)))
            orientation = float(getattr(current, "orientation", 0.0))
            steering = float(getattr(current, "steering_angle", 0.0))

            next_velocity = max(0.0, velocity - max_deceleration * dt)

            # Center steering gradually instead of snapping it to zero.
            if abs(steering) <= max_steering_step:
                next_steering = 0.0
            else:
                next_steering = steering - np.sign(steering) * max_steering_step

            # Trapezoidal integration with average speed and steering angle.
            mean_velocity = 0.5 * (velocity + next_velocity)
            mean_steering = 0.5 * (steering + next_steering)
            displacement = mean_velocity * dt

            # Slip-free bicycle model: kappa=tan(delta)/wheelbase, yaw_rate=v*kappa.
            mean_yaw_rate = mean_velocity * np.tan(mean_steering) / wheelbase
            orientation_change = mean_yaw_rate * dt

            # Use midpoint heading for position integration.
            midpoint_orientation = orientation + 0.5 * orientation_change
            next_position = np.asarray(current.position, dtype=float) + displacement * np.array([
                np.cos(midpoint_orientation),
                np.sin(midpoint_orientation),
            ])

            # Wrap heading to [-pi, pi] while preserving continuity for planner x_0.
            unwrapped_orientation = orientation + orientation_change
            next_orientation = np.arctan2(
                np.sin(unwrapped_orientation),
                np.cos(unwrapped_orientation),
            )

            # Endpoint yaw_rate must match endpoint speed and steering angle.
            next_yaw_rate = next_velocity * np.tan(next_steering) / wheelbase

            next_state = copy.deepcopy(current)

            next_state.position = next_position
            next_state.velocity = next_velocity
            next_state.acceleration = -max_deceleration if next_velocity > 0.0 else 0.0
            next_state.orientation = next_orientation
            next_state.steering_angle = next_steering
            next_state.yaw_rate = next_yaw_rate
            if hasattr(next_state, "slip_angle"):
                next_state.slip_angle = 0.0

            next_state.time_step = int(getattr(current, "time_step", 0)) + 1

            states.append(next_state)
            current = next_state

        # Append the generated states to the global state list used by simulation
        # and GIF rendering.
        append_states_to_list_ver(state_list, states, replanning_frequency)

        # The segment end becomes state_current for the next state-machine step.
        next_state = states[-1]

        # Log continuity of velocity, heading, and steering through this segment.
        print(
            f"[fallback-brake] v={float(state_current.velocity):.2f} -> "
            f"{float(next_state.velocity):.2f} m/s, "
            f"orientation={float(state_current.orientation):.3f} -> "
            f"{float(next_state.orientation):.3f} rad, "
            f"steering={float(getattr(state_current, 'steering_angle', 0.0)):.3f} -> "
            f"{float(getattr(next_state, 'steering_angle', 0.0)):.3f} rad"
        )
        return next_state

    def _plan_and_optimize(self, planner: CommonRoadReactivePlanner, config: Any, state_list: list,
                           is_stopping: bool = False) -> Tuple[State, object]:
        """Execute one CommonRoad reactive-planner cycle."""
        state_name = self.get_current_state_name()

        # Sampling levels are zero-based internally:
        # sampling_profile order is (time_level, longitudinal_level, lateral_level).
        sampling_level = 0
        sampling_profile = (0, 0, 0)

        t0 = time.perf_counter()

        # First attempt: time, longitudinal, and lateral dimensions use Level 1.
        # A single planner.plan() returning None only means this sampling profile failed.
        trajectory = planner.plan(
            current_sampling_level=sampling_level,
            sampling_profile=sampling_profile,
        )

        # BEFORE_STOPPING_MERGE-specific retry: keep time and longitudinal at
        # Level 1, but densify lateral samples to Level 3. This adds lateral
        # shape options for kappa/kappa_dot feasibility without densifying every
        # dimension.
        if trajectory is None and state_name == "BEFORE_STOPPING_MERGE":
            sampling_profile = (0, 0, 2)
            trajectory = planner.plan(
                current_sampling_level=sampling_level,
                sampling_profile=sampling_profile,
            )

        # If the low-cost attempts fail, raise all dimensions to Level 2.
        if trajectory is None:
            sampling_level = 1
            sampling_profile = (1, 1, 1)
            trajectory = planner.plan(
                current_sampling_level=sampling_level,
                sampling_profile=sampling_profile,
            )
        base_plan_time = time.perf_counter() - t0

        # Only when all attempts return None do we treat the cycle as imminent
        # collision risk: no collision-free and dynamically feasible trajectory.
        if trajectory is None:
            reason = "imminent_collision_risk: no collision-free and dynamically feasible trajectory"
            self._record_fallback_diagnostics(planner, config, reason)
            logger.warning("Imminent collision risk; starting EMERGENCY_BRAKE controlled fallback")

            self._enter_emergency_brake(config, planner.x_0, reason)

            next_state = self._execute_fallback_braking(planner.x_0, state_list)

            self.fallback_logs[-1]["result"] = "controlled braking to standstill"

            self.timing_logs.append({
                "state": state_name,
                "sampling_level": sampling_level + 1,
                "sampling_profile": sampling_profile,
                "plan_time": base_plan_time,
                "total_time": base_plan_time,
                "Treplan": config.planning.replanning_frequency,
                "T": config.planning.time_steps_computation * config.planning.dt,
                "fallback_braking": True,
                "emergency_brake": True,
            })

            # A None trajectory means this step used state-machine braking.
            return next_state, None

        selected_trajectory = getattr(planner, "best_sample", None)
        if self._selected_trajectory_has_imminent_collision_risk(planner, config, selected_trajectory):
            reason = "imminent_collision_risk: selected trajectory collides within emergency horizon"
            self._record_fallback_diagnostics(planner, config, reason)
            logger.warning("Selected trajectory has imminent collision risk; starting EMERGENCY_BRAKE")
            self._enter_emergency_brake(config, planner.x_0, reason)
            next_state = self._execute_fallback_braking(planner.x_0, state_list)
            self.fallback_logs[-1]["result"] = "controlled braking to standstill"
            self.timing_logs.append({
                "state": state_name,
                "sampling_level": sampling_level + 1,
                "sampling_profile": sampling_profile,
                "plan_time": base_plan_time,
                "total_time": base_plan_time,
                "Treplan": config.planning.replanning_frequency,
                "T": config.planning.time_steps_computation * config.planning.dt,
                "fallback_braking": True,
                "emergency_brake": True,
            })
            return next_state, None

        # When any sampling attempt succeeds, execute through the
        # replanning_frequency endpoint and append intermediate dt states for
        # continuous simulation and GIF rendering.
        next_state = trajectory[0].state_list[config.planning.replanning_frequency]

        append_states_to_list_ver(
            state_list,
            trajectory[0].state_list,
            config.planning.replanning_frequency
        )

        self.timing_logs.append({
            "state": state_name,
            "sampling_level": sampling_level + 1,
            "sampling_profile": sampling_profile,
            "plan_time": base_plan_time,
            "total_time": base_plan_time,
            "Treplan": config.planning.replanning_frequency,
            "T": config.planning.time_steps_computation * config.planning.dt
        })
        self._last_planner = planner
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
            "emergency_horizon": self._emergency_horizon_seconds(config),
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

    def _emergency_recovery_allowed(self, state_current: State, restart_state: PlannerState) -> bool:
        """Return True when stopped ego can safely restart in the selected normal state."""
        anchors = (
            self._before_stopping_entry_pose,
            self._before_stopping_entry_heading,
            self._departing_entry_pose,
        )
        try:
            imminent_collision_risk = self._probe_imminent_collision_risk(state_current, restart_state)
        finally:
            (
                self._before_stopping_entry_pose,
                self._before_stopping_entry_heading,
                self._departing_entry_pose,
            ) = anchors

        return not imminent_collision_risk

    def _probe_imminent_collision_risk(
            self,
            state_current: State,
            restart_state: Optional[PlannerState] = None,
    ) -> bool:
        """Evaluate imminent_collision_risk for recovery without appending trajectory states."""
        state = restart_state if restart_state is not None else self.state_before_emergency
        if state is None:
            return False

        if isinstance(self, BusStopBayPlanner) and isinstance(state, StoppingState):
            return False

        active_state_before_probe = self._active_planning_state_name
        try:
            coord_sys = self._get_coord_system_for_state(state, state_current)
            if coord_sys is None:
                return False

            planner, config = self._create_planner(state, coord_sys, state_current)
            self._configure_planner_for_state(planner, config, state, state_current)
            trajectory = self._run_planner_sampling_attempts(planner, state.get_state_name())
            if trajectory is None:
                return True
            return self._selected_trajectory_has_imminent_collision_risk(
                planner,
                config,
                getattr(planner, "best_sample", None),
                conservative_on_error=True,
            )
        except Exception as exc:
            logger.warning("Emergency recovery probe failed; keeping EMERGENCY_BRAKE active: %s", exc)
            return True
        finally:
            self._active_planning_state_name = active_state_before_probe

    def _get_coord_system_for_state(self, state: PlannerState, state_current: State) -> Optional[CoordinateSystem]:
        if isinstance(self, BusStopBayPlanner):
            if isinstance(state, DepartingState):
                return self._get_departing_coord_system(state_current)
            if isinstance(state, BeforeStoppingState):
                return self._get_before_stopping_coord_system(state, state_current)
        return state.get_coordinate_system(self.scenario, self.planning_problem)

    def _configure_planner_for_state(
            self,
            planner: CommonRoadReactivePlanner,
            config: Any,
            state: PlannerState,
            state_current: State,
    ) -> None:
        """Apply state-specific target settings to a planner used for recovery probing."""
        if isinstance(state, DepartingState):
            if self.scenario_type == "bus_stop_bulb":
                planner._low_vel_mode = False
            elif self.scenario_type == "bus_stop_bay":
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
            return

        if isinstance(state, StoppingState):
            planner._low_vel_mode = True
            planner._desired_speed = 0
            planner.set_desired_lon_position(lon_position=self.lon_goal)
            return

        if isinstance(state, BeforeStoppingFinalState):
            planner._low_vel_mode = True
            planner._desired_speed = 0
            goal_s, _ = planner.coordinate_system.convert_to_curvilinear_coords(self.goal_x, self.goal_y)
            planner.set_desired_lon_position(lon_position=goal_s)
            return

        desired_velocity = config.sampling.desire_velocity
        if (
            isinstance(state, BeforeStoppingMergeState)
            and self._before_stopping_entry_pose is not None
        ):
            entry_y = self._before_stopping_entry_pose[1]
            lateral_distance = abs(float(self.goal_y) - entry_y)
            if lateral_distance > 1e-6:
                lateral_progress = abs(float(state_current.position[1]) - entry_y) / lateral_distance
                if lateral_progress >= 0.30:
                    desired_velocity = min(2.6, float(config.sampling.v_max))

        planner._desired_speed = desired_velocity
        planner.set_desired_velocity(
            current_speed=state_current.velocity,
            desired_velocity=desired_velocity,
        )

    def _run_planner_sampling_attempts(
            self,
            planner: CommonRoadReactivePlanner,
            state_name: str,
    ) -> Optional[Tuple]:
        sampling_level = 0
        sampling_profile = (0, 0, 0)
        trajectory = planner.plan(
            current_sampling_level=sampling_level,
            sampling_profile=sampling_profile,
        )

        if trajectory is None and state_name == "BEFORE_STOPPING_MERGE":
            trajectory = planner.plan(
                current_sampling_level=sampling_level,
                sampling_profile=(0, 0, 2),
            )

        if trajectory is None:
            trajectory = planner.plan(
                current_sampling_level=1,
                sampling_profile=(1, 1, 1),
            )

        return trajectory

    def _selected_trajectory_has_imminent_collision_risk(
            self,
            planner: CommonRoadReactivePlanner,
            config: Any,
            trajectory_sample: Any,
            conservative_on_error: bool = False,
    ) -> bool:
        """Check whether the selected ego trajectory collides inside the emergency horizon."""
        if trajectory_sample is None or getattr(trajectory_sample, "cartesian", None) is None:
            return False

        cartesian = trajectory_sample.cartesian
        horizon_steps = self._emergency_horizon_steps(config)
        steps = min(cartesian.length(), horizon_steps)
        if steps <= 0:
            return False

        collision_checker = planner.collision_checker
        if collision_checker is None:
            return False

        vehicle_params = planner.vehicle_params
        half_length = 0.5 * float(vehicle_params.length)
        half_width = 0.5 * float(vehicle_params.width)
        rear_axle_offset = float(getattr(vehicle_params, "wb_rear_axle", 0.0))
        initial_time_step = int(getattr(planner.x_0, "time_step", 0))

        for i in range(steps):
            theta = float(cartesian.theta[i])
            pos_x = float(cartesian.x[i]) + rear_axle_offset * np.cos(theta)
            pos_y = float(cartesian.y[i]) + rear_axle_offset * np.sin(theta)
            ego_collision_rect = pycrcc.RectOBB(half_length, half_width, theta, pos_x, pos_y)
            try:
                if collision_checker.time_slice(initial_time_step + i).collide(ego_collision_rect):
                    return True
            except Exception as exc:
                logger.warning("Emergency horizon collision probe failed at step %s: %s", i, exc)
                return conservative_on_error

        return False

    @staticmethod
    def _emergency_horizon_seconds(config: Any) -> float:
        dt = max(1e-6, float(config.planning.dt))
        explicit_horizon = getattr(config.planning, "emergency_horizon", None)
        if explicit_horizon is None:
            explicit_horizon = getattr(config.planning, "emergency_brake_horizon", None)
        if explicit_horizon is not None:
            return max(dt, float(explicit_horizon))

        v_max = getattr(config.sampling, "v_max", None)
        if v_max is None:
            v_max = getattr(config.sampling, "desire_velocity", None)
        if v_max is None:
            return float(config.planning.time_steps_computation) * dt

        a_max = max(0.1, float(config.vehicle.a_max))
        a_brake = min(2.0, a_max)
        safe_margin = 1.0
        return max(dt, float(v_max) / a_brake + safe_margin)

    @classmethod
    def _emergency_horizon_steps(cls, config: Any) -> int:
        dt = max(1e-6, float(config.planning.dt))
        horizon = cls._emergency_horizon_seconds(config)
        return max(1, int(np.ceil(float(horizon) / dt)) + 1)

    def _check_state_transition(self, next_state: State, config: Any) -> None:
        """Check if state transition is needed using state class method"""
        # Keep the logical state fixed during fallback braking so the reference
        # path and config do not change mid-brake.
        if self._fallback_braking_active:
            return
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
        self.state_before_emergency = None
        self._fallback_braking_active = False
        self._fallback_braking_params = None
        self._emergency_standstill_cycles = 0
        self._clear_motion_phase_anchors()
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
        if self._stopped_outside_service_window(next_state):
            pos = next_state.position
            raise VehicleLeftScenarioError(
                "STOPPING reached standstill outside the bulb service window "
                f"at position ({pos[0]:.2f}, {pos[1]:.2f}); aborting to avoid a dead loop."
            )
        return next_state

    def _stopped_outside_service_window(self, state_current: State) -> bool:
        """Return True if STOPPING can no longer make progress into the service window."""
        ego_v = abs(float(getattr(state_current, "velocity", 0.0)))
        if ego_v >= 1e-5:
            return False
        return float(state_current.position[0]) > float(self.goal_boundary)

    def _check_stopping_condition(self, state_current: State, state_list: list) -> bool:
        """Check stopping condition for bulb scenario"""
        ego_x = state_current.position[0]
        ego_v = state_current.velocity

        condition = ego_x <= self.goal_boundary and ego_v < 1e-5

        if condition:
            append_state_to_list(state_list, state_current)
            self.stopping_counter = 1
            self.stopping_next_state = state_current
            return True

        return False


class BusStopBayPlanner(BaseStateMachinePlanner):
    """
    State machine planner for bus stop bay scenarios.
    State flow: DEPARTING → HEADING → ARRIVING → BEFORE_STOPPING_MERGE
                → BEFORE_STOPPING_ALIGN → BEFORE_STOPPING_FINAL → STOPPING
    """

    MISSION_COMPLETE_DISTANCE = 80.0

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
        progress = progress ** 3 * (10.0 + progress * (-15.0 + 6.0 * progress))
        y_values = start_y * (1.0 - progress) + target_y_values * progress
        return np.column_stack((x_values, y_values))

    def _execute_departing(self, state_current: State, state_list: list) -> State:
        """Execute DEPARTING state for bay scenario"""
        coord_sys = self._get_departing_coord_system(state_current)
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
        """Execute staged BEFORE_STOPPING states for bay scenario."""
        coord_sys = self._get_before_stopping_coord_system(self.current_state, state_current)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)

        if isinstance(self.current_state, BeforeStoppingFinalState):
            planner._low_vel_mode = True
            planner._desired_speed = 0
            goal_s, _ = planner.coordinate_system.convert_to_curvilinear_coords(self.goal_x, self.goal_y)
            planner.set_desired_lon_position(lon_position=goal_s)
        else:
            desired_velocity = config.sampling.desire_velocity
            if (
                isinstance(self.current_state, BeforeStoppingMergeState)
                and self._before_stopping_entry_pose is not None
            ):
                # Keep the early merge slow to avoid dynamic constraints error, then speed up inside the bay.
                entry_y = self._before_stopping_entry_pose[1]
                lateral_distance = abs(float(self.goal_y) - entry_y)
                if lateral_distance > 1e-6:
                    lateral_progress = abs(float(state_current.position[1]) - entry_y) / lateral_distance
                    if lateral_progress >= 0.30:
                        desired_velocity = min(2.6, float(config.sampling.v_max))

            planner._desired_speed = desired_velocity
            planner.set_desired_velocity(
                current_speed=state_current.velocity,
                desired_velocity=desired_velocity,
            )

        fallback_count_before = len(self.fallback_logs)
        next_state, _ = self._plan_and_optimize(planner, config, state_list)
        if len(self.fallback_logs) > fallback_count_before:
            self.fallback_logs[-1]["distance_to_goal_x"] = abs(
                float(self.goal_x) - float(state_current.position[0])
            )
        self._check_state_transition(next_state, config)

        return next_state

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

        if isinstance(from_state, BeforeStoppingFinalState) and isinstance(to_state, StoppingState):
            steering = abs(float(getattr(next_state, "steering_angle", 0.0)))
            velocity = float(getattr(next_state, "velocity", 0.0))
            longitudinal_goal_error = abs(float(next_state.position[0]) - float(self.goal_x))
            lateral_goal_error = abs(float(next_state.position[1]) - float(self.goal_y))
            if longitudinal_goal_error > 1.5:
                print(
                    f"[transition] BEFORE_STOPPING_FINAL held: longitudinal goal error "
                    f"{longitudinal_goal_error:.2f} m > 1.50 m"
                )
                return False
            if velocity > 0.55:
                print(f"[transition] BEFORE_STOPPING_FINAL held: velocity {velocity:.2f} m/s > 0.55 m/s")
                return False
            if steering > 0.12:
                print(f"[transition] BEFORE_STOPPING_FINAL held: steering {steering:.3f} rad > 0.120 rad")
                return False
            if lateral_goal_error > 1.2:
                print(
                    f"[transition] BEFORE_STOPPING_FINAL held: lateral goal error "
                    f"{lateral_goal_error:.2f} m > 1.20 m"
                )
                return False

        return True

    def _execute_direct_stop_braking(self, state_current: State, state_list: list, config: Any) -> State:
        """Brake monotonically in STOPPING without chasing a longitudinal target."""
        dt = float(config.planning.dt)
        replanning_frequency = int(config.planning.replanning_frequency)
        max_deceleration = min(1.2, max(0.1, float(config.vehicle.a_max)))
        max_steering_step = max(0.0, float(config.vehicle.v_delta_max)) * dt
        wheelbase = max(1e-6, float(config.vehicle.wheelbase))

        states = [copy.deepcopy(state_current)]
        current = states[0]
        for _ in range(replanning_frequency):
            velocity = max(0.0, float(getattr(current, "velocity", 0.0)))
            orientation = float(getattr(current, "orientation", 0.0))
            steering = float(getattr(current, "steering_angle", 0.0))

            next_velocity = max(0.0, velocity - max_deceleration * dt)
            if abs(steering) <= max_steering_step:
                next_steering = 0.0
            else:
                next_steering = steering - np.sign(steering) * max_steering_step

            mean_velocity = 0.5 * (velocity + next_velocity)
            mean_steering = 0.5 * (steering + next_steering)
            mean_yaw_rate = mean_velocity * np.tan(mean_steering) / wheelbase
            orientation_change = mean_yaw_rate * dt
            midpoint_orientation = orientation + 0.5 * orientation_change
            next_position = np.asarray(current.position, dtype=float) + mean_velocity * dt * np.array([
                np.cos(midpoint_orientation),
                np.sin(midpoint_orientation),
            ])
            next_orientation = np.arctan2(
                np.sin(orientation + orientation_change),
                np.cos(orientation + orientation_change),
            )

            next_state = copy.deepcopy(current)
            next_state.position = next_position
            next_state.velocity = next_velocity
            next_state.acceleration = -max_deceleration if next_velocity > 0.0 else 0.0
            next_state.orientation = next_orientation
            next_state.steering_angle = next_steering
            next_state.yaw_rate = next_velocity * np.tan(next_steering) / wheelbase
            if hasattr(next_state, "slip_angle"):
                next_state.slip_angle = 0.0
            next_state.time_step = int(getattr(current, "time_step", 0)) + 1

            states.append(next_state)
            current = next_state

        append_states_to_list_ver(state_list, states, replanning_frequency)
        next_state = states[-1]
        print(
            f"[direct-stop] v={float(state_current.velocity):.2f} -> "
            f"{float(next_state.velocity):.2f} m/s"
        )
        return next_state

    def _execute_stopping(self, state_current: State, state_list: list) -> State:
        """Execute STOPPING state for bay scenario"""
        # Check if stopping condition is already met
        if self._check_stopping_condition(state_current, state_list):
            return state_current

        config = ReactivePlannerConfiguration.load(str(self.config_root / self.current_state.get_config_filename()))
        config.update(self.scenario, planning_problem=self.planning_problem)
        next_state = self._execute_direct_stop_braking(state_current, state_list, config)
        if self._stopped_outside_service_window(next_state):
            pos = next_state.position
            raise VehicleLeftScenarioError(
                "STOPPING reached standstill outside the bay service window "
                f"at position ({pos[0]:.2f}, {pos[1]:.2f}); aborting to avoid a dead loop."
            )
        return next_state

    def _stopped_outside_service_window(self, state_current: State) -> bool:
        """Return True if direct STOPPING braking can no longer enter the goal area."""
        ego_v = abs(float(getattr(state_current, "velocity", 0.0)))
        if ego_v >= 1e-5:
            return False
        return not self._is_inside_service_window(state_current)

    def _is_inside_service_window(self, state_current: State) -> bool:
        """Check the bay goal rectangle without mutating stop counters or state_list."""
        goal_state = self.planning_problem.goal.state_list[0]
        half_length = 0.5 * float(getattr(goal_state.position, "length", 24.0))
        half_width = 0.5 * float(getattr(goal_state.position, "width", 6.0))
        longitudinal_ok = abs(float(state_current.position[0]) - float(self.goal_x)) <= half_length
        lateral_ok = abs(float(state_current.position[1]) - float(self.goal_y)) <= half_width
        return longitudinal_ok and lateral_ok

    def _check_stopping_condition(self, state_current: State, state_list: list) -> bool:
        """Check stopping condition for bay scenario"""
        ego_v = state_current.velocity

        condition = self._is_inside_service_window(state_current) and ego_v < 1e-5

        if condition:
            append_state_to_list(state_list, state_current)
            self.stopping_counter = 1
            self.stopping_next_state = state_current
            return True

        return False

    def _get_before_stopping_coord_system(self, state: PlannerState, state_current: State) -> CoordinateSystem:
        """Create the staged reference path for the active BEFORE_STOPPING state."""
        if isinstance(state, BeforeStoppingAlignState):
            return self._get_goal_alignment_coord_system(state_current)
        if isinstance(state, BeforeStoppingMergeState):
            return self._get_before_stopping_merge_coord_system(state_current)
        if isinstance(state, BeforeStoppingFinalState):
            return self._get_before_stopping_final_coord_system(state_current)
        return state.get_coordinate_system(self.scenario, self.planning_problem)

    def _get_goal_alignment_coord_system(self, state_current: State) -> CoordinateSystem:
        """Blend from the current pose into a horizontal stop line inside the bay."""
        x0, y0 = map(float, state_current.position)
        theta = float(getattr(state_current, "orientation", 0.0))
        slope0 = float(np.clip(np.tan(theta), -0.45, 0.45))

        align_end_x = max(float(self.goal_x) + 4.0, x0 + 12.0)
        start_x = x0 - 20.0
        end_x = align_end_x + 70.0
        n_pts = max(160, int((end_x - start_x) * 10))
        x_values = np.linspace(start_x, end_x, n_pts)
        y_values = np.empty_like(x_values)

        before = x_values < x0
        y_values[before] = y0 + slope0 * (x_values[before] - x0)

        blend = (x_values >= x0) & (x_values <= align_end_x)
        u = (x_values[blend] - x0) / max(align_end_x - x0, 1e-6)
        h00 = 2.0 * u ** 3 - 3.0 * u ** 2 + 1.0
        h10 = u ** 3 - 2.0 * u ** 2 + u
        h01 = -2.0 * u ** 3 + 3.0 * u ** 2
        dx = align_end_x - x0
        y_values[blend] = h00 * y0 + h10 * dx * slope0 + h01 * float(self.goal_y)

        after = x_values > align_end_x
        y_values[after] = float(self.goal_y)
        return create_coordinate_system(np.column_stack((x_values, y_values)))

    def _get_bay_stop_line_coord_system(self, state_current: State) -> CoordinateSystem:
        """Use the horizontal bay stop line as the final low-speed reference."""
        x0 = float(state_current.position[0])
        x_values = np.linspace(x0 - 30.0, float(self.goal_x) + 90.0, 500)
        y_values = np.full_like(x_values, float(self.goal_y))
        return create_coordinate_system(np.column_stack((x_values, y_values)))

    def _get_entry_to_alignment_coord_system(
            self,
            entry_x: float,
            entry_y: float,
            entry_heading: float,
    ) -> CoordinateSystem:
        """Merge into the bay without forcing a horizontal pose at the handoff."""
        goal_y = float(self.goal_y)
        lateral_distance = abs(goal_y - entry_y)
        handoff_x = min(float(self.goal_x) - 14.0, entry_x + 42.0)
        align_end_x = max(float(self.goal_x) + 4.0, handoff_x + 14.0)
        handoff_y = goal_y + np.sign(entry_y - goal_y) * min(1.5, lateral_distance * 0.22)
        slope0 = float(np.clip(np.tan(entry_heading), -0.22, -0.04))
        slope1 = -0.14 if goal_y < entry_y else 0.14

        start_x = entry_x - 20.0
        end_x = align_end_x + 70.0
        n_pts = max(220, int((end_x - start_x) * 10))
        x_values = np.linspace(start_x, end_x, n_pts)
        y_values = np.empty_like(x_values)

        before = x_values < entry_x
        y_values[before] = entry_y + slope0 * (x_values[before] - entry_x)

        first = (x_values >= entry_x) & (x_values <= handoff_x)
        u = (x_values[first] - entry_x) / max(handoff_x - entry_x, 1e-6)
        h00 = 2.0 * u ** 3 - 3.0 * u ** 2 + 1.0
        h10 = u ** 3 - 2.0 * u ** 2 + u
        h01 = -2.0 * u ** 3 + 3.0 * u ** 2
        h11 = u ** 3 - u ** 2
        dx = handoff_x - entry_x
        y_values[first] = h00 * entry_y + h10 * dx * slope0 + h01 * handoff_y + h11 * dx * slope1

        second = (x_values > handoff_x) & (x_values <= align_end_x)
        u = (x_values[second] - handoff_x) / max(align_end_x - handoff_x, 1e-6)
        h00 = 2.0 * u ** 3 - 3.0 * u ** 2 + 1.0
        h10 = u ** 3 - 2.0 * u ** 2 + u
        h01 = -2.0 * u ** 3 + 3.0 * u ** 2
        h11 = u ** 3 - u ** 2
        dx = align_end_x - handoff_x
        y_values[second] = h00 * handoff_y + h10 * dx * slope1 + h01 * goal_y + h11 * dx * 0.0

        after = x_values > align_end_x
        y_values[after] = goal_y
        return create_coordinate_system(np.column_stack((x_values, y_values)))

    def _get_before_stopping_merge_coord_system(self, state_current: State) -> CoordinateSystem:
        """Use a long, curvature-friendly lateral shift from current lane to goal_y."""
        if self._before_stopping_entry_pose is None:
            self._before_stopping_entry_pose = (
                float(state_current.position[0]),
                float(state_current.position[1]),
                max(0.0, float(state_current.velocity)),
            )
            self._before_stopping_entry_heading = float(getattr(state_current, "orientation", 0.0))
        entry_x, entry_y, _ = self._before_stopping_entry_pose
        return self._get_entry_to_alignment_coord_system(
            entry_x=entry_x,
            entry_y=entry_y,
            entry_heading=float(self._before_stopping_entry_heading or 0.0),
        )

    def _get_before_stopping_final_coord_system(self, state_current: State) -> CoordinateSystem:
        """Brake on the bay stop line after ALIGN has settled the pose."""
        return self._get_bay_stop_line_coord_system(state_current)

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
        # Use a shorter departure merge to leave DEPARTING sooner without changing the target lane.
        merge_end_x = merge_start_x + 40.0
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
        return distance_after_stop > self.MISSION_COMPLETE_DISTANCE


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
