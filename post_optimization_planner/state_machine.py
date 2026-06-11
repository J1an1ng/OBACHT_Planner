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
    BeforeStoppingState, BeforeStoppingAlignState, BeforeStoppingMergeState,
    BeforeStoppingFinalState, StoppingState, create_initial_state,
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
        # True 表示常规规划已经失败，当前由状态机直接生成受控制动轨迹。
        self._fallback_braking_active = False
        # 保存一次 fallback 全程固定使用的离散步长、车辆轴距和控制上限。
        self._fallback_braking_params: Optional[Dict[str, float]] = None
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

        # fallback 激活后，后续 step 不再调用常规 planner，直到车辆完全停稳。
        if self._fallback_braking_active:
            # 速度、加速度和转角均归零后，制动阶段结束。
            if self._fallback_braking_complete(state_current):
                # 退出 fallback 模式；本 step 随后继续执行当前状态的常规规划 handler。
                self._fallback_braking_active = False
                self._fallback_braking_params = None
                # state_current 就是停车末状态，也会成为新 planner 的 x_0。
                print(
                    f"[fallback-brake] Restart x0: "
                    f"position=({state_current.position[0]:.3f}, "
                    f"{state_current.position[1]:.3f}), "
                    f"orientation={float(state_current.orientation):.6f} rad, "
                    f"velocity={float(state_current.velocity):.3f} m/s, "
                    f"steering={float(getattr(state_current, 'steering_angle', 0.0)):.6f} rad; "
                    f"restarting {self.get_current_state_name()} planning"
                )
            else:
                # 尚未停稳：只执行下一个制动时间段，并立即返回制动后的状态。
                return self._execute_fallback_braking(state_current, state_list)

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

    def _activate_fallback_braking(self, config: Any, state_current: State) -> None:
        """Freeze fallback parameters at the trigger state and enter braking mode."""
        # 读取车辆配置的最大加速度绝对值，并避免异常的零上限
        a_max = max(0.1, float(config.vehicle.a_max))

        # 固定本次 fallback 使用的车辆参数和控制上限；运动状态始终从每一帧的 current state 继续积分。
        self._fallback_braking_params = {
            "dt": float(config.planning.dt),
            "replanning_frequency": int(config.planning.replanning_frequency),
            # 紧急制动减速度不超过车辆 a_max
            "max_deceleration": min(2.0, a_max),
            # 每个 dt 内允许的最大方向盘转角变化由 v_delta_max 决定
            "max_steering_rate": max(0.0, float(config.vehicle.v_delta_max)),
            # 自行车模型用轴距把方向盘转角转换成曲率和横摆角速度。
            "wheelbase": max(1e-6, float(config.vehicle.wheelbase)),
        }

        # 从下一次 step 开始，状态机将不再调用 planner，而是直接生成受控制的制动状态，直到车辆完全停稳
        self._fallback_braking_active = True

    @staticmethod
    def _fallback_braking_complete(state: State) -> bool:
        """Return True only when the vehicle and steering have settled."""
        return (
            # 车辆纵向速度已经完全停稳，允许微小数值误差
            abs(float(getattr(state, "velocity", 0.0))) <= 1e-3
            # 制动结束后加速度也必须恢复为 0
            and abs(float(getattr(state, "acceleration", 0.0))) <= 1e-3
            # 方向盘转角必须回正
            and abs(float(getattr(state, "steering_angle", 0.0))) <= 1e-3
        )

    def _execute_fallback_braking(self, state_current: State, state_list: list) -> State:
        """Generate one dynamically consistent emergency-braking segment."""
        if self._fallback_braking_params is None:
            raise RuntimeError("Fallback braking is active without braking parameters")

        # 读取激活 fallback 时冻结的参数，确保整个制动过程使用同一车辆模型和控制上限。
        params = self._fallback_braking_params
        dt = params["dt"]
        max_deceleration = params["max_deceleration"]
        wheelbase = params["wheelbase"]
        # 将最大转角速度转换为每个 dt 内的最大转角变化，确保制动过程中方向盘逐步回正，而不是数值瞬间跳到 0
        max_steering_step = params["max_steering_rate"] * dt
        replanning_frequency = int(params["replanning_frequency"])

        # 第一项是 fallback 触发时的真实车辆状态；后续状态都从前一项连续积分得到。
        states = [copy.deepcopy(state_current)]
        current = states[0]

        # 一次调用生成 replanning_frequency 个未来状态
        for _ in range(replanning_frequency):
            # 读取本步初始速度、车身朝向和方向盘转角，并防止数值误差产生负速度
            velocity = max(0.0, float(getattr(current, "velocity", 0.0)))
            orientation = float(getattr(current, "orientation", 0.0))
            steering = float(getattr(current, "steering_angle", 0.0))

            #按固定最大减速度更新纵向速度，且速度最低只能到 0
            next_velocity = max(0.0, velocity - max_deceleration * dt)

            # 按方向盘转角速率限制逐步回正，避免 steering_angle 瞬间跳变
            if abs(steering) <= max_steering_step:
                next_steering = 0.0
            else:
                next_steering = steering - np.sign(steering) * max_steering_step

            # 使用本 dt 前后的平均速度和平均转角进行梯形积分
            mean_velocity = 0.5 * (velocity + next_velocity)
            mean_steering = 0.5 * (steering + next_steering)
            displacement = mean_velocity * dt

            # 无侧滑自行车模型 kappa=tan(delta)/wheelbase，yaw_rate=v*kappa。转角回正时，yaw_rate 也会逐步回正
            mean_yaw_rate = mean_velocity * np.tan(mean_steering) / wheelbase
            orientation_change = mean_yaw_rate * dt

            # 用本 dt 中点朝向积分位置变化，避免朝向变化过大时位置积分误差过大；同时确保朝向在 [-pi, pi] 范围内
            midpoint_orientation = orientation + 0.5 * orientation_change
            next_position = np.asarray(current.position, dtype=float) + displacement * np.array([
                np.cos(midpoint_orientation),
                np.sin(midpoint_orientation),
            ])

            # 积分得到本 dt 末朝向，并确保在 [-pi, pi] 范围内；这样可以避免朝向数值过大时的异常情况，同时也让后续 planner 的 x_0 朝向连续于制动前状态，避免 planner 因为朝向突变而无法找到可行轨迹
            unwrapped_orientation = orientation + orientation_change
            next_orientation = np.arctan2(
                np.sin(unwrapped_orientation),
                np.cos(unwrapped_orientation),
            )

            #末端 yaw_rate 必须与末端速度和转角一致，确保整个制动轨迹在动力学上连续可行
            next_yaw_rate = next_velocity * np.tan(next_steering) / wheelbase

            next_state = copy.deepcopy(current)

            # 将离散自行车模型计算出的下一个状态的 position、velocity、acceleration、orientation、steering_angle 和 yaw_rate 更新到next state
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

        # 将新生成的状态追加到全局 state_list，供仿真和 GIF 使用
        append_states_to_list_ver(state_list, states, replanning_frequency)

        # 返回本次制动时间段的末状态，作为下一次状态机 step 的 state_current
        next_state = states[-1]

        # 输出本段制动前后的速度、朝向和转角，便于检查三个量是否连续变化
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

        # Sampling level 在代码内部从 0 开始计数：
        # sampling_profile 的顺序是 (time_level, longitudinal_level, lateral_level)。
        sampling_level = 0
        sampling_profile = (0, 0, 0)

        t0 = time.perf_counter()

        # 第一次尝试：时间、纵向和横向都使用 Level 1
        # 单次 planner.plan() 返回 None 只表示当前 sampling 配置失败
        trajectory = planner.plan(
            current_sampling_level=sampling_level,
            sampling_profile=sampling_profile,
        )

        # BEFORE_STOPPING_MERGE的专项重试：
        # 保持时间和纵向为 Level 1，只将横向加密到 Level 3
        # 这样可以增加不同横向轨迹形状，改善 kappa/kappa_dot 可行性
        # 同时避免三个维度全部加密带来的大规模采样开销
        if trajectory is None and state_name == "BEFORE_STOPPING_MERGE":
            sampling_profile = (0, 0, 2)
            trajectory = planner.plan(
                current_sampling_level=sampling_level,
                sampling_profile=sampling_profile,
            )

        # 如果前面的低成本采样仍然失败，三个维度统一升级到 Level 2
        # 只要这里的 Level 2 成功，该 planning cycle 就正常继续
        if trajectory is None:
            sampling_level = 1
            sampling_profile = (1, 1, 1)
            trajectory = planner.plan(
                current_sampling_level=sampling_level,
                sampling_profile=sampling_profile,
            )
        base_plan_time = time.perf_counter() - t0

        # 只有所有上述采样尝试都返回 None，才进入真正的车辆 fallback
        if trajectory is None:
            # 保存无可行轨迹的原因、位置、速度和约束失败统计。
            self._record_fallback_diagnostics(planner, config, "planner returned no trajectory")
            logger.warning("Planner returned no trajectory; starting controlled braking fallback")

            # 固定车辆模型和控制上限，并将状态机切换到 fallback braking 模式。
            self._activate_fallback_braking(config, planner.x_0)

            # 在当前 planning cycle 内立即按自行车模型生成第一段制动轨迹。
            next_state = self._execute_fallback_braking(planner.x_0, state_list)

            # 记录本次 planning cycle 的最终处理结果。
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
            })

            # trajectory 返回 None，表示该 step 执行的是状态机生成的制动状态。
            return next_state, None

        # 任一 sampling 尝试成功后，取 replanning_frequency 对应的执行终点，
        # 并把中间的每个 dt 状态追加到完整轨迹，供仿真和 GIF 连续显示。
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
        # fallback 刹停期间保持原状态机阶段，避免制动中途切换规划参考线或配置。
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
    State flow: DEPARTING → HEADING → ARRIVING → BEFORE_STOPPING_ALIGN
                → BEFORE_STOPPING_MERGE → BEFORE_STOPPING_FINAL → STOPPING
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
        progress = progress ** 3 * (10.0 + progress * (-15.0 + 6.0 * progress))
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
        """Execute staged BEFORE_STOPPING states for bay scenario."""
        coord_sys = self._get_before_stopping_coord_system(self.current_state, state_current)
        planner, config = self._create_planner(self.current_state, coord_sys, state_current)

        if isinstance(self.current_state, BeforeStoppingFinalState):
            planner._low_vel_mode = True
            planner._desired_speed = 0
            goal_s, _ = planner.coordinate_system.convert_to_curvilinear_coords(self.goal_x, self.goal_y)
            planner.set_desired_lon_position(lon_position=goal_s)
        else:
            planner._desired_speed = config.sampling.desire_velocity
            planner.set_desired_velocity(
                current_speed=state_current.velocity,
                desired_velocity=config.sampling.desire_velocity,
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
            lateral_goal_error = abs(float(next_state.position[1]) - float(self.goal_y))
            if velocity > 0.8:
                print(f"[transition] BEFORE_STOPPING_FINAL held: velocity {velocity:.2f} m/s > 0.80 m/s")
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

    def _get_before_stopping_coord_system(self, state: PlannerState, state_current: State) -> CoordinateSystem:
        """Create the staged reference path for the active BEFORE_STOPPING state."""
        if isinstance(state, BeforeStoppingAlignState):
            return self._get_before_stopping_align_coord_system(state_current)
        if isinstance(state, BeforeStoppingMergeState):
            return self._get_before_stopping_merge_coord_system(state_current)
        if isinstance(state, BeforeStoppingFinalState):
            return self._get_before_stopping_final_coord_system(state_current)
        return state.get_coordinate_system(self.scenario, self.planning_problem)

    def _get_before_stopping_align_coord_system(self, state_current: State) -> CoordinateSystem:
        """Follow the current pose to settle steering/yaw before starting the merge."""
        x0, y0 = map(float, state_current.position)
        theta = float(getattr(state_current, "orientation", 0.0))
        s_values = np.linspace(-25.0, 100.0, 260)
        x_values = x0 + np.cos(theta) * s_values
        y_values = y0 + np.sin(theta) * s_values
        return create_coordinate_system(np.column_stack((x_values, y_values)))

    def _get_before_stopping_merge_coord_system(self, state_current: State) -> CoordinateSystem:
        """Use a long, curvature-friendly lateral shift from current lane to goal_y."""
        if self._before_stopping_entry_pose is None:
            self._before_stopping_entry_pose = (
                float(state_current.position[0]),
                float(state_current.position[1]),
                max(0.0, float(state_current.velocity)),
            )
        entry_x, entry_y, entry_v = self._before_stopping_entry_pose
        merge_start_x = entry_x + max(2.0, entry_v * 0.5)
        merge_end_x = max(merge_start_x + 50.0, float(self.goal_x) + 6.0)
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

    def _get_before_stopping_final_coord_system(self, state_current: State) -> CoordinateSystem:
        """Hold the current pose while targeting the stop position longitudinally."""
        return self._get_before_stopping_align_coord_system(state_current)

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
