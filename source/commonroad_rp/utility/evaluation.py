__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2024.1"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"


import warnings
from typing import List, Union

import numpy as np
from commonroad.common.solution import (
    CostFunction,
    PlanningProblemSolution,
    Solution,
    VehicleModel,
    VehicleType,
)
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState, InputState, State, TraceState
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.feasibility.feasibility_checker import (
    VehicleDynamics,
    _angle_diff,
    position_orientation_feasibility_criteria,
    position_orientation_objective,
    state_transition_feasibility,
)
from commonroad_dc.feasibility.solution_checker import valid_solution
from matplotlib import pyplot as plt

from commonroad_rp.reactive_planner import ReactivePlannerState
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.visualization import plot_final_trajectory


def run_evaluation(
    config: ReactivePlannerConfiguration,
    state_list: List[ReactivePlannerState],
    input_list: List[InputState],
):
    """
    Creates a CommonRoad solution Trajectory from the planning results, evaluates state and input feasibility, plots
    solution Trajectory
    :return cr_solution: Planner solution as CR solution object
    :return feasibility_list: List[Bool] indicating feasibility of each state transition
    """
    ego_vehicle_solution = create_full_solution_ego_vehicle(config, state_list)
    ego_solution_trajectory = ego_vehicle_solution.prediction.trajectory
    plot_final_trajectory(
        config.scenario,
        config.planning_problem,
        ego_vehicle_solution.prediction.trajectory.state_list,
        config,
        ego_vehicle=ego_vehicle_solution,
    )
    cr_solution, feasibility_list = evaluate_results(
        config, ego_solution_trajectory, input_list
    )

    return cr_solution, feasibility_list


def evaluate_results(
    config: ReactivePlannerConfiguration, ego_solution_trajectory, record_input_list
):
    """
    Solution is evaluated via input reconstruction from commonroad_dc.feasibility.feasibility_checker
    For each state transition the inputs are reconstructed. To check feasibility, the reconstructed input is used for
    forward simulation and the error between the forward simulated (reconstructed) state and the planned state is
    evaluated
    """
    # create CR solution
    solution = create_planning_problem_solution(
        config, ego_solution_trajectory, config.scenario, config.planning_problem
    )

    # check feasibility
    # reconstruct inputs (state transition optimizations)
    feasible, reconstructed_inputs = reconstruct_inputs(
        config, solution.planning_problem_solutions[0]
    )

    # reconstruct states from inputs
    reconstructed_states = reconstruct_states(
        config, ego_solution_trajectory.state_list, reconstructed_inputs
    )

    # check acceleration correctness
    check_acceleration(config, ego_solution_trajectory.state_list)

    # plot
    plot_states(
        config,
        ego_solution_trajectory.state_list,
        reconstructed_states,
        plot_bounds=False,
    )
    plot_inputs(config, record_input_list[1:], reconstructed_inputs, plot_bounds=True)

    # CR validity check
    print("Feasibility Check Result: ")
    print(valid_solution(config.scenario, config.planning_problem_set, solution))

    return solution, feasible


def create_full_solution_ego_vehicle(
    config: ReactivePlannerConfiguration, state_list: List[ReactivePlannerState]
) -> DynamicObstacle:
    """
    Create CR Dynamic obstacle for ego vehicle from the recorded state list of the reactive planner
    """
    ego_traj = create_full_solution_trajectory(config, state_list)
    ego_shape = Rectangle(config.vehicle.length, config.vehicle.width)
    ego_prediction = TrajectoryPrediction(ego_traj, ego_shape)
    ego_init_state = InitialState()
    ego_init_state = ego_traj.state_list[0].convert_state_to_state(ego_init_state)

    ego_vehicle = DynamicObstacle(
        obstacle_id=9999,
        obstacle_type=ObstacleType.CAR,
        obstacle_shape=ego_shape,
        initial_state=ego_init_state,
        prediction=ego_prediction,
    )
    return ego_vehicle


def create_full_solution_trajectory(
    config: ReactivePlannerConfiguration, state_list: List[ReactivePlannerState]
) -> Trajectory:
    """
    Create CR solution trajectory from recorded state list of the reactive planner
    Positions are shifted from rear axis to vehicle center due to CR position convention
    """
    new_state_list = list()
    for state in state_list:
        new_state_list.append(
            ReactivePlannerState.shift_state_to_center(
                state, config.vehicle.wb_rear_axle
            )
        )
    return Trajectory(
        initial_time_step=new_state_list[0].time_step, state_list=new_state_list
    )


def create_planning_problem_solution(
    config: ReactivePlannerConfiguration,
    solution_trajectory: Trajectory,
    scenario: Scenario,
    planning_problem: PlanningProblem,
) -> Solution:
    """
    Creates CommonRoad Solution object
    """
    pps = PlanningProblemSolution(
        planning_problem_id=planning_problem.planning_problem_id,
        vehicle_type=VehicleType(config.vehicle.id_type_vehicle),
        vehicle_model=VehicleModel.KS,
        cost_function=CostFunction.JB1,
        trajectory=solution_trajectory,
    )

    # create solution object
    solution = Solution(scenario.scenario_id, [pps])
    return solution


def reconstruct_states(
    config: ReactivePlannerConfiguration,
    states: List[Union[ReactivePlannerState, TraceState]],
    inputs: List[InputState],
):
    """reconstructs states from a given list of inputs by forward simulation"""
    vehicle_dynamics = VehicleDynamics.from_model(
        VehicleModel.KS, VehicleType(config.vehicle.id_type_vehicle)
    )

    x_sim_list = list()
    x_sim_list.append(states[0])
    for idx, inp in enumerate(inputs):
        x0, x0_ts = vehicle_dynamics.state_to_array(states[idx])
        u0 = vehicle_dynamics.input_to_array(inp)[0]
        x1_sim = vehicle_dynamics.forward_simulation(
            x0, u0, config.planning.dt, throw=False
        )
        x_sim_list.append(vehicle_dynamics.array_to_state(x1_sim, x0_ts + 1))
    return x_sim_list


def reconstruct_inputs(
    config: ReactivePlannerConfiguration, pps: PlanningProblemSolution
):
    """
    reconstructs inputs for each state transition using the feasibility checker
    """
    vehicle_dynamics = VehicleDynamics.from_model(pps.vehicle_model, pps.vehicle_type)

    feasible_state_list = []
    reconstructed_inputs = []
    for x0, x1 in zip(pps.trajectory.state_list[:-1], pps.trajectory.state_list[1:]):
        # reconstruct inputs using optimization
        feasible_state, reconstructed_input_state = state_transition_feasibility(
            x0,
            x1,
            vehicle_dynamics,
            config.planning.dt,
            position_orientation_objective,
            position_orientation_feasibility_criteria,
            1e-8,
            np.array([2e-2, 2e-2, 3e-2]),
            4,
            100,
            False,
        )
        feasible_state_list.append(feasible_state)
        reconstructed_inputs.append(reconstructed_input_state)
    return feasible_state_list, reconstructed_inputs


def check_acceleration(
    config: ReactivePlannerConfiguration,
    state_list: List[Union[ReactivePlannerState, TraceState]],
):
    """Checks whether the computed acceleration the trajectory matches the velocity difference (dv/dt), i.e., assuming
    piecewise constant acceleration input"""
    # computed acceleration of trajectory
    a_planned = np.array([state.acceleration for state in state_list])
    a_piecewise_constant = np.array(
        [(a_planned[i] + a_planned[i + 1]) / 2 for i in range(len(a_planned) - 1)]
    )
    # recalculated acceleration via velocity
    v = np.array([state.velocity for state in state_list])
    a_recalculated = np.diff(v) / config.planning.dt
    # check
    diff = np.abs(a_piecewise_constant - a_recalculated)
    acc_correct = np.all(diff < 1e-01)
    print(
        "Acceleration correct: %s, with max deviation %s" % (acc_correct, np.max(diff))
    )

    if config.debug.show_evaluation_plots:
        plt.figure(figsize=(7, 3.5))
        plt.suptitle("Acceleration check")
        plt.plot(
            list(range(len(a_planned[1:]))),
            a_planned[1:],
            color="black",
            label="planned acceleration",
        )
        plt.plot(
            list(range(len(a_piecewise_constant))),
            a_piecewise_constant,
            color="green",
            label="planned (piecewise constant)",
        )
        plt.plot(
            list(range(len(a_recalculated))),
            a_recalculated,
            color="orange",
            label="recomputed (dv/dt)",
        )
        plt.xlabel("t in s")
        plt.ylabel("a_long in m/s^2")
        plt.legend()
        plt.tight_layout()
        plt.show()


def plot_states(
    config: ReactivePlannerConfiguration,
    state_list: List[Union[ReactivePlannerState, TraceState]],
    reconstructed_states=None,
    plot_bounds=False,
):
    """
    Plots states of trajectory from a given state_list
    state_list must contain the following states: steering_angle, velocity, orientation and yaw_rate
    """
    plt.figure(figsize=(7, 8.0))
    plt.suptitle("States")

    # x, y position
    plt.subplot(5, 1, 1)
    plt.plot(
        [state.position[0] for state in state_list],
        [state.position[1] for state in state_list],
        color="black",
        label="planned",
    )
    if reconstructed_states:
        plt.plot(
            [state.position[0] for state in reconstructed_states],
            [state.position[1] for state in reconstructed_states],
            color="blue",
            label="reconstructed",
        )
    plt.xlabel("x")
    plt.ylabel("y")

    # steering angle
    plt.subplot(5, 1, 2)
    plt.plot(
        list(range(len(state_list))),
        [state.steering_angle for state in state_list],
        color="black",
        label="planned",
    )
    if reconstructed_states:
        plt.plot(
            list(range(len(reconstructed_states))),
            [state.steering_angle for state in reconstructed_states],
            color="blue",
            label="reconstructed",
        )
    if plot_bounds:
        plt.plot(
            [0, len(state_list)],
            [config.vehicle.delta_min, config.vehicle.delta_min],
            color="red",
            label="bounds",
        )
        plt.plot(
            [0, len(state_list)],
            [config.vehicle.delta_max, config.vehicle.delta_max],
            color="red",
        )
    plt.ylabel("delta")

    # velocity
    plt.subplot(5, 1, 3)
    plt.plot(
        list(range(len(state_list))),
        [state.velocity for state in state_list],
        color="black",
        label="planned",
    )
    if reconstructed_states:
        plt.plot(
            list(range(len(reconstructed_states))),
            [state.velocity for state in reconstructed_states],
            color="blue",
            label="reconstructed",
        )
    plt.legend()
    plt.ylabel("velocity [m/s]")
    plt.xlabel("timestep")

    # orientation
    plt.subplot(5, 1, 4)
    plt.plot(
        list(range(len(state_list))),
        [state.orientation for state in state_list],
        color="black",
        label="planned",
    )
    if reconstructed_states:
        plt.plot(
            list(range(len(reconstructed_states))),
            [state.orientation for state in reconstructed_states],
            color="blue",
            label="reconstructed",
        )
    plt.ylabel("theta")

    # yaw rate
    plt.subplot(5, 1, 5)
    plt.plot(
        list(range(len(state_list))),
        [state.yaw_rate for state in state_list],
        color="black",
        label="planned",
    )
    reconstructed_yaw_rate = (
        np.diff(np.array([state.orientation for state in reconstructed_states]))
        / config.planning.dt
    )
    reconstructed_yaw_rate = np.insert(
        reconstructed_yaw_rate, 0, state_list[0].yaw_rate, axis=0
    )
    plt.plot(
        list(range(len(reconstructed_yaw_rate))),
        reconstructed_yaw_rate,
        color="blue",
        label="reconstructed",
    )
    plt.ylabel("theta_dot")
    plt.tight_layout()

    if config.debug.show_evaluation_plots:
        plt.show()

    # plot errors in position, velocity, orientation
    if reconstructed_states:
        plt.figure(figsize=(7, 8.0))
        plt.suptitle("State Errors (planned vs. forward simulated)")

        plt.subplot(5, 1, 1)
        plt.plot(
            list(range(len(state_list))),
            [
                abs(state_list[i].position[0] - reconstructed_states[i].position[0])
                for i in range(len(state_list))
            ],
            color="black",
        )
        plt.ylabel("x error")
        plt.subplot(5, 1, 2)
        plt.plot(
            list(range(len(state_list))),
            [
                abs(state_list[i].position[1] - reconstructed_states[i].position[1])
                for i in range(len(state_list))
            ],
            color="black",
        )
        plt.ylabel("y error")
        plt.subplot(5, 1, 3)
        plt.plot(
            list(range(len(state_list))),
            [
                abs(
                    _angle_diff(
                        state_list[i].steering_angle,
                        reconstructed_states[i].steering_angle,
                    )
                )
                for i in range(len(state_list))
            ],
            color="black",
        )
        plt.ylabel("delta error")
        plt.subplot(5, 1, 4)
        plt.plot(
            list(range(len(state_list))),
            [
                abs(state_list[i].velocity - reconstructed_states[i].velocity)
                for i in range(len(state_list))
            ],
            color="black",
        )
        plt.ylabel("velocity error")
        plt.subplot(5, 1, 5)
        plt.plot(
            list(range(len(state_list))),
            [
                abs(
                    _angle_diff(
                        state_list[i].orientation, reconstructed_states[i].orientation
                    )
                )
                for i in range(len(state_list))
            ],
            color="black",
        )
        plt.ylabel("theta error")
        plt.tight_layout()

        if config.debug.show_evaluation_plots:
            plt.show()


def plot_inputs(
    config: ReactivePlannerConfiguration,
    input_list: List[InputState],
    reconstructed_inputs=None,
    plot_bounds=False,
):
    """
    Plots inputs of trajectory from a given input_list
    input_list must contain the following states: steering_angle_speed, acceleration
    optionally plots reconstructed_inputs
    """
    plt.figure()
    plt.suptitle("Inputs")

    # steering angle speed
    plt.subplot(2, 1, 1)
    plt.plot(
        list(range(len(input_list))),
        [state.steering_angle_speed for state in input_list],
        color="black",
        label="planned",
    )
    if reconstructed_inputs:
        plt.plot(
            list(range(len(reconstructed_inputs))),
            [state.steering_angle_speed for state in reconstructed_inputs],
            color="blue",
            label="reconstructed",
        )
    if plot_bounds:
        plt.plot(
            [0, len(input_list)],
            [config.vehicle.v_delta_min, config.vehicle.v_delta_min],
            color="red",
            label="bounds",
        )
        plt.plot(
            [0, len(input_list)],
            [config.vehicle.v_delta_max, config.vehicle.v_delta_max],
            color="red",
        )
    plt.legend()
    plt.ylabel("v_delta in rad/s")

    # acceleration
    plt.subplot(2, 1, 2)
    plt.plot(
        np.array(list(range(len(input_list)))),
        [state.acceleration for state in input_list],
        color="black",
        label="planned",
    )
    if reconstructed_inputs:
        plt.plot(
            list(range(len(reconstructed_inputs))),
            [state.acceleration for state in reconstructed_inputs],
            color="blue",
            label="reconstructed",
        )
    if plot_bounds:
        plt.plot(
            [0, len(input_list)],
            [-config.vehicle.a_max, -config.vehicle.a_max],
            color="red",
            label="bounds",
        )
        plt.plot(
            [0, len(input_list)],
            [config.vehicle.a_max, config.vehicle.a_max],
            color="red",
        )
    plt.ylabel("a_long in m/s^2")
    plt.tight_layout()

    if config.debug.show_evaluation_plots:
        plt.show()


def concatenate_trajectories(traj1: Trajectory, traj2: Trajectory) -> Trajectory:
    """
    将 traj1 和 traj2 拼接成一个新的 Trajectory。
    要点：
    1) 复制 traj1 的所有 state。
    2) 调整 traj2 的 time_step（可选，若你需要无缝衔接）使之接在 traj1 之后。
    3) 把 traj2 的状态逐个 append 到新的列表，生成新的 Trajectory。
    """

    # 1) 复制 traj1 的 state_list 作为新轨迹的初始状态列表
    new_state_list = list(traj1.state_list)  # 浅拷贝即可

    # 2) 计算需要平移的时间步 offset
    #    例如希望第二段轨迹的起始时刻刚好是 traj1.final_state.time_step + 1
    offset = (traj1.final_state.time_step + 1) - traj2.initial_time_step

    # 3) 把 traj2 的状态复制并 time_step += offset，再 append 到新列表
    for old_state in traj2.state_list:
        # 复制 state （确保不会修改原轨迹的对象）
        new_state = _copy_state(old_state)
        # 调整 time_step，让它衔接到 traj1 的尾部
        new_state.time_step = new_state.time_step + offset
        # 加入到 new_state_list
        new_state_list.append(new_state)

    # 4) 创建一个新的 Trajectory，初始时刻依旧是 traj1 的 initial_time_step
    new_trajectory = Trajectory(traj1.initial_time_step, new_state_list)
    return new_trajectory


def _copy_state(original_state: State) -> State:
    """这里演示手动复制一个 State 的简单做法。
    你也可以使用 deepcopy(original_state)。
    不同 State 类型（InitialState, KSState等）都继承了 State，所以尽量保持同一种类型。
    """
    # 常见写法是用 `copy.deepcopy(original_state)` 即可
    import copy

    return copy.deepcopy(original_state)


import copy

from commonroad.scenario.state import TraceState
from commonroad.scenario.trajectory import Trajectory


def shift_trajectory_time_steps(traj: Trajectory, offset: int) -> Trajectory:
    """
    将传入的 traj 的所有状态 time_step += offset，并返回一个新的 Trajectory。
    不会在原地修改 traj，而是创建并返回一个新的对象。
    """
    new_state_list = []
    for old_state in traj.state_list:
        new_state = copy.deepcopy(old_state)  # 深拷贝，避免破坏原始数据
        new_state.time_step += offset
        new_state_list.append(new_state)

    # 新的 initial_time_step 也要相应地加同样的 offset
    new_initial_time_step = traj.initial_time_step + offset

    # 构造并返回新的 Trajectory
    new_trajectory = Trajectory(new_initial_time_step, new_state_list)
    return new_trajectory


def shift_trajectory_to_start_at_zero(traj: Trajectory) -> Trajectory:
    """
    将传入的 traj 平移，使它从 time_step=0 开始。
    即让 traj.initial_time_step 变为 0，所有状态的 time_step 相应平移。
    """
    # 平移量 = 0 - 当前的 initial_time_step
    offset = -traj.initial_time_step
    return shift_trajectory_time_steps(traj, offset)


def concatenate_trajectories_from_zero(
    traj1: Trajectory, traj2: Trajectory
) -> Trajectory:
    """
    将 traj1 和 traj2 拼接成一个新的 Trajectory。
    1) 先把 traj1 平移到从 time_step=0 开始
    2) 复制 traj1 的所有 state
    3) 计算 traj2 的 time_step offset，使它能无缝接在 traj1 之后
    4) 复制并平移 traj2 的各个 state，然后 append 到新轨迹里
    5) 最终生成拼接后的 Trajectory，依旧从 time_step=0 开始
    """
    # 1) 复制 traj1 的所有状态作为新轨迹的“前半段”
    new_state_list = list(
        traj1.state_list
    )  # 浅拷贝列表本身，但别忘了其中的 State 也可以用深拷贝
    # 不过对 state 对象本身，一般不需要改它的属性（除非你想单独再 shift 这部分）
    # 所以这里可以直接用原引用。如果要完全独立也可以改成 copy.deepcopy(state)

    # 2) 计算拼接偏移量 offset
    #    使得第二条轨迹的起始时刻 = traj1.final_state.time_step + 1
    #    offset = (目标起始时刻) - traj2.initial_time_step
    offset = (traj1.final_state.time_step + 1) - traj2.initial_time_step

    # 3) 把 traj2 的各状态复制并 time_step += offset，然后拼到 new_state_list 里
    import copy

    for old_state in traj2.state_list:
        new_state = copy.deepcopy(old_state)
        new_state.time_step += offset
        new_state_list.append(new_state)

    # 4) 创建并返回新的 Trajectory
    #    它的 initial_time_step 通常保持和 traj1 一样（假如你已经让 traj1 从 0 开始，则此处还是 0）
    new_trajectory = Trajectory(traj1.initial_time_step, new_state_list)
    return new_trajectory
