import os
from typing import Dict, List, Optional, Tuple, Union

import matplotlib.pyplot as plt
import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState
from commonroad.visualization.draw_params import DynamicObstacleParams, MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib.animation import FuncAnimation
from sumocr.interface.ego_vehicle import EgoVehicle


def create_video(
    scenario: Scenario,
    output_folder: str,
    planning_problem_set: PlanningProblemSet = None,
    trajectory_pred: Union[Dict[int, EgoVehicle], List[TrajectoryPrediction]] = None,
    follow_ego: bool = False,
    follow_ego_area_size: Union[float, Tuple[float, float]] = 120,
    suffix: str = "",
    file_type: str = "mp4",
    ego_obstacle_type: ObstacleType = ObstacleType.CAR,
    ego_dimensions: Optional[Tuple[float, float]] = None,
    ego_color: str = "green",
    other_vehicle_color: Optional[str] = None,
    figsize: Tuple[float, float] = (5, 5),
    dpi: int = 150,
) -> str:
    """
    Create video for a simulated scenario and the list of ego vehicles.

    :param scenario: Final commonroad scenario
    :param output_folder: path to output folder
    :param planning_problem_set: possibility to plot a Commonroad planning problem
    :param trajectory_pred: list of one or more ego vehicles or their trajectory predictions
    :param follow_ego: focus video on the ego vehicle(s)
    :param follow_ego_area_size: half-width and half-height of the ego-centered view in meters
    :param suffix: possibility to add suffix to file name
    :param file_type: mp4 or gif files supported
    :param ego_obstacle_type: CommonRoad type used to draw the ego vehicle
    :param ego_dimensions: optional ego length and width in meters
    :param ego_color: ego vehicle fill color
    :param other_vehicle_color: optional fill color for non-ego dynamic obstacles
    :param figsize: output figure size in inches
    :param dpi: output resolution
    :return:
    """
    assert file_type in ("mp4", "gif")

    # add short padding to create a short break before the loop (1sec)
    frame_count_padding = int(1 / scenario.dt)
    frame_count = (
        max([obstacle.prediction.final_time_step for obstacle in scenario.obstacles])
        + frame_count_padding
    )
    dynamic_obstacles_ego = []
    if trajectory_pred is None:
        trajectory_pred = []

    if len(trajectory_pred) > 0 and isinstance(
        list(trajectory_pred.values())[0], EgoVehicle
    ):
        trajectories_tmp = []
        for _, e in trajectory_pred.items():
            trajectories_tmp.append(e.driven_trajectory)
        trajectory_pred = trajectories_tmp

    for prediction in trajectory_pred:
        frame_count = len(prediction.trajectory.state_list) + frame_count_padding
        prediction_shape = prediction.shape
        if ego_dimensions is not None:
            prediction_shape = Rectangle(
                length=ego_dimensions[0],
                width=ego_dimensions[1],
            )
            prediction = TrajectoryPrediction(
                prediction.trajectory,
                prediction_shape,
            )

        # create the ego vehicle prediction using the trajectory and the shape of the obstacle
        dynamic_obstacle_initial_state = prediction.trajectory.state_list[0]

        if not isinstance(dynamic_obstacle_initial_state, InitialState):
            dynamic_obstacle_initial_state = InitialState(
                position=dynamic_obstacle_initial_state.position,
                orientation=dynamic_obstacle_initial_state.orientation,
                velocity=dynamic_obstacle_initial_state.velocity,
                acceleration=getattr(
                    dynamic_obstacle_initial_state, "acceleration", None
                ),
                yaw_rate=getattr(dynamic_obstacle_initial_state, "yaw_rate", None),
                slip_angle=getattr(dynamic_obstacle_initial_state, "slip_angle", None),
                time_step=dynamic_obstacle_initial_state.time_step,
            )

            print(dynamic_obstacle_initial_state)

        # generate the dynamic obstacle according to the specification
        dynamic_obstacle_id = scenario.generate_object_id()
        ego_dynamic_obstacle = DynamicObstacle(
            dynamic_obstacle_id,
            ego_obstacle_type,
            prediction_shape,
            dynamic_obstacle_initial_state,
            prediction,
        )
        dynamic_obstacles_ego.append(ego_dynamic_obstacle)

    if follow_ego:
        if trajectory_pred:
            # a dictionary that holds the plot limits at each time step
            dict_plot_limits = get_dynamic_plot_limits(
                trajectory_pred,
                frame_count,
                area_size=follow_ego_area_size,
            )
        else:
            # warnings.warn("Unable to follow the ego vehicle as no trajectory is provided!")
            dict_plot_limits = get_plot_limits(scenario, frame_count)
    else:
        dict_plot_limits = get_plot_limits(scenario, frame_count)

    interval = (
        1000 * scenario.dt
    )  # delay between frames in milliseconds, 1 second * dt to get actual time in ms

    draw_params = MPDrawParams()
    draw_params.axis_visible = False
    rnd = MPRenderer(figsize=figsize, draw_params=draw_params)
    rnd.ax.axes.get_xaxis().set_visible(False)
    rnd.ax.axes.get_yaxis().set_visible(False)
    rnd.f.subplots_adjust(left=0, right=1, bottom=0, top=1)
    (ln,) = plt.plot([], [], animated=True)

    ego_params = DynamicObstacleParams()
    ego_params.draw_icon = True
    ego_params.draw_shape = True
    ego_params.show_label = False
    ego_params.vehicle_shape.occupancy.draw_occupancies = True
    ego_params.vehicle_shape.occupancy.shape.facecolor = ego_color
    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#C55A11"
    ego_params.vehicle_shape.occupancy.shape.opacity = 1
    ego_params.vehicle_shape.occupancy.shape.zorder = 200

    other_params = MPDrawParams()
    other_params.axis_visible = False
    other_params.dynamic_obstacle.draw_icon = True
    other_params.dynamic_obstacle.draw_shape = True
    other_params.dynamic_obstacle.show_label = False
    if other_vehicle_color is not None:
        other_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = other_vehicle_color
        other_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#1B5E20"
        other_params.dynamic_obstacle.vehicle_shape.occupancy.shape.opacity = 1

    def init_plot():
        plt.cla()
        if planning_problem_set is not None:
            planning_problem_set.draw(rnd)

        if dynamic_obstacles_ego is not None:
            ego_params.time_begin = 0
            ego_params.time_end = 0
            rnd.draw_list(dynamic_obstacles_ego, ego_params)

        other_params.time_begin = 0
        other_params.time_end = 0
        scenario.draw(renderer=rnd, draw_params=other_params)
        rnd.plot_limits = dict_plot_limits[0]
        rnd.render()
        plt.draw()
        return (ln,)

    def animate_plot(frame):
        rnd.clear(keep_static_artists=False)
        if planning_problem_set is not None:
            planning_problem_set.draw(rnd)

        other_params.time_begin = frame
        other_params.time_end = frame
        rnd.draw_list(scenario.dynamic_obstacles, draw_params=other_params)

        if dynamic_obstacles_ego is not None:
            ego_params.time_begin = frame
            ego_params.time_end = frame
            rnd.draw_list(dynamic_obstacles_ego, draw_params=ego_params)

        lanelet_params = MPDrawParams()
        lanelet_params.time_begin = 0
        lanelet_params.time_end = 0
        scenario.lanelet_network.draw(renderer=rnd, draw_params=lanelet_params)

        rnd.plot_limits = dict_plot_limits[frame]
        rnd.render()
        return (ln,)

    anim = FuncAnimation(
        rnd.f,
        animate_plot,
        frames=frame_count,
        init_func=init_plot,
        blit=True,
        interval=interval,
    )

    file_name = str(scenario.scenario_id) + suffix + os.extsep + file_type
    anim.save(os.path.join(output_folder, file_name), dpi=dpi, writer="ffmpeg")
    plt.close(rnd.f)
    return file_name


def get_plot_limits(scenario: Scenario, frame_count):
    """
    Return fixed limits containing the complete lanelet geometry.
    """
    boundary_vertices = np.vstack(
        [
            vertices
            for lanelet in scenario.lanelet_network.lanelets
            for vertices in (lanelet.left_vertices, lanelet.right_vertices)
        ]
    )
    min_coords = np.min(boundary_vertices, axis=0)
    max_coords = np.max(boundary_vertices, axis=0)
    margin_x = 2.0
    margin_y = 2.0
    dict_plot_limits = [
        [
            min_coords[0] - margin_x,
            max_coords[0] + margin_x,
            min_coords[1] - margin_y,
            max_coords[1] + margin_y,
        ]
    ] * frame_count

    return dict_plot_limits


def get_dynamic_plot_limits(
    trajectories: List[TrajectoryPrediction],
    frame_count,
    area_size: Union[float, Tuple[float, float]] = 120,
):
    """
    The plot limits track the center of the ego vehicles.
    """
    # trajectories = trajectories.tra
    num_time_step_trajectories_max = max(
        [len(trajectory.trajectory.state_list) for trajectory in trajectories]
    )

    if isinstance(area_size, (tuple, list)):
        area_x, area_y = area_size
    else:
        area_x = area_y = area_size

    dict_plot_limits = list()
    for i in range(frame_count):
        if i < num_time_step_trajectories_max:
            list_states_vehicles = get_state_list_of_trajectories(i, trajectories)
        else:
            list_states_vehicles = get_state_list_of_trajectories(
                num_time_step_trajectories_max - 1, trajectories
            )

        x_min = min([state.position[0] for state in list_states_vehicles]) - area_x
        x_max = max([state.position[0] for state in list_states_vehicles]) + area_x
        y_min = min([state.position[1] for state in list_states_vehicles]) - area_y
        y_max = max([state.position[1] for state in list_states_vehicles]) + area_y

        dict_plot_limits.append([x_min, x_max, y_min, y_max])

    return dict_plot_limits


def get_state_list_of_trajectories(
    time_step: int, trajectories: List[TrajectoryPrediction]
):
    """
    Returns the list of states of trajectories at the specified time step.
    """
    list_states = []
    for trajectory in trajectories:
        try:
            list_states.append(trajectory.trajectory.state_list[time_step])
        except IndexError:
            pass

    return list_states
