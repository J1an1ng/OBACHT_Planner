import copy
from dataclasses import dataclass
from typing import List

import numpy as np

from commonroad.scenario.state import KSState, FloatExactOrInterval, InitialState
from commonroad.scenario.trajectory import State


@dataclass(eq=False)
class ReactivePlannerState(KSState):
    """
    State class used for output trajectory of reactive planner with the following additions to KSState:
    * Position is here defined w.r.t. rear-axle position (in KSState: position is defined w.r.t. vehicle center)
    * Extends KSState attributes by acceleration and yaw rate
    """
    def __repr__(self):
        return f"(time_step={self.time_step}, position={self.position},steering_angle={self.steering_angle}, " \
               f"velocity={self.velocity}, orientation={self.orientation}, acceleration={self.acceleration}, " \
               f"yaw_rate = {self.yaw_rate})"

    acceleration: FloatExactOrInterval = None
    yaw_rate: FloatExactOrInterval = None

    @classmethod
    def shift_state_to_center(
        cls,
        state: "ReactivePlannerState",
        wb_rear_axle: float
    ) -> "ReactivePlannerState":
        """
        Shifts position from rear-axle to vehicle center
        :param state: original state with position defined on the rear axle
        :param wb_rear_axle: distance between rear-axle and vehicle center
        :return new state, where the positions are shifted to vehicle center
        """
        # shift positions from rear axle to center
        orientation = state.orientation
        pos_x = state.position[0] + wb_rear_axle * np.cos(orientation)
        pos_y = state.position[1] + wb_rear_axle * np.sin(orientation)
        
        return ReactivePlannerState(
            time_step=state.time_step,
            position=np.array([pos_x, pos_y]),
            velocity=state.velocity,
            acceleration=state.acceleration,
            orientation=state.orientation,
            steering_angle=state.steering_angle,
            yaw_rate=state.yaw_rate
        )

    @classmethod
    def create_from_initial_state(
        cls,
        initial_state: InitialState,
        wheelbase: float,
        wb_rear_axle: float
    )-> "ReactivePlannerState":
        """
        Converts InitialState object to ReactivePlannerState object by:
        * adding initial acceleration (if not existing)
        * adding initial steering angle
        * removing initial slip angle
        * shifting position from center to rear axle
        :param initial_state: InitialState object
        :param wheelbase: wheelbase of the vehicle
        :param wb_rear_axle: distance between rear-axle and vehicle center
        """
        assert type(initial_state) == InitialState, "<ReactivePlannerState.create_from_initial_state()>: Input must be" \
                                                    "of type InitialState"
        # add acceleration (if not existing)
        if not hasattr(initial_state, 'acceleration'):
            initial_state.acceleration = 0.

        # remove slip angle
        try:
            delattr(initial_state, "slip_angle")
        except AttributeError:
            pass

        # shift initial position from center to rear axle
        orientation = initial_state.orientation
        initial_state_shifted = initial_state.translate_rotate(np.array([-wb_rear_axle * np.cos(orientation),
                                                                         -wb_rear_axle * np.sin(orientation)]), 0.0)

        # convert to ReactivePlannerState
        x0_planner = ReactivePlannerState()
        x0_planner = initial_state_shifted.convert_state_to_state(x0_planner)

        # add steering angle
        x0_planner.steering_angle = np.arctan2(wheelbase * x0_planner.yaw_rate, x0_planner.velocity)

        return x0_planner


def append_state_to_list(state_list: List[State], new_state: State) -> None:

    new_state = copy.deepcopy(new_state)

    if not isinstance(state_list, list) or any(
        not isinstance(s, State) for s in state_list
    ):
        raise TypeError("state_list must be List[State] and every element must be a State")

    if not isinstance(new_state, State):
        raise TypeError("new_state must be a State")

    if not state_list:
        raise ValueError("state_list cannot be empty")

    ref_state = state_list[0]
    ref_attrs = set(ref_state.used_attributes)
    new_attrs = set(new_state.used_attributes)

    missing = ref_attrs - new_attrs
    for attr in missing:
        setattr(new_state, attr, copy.deepcopy(getattr(ref_state, attr)))

    extra = new_attrs - ref_attrs
    for attr in extra:
        delattr(new_state, attr)

    new_state.time_step = state_list[-1].time_step + 1

    state_list.append(new_state)


def append_states_to_list_ver(
    target_list: List[State], source_list: List[State], n: int
) -> None:
    """
    Append multiple states from the source list to the target list starting from the second state

    :param target_list: The target list of states (will be modified)
    :param source_list: The source list of states (from which states are taken)
    :param n: The number of states to append (starting from index 1)
    """
    start_index = 1

    if not target_list:
        raise ValueError("target_list cannot be empty")

    if not isinstance(target_list, list) or any(
        not isinstance(s, State) for s in target_list
    ):
        raise TypeError(
            "target_list must be List[State] with all elements being State instances"
        )

    if not isinstance(source_list, list) or any(
        not isinstance(s, State) for s in source_list
    ):
        raise TypeError(
            "source_list must be List[State] with all elements being State instances"
        )

    if len(source_list) < 2:
        raise ValueError(
            f"Source list must contain at least 2 states, but got {len(source_list)}"
        )

    if n <= 0:
        raise ValueError("n must be a positive integer")

    if start_index + n > len(source_list):
        raise IndexError(
            f"Source list has {len(source_list)} states, cannot append {n} states starting from index 1"
        )

    ref_state = target_list[0]
    ref_attrs = set(ref_state.used_attributes)
    last_time_step = target_list[-1].time_step

    for i in range(n):
        source_state = source_list[start_index + i]
        new_state = copy.deepcopy(source_state)
        new_attrs = set(new_state.used_attributes)

        missing = ref_attrs - new_attrs
        for attr in missing:
            setattr(new_state, attr, copy.deepcopy(getattr(ref_state, attr)))

        extra = new_attrs - ref_attrs
        for attr in extra:
            delattr(new_state, attr)

        new_state.time_step = last_time_step + i + 1
        target_list.append(new_state)
