from abc import ABC, abstractmethod
from typing import Optional
import numpy as np
from source.commonroad_rp.utility.utils_coordinate_system import CoordinateSystem, create_coordinate_system


def _extend_path(vertices: np.ndarray, extra_m: float = 80.0) -> np.ndarray:
    """
    Append `extra_m` metres of straight extension beyond the last vertex,
    keeping the direction of the final segment.  This ensures the projection
    domain is long enough to cover the full planning horizon even when the
    vehicle is near the end of the lanelet.
    """
    if len(vertices) < 2:
        return vertices
    direction = vertices[-1] - vertices[-2]
    norm = np.linalg.norm(direction)
    if norm < 1e-9:
        return vertices
    unit = direction / norm
    n_pts = max(2, int(extra_m / 0.5))          # one point every 0.5 m
    extras = vertices[-1] + unit * np.linspace(0.5, extra_m, n_pts)[:, None]
    return np.vstack([vertices, extras])


class PlannerState(ABC):
    """
    Abstract base class for planner states.
    Each state knows its own configuration file and how to create its coordinate system.
    """
    
    def __init__(self, scenario_type: str):
        self.scenario_type = scenario_type
        self._coord_system = None
    
    @abstractmethod
    def get_config_filename(self) -> Optional[str]:
        """Return the YAML configuration filename for planner-backed states."""
        pass
    
    @abstractmethod
    def create_coordinate_system(self, scenario, planning_problem) -> CoordinateSystem:
        """Create and return the coordinate system for this state"""
        pass
    
    @abstractmethod
    def get_state_name(self) -> str:
        """Return the string name of this state"""
        pass
    
    @abstractmethod
    def check_transition(self, next_state, config, goal_x: float) -> Optional['PlannerState']:
        """
        Check if transition to another state should occur.
        Returns the next state instance if transition should happen, None otherwise.
        """
        pass
    
    def get_coordinate_system(self, scenario, planning_problem) -> CoordinateSystem:
        """Get or create the coordinate system for this state"""
        if self._coord_system is None:
            self._coord_system = self.create_coordinate_system(scenario, planning_problem)
        return self._coord_system
    
    def is_valid_for_scenario(self) -> bool:
        """Check if this state is valid for the current scenario type"""
        return True  # Override in subclasses if needed


class DepartingState(PlannerState):
    """DEPARTING state implementation"""
    
    def get_config_filename(self) -> str:
        return "departure.yaml"
    
    def get_state_name(self) -> str:
        return "DEPARTING"
    
    def create_coordinate_system(self, scenario, planning_problem) -> CoordinateSystem:
        if self.scenario_type == "bus_stop_bulb":
            # For bulb scenario, use lane 2
            return create_coordinate_system(
                scenario.lanelet_network.find_lanelet_by_id(2).center_vertices
            )
        elif self.scenario_type == "bus_stop_bay":
            # For bay scenario, coordinate system is dynamic and created during execution
            # Return None here, will be handled in the planner
            return None
        else:
            raise ValueError(f"Unknown scenario type: {self.scenario_type}")
    
    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        # DEPARTING doesn't have automatic transitions in the current implementation

        orientation = abs(float(getattr(next_state, "orientation", 0.0)))
        acceleration = abs(float(getattr(next_state, "acceleration", 0.0)))
        if (
            next_state.velocity >= config.sampling.desire_velocity
            and orientation < 0.02
            and acceleration < 0.2
        ):
             return HeadingState(self.scenario_type)
        return None
    

class HeadingState(PlannerState):
    """HEADING state implementation"""
    
    def get_config_filename(self) -> str:
        return "heading_to_next.yaml"
    
    def get_state_name(self) -> str:
        return "HEADING"
    
    def create_coordinate_system(self, scenario, planning_problem) -> CoordinateSystem:
        if self.scenario_type == "bus_stop_bulb":
            # For bulb scenario, use lane 1
            vertices = scenario.lanelet_network.find_lanelet_by_id(1).center_vertices
            return create_coordinate_system(_extend_path(vertices))
        elif self.scenario_type == "bus_stop_bay":
            # For bay scenario, use lane 2
            vertices = scenario.lanelet_network.find_lanelet_by_id(2).center_vertices
            return create_coordinate_system(_extend_path(vertices))
        else:
            raise ValueError(f"Unknown scenario type: {self.scenario_type}")
    
    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        distance_threshold = config.planning.distance_heading_to_next_to_arriving

        if abs(goal_x -next_state.position[0]) < distance_threshold and next_state.position[0] < goal_x :

            return ArrivingState(self.scenario_type)
        return None


class ArrivingState(PlannerState):
    """ARRIVING state implementation"""
    
    def get_config_filename(self) -> str:
        return "arriving.yaml"
    
    def get_state_name(self) -> str:
        return "ARRIVING"
    
    def create_coordinate_system(self, scenario, planning_problem) -> CoordinateSystem:
        if self.scenario_type == "bus_stop_bulb":
            # For bulb scenario, use lane 2
            vertices = scenario.lanelet_network.find_lanelet_by_id(2).center_vertices
            return create_coordinate_system(_extend_path(vertices))
        elif self.scenario_type == "bus_stop_bay":
            # For bay scenario, use lane 1
            vertices = scenario.lanelet_network.find_lanelet_by_id(1).center_vertices
            return create_coordinate_system(_extend_path(vertices))
        else:
            raise ValueError(f"Unknown scenario type: {self.scenario_type}")

    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        if self.scenario_type == "bus_stop_bulb":
            threshold = config.planning.distance_arriving_to_stopping
            if abs(goal_x - next_state.position[0]) < threshold:
                return StoppingState(self.scenario_type)
        elif self.scenario_type == "bus_stop_bay":
            threshold = config.planning.distance_arriving_to_next_to_before_stopping
            if abs(goal_x - next_state.position[0]) < threshold:
                return BeforeStoppingMergeState(self.scenario_type)
        return None


class BeforeStoppingState(PlannerState):
    """Base BEFORE_STOPPING state implementation (only for bay scenario)."""
    
    def get_config_filename(self) -> str:
        return "before_stopping.yaml"
    
    def get_state_name(self) -> str:
        return "BEFORE_STOPPING"
    
    def create_coordinate_system(self, scenario, planning_problem) -> CoordinateSystem:
        if self.scenario_type != "bus_stop_bay":
            raise ValueError(f"BEFORE_STOPPING state only valid for bus_stop_bay scenario")
        
        goal_y = planning_problem.goal.state_list[0].position.center[1]
        left_vertex = scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[0][0]
        right_vertex = scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[-1][0]
        num = (right_vertex - left_vertex) * 10
        x_values = np.linspace(left_vertex, right_vertex, int(num))
        y_values = np.full_like(x_values, goal_y)
        return create_coordinate_system(np.column_stack((x_values, y_values)))
    
    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        threshold = config.planning.distance_before_stopping_to_stopping
        distance_to_goal = goal_x - float(next_state.position[0])
        velocity = float(getattr(next_state, "velocity", 0.0))
        if abs(distance_to_goal) < threshold or (distance_to_goal < 0.0 and velocity < 0.8):
            return StoppingState(self.scenario_type)
        return None
    
    def is_valid_for_scenario(self) -> bool:
        return self.scenario_type == "bus_stop_bay"


class BeforeStoppingAlignState(BeforeStoppingState):
    """Align the bus with the bay stop line after the merge."""

    def get_config_filename(self) -> str:
        return "before_stopping_align.yaml"

    def get_state_name(self) -> str:
        return "BEFORE_STOPPING_ALIGN"

    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        threshold = config.planning.distance_arriving_to_next_to_before_stopping
        if abs(goal_x - next_state.position[0]) < threshold:
            return BeforeStoppingFinalState(self.scenario_type)
        return None


class BeforeStoppingMergeState(BeforeStoppingState):
    """Perform the curvature-friendly lateral shift into the bay."""

    def get_config_filename(self) -> str:
        return "before_stopping_merge.yaml"

    def get_state_name(self) -> str:
        return "BEFORE_STOPPING_MERGE"

    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        threshold = config.planning.distance_arriving_to_stopping
        if abs(goal_x - next_state.position[0]) < threshold:
            return BeforeStoppingAlignState(self.scenario_type)
        return None


class BeforeStoppingFinalState(BeforeStoppingState):
    """Prepare for the final STOPPING state inside the goal lane."""

    def get_config_filename(self) -> str:
        return "before_stopping_final.yaml"

    def get_state_name(self) -> str:
        return "BEFORE_STOPPING_FINAL"

    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        threshold = config.planning.distance_before_stopping_to_stopping
        distance_to_goal = goal_x - float(next_state.position[0])
        velocity = float(getattr(next_state, "velocity", 0.0))
        if abs(distance_to_goal) < threshold or (distance_to_goal < 0.0 and velocity < 0.8):
            return StoppingState(self.scenario_type)
        return None


class StoppingState(PlannerState):
    """STOPPING state implementation"""
    
    def get_config_filename(self) -> str:
        return "stopping.yaml"
    
    def get_state_name(self) -> str:
        return "STOPPING"
    
    def create_coordinate_system(self, scenario, planning_problem) -> CoordinateSystem:
        if self.scenario_type == "bus_stop_bulb":
            # For bulb scenario, use lane 2
            return create_coordinate_system(
                scenario.lanelet_network.find_lanelet_by_id(2).center_vertices
            )
        elif self.scenario_type == "bus_stop_bay":
            # For bay scenario, use goal region coordinate system
            goal_state = planning_problem.goal.state_list[0]
            goal_y = goal_state.position.center[1]
            
            left_vertex = scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[0][0]
            right_vertex = scenario.lanelet_network.find_lanelet_by_id(1).left_vertices[-1][0]
            num = (right_vertex - left_vertex) * 10
            x_values = np.linspace(left_vertex, right_vertex, int(num))
            y_values = np.full_like(x_values, goal_y)
            return create_coordinate_system(np.column_stack((x_values, y_values)))
        else:
            raise ValueError(f"Unknown scenario type: {self.scenario_type}")
    
    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        # STOPPING state transitions are handled differently (with counter)
        return None


class EmergencyBrakeState(PlannerState):
    """Safety fallback state for controlled braking under imminent collision risk."""

    def get_config_filename(self) -> Optional[str]:
        return None

    def get_state_name(self) -> str:
        return "EMERGENCY_BRAKE"

    def create_coordinate_system(self, scenario, planning_problem) -> CoordinateSystem:
        return None

    def check_transition(self, next_state, config, goal_x: float) -> Optional[PlannerState]:
        # Recovery is handled by the state machine after the vehicle has stopped
        # and imminent_collision_risk is false.
        return None


# Factory function to create initial state
def create_initial_state(scenario_type: str, state_name: str = "HEADING") -> PlannerState:
    """Create an initial state instance based on scenario type and state name"""
    state_map = {
        "DEPARTING": DepartingState,
        "HEADING": HeadingState,
        "ARRIVING": ArrivingState,
        "BEFORE_STOPPING": BeforeStoppingState,
        "BEFORE_STOPPING_ALIGN": BeforeStoppingAlignState,
        "BEFORE_STOPPING_MERGE": BeforeStoppingMergeState,
        "BEFORE_STOPPING_FINAL": BeforeStoppingFinalState,
        "STOPPING": StoppingState,
        "EMERGENCY_BRAKE": EmergencyBrakeState,
    }
    
    if state_name not in state_map:
        raise ValueError(f"Unknown state name: {state_name}")
    
    state_instance = state_map[state_name](scenario_type)
    
    # Validate state is valid for scenario
    if not state_instance.is_valid_for_scenario():
        raise ValueError(f"State '{state_name}' is not valid for scenario type '{scenario_type}'")
    
    return state_instance


# Helper function to get valid states for a scenario
def get_valid_states_for_scenario(scenario_type: str) -> set:
    """Get the set of valid state names for a given scenario type"""
    if scenario_type == "bus_stop_bulb":
        return {"DEPARTING", "HEADING", "ARRIVING", "STOPPING", "EMERGENCY_BRAKE"}
    elif scenario_type == "bus_stop_bay":
        return {
            "DEPARTING",
            "HEADING",
            "ARRIVING",
            "BEFORE_STOPPING_ALIGN",
            "BEFORE_STOPPING_MERGE",
            "BEFORE_STOPPING_FINAL",
            "STOPPING",
            "EMERGENCY_BRAKE",
        }
    else:
        raise ValueError(f"Unknown scenario type: {scenario_type}")
