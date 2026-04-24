import copy
import logging
import warnings
from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import numpy as np
from commonroad.visualization.mp_renderer import MPRenderer

from source.crmonitor.common.config import (
    get_evaluation_config,
    get_evaluation_config_pt,
    get_traffic_rule_config,
)
from source.crmonitor.common.helper import create_ego_vehicle_param, merge_dicts_recursively
from source.crmonitor.common.vehicle import Vehicle
from source.crmonitor.common.world import World
from source.crmonitor.evaluation.visitor import (
    AstNodeValueCollectorMonitorTreeVisitor,
    EvaluationMonitorTreeVisitor,
    MonitorCreationRuleTreeVisitor,
    PredicateCollectorMonitorTreeVisitor,
    PredicateVisualizerMonitorTreeVisitor,
    ResetMonitorTreeVisitor,
    RuleTreeVisitor,
)
from source.crmonitor.monitor.rtamt_monitor_stl import OutputType
from source.crmonitor.predicates.base import BasePredicateEvaluator
from source.crmonitor.predicates.predicate_factory import PredicateFactory
from source.crmonitor.rule.rule_factory import RuleFactory
from source.crmonitor.rule.rule_node import VisitorNode

logger = logging.getLogger(__name__)


class RuleEvaluator:
    @classmethod
    def create_from_config(
        cls,
        world: World,
        ego_id: Optional[Union[int, Vehicle]],
        rule: str = "R_G1",
        traffic_rules_config=None,
        use_boolean: bool = False,
        output_type: OutputType = OutputType.STANDARD,
    ):
        if traffic_rules_config is None:
            traffic_rules_config = get_traffic_rule_config()
        rule_str_dict = traffic_rules_config["traffic_rules"]

        # Flat copy vehicles of the world to update ego vehicle parameters
        world = copy.copy(world)
        world.vehicles = copy.copy(world.vehicles)

        if isinstance(ego_id, Vehicle):
            warnings.warn(
                "Passing a vehicle instance is deprecated and will be removed in the future!",
                DeprecationWarning,
            )
            assert ego_id is world.vehicle_by_id(ego_id.id)
            ego_id = ego_id.id

        ego_vehicle = copy.copy(world.vehicle_by_id(ego_id))
        world.vehicles.remove(world.vehicle_by_id(ego_id))

        ego_vehicle.vehicle_param = create_ego_vehicle_param(
            get_evaluation_config_pt().get("ego_vehicle_param"), world.dt
        )
        world.vehicles.add(ego_vehicle)

        rule = RuleFactory(
            PredicateFactory(traffic_rules_config["traffic_rules_param"])
        ).parse_rule(rule_str_dict[rule], name=rule)
        return cls(
            rule,
            ego_vehicle.id,
            world,
            use_boolean=use_boolean,
            output_type=output_type,
        )

    def __init__(
        self,
        rule: VisitorNode,
        ego_id: Optional[Union[Vehicle, int]] = None,
        world: Optional[World] = None,
        start_time_step=None,
        use_boolean: bool = False,
        output_type: OutputType = OutputType.STANDARD,
        monitor_creation_visitor: Optional[RuleTreeVisitor] = None,
    ):
        if monitor_creation_visitor is None:
            monitor_creation_visitor = MonitorCreationRuleTreeVisitor(
                world.dt, output_type
            )
        self._rule = rule
        self._monitor = rule.visit(monitor_creation_visitor)
        self._predicate_collector_visitor = PredicateCollectorMonitorTreeVisitor()
        self._ast_node_value_collector_visitor = (
            AstNodeValueCollectorMonitorTreeVisitor()
        )
        self._visualizer_visitor = PredicateVisualizerMonitorTreeVisitor()
        self._eval_visitor = EvaluationMonitorTreeVisitor(
            use_boolean=use_boolean, output_type=output_type
        )
        self._last_evaluation_time_step = -1
        self._rule_value_course = []
        self._ego_id = None
        self._world = None
        if ego_id is not None:
            assert world is not None
            self.reset(ego_id, world, start_time_step)

    @property
    def current_time(self) -> int:
        return self._last_evaluation_time_step

    def get_predicates(self) -> Dict[str, float]:
        predicate_values = dict(self._monitor.visit(self._predicate_collector_visitor))
        return predicate_values

    def ast_node_values(self) -> Dict[str, float]:
        node_values = dict(self._monitor.visit(self._ast_node_value_collector_visitor))
        return node_values

    @property
    def ego_vehicle(self):
        return self._world.vehicle_by_id(self._ego_id)

    def update(self) -> float:
        """
        Advance the monitor state by one time step and return the corresponding
        rule evaluation value.

        :return: robustness or boolean rule value
        """
        self._last_evaluation_time_step += 1
        if (
            self.ego_vehicle.start_time > self._last_evaluation_time_step
            or self._last_evaluation_time_step > self.ego_vehicle.end_time
        ):
            logger.warning("Evaluating vehicle outside its lifetime!")
            return np.inf
        rule_value = self._eval_visitor.walk(
            self._monitor,
            self._world,
            self._last_evaluation_time_step,
            self.ego_vehicle,
        )
        rule_value = (
            rule_value if np.isfinite(rule_value) else np.sign(rule_value) * 1.0
        )
        self._rule_value_course.append((self._last_evaluation_time_step, rule_value))
        return rule_value

    def evaluate(self) -> np.ndarray:
        """
        Evaluate the rule exhaustively until the final time step of the vehicle object
        is reached.

        Caution: This will change the time step of the world object!

        :return: Array of all rule values for all time steps of the vehicle's known
            trajectory
        """
        robustness_values = []
        for i in range(
            self._last_evaluation_time_step + 1, self.ego_vehicle.end_time + 1
        ):
            robustness_values.append(self.update())
        return np.array(robustness_values)

    def __iter__(self):
        return self

    def __next__(self):
        if self._last_evaluation_time_step + 1 < self.ego_vehicle.end_time + 1:
            return self.update()
        else:
            raise StopIteration

    def visualize_predicates(
        self,
        vehicle2draw_params: Dict,
        visualization_config: Dict[str, any],
    ) -> Tuple[
        Dict[str, BasePredicateEvaluator],
        Dict[Any, Dict],
        List,
        List[Callable[[MPRenderer], None]],
    ]:
        """
        Renders a scenario visualization using the MPRenderer and adds plots of the
        predicates. In general, only
        predicate instances belonging to an effective group within all enclosing all-
        and exist-quantifiers of the
        considered rule are visualized; here, "effective group" denotes the group
        giving the minimum resp. maximum
        value for an all- resp. exist-quantifier.
        :visualization_config: predicate-name | 'default' -> {
            show_non_effective_predicate_instances_for_vehicles: List[Tuple[int]],
            # show predicate value for certain
            # vehicle-ids
        }. Allows predicate-type wise configurations of the visualization
        :plot_scenario_legend: whether the legend for the scenario visualization
        should be plotted. If None, it is
        plotted for the first time-step only
        :scenario_fig_size: figure size of the scenario only
        :scenario_scale_compared_to_other_plots: scale describing how much larger
        than the other bar-chart and the
        rule-robustness chart the scenario should be drawn
        :plot_predicate_bar_chart: whether a bar chart showing the predicate values
        should be plotted. The
        predicate instances included in the visualization are the same as the ones
        shown in the scenario visualization
        :bar_chart_plot_limits: minimum and maximum value of the bar-chart
        :plot_rule_robustness_course: whether the rule robustness should be plotted
        :rule_robustness_course_plot_limits: minimum and maximum y-value of the rule
        robustness course
        :scenario_plot_limits: [xmin, xmax, ymin, ymax] for the scenario plotting
        """

        def add_vehicle_draw_params(vehicle_id: int, draw_params: any):
            vehicle2draw_params[vehicle_id] = merge_dicts_recursively(
                vehicle2draw_params.get(vehicle_id, {}), draw_params
            )

        predicate_names2vehicle_ids2values = defaultdict(dict)

        predicate_name2predicate_evaluator = {}

        draw_functions = self._monitor.visit(
            self._visualizer_visitor,
            add_vehicle_draw_params,
            predicate_names2vehicle_ids2values,
            predicate_name2predicate_evaluator,
            self._world,
            self.current_time,
            visualization_config,
        )

        return (
            predicate_name2predicate_evaluator,
            predicate_names2vehicle_ids2values,
            self._rule_value_course,
            draw_functions,
        )

    @property
    def other_ids(self) -> Tuple[int]:
        return self._eval_visitor.other_ids[1:]

    def reset(self, ego_id: Union[Vehicle, int], world: World, start_time_step=None):
        if isinstance(ego_id, Vehicle):
            warnings.warn(
                "Passing a vehicle instance is deprecated and will be removed in the future!",
                DeprecationWarning,
            )
            assert ego_id is world.vehicle_by_id(ego_id.id)
            ego_id = ego_id.id
        self._ego_id = ego_id
        self._world = world
        self._last_evaluation_time_step = (
            start_time_step - 1
            if start_time_step is not None
            else self.ego_vehicle.start_time - 1
        )
        self._rule_value_course = []
        # Reset monitor
        reset_visitor = ResetMonitorTreeVisitor()
        self._monitor.visit(reset_visitor)
