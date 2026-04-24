from typing import Optional

from source.crmonitor.common.vehicle import Vehicle
from source.crmonitor.common.world import World
from source.crmonitor.evaluation.evaluation import RuleEvaluator
from source.crmonitor.evaluation.visitor import (
    BaseValueMonitorTreeVisitor,
    MonitorCreationRuleTreeVisitor,
    RuleTreeVisitor,
)
from source.crmonitor.monitor.monitor_node import RuleMonitorNode
from source.crmonitor.monitor.proposition_robustness import PropositionRobustnessMonitor
from source.crmonitor.monitor.rtamt_monitor_stl import OutputType
from source.crmonitor.rule.rule_node import PredicateNode, RuleNode, VisitorNode


class PropositionMonitorRuleTreeVisitor(MonitorCreationRuleTreeVisitor):
    def visit_rule_node(self, rule_node: RuleNode, *ctx):
        children = [c.visit(self, *ctx) for c in rule_node.children]
        monitor = PropositionRobustnessMonitor.create_from_rule_node(
            rule_node, self.dt, self.output_type
        )
        return RuleMonitorNode(rule_node.name, children, monitor)


class PropositionCollectorMonitorTreeVisitor(BaseValueMonitorTreeVisitor):
    @staticmethod
    def visit_rule_node(rule_node: "RuleMonitorNode", *ctx):
        return list(rule_node.monitor.ast_node_values.items())

    def visit_predicate_node(self, predicate_node: PredicateNode, *ctx):
        raise NotImplementedError()


class PropositionRuleEvaluator(RuleEvaluator):
    def __init__(
        self,
        rule: VisitorNode,
        ego_id: int,
        world: World,
        start_time_step=None,
        use_boolean: bool = False,
        output_type: OutputType = OutputType.STANDARD,
    ):
        monitor_creation_visitor = PropositionMonitorRuleTreeVisitor(
            world.dt, output_type
        )
        self.proposition_collector = PropositionCollectorMonitorTreeVisitor()
        super().__init__(
            rule,
            ego_id,
            world,
            start_time_step,
            use_boolean,
            output_type,
            monitor_creation_visitor,
        )

    def get_propositions(self):
        """
        Calculates the proposition robustness (mainly used for trajectory repairing)
        Calculations are done for the non-ego vehicle that conforms to the rule with the lowest feasibility.

        Returns:
        props (dict{prop, value}): Robustness values of each proposition, obtained using _props attribute of the
        RtamtStlMonitor, set using the RtamtStlMonitor.collect_prop_rob method. If quantifier nodes exist, the Monitor
        that monitors the ego vehicle against the worst-case non-ego vehicle is used.
        other_id (int): The vehicle against which the values were obtained. Ego if the rule concerns the ego vehicle.
        time (int): Timestep at which the values were obtained.
        """
        other_id = (
            self._eval_visitor.other_ids[-1]
            if len(self._eval_visitor.other_ids) > 0
            else self.ego_vehicle.id
        )
        if hasattr(self._monitor, "monitors"):
            other_id = self._eval_visitor.other_ids[-1]
            props = self._monitor.monitors[other_id].monitor._propositions
        else:
            if any(hasattr(child, "monitors") for child in self._monitor.children):
                other_id = self._eval_visitor.other_ids[-1]
                props = self._monitor.monitor._propositions
                quant_nodes = [
                    node for node in self._monitor.children if hasattr(node, "monitors")
                ]
                # for quant_node in quant_nodes:
                #    for key in [key for key in props.keys() if quant_node.name in key]:
                #        props.pop(key)
                #    props.update(quant_node.monitors[other_id].monitor._props)
            else:
                props = self._monitor.monitor._propositions

        return props, other_id, self._last_evaluation_time_step
