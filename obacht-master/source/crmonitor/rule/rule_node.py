import copy
from abc import ABCMeta, abstractmethod
from enum import Enum

from source.crmonitor.monitor.monitor_node import MonitorNode


class IOType(Enum):
    OUTPUT = "output"
    INPUT = "input"


class VisitorNode(metaclass=ABCMeta):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @abstractmethod
    def visit(self, visitor, **ctx):
        pass


class RuleNode(VisitorNode):
    def __init__(self, children, rule_str, name):
        self.children = children
        self.name = name
        self.rule_str = rule_str

    def visit(self, visitor, *ctx):
        return visitor.visit_rule_node(self, *ctx)


class AllNode(VisitorNode):
    def __init__(self, children, quantified_vehicle, name):
        self.children = children
        self.name = name
        self.quantified_vehicle = quantified_vehicle

    def visit(self, visitor, *ctx):
        return visitor.visit_all_node(self, *ctx)


class ExistNode(VisitorNode):
    def __init__(self, children, quantified_vehicle, name):
        self.children = children
        self.name = name
        self.quantified_vehicle = quantified_vehicle

    def visit(self, visitor, *ctx):
        return visitor.visit_exist_node(self, *ctx)


class PredicateNode(MonitorNode, VisitorNode):
    def __init__(self, full_name, agent_placeholders, evaluator, io_type=IOType.OUTPUT):
        assert len(agent_placeholders) == evaluator.arity, (
            f"The arity of the evaluator for {full_name} should be "
            f"{len(agent_placeholders)}, but is {evaluator.arity}!"
        )
        super().__init__(full_name)
        self.agent_placeholders = tuple(agent_placeholders)
        self.evaluator = evaluator
        self.io_type = io_type
        self.latest_value = None
        self.latest_vehicle_ids = None

    def evaluate_boolean(self, world, time_step, vehicle_ids):
        value = self.evaluator.evaluate_boolean(world, time_step, vehicle_ids)
        self.latest_value = 1.0 if value else -1.0
        self.latest_vehicle_ids = tuple(vehicle_ids)
        return value

    def evaluate_robustness(self, world, time_step, vehicle_ids):
        value = self.evaluator.evaluate_robustness_with_cache(
            world, time_step, vehicle_ids
        )
        self.latest_value = value
        self.latest_vehicle_ids = tuple(vehicle_ids)
        return value

    def visit(self, visitor, *ctx):
        return visitor.visit_predicate_node(self, *ctx)

    @property
    def base_name(self):
        return self.evaluator.predicate_name

    @property
    def num_dependencies(self):
        return len(self.agent_placeholders)

    def __eq__(self, o) -> bool:
        return self.name == o.name and self.agent_placeholders == o.agent_placeholders

    def __hash__(self) -> int:
        return hash((self.name, self.agent_placeholders))

    def copy(self):
        return copy.copy(self)

    def reset(self):
        self.latest_value = None
        self.latest_vehicle_ids = None
