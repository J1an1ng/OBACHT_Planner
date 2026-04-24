import copy
from enum import Enum
from functools import lru_cache
from typing import Any, Dict, List, Tuple

import rtamt

from source.crmonitor.rule.rule_node import IOType, RuleNode

from .specification_dict import stl_discrete_time_online_specification_factory


class OutputType(Enum):
    STANDARD = rtamt.Semantics.STANDARD
    OUTPUT_ROBUSTNESS = rtamt.Semantics.OUTPUT_ROBUSTNESS


@lru_cache(None)
def _template_spec(
    logic_formula: str, output_type: OutputType, predicates, dt
) -> rtamt.STLSpecification:
    if output_type != OutputType.STANDARD:
        # Workaround for rtamt when working with output-robustness and input vacuity
        for pred in predicates:
            logic_formula = logic_formula.replace(
                pred[0].name, f"({pred[0].name} >= 0)"
            )

    spec = stl_discrete_time_online_specification_factory(semantics=output_type.value)
    for var, io_type in predicates:
        spec.declare_var(var.name, "float")
        if io_type == IOType.INPUT:
            spec.set_var_io_type(var.name, "input")
        else:
            spec.set_var_io_type(var.name, "output")
    spec.declare_var("out", "float")

    spec.iosem = output_type
    spec.unit = "s"
    spec.spec = f"out = {logic_formula}"
    spec.set_sampling_period(dt, "s")
    spec.parse()
    spec.pastify()
    # new ast of online interpreter is not set until the update method
    # of AbstractOnlineSpecification is called
    spec.online_interpreter.set_ast(spec.ast)

    return spec


def _create_spec(
    rule_str: str, output_type: OutputType, predicates: List[Tuple[str, Any]], dt: float
) -> rtamt.STLSpecification:
    template_spec = _template_spec(rule_str, output_type, tuple(predicates), dt)
    # The dynamic part of the template spec has to be replaced.
    spec = copy.copy(template_spec)
    # Create a dummy spec to obtain a new interpreter
    dummy_spec = stl_discrete_time_online_specification_factory(output_type.value)
    spec.online_interpreter = dummy_spec.online_interpreter
    # new ast of online interpreter is not set until the
    # update method of AbstractOnlineSpecification is called
    dummy_spec.online_interpreter.set_sampling_period(dt, "s")
    spec.online_interpreter.set_ast(spec.ast)
    spec.reset()
    return spec


class RtamtStlMonitor:
    """
    Represents single formalized STL rule
    """

    @classmethod
    def create_from_rule_node(
        cls, rule_node: RuleNode, dt: float, output_type=OutputType.STANDARD
    ):
        predicates = [
            (c, c.io_type if hasattr(c, "io_type") else IOType.OUTPUT)
            for c in rule_node.children
        ]
        return cls(rule_node.rule_str, predicates, dt, output_type)

    def __init__(self, rule_str, predicates, dt, output_type=OutputType.STANDARD):
        self._rule = rule_str
        self._predicates = predicates
        self._output_type = output_type
        self._dt = dt

        # Flat copy spec and only recreate the online evaluator to
        # avoid parsing the rule.
        self._spec = _create_spec(rule_str, output_type, predicates, dt)

    @property
    def dt(self) -> float:
        return self._dt

    @property
    def ast_node_values(self) -> Dict[str, float]:
        return self._spec.online_interpreter.updateVisitor.ast_node_values

    def evaluate_monitor_online(
        self, time_step: int, predicates: List[Tuple[str, float]]
    ):
        time = time_step * self.dt
        rob = self._spec.update(time, predicates)
        return rob

    def copy(self):
        return RtamtStlMonitor(self._rule, self._predicates, self.dt, self._output_type)

    def reset(self):
        self._spec.reset()
