from typing import List, Tuple

from rtamt.syntax.node.ltl.conjunction import Conjunction
from rtamt.syntax.node.ltl.disjunction import Disjunction
from rtamt.syntax.node.ltl.implies import Implies
from rtamt.syntax.node.ltl.neg import Neg
from rtamt.syntax.node.ltl.predicate import Predicate
from rtamt.syntax.node.ltl.previous import Previous
from rtamt.syntax.node.ltl.variable import Variable
from rtamt.syntax.node.stl.timed_always import TimedAlways
from rtamt.syntax.node.stl.timed_eventually import TimedEventually
from rtamt.syntax.node.stl.timed_historically import TimedHistorically
from rtamt.syntax.node.stl.timed_once import TimedOnce
from rtamt.syntax.node.unary_node import UnaryNode

from crmonitor.monitor.rtamt_monitor_stl import OutputType, RtamtStlMonitor


class PropositionRobustnessMonitor(RtamtStlMonitor):
    def __init__(self, rule_str, predicates, dt, output_type=OutputType.STANDARD):
        super().__init__(rule_str, predicates, dt, output_type)
        self._propositions = {}

    def evaluate_monitor_online(
        self, time_step: int, predicates: List[Tuple[str, float]]
    ):
        robustness = super().evaluate_monitor_online(time_step, predicates)
        self.collect_prop_rob(self._spec.ast.specs[0], self._propositions)
        return robustness

    def collect_prop_rob(self, specs_node=None, prop_list=None):
        """
        Collects the propositions (abstractions) recursively to pass them to the monitor wrapper.
        If a sub-formula is encapsulated by an LTL/STL indicator, it constitutes a proposition.
        If negations exist, the non-negated formula that follows the negation is considered.
        If a predicate is not encapsulated by an LTL/STL indicator, it constitutes a proposition alone.
        Formulas may contain only: Implications, Con/Disjunctions, Negations, LTL/STL indicators.
        The values are obtained directly from the Rtamt.

        Returns:
        None. Acts directly on the dict that was passed as an argument: dict{proposition, robustness_value}
        """
        if specs_node is None:
            specs_node = self._spec.ast.specs[0]
        if isinstance(specs_node, UnaryNode):
            if isinstance(specs_node, Neg):
                self.collect_prop_rob(specs_node.children[0], prop_list)
            if (
                isinstance(specs_node, TimedOnce)
                or isinstance(specs_node, Previous)
                or isinstance(specs_node, TimedAlways)
                or isinstance(specs_node, TimedHistorically)
                or isinstance(specs_node, TimedEventually)
            ):
                prop_list[specs_node.name] = self.ast_node_values[specs_node.name]
        elif isinstance(specs_node, Predicate) or isinstance(specs_node, Variable):
            prop_list[specs_node.name] = self.ast_node_values[specs_node.name]
        else:
            if isinstance(specs_node, Implies):
                if isinstance(specs_node.children[0], Predicate) or isinstance(
                    specs_node.children[0], Variable
                ):
                    prop_list[specs_node.children[0].name] = self.ast_node_values[
                        specs_node.children[0].name
                    ]
                    self.collect_prop_rob(specs_node.children[1], prop_list)
                elif isinstance(specs_node.children[1], Predicate) or isinstance(
                    specs_node.children[1], Variable
                ):
                    prop_list[specs_node.children[1].name] = self.ast_node_values[
                        specs_node.children[1].name
                    ]
                    self.collect_prop_rob(specs_node.children[0], prop_list)
            if isinstance(specs_node, Conjunction) or isinstance(
                specs_node, Disjunction
            ):
                self.collect_prop_rob(specs_node.children[0], prop_list)
                self.collect_prop_rob(specs_node.children[1], prop_list)

    def copy(self):
        return PropositionRobustnessMonitor(
            self._rule, self._predicates, self.dt, self._output_type
        )
