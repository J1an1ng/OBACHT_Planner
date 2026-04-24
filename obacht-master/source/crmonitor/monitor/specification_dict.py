from typing import Dict

from rtamt import Language, StlDiscreteTimeSpecification
from rtamt.semantics.abstract_discrete_time_online_interpreter import (
    DiscreteTimeOnlineUpdateVisitor,
)


class DiscreteTimeOnlineUpdateVisitorDict(DiscreteTimeOnlineUpdateVisitor):
    def __init__(self) -> None:
        self._ast_node_values = dict()

    @property
    def ast_node_values(self) -> Dict[str, float]:
        return self._ast_node_values

    def visit(self, node, *args, **kwargs):
        result = super(DiscreteTimeOnlineUpdateVisitorDict, self).visit(
            node, *args, **kwargs
        )
        self._ast_node_values.update({node.name: result})
        return result


def stl_discrete_time_online_specification_factory(semantics):
    spec = StlDiscreteTimeSpecification(semantics, Language.PYTHON)
    spec.online_interpreter.updateVisitor = DiscreteTimeOnlineUpdateVisitorDict()
    return spec
