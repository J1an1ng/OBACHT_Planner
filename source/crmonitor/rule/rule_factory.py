from typing import Optional

from antlr4 import CommonTokenStream
from antlr4.InputStream import InputStream

from source.crmonitor.predicates.predicate_factory import PredicateFactory
from source.crmonitor.rule.fastl.FaStlLexer import FaStlLexer
from source.crmonitor.rule.fastl.FaStlParser import FaStlParser
from source.crmonitor.rule.parse_tree_visitor import TrafficRuleParseTreeVisitor


class RuleFactory:
    def __init__(self, predicate_factory: Optional[PredicateFactory] = None):
        self._predicate_factory = predicate_factory or PredicateFactory()

    def parse_rule(self, full_rule_str, name=None):
        if name is None:
            name = full_rule_str
        stream = InputStream(full_rule_str)
        lexer = FaStlLexer(stream)
        stream = CommonTokenStream(lexer)
        parser = FaStlParser(stream)
        tree = parser.compile_unit()
        visitor = TrafficRuleParseTreeVisitor(stream)
        node = visitor.visit(tree)[0]
        node.name = name
        return node
