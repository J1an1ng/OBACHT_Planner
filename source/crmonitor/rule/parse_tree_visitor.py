from typing import Optional

from antlr4.TokenStreamRewriter import TokenStreamRewriter
from interval import Interval

from source.crmonitor.predicates.predicate_factory import PredicateFactory
from source.crmonitor.rule.fastl.FaStlParser import FaStlParser
from source.crmonitor.rule.fastl.FaStlParserVisitor import FaStlParserVisitor
from source.crmonitor.rule.rule_node import AllNode, ExistNode, IOType, PredicateNode, RuleNode


class TrafficRuleParseTreeVisitor(FaStlParserVisitor):
    """
    Build a modified tree from the original parse tree.
    """

    def __init__(self, tokens, predicate_factory: Optional[PredicateFactory] = None):
        self._predicate_factory = predicate_factory or PredicateFactory()
        self._rewriter: Optional[TokenStreamRewriter] = TokenStreamRewriter(tokens)
        self._sub_rule_counter = 0

    def defaultResult(self):
        return []

    def aggregateResult(self, aggregate, nextResult):
        aggregate.extend(nextResult)
        return aggregate

    def visitVehicle(self, ctx: FaStlParser.VehicleContext):
        # Get the placeholder id
        return [int(ctx.IntegerLiteral().getText())]

    def visitPredicate(self, ctx: FaStlParser.PredicateContext):
        vehicle_ids = self.visitChildren(ctx)
        pred_basename = ctx.Identifier().getText()
        if ctx.IO_TYPE_INPUT() is not None:
            io_type = IOType.INPUT
            token_index = ctx.IO_TYPE_INPUT().symbol.tokenIndex
            # Delete the input indicator, only keep the predicate name.
            self._rewriter.delete("predicate", token_index, token_index)
        else:
            io_type = IOType.OUTPUT
        predicate_evaluator = self._predicate_factory.get_predicate(pred_basename)
        # Rewrite the predicate name into RTAMT compliant syntax
        suffix = "__" + "_".join(str(i) for i in vehicle_ids)
        self._rewriter.replace(
            "predicate",
            ctx.LPAREN().symbol.tokenIndex,
            ctx.RPAREN().symbol.tokenIndex,
            suffix,
        )
        p = PredicateNode(
            pred_basename + suffix,
            vehicle_ids,
            predicate_evaluator,
            io_type,
        )
        return [p]

    def visitSpecQuantForall(self, ctx: FaStlParser.SpecQuantForallContext):
        children = self.visit(ctx.spec())
        quantified_vehicle = self.visitVehicle(ctx.vehicle())[0]
        self._sub_rule_counter += 1
        node_name = f"g{self._sub_rule_counter}"
        node = AllNode(children, quantified_vehicle, node_name)
        # Replace sub-formula inside the quantification by a "virtual" predicate g...
        self._rewriter.replace(
            "predicate",
            ctx.start.tokenIndex,
            ctx.stop.tokenIndex,
            node_name,
        )
        return [node]

    def visitSpecQuantExist(self, ctx: FaStlParser.SpecQuantExistContext):
        children = self.visit(ctx.spec())
        quantified_vehicle = self.visitVehicle(ctx.vehicle())[0]
        self._sub_rule_counter += 1
        node_name = f"g{self._sub_rule_counter}"
        node = ExistNode(children, quantified_vehicle, node_name)
        # Replace sub-formula inside the quantification by a "virtual" predicate g...
        self._rewriter.replace(
            "predicate",
            ctx.start.tokenIndex,
            ctx.stop.tokenIndex,
            node_name,
        )
        return [node]

    def visitSpecNested(self, ctx: FaStlParser.SpecNestedContext):
        children = self.visitChildren(ctx)
        # De-duplicate
        children = list(dict.fromkeys(children))
        Interval1 = Interval(ctx.start.tokenIndex, ctx.stop.tokenIndex)

        if not isinstance(ctx.parentCtx, FaStlParser.SpecNestedContext):
            # Flatten tree to evaluate with rtamt
            return [
                RuleNode(
                    children,
                    self._rewriter.getText(
                        "predicate", ctx.start.tokenIndex, ctx.stop.tokenIndex
                    ),
                    # self._rewriter.getText(
                    #     "predicate",Interval1
                    # ),
                    f"g{self._sub_rule_counter}",
                )
            ]
        else:
            return children
