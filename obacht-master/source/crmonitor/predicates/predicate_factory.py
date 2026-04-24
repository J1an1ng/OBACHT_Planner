import inspect
import re
import sys
from typing import Optional

from source.crmonitor.common.config import get_traffic_rule_config

# by setting __all__ in __init__.py, all relevant modules are imported
# noinspection PyUnresolvedReferences
from source.crmonitor.predicates import *  # noqa: F401,F403
from source.crmonitor.predicates.base import BasePredicateEvaluator


class PredicateFactory:
    def __init__(self, traffic_rule_params: Optional[dict] = None):
        self._traffic_rule_params = (
            traffic_rule_params or get_traffic_rule_config()["traffic_rules_param"]
        )
        self._evaluators = self._get_all_predicate_evaluators()

    @staticmethod
    def _get_all_predicate_evaluators():
        modules = inspect.getmembers(
            sys.modules["source.crmonitor.predicates"], inspect.ismodule
        )
        classes = []
        for _, module in modules:
            classes += inspect.getmembers(module, inspect.isclass)

        predicate_class_map = {
            cls.predicate_name: cls
            for name, cls in classes
            if re.match(r"^Pred[A-Z].*$", name) is not None
        }
        return predicate_class_map

    def get_predicate(self, predicate_name: str) -> BasePredicateEvaluator:
        try:
            evaluator = self._evaluators[predicate_name]
        except KeyError:
            raise KeyError(f"Unknown predicate '{predicate_name}'")
        return evaluator(self._traffic_rule_params)
