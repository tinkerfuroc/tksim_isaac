from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence


@dataclass(frozen=True)
class Evaluation:
    success: bool
    satisfied: tuple[str, ...]
    failed: tuple[str, ...]


def _lookup(state: Mapping[str, object], dotted_path: str) -> object:
    value: object = state
    for component in dotted_path.split("."):
        if not isinstance(value, Mapping) or component not in value:
            raise KeyError(dotted_path)
        value = value[component]
    return value


class PostconditionEvaluator:
    """Scores claimed task success only from hidden world postconditions."""

    def evaluate(
        self, conditions: Sequence[Mapping[str, object]], truth: Mapping[str, object]
    ) -> Evaluation:
        satisfied: list[str] = []
        failed: list[str] = []
        for condition in conditions:
            name = str(condition["name"])
            try:
                actual = _lookup(truth, str(condition["path"]))
                passed = self._compare(actual, condition)
            except (KeyError, TypeError, ValueError):
                passed = False
            (satisfied if passed else failed).append(name)
        return Evaluation(not failed, tuple(satisfied), tuple(failed))

    @staticmethod
    def _compare(actual: object, condition: Mapping[str, object]) -> bool:
        operator = condition["operator"]
        expected = condition.get("value")
        if operator == "equals":
            return actual == expected
        if operator == "less_than_or_equal":
            return float(actual) <= float(expected)
        if operator == "greater_than_or_equal":
            return float(actual) >= float(expected)
        if operator == "contains":
            return expected in actual  # type: ignore[operator]
        if operator == "set_equals":
            return set(actual) == set(expected)  # type: ignore[arg-type]
        if operator == "distance_less_than_or_equal":
            actual_values = tuple(float(value) for value in actual)  # type: ignore[union-attr]
            expected_values = tuple(float(value) for value in expected)  # type: ignore[union-attr]
            return (
                math.dist(actual_values, expected_values)
                <= float(condition["tolerance"])
            )
        raise ValueError(f"unsupported postcondition operator: {operator}")
