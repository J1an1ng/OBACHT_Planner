import math
from abc import ABCMeta, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


class IRobustnessScaler(metaclass=ABCMeta):
    @abstractmethod
    def scale_speed(self, x):
        pass

    @abstractmethod
    def scale_acc(self, x):
        pass

    @abstractmethod
    def scale_lon_dist(self, x):
        pass

    @abstractmethod
    def scale_lat_dist(self, x):
        pass

    @abstractmethod
    def scale_angle(self, x):
        pass


@dataclass(frozen=True)
class RobustnessScalingConstants:
    MAX_LONG_DIST: float = 200.0
    MAX_LAT_DIST: float = 20.0
    MAX_SPEED: float = 250.0 / 3.6
    MAX_ACC: float = 10.5
    MAX_ANGLE: float = math.pi


class RobustnessScaler(IRobustnessScaler):
    def __init__(
        self,
        scale: bool = True,
        scale_constants: Optional[RobustnessScalingConstants] = None,
    ):
        self._scale_constants = scale_constants or RobustnessScalingConstants()
        self.scale = scale

    def _scale(self, x, max_value):
        return np.clip(x / max_value, -1.0, 1.0) if self.scale else x

    def scale_speed(self, x):
        return self._scale(x, self._scale_constants.MAX_SPEED)

    def scale_acc(self, x):
        return self._scale(x, self._scale_constants.MAX_ACC)

    def scale_lon_dist(self, x):
        return self._scale(x, self._scale_constants.MAX_LONG_DIST)

    def scale_lat_dist(self, x):
        return self._scale(x, self._scale_constants.MAX_LAT_DIST)

    def scale_angle(self, x):
        return self._scale(x, self._scale_constants.MAX_ANGLE)
