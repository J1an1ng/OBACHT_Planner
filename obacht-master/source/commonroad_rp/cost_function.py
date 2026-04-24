from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

import source.commonroad_rp.trajectories


class CostFunction(ABC):
    """
    Abstract base class for new cost functions
    """

    def __init__(self):
        pass

    @abstractmethod
    def evaluate(self, trajectory: source.commonroad_rp.trajectories.TrajectorySample) -> float:
        """
        Computes the costs of a given trajectory sample
        :param trajectory: The trajectory sample for the cost computation
        :return: The cost of the given trajectory sample
        """
        pass


class DefaultCostFunction(CostFunction):
    """
    Default cost function for comfort driving
    """

    def __init__(self, desired_speed: Optional[float] = None, desired_d: float = 0.0,
                 desired_s: Optional[float] = None):
        super(DefaultCostFunction, self).__init__()
        # target states
        self.desired_speed = desired_speed
        self.desired_d = desired_d
        self.desired_s = desired_s

        # weights
        self.w_a = 5    # acceleration weight
        self.w_jerk = 20
    def evaluate(self, trajectory: source.commonroad_rp.trajectories.TrajectorySample):
        costs = 0.0
        # acceleration costs
        costs += np.sum((self.w_a * trajectory.cartesian.a) ** 2)
        # velocity costs
        jerk = np.diff(trajectory.cartesian.a) / trajectory.dt
        costs += np.sum((self.w_jerk * jerk) ** 2)

        if self.desired_speed is not None:
            costs += np.sum((5 * (trajectory.cartesian.v - self.desired_speed)) ** 2) + \
                     (50 * (trajectory.cartesian.v[-1] - self.desired_speed) ** 2) + \
                     (100 * (trajectory.cartesian.v[int(len(trajectory.cartesian.v)/2)] - self.desired_speed) ** 2)
        if self.desired_s is not None:
            costs += np.sum((0.25 * (self.desired_s - trajectory.curvilinear.s)) ** 2) + \
                 (20 * (self.desired_s - trajectory.curvilinear.s[-1])) ** 2

        # distance costs
        costs += np.sum((0.25 * (self.desired_d - trajectory.curvilinear.d)) ** 2) + \
                 (20 * (self.desired_d - trajectory.curvilinear.d[-1])) ** 2

        # orientation costs
        costs += np.sum((0.25 * np.abs(trajectory.curvilinear.theta)) ** 2) + (
                5 * (np.abs(trajectory.curvilinear.theta[-1]))) ** 2
        # costs += np.sum((250 * np.abs(trajectory.curvilinear.theta)) ** 2) + (
        #         250 * (np.abs(trajectory.curvilinear.theta[-1]))) ** 2

        return costs


class DefaultCostFunctionFailSafe(CostFunction):
    """
    Default cost function for fail-safe trajectory planning
    """

    def __init__(self):
        super(DefaultCostFunctionFailSafe, self).__init__()

    def evaluate(self, trajectory: source.commonroad_rp.trajectories.TrajectorySample):

        # acceleration costs
        costs = np.sum((1 * trajectory.cartesian.a) ** 2)
        # distance costs
        costs += np.sum((0.25 * trajectory.curvilinear.d) ** 2) + (20 * trajectory.curvilinear.d[-1]) ** 2
        # orientation costs
        costs += np.sum((0.25 * np.abs(trajectory.curvilinear.theta)) ** 2) + (
                5 * (np.abs(trajectory.curvilinear.theta[-1]))) ** 2

        return costs