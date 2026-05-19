import logging
from typing import Dict, List, Optional, Tuple, Type, Union
import numpy as np
from commonroad.scenario.trajectory import Trajectory
import math
from source.commonroad_rp.utility.utils_coordinate_system import interpolate_angle
from scipy.optimize import minimize
logger = logging.getLogger("RP_LOGGER")

try:
    from commonroad_clcs.pycrccosy import (  # type: ignore[attr-defined]
        CurvilinearProjectionDomainLateralError,
        CurvilinearProjectionDomainLongitudinalError,
    )
    _PROJECTION_DOMAIN_ERRORS = (
        CurvilinearProjectionDomainLateralError,
        CurvilinearProjectionDomainLongitudinalError,
    )
except (ImportError, AttributeError):
    _PROJECTION_DOMAIN_ERRORS = (ValueError, RuntimeError)  # type: ignore[assignment]


from source.commonroad_rp.trajectories import (
    CartesianSample,
    CurviLinearSample,
    FeasibilityStatus,
    TrajectoryBundle,
    TrajectorySample,
)
from source.commonroad_rp.utility.config import (
    ReactivePlannerConfiguration,
    VehicleConfiguration,
)

from source.commonroad_rp.reactive_planner import ReactivePlanner




class PostOptimizerPlanner(ReactivePlanner):
    """
    A planner that inherits from ReactivePlanner and adds post-optimization functionality
    for optimal trajectories. It uses scipy.optimize.minimize to fine-tune the terminal
    states of trajectories for better cost.
    """

    def __init__(self, config: ReactivePlannerConfiguration):
        """
        Initialize the post-optimization planner.
        """
        super().__init__(config)

    def _optimization_cost_function(self, x_array, mode, additional_params=None):
        """
        Unified cost function for optimization.

        Args:
            x_array: Optimization variables
            mode: "velocity_keeping" or "stopping"
            additional_params: Additional parameters needed for trajectory generation
        """
        if mode == "velocity_keeping":
            d_val, v_val = float(x_array[0]), float(x_array[1])
            delta_tau = additional_params
            traj = self._generate_trajectory_for_optimization(d_val, v_val, delta_tau, mode)
        else:  # stopping
            s_val, d_val, tau_val = map(float, x_array)
            traj = self._generate_trajectory_for_optimization(d_val, s_val, tau_val, mode)

        if traj is None:
            return 1e10

        if self._check_single_kinematics(traj) < 0:
            return 1e10

        if self._check_collision_traj_sample(traj):
            return 1e10

        return self.cost_function.evaluate(traj)

    def _optimization_feasibility_constraint(self, x_array, mode, additional_params=None):
        """
        Unified feasibility constraint for optimization.

        Args:
            x_array: Optimization variables
            mode: "velocity_keeping" or "stopping"
            additional_params: Additional parameters needed for trajectory generation
        """
        if mode == "velocity_keeping":
            d_val, v_val = float(x_array[0]), float(x_array[1])
            delta_tau = additional_params
            traj = self._generate_trajectory_for_optimization(d_val, v_val, delta_tau, mode)
        else:  # stopping
            s_val, d_val, tau_val = map(float, x_array)
            traj = self._generate_trajectory_for_optimization(d_val, s_val, tau_val, mode)

        if traj is None:
            return -1.0

        if self._check_single_kinematics(traj) < 0:
            return -1.0

        if self._check_collision_traj_sample(traj):
            return -1.0

        return 1.0

    def _execute_optimization(self, x0, bounds, mode, additional_params=None):
        """
        Execute optimization with unified parameters.

        Args:
            x0: Initial guess
            bounds: Optimization bounds
            mode: "velocity_keeping" or "stopping"
            additional_params: Additional parameters for trajectory generation
        """
        constraints = [{
            "type": "ineq",
            "fun": lambda x: self._optimization_feasibility_constraint(x, mode, additional_params)
        }]

        return minimize(
            fun=lambda x: self._optimization_cost_function(x, mode, additional_params),
            x0=x0,
            bounds=bounds,
            constraints=constraints,
            method="SLSQP",
            options={'maxiter': 100, 'ftol': 1e-4}
        )

    def post_optimize(self, optimal_trajectory: TrajectorySample) -> Tuple[Trajectory, List, List]:
        """
        Post-optimize the trajectory by fine-tuning terminal lateral position d and velocity v.
        """
        if optimal_trajectory is None:
            print("post_optimize: Input trajectory is None!")
            return None, [], []

        # Extract parameters from current trajectory
        d_end = optimal_trajectory.curvilinear.d[-1]
        v_end = optimal_trajectory.trajectory_long.x_d[0]
        delta_tau = optimal_trajectory.trajectory_long.delta_tau

        # Set optimization parameters
        d_bounds_tolerance = self.config.optimization.d_bounds_tolerance
        v_bounds_tolerance = self.config.optimization.v_bounds_tolerance
        x0 = [d_end, v_end]
        bounds = [(d_end - d_bounds_tolerance, d_end + d_bounds_tolerance), (v_end - v_bounds_tolerance, v_end + v_bounds_tolerance)]

        # Execute optimization
        result = self._execute_optimization(x0, bounds, "velocity_keeping", delta_tau)

        # Process results
        if result.success:
            d_opt, v_opt = result.x[0], result.x[1]
            if abs(d_opt - d_end) > 0.01 or abs(v_opt - v_end) > 0.1:
                logger.debug(
                    f"Optimization improved trajectory: d: {d_end:.3f}->{d_opt:.3f}, v: {v_end:.3f}->{v_opt:.3f}")
        else:
            logger.debug(f"Optimization did not improve trajectory: {result.message}")
            d_opt, v_opt = d_end, v_end

        # Generate final optimized trajectory
        final_trajectory = self._generate_trajectory_for_optimization(
            d_opt, v_opt, delta_tau, "velocity_keeping"
        )


        return self._process_and_validate_optimized_trajectory(final_trajectory)

    def post_optimize_for_stopping(self, optimal_trajectory: TrajectorySample) -> Tuple[Trajectory, List, List]:
        """
        Post-optimize stopping trajectory by fine-tuning terminal longitudinal position s,
        lateral position d and time tau.
        """
        if optimal_trajectory is None:
            logger.warning("post_optimize_for_stopping: Input trajectory is None!")
            return None, [], []

        # Extract parameters from current trajectory
        s_end = optimal_trajectory.curvilinear.s[-1]
        d_end = optimal_trajectory.curvilinear.d[-1]
        delta_tau = optimal_trajectory.trajectory_long.delta_tau

        # Set optimization parameters
        x0 = [s_end, d_end, delta_tau]
        max_delta_tau = (self.N - 1) * self.dt
        s_bounds_tolerance = self.config.optimization.s_bounds_tolerance
        d_bounds_tolerance = self.config.optimization.d_bounds_tolerance_for_stopping
        bounds = [
            (s_end - s_bounds_tolerance , s_end + s_bounds_tolerance ),
            (d_end - d_bounds_tolerance, d_end + d_bounds_tolerance),
            (0.0, min(delta_tau, max_delta_tau))
        ]

        # Execute optimization
        result = self._execute_optimization(x0, bounds, "stopping")

        # Process results
        if result.success:
            s_opt, d_opt, tau_opt = result.x[0], result.x[1], result.x[2]
            if abs(s_opt - s_end) > 0.01 or abs(d_opt - d_end) > 0.01:
                logger.debug(f"Stopping optimization: s: {s_end:.2f}->{s_opt:.2f}, d: {d_end:.2f}->{d_opt:.2f}")

        else:
            s_opt, d_opt, tau_opt = s_end, d_end, delta_tau

        # Generate final optimized trajectory
        final_trajectory = self._generate_trajectory_for_optimization(
            d_opt, s_opt, tau_opt, "stopping"
        )

        return self._process_and_validate_optimized_trajectory(final_trajectory)

    def _generate_trajectory_for_optimization(
            self, d: float, lon_sample: float, delta_tau: float, mode: str
    ) -> Union[TrajectorySample, None]:
        """
        Internal method to generate trajectory for optimization process.

        Args:
            d: Lateral position
            lon_sample: Longitudinal sample value (velocity or position, depending on mode)
            delta_tau: Time duration
            mode: "velocity_keeping" or "stopping"
        """
        # Ensure d is a float
        if isinstance(d, (np.ndarray, list, tuple)):
            d = float(d[0]) if len(d) > 0 else 0.0

        # Set lateral terminal state
        end_state_lat = np.array([d, 0.0, 0.0])

        # Get initial states
        x_0_lon, x_0_lat = self.x_0_cl

        # Generate longitudinal trajectory
        try:
            lon_tra = self.sampling_space._generate_lon_trajectory(
                delta_tau=delta_tau,
                x_0=np.array(x_0_lon),
                lon_sample=lon_sample,
                mode=mode,
            )

            if lon_tra is None:
                return None

            # Generate lateral trajectory
            lat_tra = self.sampling_space._generate_lat_trajectory(
                delta_tau=lon_tra.delta_tau,
                x_0=np.array(x_0_lat),
                x_d=end_state_lat,
            )

            # Create trajectory sample
            return TrajectorySample(delta_tau, self.dt, lon_tra, lat_tra)

        except Exception as e:
            return None

    def _process_and_validate_optimized_trajectory(
            self, trajectory: TrajectorySample
    ) -> Tuple[Trajectory, List, List]:
        """
        Process and validate the optimized trajectory, then generate final output.
        """
        if trajectory is None:
            return None, [], []

        # Perform kinematic check, which populates cartesian and curvilinear attributes
        if self._check_single_kinematics(trajectory) < 0:
            return None, [], []

        # Perform collision check
        if self._check_collision_traj_sample(trajectory):
            return None, [], []

        # Calculate cost (since cartesian has been populated)
        trajectory._cost = self.cost_function.evaluate(trajectory)

        # Update planner state
        self._cost_value = trajectory.cost
        self.optimal_longitudinal = trajectory.trajectory_long
        self.optimal_lateral = trajectory.trajectory_lat
        self.best_sample = trajectory

        # Create CommonRoad output
        planning_result = self._create_output(trajectory)
        if planning_result is None:
            return None, [], []

        return planning_result

    def _check_single_kinematics(self, trajectory: TrajectorySample) -> float:
        """
        Checks the kinematics of a single trajectory sample and returns the feasibility result.

        :param trajectory: The trajectory sample to check.
        :return: 1.0 if feasible, -1.0 if infeasible.
        """


        # Precision value
        _EPS = 1e-5

        # Create time array and precompute time interval information
        t = np.arange(
            0, np.round(trajectory.trajectory_long.delta_tau + self.dt, 5), self.dt
        )
        t2 = np.square(t)
        t3 = t2 * t
        t4 = np.square(t2)
        t5 = t4 * t

        # Initialize state vectors
        s = np.zeros(self.N + 1)
        s_velocity = np.zeros(self.N + 1)
        s_acceleration = np.zeros(self.N + 1)
        d = np.zeros(self.N + 1)
        d_velocity = np.zeros(self.N + 1)
        d_acceleration = np.zeros(self.N + 1)

        traj_len = len(t)

        # 1) Calculate longitudinal s, s_dot, s_ddot
        s[:traj_len] = trajectory.trajectory_long.calc_position(t, t2, t3, t4, t5)
        s_velocity[:traj_len] = trajectory.trajectory_long.calc_velocity(t, t2, t3, t4)
        s_acceleration[:traj_len] = trajectory.trajectory_long.calc_acceleration(
            t, t2, t3
        )

        # 2) Calculate lateral d, d_dot, d_ddot
        if not self._low_vel_mode:
            # High-speed mode: lateral polynomial uses time t as variable
            d[:traj_len] = trajectory.trajectory_lat.calc_position(t, t2, t3, t4, t5)
            d_velocity[:traj_len] = trajectory.trajectory_lat.calc_velocity(
                t, t2, t3, t4
            )
            d_acceleration[:traj_len] = trajectory.trajectory_lat.calc_acceleration(
                t, t2, t3
            )
        else:
            # Low-speed mode: lateral polynomial uses s (arc length traveled) as variable
            s1 = s[:traj_len] - s[0]
            s2 = s1 * s1
            s3 = s2 * s1
            s4 = s2 * s2
            s5 = s4 * s1
            d[:traj_len] = trajectory.trajectory_lat.calc_position(s1, s2, s3, s4, s5)
            d_velocity[:traj_len] = trajectory.trajectory_lat.calc_velocity(
                s1, s2, s3, s4
            )
            d_acceleration[:traj_len] = trajectory.trajectory_lat.calc_acceleration(
                s1, s2, s3
            )

        s_velocity[np.abs(s_velocity) < _EPS] = 0.0
        d_velocity[np.abs(d_velocity) < _EPS] = 0.0

        x = np.zeros(self.N + 1)  # global X
        y = np.zeros(self.N + 1)  # global Y
        v = np.zeros(self.N + 1)  # global velocity
        a = np.zeros(self.N + 1)  # global acceleration
        theta_gl = np.zeros(self.N + 1)  # global orientation
        theta_cl = np.zeros(self.N + 1)  # curvilinear orientation
        kappa_gl = np.zeros(self.N + 1)  # global curvature
        kappa_cl = np.zeros(self.N + 1)  # relative curvature (kappa_gl - k_r)

        feasible = True

        if not self._draw_traj_set:
            if np.any(np.abs(s_acceleration) > self.vehicle_params.a_max):
                self._infeasible_reason_dict["acceleration"] += 1
                feasible = False
            elif np.any(s_velocity < -_EPS):
                self._infeasible_reason_dict["velocity"] += 1
                feasible = False

        if not feasible:
            trajectory.feasibility_label = FeasibilityStatus.INFEASIBLE_KINEMATIC
            return -1.0

        for i in range(traj_len):
            # ---------- (1) Lateral derivatives, second derivatives dp, dpp ----------
            if not self._low_vel_mode:
                # High-speed mode: dp = d_dot / s_dot, dpp = ...
                if s_velocity[i] > 0.001:
                    dp = d_velocity[i] / (s_velocity[i] if s_velocity[i] != 0 else 1e-6)
                else:
                    dp = 0.0

                # d' is first derivative with respect to s, so dpp = (d_dot_dot - dp * s_ddot) / s_dot^2
                ddot = d_acceleration[i] - dp * s_acceleration[i]
                if s_velocity[i] > 0.001:
                    dpp = ddot / (s_velocity[i] ** 2)
                else:
                    dpp = 0.0
            else:
                # Low-speed mode: dp = d_dot, dpp = d_ddot (they are derivatives with respect to s)
                dp = d_velocity[i]
                dpp = d_acceleration[i]

            # ---------- (2) Interpolate curvature k_r, k_r_d, and ref theta at s_i on reference line ----------
            s_idx = np.argmax(self._co.ref_pos > s[i]) - 1
            # If s[i] exceeds reference line definition range, mark as infeasible
            if s_idx + 1 >= len(self._co.ref_pos) or s_idx < 0:
                feasible = False
                # break

            s_lambda = (s[i] - self._co.ref_pos[s_idx]) / (
                    self._co.ref_pos[s_idx + 1] - self._co.ref_pos[s_idx]
            )
            k_r = (
                          self._co.ref_curv[s_idx + 1] - self._co.ref_curv[s_idx]
                  ) * s_lambda + self._co.ref_curv[s_idx]
            k_r_d = (
                            self._co.ref_curv_d[s_idx + 1] - self._co.ref_curv_d[s_idx]
                    ) * s_lambda + self._co.ref_curv_d[s_idx]

            # ---------- (3) Calculate yaw angle theta_cl[i] in curvilinear coordinates and global yaw angle theta_gl[i] ----------
            # Reference: Werling Dissertation - Appendix A
            if not self._low_vel_mode:
                if s_velocity[i] > 0.001:
                    # theta_cl = arctan(d') = arctan(dp)
                    theta_cl[i] = np.arctan2(dp, 1.0)
                    # theta_gl = theta_cl + reference line direction
                    theta_gl[i] = theta_cl[i] + interpolate_angle(
                        s[i],
                        self._co.ref_pos[s_idx],
                        self._co.ref_pos[s_idx + 1],
                        self._co.ref_theta[s_idx],
                        self._co.ref_theta[s_idx + 1],
                    )
                else:
                    # When velocity is approximately 0, maintain previous global angle or initial angle
                    if i == 0:
                        theta_gl[i] = self.x_0.orientation
                    else:
                        theta_gl[i] = theta_gl[i - 1]
                    theta_cl[i] = theta_gl[i] - interpolate_angle(
                        s[i],
                        self._co.ref_pos[s_idx],
                        self._co.ref_pos[s_idx + 1],
                        self._co.ref_theta[s_idx],
                        self._co.ref_theta[s_idx + 1],
                    )
            else:
                # In low-speed mode, d' = dp is already derivative with respect to s
                theta_cl[i] = np.arctan2(dp, 1.0)
                theta_gl[i] = theta_cl[i] + interpolate_angle(
                    s[i],
                    self._co.ref_pos[s_idx],
                    self._co.ref_pos[s_idx + 1],
                    self._co.ref_theta[s_idx],
                    self._co.ref_theta[s_idx + 1],
                )

            # ---------- (4) Calculate global curvature kappa_gl[i] ----------
            # kappa_gl = ...
            oneKrD = 1.0 - k_r * d[i]
            cosTheta = math.cos(theta_cl[i])
            tanTheta = math.tan(theta_cl[i])
            # global curvature
            kappa_gl[i] = (dpp + (k_r * dp + k_r_d * d[i]) * tanTheta) * cosTheta * (
                    (cosTheta / oneKrD) ** 2
            ) + (cosTheta / oneKrD) * k_r
            # relative curvature
            kappa_cl[i] = kappa_gl[i] - k_r

            # ---------- (5) Calculate global velocity v[i], acceleration a[i] ----------
            # v[i] = s_dot * (1 - k_r * d) / cos(theta_cl)
            v[i] = s_velocity[i] * oneKrD / (cosTheta if abs(cosTheta) > 1e-6 else 1e-6)

            # a[i] = ...
            a[i] = (
                    s_acceleration[i]
                    * oneKrD
                    / (cosTheta if abs(cosTheta) > 1e-6 else 1e-6)
            )
            a[i] += (
                            s_velocity[i] ** 2 / (cosTheta if abs(cosTheta) > 1e-6 else 1e-6)
                    ) * (
                            oneKrD
                            * tanTheta
                            * (
                                    kappa_gl[i] * oneKrD / (cosTheta if abs(cosTheta) > 1e-6 else 1e-6)
                                    - k_r
                            )
                            - (k_r_d * d[i] + k_r * dp)
                    )

            # ---------- (6) Check various dynamics constraints -----------
            if feasible:
                # velocity >= 0
                if "velocity" in self.config.planning.constraints_to_check:
                    if v[i] < -_EPS:
                        self._infeasible_reason_dict["velocity"] += 1
                        feasible = False
                        # break

                # curvature constraint
                kappa_max = (
                        np.tan(self.vehicle_params.delta_max)
                        / self.vehicle_params.wheelbase
                )
                if "kappa" in self.config.planning.constraints_to_check:
                    if abs(kappa_gl[i]) > kappa_max:
                        self._infeasible_reason_dict["kappa"] += 1
                        feasible = False
                        # break

                # yaw_rate
                if "yaw_rate" in self.config.planning.constraints_to_check and i > 0:
                    yaw_rate = (theta_gl[i] - theta_gl[i - 1]) / self.dt
                    theta_dot_max = kappa_max * v[i]
                    if abs(yaw_rate) > theta_dot_max:
                        self._infeasible_reason_dict["yaw_rate"] += 1
                        feasible = False
                        # break

                # kappa_dot
                if "kappa_dot" in self.config.planning.constraints_to_check and i > 0:
                    kappa_dot = (kappa_gl[i] - kappa_gl[i - 1]) / self.dt
                    # Formula similar to yaw_rate, or use vehicle parameter v_delta_max
                    steering_angle = np.arctan2(
                        self.vehicle_params.wheelbase * kappa_gl[i], 1.0
                    )
                    kappa_dot_max = self.vehicle_params.v_delta_max / (
                            self.vehicle_params.wheelbase * math.cos(steering_angle) ** 2
                    )
                    if abs(kappa_dot) > kappa_dot_max:
                        self._infeasible_reason_dict["kappa_dot"] += 1
                        feasible = False
                        # break

                # acceleration
                if "acceleration" in self.config.planning.constraints_to_check:
                    v_switch = self.vehicle_params.v_switch
                    if v[i] > v_switch:
                        a_max = self.vehicle_params.a_max * v_switch / (v[i] + 1e-6)
                    else:
                        a_max = self.vehicle_params.a_max
                    a_min = -self.vehicle_params.a_max
                    if not (a_min <= a[i] <= a_max):
                        self._infeasible_reason_dict["acceleration"] += 1
                        feasible = False
                        # break

            if not feasible:
                trajectory.feasibility_label = FeasibilityStatus.INFEASIBLE_KINEMATIC
                # break

        # 6) If still feasible, convert (s, d) back to global coordinates (x, y) and check if within reference line projection range
        if feasible:
            for i in range(traj_len):
                try:
                    pos = self._co.convert_to_cartesian_coords(s[i], d[i])
                except _PROJECTION_DOMAIN_ERRORS:
                    # s or d falls outside the projection domain – treat as infeasible
                    pos = None
                if pos is None:
                    # Outside reference line projectable area
                    feasible = False
                    break
                x[i], y[i] = pos

        # 7) Final feasible/infeasible determination
        if feasible:
            # Mark as FEASIBLE
            trajectory.feasibility_label = FeasibilityStatus.FEASIBLE

            # Populate CartesianSample
            trajectory.cartesian = CartesianSample(
                x=x,
                y=y,
                theta=theta_gl,
                v=v,
                a=a,
                kappa=kappa_gl,
                kappa_dot=np.append([0], np.diff(kappa_gl)),
                current_time_step=traj_len,
            )

            # Populate CurviLinearSample
            trajectory.curvilinear = CurviLinearSample(
                s=s,
                d=d,
                theta=theta_cl,
                ss=s_velocity,
                sss=s_acceleration,
                dd=d_velocity,
                ddd=d_acceleration,
                current_time_step=traj_len,
            )

            # If trajectory is shorter than planning horizon, enlarge it
            if self.N + 1 > trajectory.cartesian.current_time_step:
                trajectory.enlarge(self.dt)

            return 1.0
        else:
            trajectory.feasibility_label = FeasibilityStatus.INFEASIBLE_KINEMATIC
            return -1.0