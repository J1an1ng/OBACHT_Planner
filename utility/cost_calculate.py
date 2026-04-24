# import numpy as np
# from typing import Dict, List, Optional, Tuple
# from collections import defaultdict
# import pandas as pd
#
#
# class TrajectoryCostTracker_1:
#     """
#     准确的轨迹成本追踪器
#     只计算实际执行的轨迹点的成本，但考虑规划轨迹终点的影响
#     """
#
#     def __init__(self):
#         # 按状态存储成本
#         self.state_costs = defaultdict(lambda: {
#             'velocity': [],
#             'acceleration': [],
#             'jerk': [],
#             'orientation': [],
#             'distance': [],
#             'total': [],
#             'executed_segments': []  # 存储实际执行的轨迹段信息
#         })
#
#         # 存储每个执行点的详细成本
#         self.point_costs = {
#             'time_step': [],  # 时间步
#             'state': [],  # 所在状态
#             'velocity_cost': [],  # 速度成本
#             'acceleration_cost': [],  # 加速度成本
#             'jerk_cost': [],  # 加加速度成本
#             'orientation_cost': [],  # 方向成本
#             'distance_cost': [],  # 距离成本
#             'total_cost': [],  # 总成本
#             # 执行点的实际状态
#             'actual_v': [],  # 实际速度
#             'actual_a': [],  # 实际加速度
#             'actual_d': [],  # 实际横向位置
#             'actual_s': [],  # 实际纵向位置
#             'actual_theta': [],  # 实际方向角
#             # 期望值
#             'desired_v': [],  # 期望速度
#             'desired_d': [],  # 期望横向位置
#             'desired_s': [],  # 期望纵向位置（如果有）
#         }
#
#         # 追踪规划信息
#         self.planning_info = []
#
#         # 当前时间步
#         self.current_time_step = 0
#
#     def calculate_point_cost(self, point_idx: int, trajectory_sample, cost_function,
#                              planning_final_state: Dict,executed_length: int) -> Dict[str, float]:
#         """
#         计算单个执行点的成本
#
#         Args:
#             point_idx: 执行点在轨迹中的索引
#             trajectory_sample: 完整的规划轨迹
#             cost_function: 成本函数
#             planning_final_state: 规划轨迹的终点状态
#
#         Returns:
#             该点的各项成本
#         """
#         costs = {}
#
#         # 获取权重
#         w_a = getattr(cost_function, 'w_a', 5)
#         w_jerk = getattr(cost_function, 'w_jerk', 20)
#
#         # 1. 加速度成本 - 只计算该点的
#         a = trajectory_sample.cartesian.a[point_idx]
#         costs['acceleration'] = (w_a * a) ** 2
#
#         # 2. 加加速度成本 - 需要前一个点
#         if point_idx > 0:
#             jerk = (trajectory_sample.cartesian.a[point_idx] -
#                     trajectory_sample.cartesian.a[point_idx - 1]) / trajectory_sample.dt
#             costs['jerk'] = (w_jerk * jerk) ** 2
#         else:
#             costs['jerk'] = 0.0
#
#         # 3. 速度成本 - 考虑终点
#         if hasattr(cost_function, 'desired_speed') and cost_function.desired_speed is not None:
#             v = trajectory_sample.cartesian.v[point_idx]
#             v_desired = cost_function.desired_speed
#             v_final = planning_final_state['v']
#
#             # 基础成本（当前点）
#             costs['velocity'] = (5 * (v - v_desired)) ** 2
#
#             # 如果这是执行段的最后一个点，需要考虑规划终点的影响
#             if point_idx == executed_length - 1:
#                costs['velocity'] += (50 * (v_final - v_desired) ** 2)
#
#
#             v_mid = trajectory_sample.cartesian.v[len(trajectory_sample.cartesian.v) // 2]
#             costs['velocity'] += (100 * (v_mid - v_desired) ** 2)
#         else:
#             costs['velocity'] = 0.0
#
#         # 4. 纵向位置成本 - 考虑终点
#         if hasattr(cost_function, 'desired_s') and cost_function.desired_s is not None:
#             s = trajectory_sample.curvilinear.s[point_idx]
#             s_desired = cost_function.desired_s
#             s_final = planning_final_state['s']
#
#             # 基础成本
#             costs['longitudinal'] = (0.25 * (s_desired - s)) ** 2
#
#             if point_idx == executed_length - 1:
#                 costs['longitudinal'] += (20 * (s_desired - s_final) ** 2)
#         else:
#             costs['longitudinal'] = 0.0
#
#
#         d = trajectory_sample.curvilinear.d[point_idx]
#         d_desired = getattr(cost_function, 'desired_d', 0.0)
#         d_final = planning_final_state['d']
#
#         # 基础成本
#         costs['distance'] = (0.25 * (d_desired - d)) ** 2
#
#         if point_idx == executed_length - 1:
#             costs['distance'] += (20 * (d_desired - d_final) ** 2)
#
#         # 6. 方向成本 - 考虑终点
#         theta = trajectory_sample.curvilinear.theta[point_idx]
#         theta_final = planning_final_state['theta']
#
#         # 基础成本
#         costs['orientation'] = (0.25 * np.abs(theta)) ** 2
#
#         if point_idx == executed_length - 1:
#             costs['orientation'] += (5 * np.abs(theta_final)) ** 2
#
#         # 总成本
#         costs['total'] = (costs['acceleration'] + costs['velocity'] +
#                           costs.get('longitudinal', 0.0) + costs['distance'] +
#                           costs['orientation'] + costs['jerk'])
#
#         return costs
#
#     def track_planning_step(self, planner, state_name: str, trajectory_sample=None,
#                             replanning_frequency: int = 1):
#         """
#         追踪一次规划步骤的成本
#
#         Args:
#             planner: ReactivePlanner实例
#             state_name: 当前状态名
#             trajectory_sample: 轨迹样本
#             replanning_frequency: 实际执行的轨迹点数
#         """
#         if trajectory_sample is None and hasattr(planner, 'best_sample'):
#             trajectory_sample = planner.best_sample
#
#         if trajectory_sample is None:
#             return
#
#         # 获取规划轨迹的终点状态
#         planning_final_state = {
#             'v': trajectory_sample.cartesian.v[-1],
#             'a': trajectory_sample.cartesian.a[-1],
#             's': trajectory_sample.curvilinear.s[-1],
#             'd': trajectory_sample.curvilinear.d[-1],
#             'theta': trajectory_sample.curvilinear.theta[-1]
#         }
#
#         # 记录规划信息
#         self.planning_info.append({
#             'time_step': self.current_time_step,
#             'state': state_name,
#             'planned_length': len(trajectory_sample.cartesian.x),
#             'executed_length': min(replanning_frequency, len(trajectory_sample.cartesian.x)),
#             'final_v': planning_final_state['v'],
#             'final_s': planning_final_state['s'],
#             'final_d': planning_final_state['d'],
#             'desired_v': planner._desired_speed,
#             'desired_d': getattr(planner.cost_function, 'desired_d', 0.0),
#             'desired_s': planner._desired_lon_position
#         })
#
#         # 计算实际执行的每个点的成本
#         executed_length = min(replanning_frequency, len(trajectory_sample.cartesian.x))
#         segment_costs = []
#
#         for i in range(executed_length):
#             # 计算该点的成本
#             point_cost = self.calculate_point_cost(i, trajectory_sample,
#                                                    planner.cost_function,
#                                                    planning_final_state,executed_length = executed_length)
#
#             # 记录详细信息
#             self.point_costs['time_step'].append(self.current_time_step + i)
#             self.point_costs['state'].append(state_name)
#             self.point_costs['velocity_cost'].append(point_cost['velocity'])
#             self.point_costs['acceleration_cost'].append(point_cost['acceleration'])
#             self.point_costs['jerk_cost'].append(point_cost['jerk'])
#             self.point_costs['orientation_cost'].append(point_cost['orientation'])
#             self.point_costs['distance_cost'].append(point_cost['distance'])
#             self.point_costs['total_cost'].append(point_cost['total'])
#
#             # 记录实际状态
#             self.point_costs['actual_v'].append(trajectory_sample.cartesian.v[i])
#             self.point_costs['actual_a'].append(trajectory_sample.cartesian.a[i])
#             self.point_costs['actual_s'].append(trajectory_sample.curvilinear.s[i])
#             self.point_costs['actual_d'].append(trajectory_sample.curvilinear.d[i])
#             self.point_costs['actual_theta'].append(trajectory_sample.curvilinear.theta[i])
#
#             # 记录期望值
#             self.point_costs['desired_v'].append(
#                 planner._desired_speed if planner._desired_speed is not None else np.nan)
#             self.point_costs['desired_d'].append(getattr(planner.cost_function, 'desired_d', 0.0))
#             self.point_costs['desired_s'].append(
#                 planner._desired_lon_position if planner._desired_lon_position is not None else np.nan)
#
#             segment_costs.append(point_cost)
#
#         # 汇总该段的成本到状态统计
#         if segment_costs:
#             for cost_type in ['velocity', 'acceleration', 'jerk', 'orientation', 'distance', 'total']:
#                 total_cost = sum(cost[cost_type] for cost in segment_costs)
#                 self.state_costs[state_name][cost_type].append(total_cost)
#
#         # 存储执行段信息
#         if executed_length > 0:
#             segment_info = {
#                 'start_time': self.current_time_step,
#                 'length': executed_length,
#                 'planning_final_state': planning_final_state,
#                 'costs': segment_costs
#             }
#             self.state_costs[state_name]['executed_segments'].append(segment_info)
#
#         # 更新时间步
#         self.current_time_step += replanning_frequency
#
#     def get_state_summary(self) -> pd.DataFrame:
#         """获取每个状态的成本统计摘要"""
#         summary_data = []
#
#         for state, costs in self.state_costs.items():
#             state_summary = {'state': state}
#
#             for cost_type in ['velocity', 'acceleration', 'jerk', 'orientation', 'distance', 'total']:
#                 if costs[cost_type]:
#                     state_summary[f'{cost_type}_mean'] = np.mean(costs[cost_type])
#                     state_summary[f'{cost_type}_std'] = np.std(costs[cost_type])
#                     state_summary[f'{cost_type}_sum'] = np.sum(costs[cost_type])
#                     state_summary[f'{cost_type}_segments'] = len(costs[cost_type])
#
#             summary_data.append(state_summary)
#
#         return pd.DataFrame(summary_data)
#
#     def get_point_costs_df(self) -> pd.DataFrame:
#         """获取每个执行点的详细成本数据"""
#         return pd.DataFrame(self.point_costs)
#
#     def get_planning_info_df(self) -> pd.DataFrame:
#         """获取规划信息数据"""
#         return pd.DataFrame(self.planning_info)
#
#     def get_total_costs(self) -> Dict[str, float]:
#         """获取整条轨迹的总成本"""
#         total_costs = defaultdict(float)
#
#         # 从点成本数据中汇总
#         df = self.get_point_costs_df()
#         if not df.empty:
#             for cost_type in ['velocity', 'acceleration', 'jerk', 'orientation', 'distance']:
#                 total_costs[cost_type] = df[f'{cost_type}_cost'].sum()
#             total_costs['total'] = df['total_cost'].sum()
#             total_costs['total_points'] = len(df)
#
#         return dict(total_costs)
#
#
#
#     def generate_report(self) -> str:
#         """生成成本分析报告"""
#         report = []
#         report.append("=" * 60)
#         report.append("轨迹成本分析报告")
#         report.append("=" * 60)
#
#         # 1. 总体统计
#         total_costs = self.get_total_costs()
#         report.append("\n1. 总体成本统计:")
#         report.append(f"   总执行点数: {total_costs.get('total_points', 0)}")
#         for cost_type in ['velocity', 'acceleration', 'jerk', 'orientation', 'distance', 'total']:
#             if cost_type in total_costs:
#                 report.append(f"   {cost_type:15s}: {total_costs[cost_type]:10.2f}")
#
#         # 2. 各状态成本统计
#         state_summary = self.get_state_summary()
#         if not state_summary.empty:
#             report.append("\n2. 各状态成本统计:")
#             report.append(state_summary.to_string())
#
#         # 3. 规划信息统计
#         planning_df = self.get_planning_info_df()
#         if not planning_df.empty:
#             report.append("\n3. 规划统计:")
#             report.append(f"   总规划次数: {len(planning_df)}")
#             report.append(f"   平均规划长度: {planning_df['planned_length'].mean():.1f}")
#             report.append(f"   平均执行长度: {planning_df['executed_length'].mean():.1f}")
#
#         return "\n".join(report)
#
#     def save_all_data(self, prefix: str = "cost_analysis"):
#         """保存所有分析数据"""
#         # 1. 状态摘要
#         self.get_state_summary().to_csv(f"{prefix}_state_summary.csv", index=False)
#
#         # 2. 详细点成本
#         self.get_point_costs_df().to_csv(f"{prefix}_point_costs.csv", index=False)
#
#         # 3. 规划信息
#         self.get_planning_info_df().to_csv(f"{prefix}_planning_info.csv", index=False)
#
#
#         # 5. 文本报告
#         with open(f"{prefix}_report.txt", 'w') as f:
#             f.write(self.generate_report())
#
#         print(f"所有分析数据已保存，前缀: {prefix}")
#
#     def reset(self):
#         """重置追踪器"""
#         self.state_costs.clear()
#         self.point_costs = {k: [] for k in self.point_costs.keys()}
#         self.planning_info = []
#         self.current_time_step = 0


import numpy as np
from typing import Dict, List, Optional, Tuple
from collections import defaultdict
import pandas as pd
import os
from datetime import datetime


class TrajectoryCostTracker_1:
    """
    Accurate trajectory cost tracker
    Only calculates costs for actually executed trajectory points, but considers the impact of planned trajectory endpoints
    """

    def __init__(self):
        # Store costs by state
        self.state_costs = defaultdict(lambda: {
            'velocity': [],
            'acceleration': [],
            'jerk': [],
            'orientation': [],
            'distance': [],
            'total': [],
            'executed_segments': []  # Store information about actually executed trajectory segments
        })

        # Store detailed costs for each executed point
        self.point_costs = {
            'time_step': [],  # Time step
            'state': [],  # Current state
            'velocity_cost': [],  # Velocity cost
            'acceleration_cost': [],  # Acceleration cost
            'jerk_cost': [],  # Jerk cost
            'orientation_cost': [],  # Orientation cost
            'distance_cost': [],  # Distance cost
            'total_cost': [],  # Total cost
            # Actual states at execution points
            'actual_v': [],  # Actual velocity
            'actual_a': [],  # Actual acceleration
            'actual_d': [],  # Actual lateral position
            'actual_s': [],  # Actual longitudinal position
            'actual_theta': [],  # Actual orientation angle
            # Desired values
            'desired_v': [],  # Desired velocity
            'desired_d': [],  # Desired lateral position
            'desired_s': [],  # Desired longitudinal position (if available)
        }

        # Track planning information
        self.planning_info = []

        # Current time step
        self.current_time_step = 0

    def calculate_point_cost(self, point_idx: int, trajectory_sample, cost_function,
                             planning_final_state: Dict, executed_length: int) -> Dict[str, float]:
        """
        Calculate cost for a single executed point

        Args:
            point_idx: Index of execution point in trajectory
            trajectory_sample: Complete planned trajectory
            cost_function: Cost function
            planning_final_state: Final state of planned trajectory
            executed_length: Length of executed segment

        Returns:
            Various costs for this point
        """
        costs = {}

        # Get weights
        w_a = getattr(cost_function, 'w_a', 5)
        w_jerk = getattr(cost_function, 'w_jerk', 20)

        # 1. Acceleration cost - only for this point
        a = trajectory_sample.cartesian.a[point_idx]
        costs['acceleration'] = (w_a * a) ** 2

        # 2. Jerk cost - needs previous point
        if point_idx > 0:
            jerk = (trajectory_sample.cartesian.a[point_idx] -
                    trajectory_sample.cartesian.a[point_idx - 1]) / trajectory_sample.dt
            costs['jerk'] = (w_jerk * jerk) ** 2
        else:
            costs['jerk'] = 0.0

        # 3. Velocity cost - considering endpoint
        if hasattr(cost_function, 'desired_speed') and cost_function.desired_speed is not None:
            v = trajectory_sample.cartesian.v[point_idx]
            v_desired = cost_function.desired_speed
            v_final = planning_final_state['v']

            # Base cost (current point)
            costs['velocity'] = (5 * (v - v_desired)) ** 2

            # If this is the last point of executed segment, consider impact of planned endpoint
            if point_idx == executed_length - 1:
                costs['velocity'] += (50 * (v_final - v_desired) ** 2)

            v_mid = trajectory_sample.cartesian.v[len(trajectory_sample.cartesian.v) // 2]
            costs['velocity'] += (100 * (v_mid - v_desired) ** 2)
        else:
            costs['velocity'] = 0.0

        # 4. Longitudinal position cost - considering endpoint
        if hasattr(cost_function, 'desired_s') and cost_function.desired_s is not None:
            s = trajectory_sample.curvilinear.s[point_idx]
            s_desired = cost_function.desired_s
            s_final = planning_final_state['s']

            # Base cost
            costs['longitudinal'] = (0.25 * (s_desired - s)) ** 2

            if point_idx == executed_length - 1:
                costs['longitudinal'] += (20 * (s_desired - s_final) ** 2)
        else:
            costs['longitudinal'] = 0.0

        # 5. Distance cost - considering endpoint
        d = trajectory_sample.curvilinear.d[point_idx]
        d_desired = getattr(cost_function, 'desired_d', 0.0)
        d_final = planning_final_state['d']

        # Base cost
        costs['distance'] = (0.25 * (d_desired - d)) ** 2

        if point_idx == executed_length - 1:
            costs['distance'] += (20 * (d_desired - d_final) ** 2)

        # 6. Orientation cost - considering endpoint
        theta = trajectory_sample.curvilinear.theta[point_idx]
        theta_final = planning_final_state['theta']

        # Base cost
        costs['orientation'] = (0.25 * np.abs(theta)) ** 2

        if point_idx == executed_length - 1:
            costs['orientation'] += (5 * np.abs(theta_final)) ** 2

        # Total cost
        costs['total'] = (costs['acceleration'] + costs['velocity'] +
                          costs.get('longitudinal', 0.0) + costs['distance'] +
                          costs['orientation'] + costs['jerk'])

        return costs

    def track_planning_step(self, planner, state_name: str, trajectory_sample=None,
                            replanning_frequency: int = 1):
        """
        Track costs for one planning step

        Args:
            planner: ReactivePlanner instance
            state_name: Current state name
            trajectory_sample: Trajectory sample
            replanning_frequency: Number of actually executed trajectory points
        """
        if trajectory_sample is None and hasattr(planner, 'best_sample'):
            trajectory_sample = planner.best_sample

        if trajectory_sample is None:
            return

        # Get final state of planned trajectory
        planning_final_state = {
            'v': trajectory_sample.cartesian.v[-1],
            'a': trajectory_sample.cartesian.a[-1],
            's': trajectory_sample.curvilinear.s[-1],
            'd': trajectory_sample.curvilinear.d[-1],
            'theta': trajectory_sample.curvilinear.theta[-1]
        }

        # Record planning information
        self.planning_info.append({
            'time_step': self.current_time_step,
            'state': state_name,
            'planned_length': len(trajectory_sample.cartesian.x),
            'executed_length': min(replanning_frequency, len(trajectory_sample.cartesian.x)),
            'final_v': planning_final_state['v'],
            'final_s': planning_final_state['s'],
            'final_d': planning_final_state['d'],
            'desired_v': planner._desired_speed,
            'desired_d': getattr(planner.cost_function, 'desired_d', 0.0),
            'desired_s': planner._desired_lon_position
        })

        # Calculate costs for each actually executed point
        executed_length = min(replanning_frequency, len(trajectory_sample.cartesian.x))
        segment_costs = []

        for i in range(executed_length):
            # Calculate cost for this point
            point_cost = self.calculate_point_cost(i, trajectory_sample,
                                                   planner.cost_function,
                                                   planning_final_state, executed_length=executed_length)

            # Record detailed information
            self.point_costs['time_step'].append(self.current_time_step + i)
            self.point_costs['state'].append(state_name)
            self.point_costs['velocity_cost'].append(point_cost['velocity'])
            self.point_costs['acceleration_cost'].append(point_cost['acceleration'])
            self.point_costs['jerk_cost'].append(point_cost['jerk'])
            self.point_costs['orientation_cost'].append(point_cost['orientation'])
            self.point_costs['distance_cost'].append(point_cost['distance'])
            self.point_costs['total_cost'].append(point_cost['total'])

            # Record actual states
            self.point_costs['actual_v'].append(trajectory_sample.cartesian.v[i])
            self.point_costs['actual_a'].append(trajectory_sample.cartesian.a[i])
            self.point_costs['actual_s'].append(trajectory_sample.curvilinear.s[i])
            self.point_costs['actual_d'].append(trajectory_sample.curvilinear.d[i])
            self.point_costs['actual_theta'].append(trajectory_sample.curvilinear.theta[i])

            # Record desired values
            self.point_costs['desired_v'].append(
                planner._desired_speed if planner._desired_speed is not None else np.nan)
            self.point_costs['desired_d'].append(getattr(planner.cost_function, 'desired_d', 0.0))
            self.point_costs['desired_s'].append(
                planner._desired_lon_position if planner._desired_lon_position is not None else np.nan)

            segment_costs.append(point_cost)

        # Aggregate segment costs to state statistics
        if segment_costs:
            for cost_type in ['velocity', 'acceleration', 'jerk', 'orientation', 'distance', 'total']:
                total_cost = sum(cost[cost_type] for cost in segment_costs)
                self.state_costs[state_name][cost_type].append(total_cost)

        # Store executed segment information
        if executed_length > 0:
            segment_info = {
                'start_time': self.current_time_step,
                'length': executed_length,
                'planning_final_state': planning_final_state,
                'costs': segment_costs
            }
            self.state_costs[state_name]['executed_segments'].append(segment_info)

        # Update time step
        self.current_time_step += replanning_frequency

    def get_state_summary(self) -> pd.DataFrame:
        """Get cost statistics summary for each state"""
        summary_data = []

        for state, costs in self.state_costs.items():
            state_summary = {'state': state}

            for cost_type in ['velocity', 'acceleration', 'jerk', 'orientation', 'distance', 'total']:
                if costs[cost_type]:
                    state_summary[f'{cost_type}_mean'] = np.mean(costs[cost_type])
                    state_summary[f'{cost_type}_std'] = np.std(costs[cost_type])
                    state_summary[f'{cost_type}_sum'] = np.sum(costs[cost_type])
                    state_summary[f'{cost_type}_segments'] = len(costs[cost_type])

            summary_data.append(state_summary)

        return pd.DataFrame(summary_data)

    def get_point_costs_df(self) -> pd.DataFrame:
        """Get detailed cost data for each executed point"""
        return pd.DataFrame(self.point_costs)

    def get_planning_info_df(self) -> pd.DataFrame:
        """Get planning information data"""
        return pd.DataFrame(self.planning_info)

    def get_total_costs(self) -> Dict[str, float]:
        """Get total costs for the entire trajectory"""
        total_costs = defaultdict(float)

        # Aggregate from point cost data
        df = self.get_point_costs_df()
        if not df.empty:
            for cost_type in ['velocity', 'acceleration', 'jerk', 'orientation', 'distance']:
                total_costs[cost_type] = df[f'{cost_type}_cost'].sum()
            total_costs['total'] = df['total_cost'].sum()
            total_costs['total_points'] = len(df)

        return dict(total_costs)

    def generate_report(self) -> str:
        """Generate cost analysis report"""
        report = []
        report.append("=" * 60)
        report.append("Trajectory Cost Analysis Report")
        report.append("=" * 60)

        # 1. Overall statistics
        total_costs = self.get_total_costs()
        report.append("\n1. Overall Cost Statistics:")
        report.append(f"   Total executed points: {total_costs.get('total_points', 0)}")
        for cost_type in ['velocity', 'acceleration', 'jerk', 'orientation', 'distance', 'total']:
            if cost_type in total_costs:
                report.append(f"   {cost_type:15s}: {total_costs[cost_type]:10.2f}")

        # 2. Cost statistics by state
        state_summary = self.get_state_summary()
        if not state_summary.empty:
            report.append("\n2. Cost Statistics by State:")
            report.append(state_summary.to_string())

        # 3. Planning information statistics
        planning_df = self.get_planning_info_df()
        if not planning_df.empty:
            report.append("\n3. Planning Statistics:")
            report.append(f"   Total planning iterations: {len(planning_df)}")
            report.append(f"   Average planned length: {planning_df['planned_length'].mean():.1f}")
            report.append(f"   Average executed length: {planning_df['executed_length'].mean():.1f}")

        return "\n".join(report)

    def save_all_data(self, prefix: str = "cost_analysis"):
        """Save all analysis data to ../experiments/output_date directory"""
        # Create output directory with date

        output_dir = os.path.join("..", "experiments", f"output_data")

        # Create directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)

        # File paths with prefix
        base_path = os.path.join(output_dir, prefix)

        # 1. State summary
        state_summary_path = f"{base_path}_state_summary.csv"
        self.get_state_summary().to_csv(state_summary_path, index=False)

        # 2. Detailed point costs
        point_costs_path = f"{base_path}_point_costs.csv"
        self.get_point_costs_df().to_csv(point_costs_path, index=False)

        # 3. Planning information
        planning_info_path = f"{base_path}_planning_info.csv"
        self.get_planning_info_df().to_csv(planning_info_path, index=False)

        # 4. Text report
        report_path = f"{base_path}_report.txt"
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(self.generate_report())

        print(f"All analysis data saved to directory: {output_dir}")
        print(f"Files saved with prefix: {prefix}")
        print(f"Saved files:")
        print(f"  - {os.path.basename(state_summary_path)}")
        print(f"  - {os.path.basename(point_costs_path)}")
        print(f"  - {os.path.basename(planning_info_path)}")
        print(f"  - {os.path.basename(report_path)}")

    def reset(self):
        """Reset the tracker"""
        self.state_costs.clear()
        self.point_costs = {k: [] for k in self.point_costs.keys()}
        self.planning_info = []
        self.current_time_step = 0