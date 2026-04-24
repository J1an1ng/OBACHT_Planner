# from commonroad.common.file_reader import CommonRoadFileReader
# from source.crmonitor.common.helper import load_yaml
# from source.crmonitor.common.world import World
# from source.crmonitor.evaluation.evaluation import RuleEvaluator
# from source.crmonitor.evaluation.visualization import plot_rule_visualization, EGO_VEHICLE_DRAW_PARAMS
# from matplotlib import pyplot as plt
# import sys
# from pathlib import Path
# import yaml
# import numpy as np
# from matplotlib.lines import Line2D
# # ========== CONFIGURABLE PARAMETERS ==========
# # Set your desired start and end time here (in seconds)
# START_TIME_SECONDS = 0.0  # Change this value as needed (e.g., 2.5, 5.0, 7.3, etc.)
# END_TIME_SECONDS = 48.9  # Change this value as needed
# #bulb 557
# #bay 489
# # Define state transition points (in seconds)
# #bus bulb
# STATE_TRANSITIONS = {
#     4.6: ('Heading to next station→Arriving', '#FF9800'),
#     31.3: ('Arriving→Boarding and Alighting', '#9E9E9E'),
#     33.4: ('Boarding and Alighting→Departure', '#2196F3'),
#     44.7: ('Departure→Heading to next station', '#4CAF50')
# }
#
#
# #bus bay
# # STATE_TRANSITIONS = {
# #     4.6: ('Heading to next station→Arriving', '#FF9800'),
# #     26.4: ('Arriving→Boarding and Alighting', '#9E9E9E'),
# #     27.6: ('Boarding and Alighting→Departure', '#2196F3'),
# #     41.0: ('Departure→Heading to next station', '#4CAF50')
# # }
#
# project_root = Path.cwd().parent
# sys.path.append(str(project_root))
# cfg = yaml.safe_load((project_root / "configurations" / "scenario.yaml").read_text())
# bus_stop = cfg["scenario"]["type"]
# scenario_file = project_root / "scenarios" / bus_stop / f"{bus_stop}.xml"
# # —— Load scenario file ——
# config_path = "../source/crmonitor/config_pt.yaml"
# config = load_yaml(str(config_path))
# rules_path = "../source/crmonitor/traffic_rules_pt.yaml"
# traffic_rules = load_yaml(str(rules_path))
# # Open the scenario
# # Make sure to call with lanelet_assignment=True
# scenario, _ = CommonRoadFileReader(scenario_file).open(lanelet_assignment=True)
# world = World.create_from_scenario(scenario, config=config)
#
# # Create a rule evaluator
# ego_vehicle = next(iter(world.vehicles))
# states = ego_vehicle.states_cr
#
# # Convert time to timesteps (assuming 0.1s intervals)
# start_timestep = int(START_TIME_SECONDS * 10)
# end_timestep = int(END_TIME_SECONDS * 10)
#
# # Validate timesteps
# max_timestep = len(states) - 1
# if start_timestep < 0:
#     print(f"Warning: start_timestep {start_timestep} is negative, setting to 0")
#     start_timestep = 0
# if start_timestep > max_timestep:
#     print(f"Warning: start_timestep {start_timestep} exceeds maximum {max_timestep}")
#     start_timestep = min(start_timestep, max_timestep)
# if end_timestep > max_timestep:
#     print(f"Warning: end_timestep {end_timestep} exceeds maximum {max_timestep}")
#     end_timestep = max_timestep
# if end_timestep < start_timestep:
#     print(f"Error: end_timestep {end_timestep} is less than start_timestep {start_timestep}")
#     end_timestep = start_timestep + 10  # Default to 1 second of data
#
# print(
#     f"Processing from timestep {start_timestep} ({start_timestep / 10:.1f}s) to {end_timestep} ({end_timestep / 10:.1f}s)")
# print(f"Total timesteps to process: {end_timestep - start_timestep + 1}")
#
# # Create a fresh rule evaluator
# rule_evaluator = RuleEvaluator.create_from_config(world, ego_vehicle, rule="RB_1", traffic_rules_config=traffic_rules)
#
# # Create time_steps array for plotting
# time_steps = [(i / 10) for i in range(start_timestep, end_timestep + 1)]
#
# # Initialize data storage
# vels = []
# predicates_robustness_all = {}
# robustness_all = []
#
# # Advance rule evaluator to start time if needed
# if start_timestep > 0:
#     print(f"Advancing rule evaluator to timestep {start_timestep}...")
#     try:
#         for i in range(start_timestep):
#             rule_evaluator.update()
#     except Exception as e:
#         print(f"Note: Could not advance rule evaluator to start time: {e}")
#         # Try to create a new rule evaluator if advancing fails
#         rule_evaluator = RuleEvaluator.create_from_config(world, ego_vehicle, rule="RB_1",
#                                                           traffic_rules_config=traffic_rules)
#         # Set current time directly if possible
#         if hasattr(rule_evaluator, 'current_time'):
#             rule_evaluator.current_time = start_timestep
#
# # Collect data from start_timestep to end_timestep
# print("Collecting data...")
# for i in range(start_timestep, end_timestep + 1):
#     try:
#         # Get velocity
#         if i < len(states):
#             vels.append(states[i].velocity)
#             position = states[i].position
#             velocity = states[i].velocity
#         else:
#             print(f"Warning: No state available for timestep {i}")
#             vels.append(0)  # Default value
#             position = "N/A"
#             velocity = 0
#
#         # Update rule evaluator
#         try:
#             robustness = rule_evaluator.update()
#             robustness_all.append(robustness)
#         except Exception as e:
#             print(f"Warning at timestep {i}: Could not update rule evaluator: {e}")
#             robustness = 0  # Default value
#             robustness_all.append(robustness)
#
#         current_time_step = i / 10  # Convert to seconds
#
#         # Print current state
#         print(f"time {current_time_step:.1f}s:")
#         print(f"  vehicle: {{pos: {position}, vel: {velocity:.3f}}}")
#         print(f"  rule rob: {robustness:.3f}")
#
#         # Get predicates robustness
#         try:
#             predicates = rule_evaluator.get_predicates()
#             print(f"  {predicates}")
#
#             predicates_robustness = predicates.items()
#             for pred, rob in predicates_robustness:
#                 if pred not in predicates_robustness_all:
#                     predicates_robustness_all[pred] = []
#                 predicates_robustness_all[pred].append(rob)
#         except Exception as e:
#             print(f"Warning: Could not get predicates at timestep {i}: {e}")
#             # Fill with default values for existing predicates
#             for pred in predicates_robustness_all:
#                 predicates_robustness_all[pred].append(0)
#
#     except Exception as e:
#         print(f"Error at timestep {i}: {e}")
#         # Add default values to maintain list consistency
#         vels.append(0)
#         robustness_all.append(0)
#         for pred in predicates_robustness_all:
#             predicates_robustness_all[pred].append(0)
#         continue
#
# # Ensure all predicate lists have the same length as time_steps
# expected_length = len(time_steps)
# for pred in predicates_robustness_all:
#     current_length = len(predicates_robustness_all[pred])
#     if current_length < expected_length:
#         print(
#             f"Warning: Predicate '{pred}' has {current_length} values, expected {expected_length}. Padding with zeros.")
#         predicates_robustness_all[pred].extend([0] * (expected_length - current_length))
#     elif current_length > expected_length:
#         print(f"Warning: Predicate '{pred}' has {current_length} values, expected {expected_length}. Truncating.")
#         predicates_robustness_all[pred] = predicates_robustness_all[pred][:expected_length]
#
# # ========== PROFESSIONAL PLOTTING WITH TIMES NEW ROMAN ==========
# print("\nGenerating professional plots with Times New Roman font...")
#
# # Set professional paper-ready plot style
# plt.rcParams.update({
#     'font.family': 'serif',
#     'font.serif': ['Times New Roman'],
#     'font.size': 10,
#     'axes.labelsize': 10,
#     'axes.titlesize': 11,
#     'axes.linewidth': 0.8,
#     'xtick.labelsize': 9,
#     'ytick.labelsize': 9,
#     'legend.fontsize': 9,
#     'figure.titlesize': 12,
#     'lines.linewidth': 1.2,
#     'mathtext.fontset': 'stix',
#     'savefig.dpi': 300,
#     'savefig.bbox': 'tight',
#     'savefig.format': 'pdf'
# })
#
#
# # Function to add state transition markers
# def add_state_markers(ax, time_steps, data, state_transitions, start_time, end_time):
#     """Add circular markers at state transition points"""
#     for time_sec, (label, color) in state_transitions.items():
#         if start_time <= time_sec <= end_time:
#             # Find the closest index for this time
#             time_idx = int((time_sec - start_time) * 10)
#             if 0 <= time_idx < len(data):
#                 ax.scatter(time_sec, data[time_idx],
#                            color=color, s=120, marker='o',
#                            edgecolors='black', linewidth=1.5,
#                            zorder=3, alpha=0.9)
#
#
# # Velocity figure with state markers
# if vels:
#     fig1, ax1 = plt.subplots(figsize=(8, 5))
#
#     # Calculate appropriate y-axis limits based on data
#     if len(vels) > 0 and max(vels) > 0:
#         vel_min = min(vels) - 0.5
#         vel_max = max(vels) + 0.5
#         # Round to nice values
#         vel_min = round(vel_min, 1)
#         vel_max = round(vel_max, 1)
#     else:
#         vel_min, vel_max = 9, 10.25  # Default values
#
#     ax1.set_ylim(vel_min, vel_max)
#     ax1.tick_params(axis='both', which='major', direction='in', top=True, right=True)
#     ax1.set_xlabel("Time (s)", fontsize=10)
#     ax1.set_ylabel("Velocity (m/s)", fontsize=10)
#
#     # Plot velocity line
#     ax1.plot(time_steps[:len(vels)], vels, color='#0066CC', linewidth=1.2, zorder=1)
#
#     # Add state transition markers
#     add_state_markers(ax1, time_steps, vels, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)
#
#     ax1.set_xlim(START_TIME_SECONDS, END_TIME_SECONDS)
#     ax1.grid(False)  # No grid for professional appearance
#
#     # Add spines on all sides for professional look
#     ax1.spines['top'].set_visible(True)
#     ax1.spines['right'].set_visible(True)
#     ax1.spines['bottom'].set_visible(True)
#     ax1.spines['left'].set_visible(True)
#
#     # Create legend for state transitions
#
#
#     legend_elements = []
#     for time_sec, (label, color) in STATE_TRANSITIONS.items():
#         legend_elements.append(
#             Line2D([0], [0], marker='o', color='w',
#                    markerfacecolor=color, markersize=8,
#                    markeredgecolor='black', markeredgewidth=1,
#                    label=f'{time_sec}s: {label}')
#         )
#
#     ax1.legend(handles=legend_elements, loc='best', fontsize=8, frameon=True,
#                fancybox=False, shadow=False, framealpha=1.0,
#                edgecolor='black', facecolor='white')
#
#     plt.tight_layout()
#     # Save velocity figure
#     fig1.savefig('velocity_with_states.pdf', dpi=300, bbox_inches='tight')
#     print("Velocity plot with state markers saved")
# else:
#     print("Warning: No velocity data to plot")
#
# # Robustness figure with state markers
# if robustness_all:
#     fig2, ax2 = plt.subplots(figsize=(8, 5))
#     ax2.set_ylim(-1, 1.1)  # Extended y-axis to give more space for legends
#     ax2.tick_params(axis='both', which='major', direction='in', top=True, right=True)
#     ax2.set_xlabel("Time (s)", fontsize=10)
#     ax2.set_ylabel("Robustness", fontsize=10)
#
#     # Plot main robustness with SOLID line
#     line1 = ax2.plot(time_steps[:len(robustness_all)], robustness_all,
#                      color='#CC0000', linewidth=1.2, linestyle='-',  # Solid line for RB_1
#                      label="RB_1 and keeps_lane_speed_limit", zorder=1)
#
#     # Add state transition markers for main robustness
#     add_state_markers(ax2, time_steps, robustness_all, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)
#
#     # Define colors for predicates
#     predicate_colors = ['#0066CC', '#009900', '#FF6600', '#6600CC', '#666666']
#     color_idx = 0
#
#     # Track if we've already plotted keeps_type_speed_limit with keeps_standing_passenger_speed_limit
#     skip_keeps_type = False
#
#     # Plot predicates
#     # for pred, robs in predicates_robustness_all.items():
#     #     if len(robs) == 0:
#     #         continue
#     #
#     #     # Process predicate name
#     #     pred_display = pred[:-3] if len(pred) > 3 and pred.endswith("_0") else pred
#     #
#     #     # Special handling for specific predicates - combine two predicates into one label
#     #     if pred_display == "keeps_standing_passenger_speed_limit":
#     #         # Check if keeps_type_speed_limit exists and has the same values
#     #         type_speed_pred = None
#     #         for p in predicates_robustness_all.keys():
#     #             p_display = p[:-3] if len(p) > 3 and p.endswith("_0") else p
#     #             if p_display == "keeps_type_speed_limit":
#     #                 type_speed_pred = p
#     #                 break
#     #
#     #         # If both exist and have same values, combine the label
#     #         if type_speed_pred and len(predicates_robustness_all[type_speed_pred]) > 0:
#     #             pred_display = "keeps_standing_passenger_speed_limit, keeps_type_speed_limit"
#     #             skip_keeps_type = True  # Mark to skip keeps_type_speed_limit later
#     #
#     #     # Skip keeps_type_speed_limit if already combined with keeps_standing_passenger_speed_limit
#     #     if pred_display == "keeps_type_speed_limit" and skip_keeps_type:
#     #         continue
#     #
#     #
#     #     # Skip keeps_lane_speed_limit_with_minmax
#     #     if pred_display == "keeps_lane_speed_limit_with_minmax":
#     #         continue
#     #
#     #     # Plot the predicate with different color and dashed line
#     #     color = predicate_colors[color_idx % len(predicate_colors)]
#     #     ax2.plot(time_steps[:len(robs)], robs,
#     #              color=color, linewidth=1.2, linestyle='--',  # All predicates use dashed lines
#     #              label=pred_display, zorder=1)
#     #
#     #     # Add state markers for this predicate line
#     #     add_state_markers(ax2, time_steps, robs, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)
#     #
#     #     color_idx += 1
#     # Plot predicates
#     for pred, robs in predicates_robustness_all.items():
#         if len(robs) == 0:
#             continue
#
#         # Process predicate name
#         pred_display = pred[:-3] if len(pred) > 3 and pred.endswith("_0") else pred
#
#         # ===== 方案1: 直接跳过 keeps_type_speed_limit =====
#         # 如果你想完全避免单独显示 keeps_type_speed_limit
#         if pred_display == "keeps_type_speed_limit":
#             continue
#
#         # Special handling for specific predicates - combine two predicates into one label
#         if pred_display == "keeps_standing_passenger_speed_limit":
#             # Check if keeps_type_speed_limit exists and has the same values
#             type_speed_pred = None
#             for p in predicates_robustness_all.keys():
#                 p_display = p[:-3] if len(p) > 3 and p.endswith("_0") else p
#                 if p_display == "keeps_type_speed_limit":
#                     type_speed_pred = p
#                     break
#
#             # If both exist and have same values, combine the label
#             if type_speed_pred and len(predicates_robustness_all[type_speed_pred]) > 0:
#                 pred_display = "keeps_standing_passenger_speed_limit, keeps_type_speed_limit"
#
#         # Skip keeps_lane_speed_limit_with_minmax
#         if pred_display == "keeps_lane_speed_limit_with_minmax":
#             continue
#
#         # Plot the predicate with different color and dashed line
#         color = predicate_colors[color_idx % len(predicate_colors)]
#         ax2.plot(time_steps[:len(robs)], robs,
#                  color=color, linewidth=1.2, linestyle='--',  # All predicates use dashed lines
#                  label=pred_display, zorder=1)
#
#         # Add state markers for this predicate line
#         add_state_markers(ax2, time_steps, robs, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)
#
#         color_idx += 1
#     # Professional layout
#     ax2.spines['top'].set_visible(True)
#     ax2.spines['right'].set_visible(True)
#     ax2.set_xlim(START_TIME_SECONDS, END_TIME_SECONDS)
#     ax2.grid(False)  # No grid for professional appearance
#
#     # Add main legend for lines (place it higher to avoid overlap)
#     legend1 = ax2.legend(loc='lower right', fontsize=9, frameon=True,
#                          fancybox=False, shadow=False, framealpha=1.0,
#                          edgecolor='black', facecolor='white')
#
#     # Add state transition legend separately (place it lower)
#     from matplotlib.lines import Line2D
#
#     legend_elements = []
#     unique_transitions = {}
#     for time_sec, (trans_name, color) in STATE_TRANSITIONS.items():
#         if '→' in trans_name:
#             a, b = [p.strip() for p in trans_name.split('→', 1)]
#             label = f"{time_sec}s: {a}→{b}"  # 用完整文案
#             # 如果担心太长，可以换行显示：
#             # label = f"{time_sec}s:\n{a}→{b}"
#         else:
#             label = f"{time_sec}s: {trans_name}"
#         unique_transitions[label] = color
#
#     for trans_name, color in unique_transitions.items():
#         legend_elements.append(
#             Line2D([0], [0], marker='o', color='w',
#                    markerfacecolor=color, markersize=8,
#                    markeredgecolor='black', markeredgewidth=1,
#                    label=trans_name)
#         )
#
#     # Add second legend for state transitions in the lower part of the plot
#     legend2 = ax2.legend(handles=legend_elements, loc='lower left',
#                          title='State Transitions', fontsize=8,
#                          frameon=True, fancybox=False, shadow=False,
#                          framealpha=1.0, edgecolor='black', facecolor='white')
#     ax2.add_artist(legend1)  # Keep the first legend
#
#     plt.tight_layout()
#     # Save robustness figure
#     fig2.savefig('robustness_with_states.pdf', dpi=300, bbox_inches='tight')
#     print("Robustness plot with state markers saved")
# else:
#     print("Warning: No robustness data to plot")
#
# plt.show()
#
# print(f"\nProcessing completed. Displayed data from {START_TIME_SECONDS:.1f}s to {END_TIME_SECONDS:.1f}s")
# print(f"Total data points processed: {len(time_steps)}")
from commonroad.common.file_reader import CommonRoadFileReader
from source.crmonitor.common.helper import load_yaml
from source.crmonitor.common.world import World
from source.crmonitor.evaluation.evaluation import RuleEvaluator
from source.crmonitor.evaluation.visualization import plot_rule_visualization, EGO_VEHICLE_DRAW_PARAMS
from matplotlib import pyplot as plt
import sys
from pathlib import Path
import yaml
import numpy as np
from matplotlib.lines import Line2D

# ========== CONFIGURABLE PARAMETERS ==========
# Set your desired start and end time here (in seconds)
START_TIME_SECONDS = 0.0  # Change this value as needed (e.g., 2.5, 5.0, 7.3, etc.)
END_TIME_SECONDS = 48.9  # Change this value as needed
# bulb 557
# bay 489
# Define state transition points (in seconds)
# bus bulb
# STATE_TRANSITIONS = {
#     4.6: ('Heading to next station→Arriving', '#FF9800'),
#     31.3: ('Arriving→Boarding and Alighting', '#9E9E9E'),
#     33.4: ('Boarding and Alighting→Departure', '#2196F3'),
#     44.7: ('Departure→Heading to next station', '#4CAF50')
# }

# bus bay
STATE_TRANSITIONS = {
    4.6: ('Heading to next station→Arriving', '#FF9800'),
    26.4: ('Arriving→Boarding and Alighting', '#9E9E9E'),
    27.6: ('Boarding and Alighting→Departure', '#2196F3'),
    41.0: ('Departure→Heading to next station', '#4CAF50')
}

project_root = Path.cwd().parent
sys.path.append(str(project_root))
cfg = yaml.safe_load((project_root / "configurations" / "scenario.yaml").read_text())
bus_stop = cfg["scenario"]["type"]
scenario_file = project_root / "scenarios" / bus_stop / f"{bus_stop}.xml"
# —— Load scenario file ——
config_path = "../source/crmonitor/config_pt.yaml"
config = load_yaml(str(config_path))
rules_path = "../source/crmonitor/traffic_rules_pt.yaml"
traffic_rules = load_yaml(str(rules_path))
# Open the scenario
# Make sure to call with lanelet_assignment=True
scenario, _ = CommonRoadFileReader(scenario_file).open(lanelet_assignment=True)
world = World.create_from_scenario(scenario, config=config)

# Create a rule evaluator
ego_vehicle = next(iter(world.vehicles))
states = ego_vehicle.states_cr

# Convert time to timesteps (assuming 0.1s intervals)
start_timestep = int(START_TIME_SECONDS * 10)
end_timestep = int(END_TIME_SECONDS * 10)

# Validate timesteps
max_timestep = len(states) - 1
if start_timestep < 0:
    print(f"Warning: start_timestep {start_timestep} is negative, setting to 0")
    start_timestep = 0
if start_timestep > max_timestep:
    print(f"Warning: start_timestep {start_timestep} exceeds maximum {max_timestep}")
    start_timestep = min(start_timestep, max_timestep)
if end_timestep > max_timestep:
    print(f"Warning: end_timestep {end_timestep} exceeds maximum {max_timestep}")
    end_timestep = max_timestep
if end_timestep < start_timestep:
    print(f"Error: end_timestep {end_timestep} is less than start_timestep {start_timestep}")
    end_timestep = start_timestep + 10  # Default to 1 second of data

print(
    f"Processing from timestep {start_timestep} ({start_timestep / 10:.1f}s) to {end_timestep} ({end_timestep / 10:.1f}s)")
print(f"Total timesteps to process: {end_timestep - start_timestep + 1}")

# Create a fresh rule evaluator
rule_evaluator = RuleEvaluator.create_from_config(world, ego_vehicle, rule="RB_1", traffic_rules_config=traffic_rules)

# Create time_steps array for plotting
time_steps = [(i / 10) for i in range(start_timestep, end_timestep + 1)]

# Initialize data storage
vels = []
predicates_robustness_all = {}
robustness_all = []

# Advance rule evaluator to start time if needed
if start_timestep > 0:
    print(f"Advancing rule evaluator to timestep {start_timestep}...")
    try:
        for i in range(start_timestep):
            rule_evaluator.update()
    except Exception as e:
        print(f"Note: Could not advance rule evaluator to start time: {e}")
        # Try to create a new rule evaluator if advancing fails
        rule_evaluator = RuleEvaluator.create_from_config(world, ego_vehicle, rule="RB_1",
                                                          traffic_rules_config=traffic_rules)
        # Set current time directly if possible
        if hasattr(rule_evaluator, 'current_time'):
            rule_evaluator.current_time = start_timestep

# Collect data from start_timestep to end_timestep
print("Collecting data...")
for i in range(start_timestep, end_timestep + 1):
    try:
        # Get velocity
        if i < len(states):
            vels.append(states[i].velocity)
            position = states[i].position
            velocity = states[i].velocity
        else:
            print(f"Warning: No state available for timestep {i}")
            vels.append(0)  # Default value
            position = "N/A"
            velocity = 0

        # Update rule evaluator
        try:
            robustness = rule_evaluator.update()
            robustness_all.append(robustness)
        except Exception as e:
            print(f"Warning at timestep {i}: Could not update rule evaluator: {e}")
            robustness = 0  # Default value
            robustness_all.append(robustness)

        current_time_step = i / 10  # Convert to seconds

        # Print current state
        print(f"time {current_time_step:.1f}s:")
        print(f"  vehicle: {{pos: {position}, vel: {velocity:.3f}}}")
        print(f"  rule rob: {robustness:.3f}")

        # Get predicates robustness
        try:
            predicates = rule_evaluator.get_predicates()
            print(f"  {predicates}")

            predicates_robustness = predicates.items()
            for pred, rob in predicates_robustness:
                if pred not in predicates_robustness_all:
                    predicates_robustness_all[pred] = []
                predicates_robustness_all[pred].append(rob)
        except Exception as e:
            print(f"Warning: Could not get predicates at timestep {i}: {e}")
            # Fill with default values for existing predicates
            for pred in predicates_robustness_all:
                predicates_robustness_all[pred].append(0)

    except Exception as e:
        print(f"Error at timestep {i}: {e}")
        # Add default values to maintain list consistency
        vels.append(0)
        robustness_all.append(0)
        for pred in predicates_robustness_all:
            predicates_robustness_all[pred].append(0)
        continue

# Ensure all predicate lists have the same length as time_steps
expected_length = len(time_steps)
for pred in predicates_robustness_all:
    current_length = len(predicates_robustness_all[pred])
    if current_length < expected_length:
        print(
            f"Warning: Predicate '{pred}' has {current_length} values, expected {expected_length}. Padding with zeros.")
        predicates_robustness_all[pred].extend([0] * (expected_length - current_length))
    elif current_length > expected_length:
        print(f"Warning: Predicate '{pred}' has {current_length} values, expected {expected_length}. Truncating.")
        predicates_robustness_all[pred] = predicates_robustness_all[pred][:expected_length]

# ========== PROFESSIONAL PLOTTING WITH TIMES NEW ROMAN ==========
print("\nGenerating professional plots with Times New Roman font...")

# Set professional paper-ready plot style
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman'],
    'font.size': 10,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'axes.linewidth': 0.8,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'legend.fontsize': 9,
    'figure.titlesize': 12,
    'lines.linewidth': 1.2,
    'mathtext.fontset': 'stix',
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.format': 'pdf'
})


# Function to add state transition markers
def add_state_markers(ax, time_steps, data, state_transitions, start_time, end_time):
    """Add circular markers at state transition points"""
    for time_sec, (label, color) in state_transitions.items():
        if start_time <= time_sec <= end_time:
            # Find the closest index for this time
            time_idx = int((time_sec - start_time) * 10)
            if 0 <= time_idx < len(data):
                ax.scatter(time_sec, data[time_idx],
                           color=color, s=120, marker='o',
                           edgecolors='black', linewidth=1.5,
                           zorder=3, alpha=0.9)


# Velocity figure with state markers
if vels:
    fig1, ax1 = plt.subplots(figsize=(8, 5))

    # Calculate appropriate y-axis limits based on data
    if len(vels) > 0 and max(vels) > 0:
        vel_min = min(vels) - 0.5
        vel_max = max(vels) + 0.5
        # Round to nice values
        vel_min = round(vel_min, 1)
        vel_max = round(vel_max, 1)
    else:
        vel_min, vel_max = 9, 10.25  # Default values

    ax1.set_ylim(vel_min, vel_max)
    ax1.tick_params(axis='both', which='major', direction='in', top=True, right=True)
    ax1.set_xlabel("Time (s)", fontsize=10)
    ax1.set_ylabel("Velocity (m/s)", fontsize=10)

    # Plot velocity line
    ax1.plot(time_steps[:len(vels)], vels, color='#0066CC', linewidth=1.2, zorder=1)

    # Add state transition markers
    add_state_markers(ax1, time_steps, vels, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)

    ax1.set_xlim(START_TIME_SECONDS, END_TIME_SECONDS)
    ax1.grid(False)  # No grid for professional appearance

    # Add spines on all sides for professional look
    ax1.spines['top'].set_visible(True)
    ax1.spines['right'].set_visible(True)
    ax1.spines['bottom'].set_visible(True)
    ax1.spines['left'].set_visible(True)

    # Create legend for state transitions
    legend_elements = []
    for time_sec, (label, color) in STATE_TRANSITIONS.items():
        legend_elements.append(
            Line2D([0], [0], marker='o', color='w',
                   markerfacecolor=color, markersize=8,
                   markeredgecolor='black', markeredgewidth=1,
                   label=f'{time_sec}s: {label}')
        )

    ax1.legend(handles=legend_elements, loc='best', fontsize=10, frameon=True,
               fancybox=False, shadow=False, framealpha=1.0,
               edgecolor='black', facecolor='white')

    plt.tight_layout()
    # Save velocity figure
    fig1.savefig('velocity_with_states.pdf', dpi=300, bbox_inches='tight')
    print("Velocity plot with state markers saved")
else:
    print("Warning: No velocity data to plot")

# MODIFIED ROBUSTNESS FIGURE WITH DISTINCT MARKERS FOR EACH PREDICATE
if robustness_all:
    fig2, ax2 = plt.subplots(figsize=(8, 5))
    ax2.set_ylim(-1, 1.1)  # Extended y-axis to give more space for legends
    ax2.tick_params(axis='both', which='major', direction='in', top=True, right=True)
    ax2.set_xlabel("Time (s)", fontsize=10)
    ax2.set_ylabel("Robustness", fontsize=10)

    # Define specific styles for each predicate
    # All predicates use dashed lines, only two have markers
    predicate_styles = {
        'RB_1': {'color': '#CC0000', 'marker': None, 'linestyle': '-'},  # 红色，实线，无标记
        'keeps_lane_speed_limit': {'color': '#0066CC', 'marker': 'o', 'linestyle': '--'},  # 蓝色，虚线，圆圈标记
        'keeps_standing_passenger_speed_limit': {'color': '#FF6600', 'marker': None, 'linestyle': '--'},  # 橙色，虚线，三角标记
        'keeps_type_speed_limit': {'color': '#6600CC', 'marker': 'x', 'linestyle': '--'},  # 紫色，虚线，叉标记
        'keeps_lane_speed_limit_with_minmax': {'color': '#0066CC', 'marker': 'o', 'linestyle': '--'},
        # 蓝色，虚线，圆圈标记（会显示为keeps_lane_speed_limit）
        'keeps_fov_speed_limit': {'color': '#00CCCC', 'marker': None, 'linestyle': '--'},  # 青色，虚线，无标记
        'keeps_brake_speed_limit': {'color': '#CC00CC', 'marker': None, 'linestyle': '--'},  # 品红，虚线，无标记
    }

    # Default style for unspecified predicates
    default_style = {'color': '#999999', 'marker': None, 'linestyle': '--'}

    # Plot main RB_1 robustness (no markers)
    ax2.plot(time_steps[:len(robustness_all)], robustness_all,
             color='#CC0000', linewidth=1.2, linestyle='-',
             label="RB_1", zorder=10)

    # Add state transition markers for main robustness
    add_state_markers(ax2, time_steps, robustness_all, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)

    # Plot each predicate separately with its own marker
    for pred, robs in predicates_robustness_all.items():
        if len(robs) == 0:
            continue

        # Process predicate name - same logic as original code
        if len(pred) > 3 and pred.endswith("_0"):
            pred_display = pred[:-3]
        elif pred.endswith("_"):
            pred_display = pred[:-1]
        else:
            pred_display = pred

        # Special renaming: keeps_lane_speed_limit_with_minmax should display as keeps_lane_speed_limit
        if pred_display == "keeps_lane_speed_limit_with_minmax":
            pred_display = "keeps_lane_speed_limit"

        # Get style for this predicate
        if pred_display in predicate_styles:
            style = predicate_styles[pred_display]
        else:
            style = default_style
            print(f"Warning: No style defined for predicate '{pred_display}', using default")

        # Special handling for keeps_type_speed_limit - higher zorder
        if pred_display == "keeps_type_speed_limit":
            plot_zorder = 1
        # Also give keeps_lane_speed_limit (renamed from keeps_lane_speed_limit_with_minmax) higher zorder
        elif pred_display == "keeps_lane_speed_limit" and pred.startswith("keeps_lane_speed_limit_with_minmax"):
            plot_zorder = 1
        else:
            plot_zorder = 10

        # Plot the predicate with its style
        # Only add markers if specified in the style
        if style['marker'] is not None:
            # Calculate markevery for predicates with markers
            markevery = max(1, len(robs) // 20)  # Show about 20 markers total
            ax2.plot(time_steps[:len(robs)], robs,
                     color=style['color'],
                     linewidth=1.2,
                     linestyle=style['linestyle'],
                     marker=style['marker'],
                     markersize=5,
                     markevery=markevery,
                     label=pred_display,
                     zorder=plot_zorder)
        else:
            # Plot without markers
            ax2.plot(time_steps[:len(robs)], robs,
                     color=style['color'],
                     linewidth=1.2,
                     linestyle=style['linestyle'],
                     label=pred_display,
                     zorder=plot_zorder)

        # Add state markers for this predicate line
        add_state_markers(ax2, time_steps, robs, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)

    # Professional layout
    ax2.spines['top'].set_visible(True)
    ax2.spines['right'].set_visible(True)
    ax2.set_xlim(START_TIME_SECONDS, END_TIME_SECONDS)
    ax2.grid(False)  # No grid for professional appearance

    # Add main legend for lines with two columns for better space usage
    legend1 = ax2.legend(loc='lower right', fontsize=8.5, frameon=True,
                         fancybox=False, shadow=False, framealpha=1.0,
                         edgecolor='black', facecolor='white',
                         ncol=2)  # Use 2 columns

    # Add state transition legend separately
    legend_elements = []
    for time_sec, (trans_name, color) in STATE_TRANSITIONS.items():
        if '→' in trans_name:
            a, b = [p.strip() for p in trans_name.split('→', 1)]
            label = f"{time_sec}s: {a}→{b}"
        else:
            label = f"{time_sec}s: {trans_name}"
        legend_elements.append(
            Line2D([0], [0], marker='o', color='w',
                   markerfacecolor=color, markersize=8,
                   markeredgecolor='black', markeredgewidth=1,
                   label=label)
        )

    # Add second legend for state transitions in the lower part of the plot
    legend2 = ax2.legend(handles=legend_elements, loc='lower left',
                         title='State Transitions', fontsize=10,
                         frameon=True, fancybox=False, shadow=False,
                         framealpha=1.0, edgecolor='black', facecolor='white')
    ax2.add_artist(legend1)  # Keep the first legend

    plt.tight_layout()
    # Save robustness figure
    fig2.savefig('robustness_with_states.pdf', dpi=300, bbox_inches='tight')
    print("Robustness plot with distinct markers saved")
else:
    print("Warning: No robustness data to plot")

plt.show()

print(f"\nProcessing completed. Displayed data from {START_TIME_SECONDS:.1f}s to {END_TIME_SECONDS:.1f}s")
print(f"Total data points processed: {len(time_steps)}")