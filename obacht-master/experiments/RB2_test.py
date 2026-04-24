from commonroad.common.file_reader import CommonRoadFileReader
from pathlib import Path
from source.crmonitor.common.helper import load_yaml
from source.crmonitor.common.world import World
from source.crmonitor.evaluation.evaluation import RuleEvaluator
from source.crmonitor.evaluation.visualization import plot_rule_visualization, EGO_VEHICLE_DRAW_PARAMS
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
import sys
import yaml
import numpy as np

project_root = Path.cwd().parent
sys.path.append(str(project_root))

# ========== CONFIGURABLE PARAMETERS ==========
# Set your desired start and end time here (in seconds)
START_TIME_SECONDS = 0.0  # Change this value as needed
END_TIME_SECONDS = 55.6  # Change this value as needed
#489
# Define state transition points (in seconds) - adjust based on your scenario
# Bus bay example (adjust these values for your specific scenario)
# bus bulb
# STATE_TRANSITIONS = {
#     4.6: ('Heading to next station→Arriving', '#FF9800'),
#     31.3: ('Arriving→Boarding and Alighting', '#9E9E9E'),
#     33.4: ('Boarding and Alighting→Departure', '#2196F3'),
#     44.7: ('Departure→Heading to next station', '#4CAF50')
# }
#bus bay
STATE_TRANSITIONS = {
    4.6: ('Heading to next station→Arriving', '#FF9800'),
    26.4: ('Arriving→Boarding and Alighting', '#9E9E9E'),
    27.6: ('Boarding and Alighting→Departure', '#2196F3'),
    41.0: ('Departure→Heading to next station', '#4CAF50')
}


def get_d(time_step, vehicle, right_most_lanelet_id: int):
    right_most_lane = world.road_network.find_lane_by_lanelet(right_most_lanelet_id)
    x, y = vehicle.states_cr[time_step].position
    s, d = right_most_lane.clcs_right.convert_to_curvilinear_coords(x, y)
    if abs(x)<=40:
        d = d - abs((vehicle.shape.width / 2))-2
    else:
       d = d - abs((vehicle.shape.width / 2))
    return d


# —— Load configuration ——
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

ego_vehicle = next(iter(world.vehicles))

# Convert to timesteps (assuming 0.1s intervals)
start_timestep = int(START_TIME_SECONDS * 10)
end_timestep = int(END_TIME_SECONDS * 10)

# Validate timesteps
max_timestep = len(ego_vehicle.states_cr) - 1
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
rule_evaluator = RuleEvaluator.create_from_config(world, ego_vehicle, rule="RB_2", traffic_rules_config=traffic_rules)
states = ego_vehicle.states_cr

# Create time_steps array for plotting
time_steps = [(i / 10) for i in range(start_timestep, end_timestep + 1)]

# Initialize data storage
vels = []
theta = []
d = []
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
        rule_evaluator = RuleEvaluator.create_from_config(world, ego_vehicle, rule="RB_2",
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
            # print(f"Warning: No state available for timestep {i}")
            vels.append(0)  # Default value
            position = "N/A"
            velocity = 0

        # Update rule evaluator
        try:
            robustness = rule_evaluator.update()
            robustness_all.append(robustness)
        except Exception as e:
            # print(f"Warning at timestep {i}: Could not update rule evaluator: {e}")
            robustness = 0  # Default value
            robustness_all.append(0)

        current_time_step = i / 10  # Convert to seconds

        # Print current state
        # print(f"time {current_time_step:.1f}s:")
        # print(f"  vehicle: {{pos: {position}, vel: {velocity:.3f}}}")
        # print(f"  rule rob: {robustness:.3f}")

        # Get d value #309
        try:
            # d.append(get_d(i, ego_vehicle, 1))
            if abs(ego_vehicle.states_cr[i].position[0])<40:
                d.append(get_d(i, ego_vehicle, 309))
            else:
                d.append(get_d(i, ego_vehicle, 1))

        except Exception as e:
            # print(f"Warning: Could not calculate d at timestep {i}: {e}")
            d.append(0)  # Default value

        # Get theta
        try:
            theta.append(ego_vehicle.get_lat_state(i).theta)
        except Exception as e:
            # print(f"Warning: Could not get theta at timestep {i}: {e}")
            theta.append(0)  # Default value

        # Get predicates robustness
        try:
            predicates = rule_evaluator.get_predicates()
            # print(f"  {predicates}")

            predicates_robustness = predicates.items()
            for pred, rob in predicates_robustness:
                if pred not in predicates_robustness_all:
                    predicates_robustness_all[pred] = []
                predicates_robustness_all[pred].append(rob)
        except Exception as e:
            # print(f"Warning: Could not get predicates at timestep {i}: {e}")
            # Fill with default values for existing predicates
            for pred in predicates_robustness_all:
                predicates_robustness_all[pred].append(0)

    except Exception as e:
        # print(f"Error at timestep {i}: {e}")
        # Add default values to maintain list consistency
        vels.append(0)
        theta.append(0)
        d.append(0)
        robustness_all.append(0)
        for pred in predicates_robustness_all:
            predicates_robustness_all[pred].append(0)
        continue

# Ensure all predicate lists have the same length as time_steps
expected_length = len(time_steps)
for pred in predicates_robustness_all:
    current_length = len(predicates_robustness_all[pred])
    if current_length < expected_length:
        # print(
            # f"Warning: Predicate '{pred}' has {current_length} values, expected {expected_length}. Padding with zeros.")
        predicates_robustness_all[pred].extend([0] * (expected_length - current_length))
    elif current_length > expected_length:
        # print(f"Warning: Predicate '{pred}' has {current_length} values, expected {expected_length}. Truncating.")
        predicates_robustness_all[pred] = predicates_robustness_all[pred][:expected_length]

# ========== PROFESSIONAL PLOTTING WITH TIMES NEW ROMAN ==========
# print("\nGenerating professional plots with Times New Roman font...")

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
        vel_min = round(vel_min, 1)
        vel_max = round(vel_max, 1)
    else:
        vel_min, vel_max = 0, 11

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

    ax1.legend(handles=legend_elements, loc='best', fontsize=8, frameon=True,
               fancybox=False, shadow=False, framealpha=1.0,
               edgecolor='black', facecolor='white')

    plt.tight_layout()
    fig1.savefig('RB2_velocity_with_states.pdf', dpi=300, bbox_inches='tight')
    print("Velocity plot with state markers saved")
else:
    print("Warning: No velocity data to plot")

# Robustness figure with state markers
if robustness_all:
    fig2, ax2 = plt.subplots(figsize=(8, 5))
    ax2.set_ylim(-1.5, 1.1)  # Extended y-axis to give more space for legends
    ax2.tick_params(axis='both', which='major', direction='in', top=True, right=True)
    ax2.set_xlabel("Time (s)", fontsize=10)
    ax2.set_ylabel("Robustness", fontsize=10)

    # Plot main robustness with SOLID line
    line1 = ax2.plot(time_steps[:len(robustness_all)], robustness_all,
                     color='#CC0000', linewidth=1.2, linestyle='-',  # Solid line for RB_2
                     label="RB_2", zorder=1)

    # Add y=0 reference line
    ax2.axhline(y=0, color='black', linestyle='-', linewidth=1.0, label='Robustness = 0', zorder=2)

    # Add state transition markers for main robustness
    add_state_markers(ax2, time_steps, robustness_all, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)

    # Define colors for predicates
    predicate_colors = ['#0066CC', '#009900', '#FF6600', '#6600CC', '#666666']
    color_idx = 0

    # Plot predicates
    for pred, robs in predicates_robustness_all.items():
        if len(robs) == 0:
            continue

        # Process predicate name
        pred_display = pred[:-3] if len(pred) > 3 and pred.endswith("_0") else pred

        # Plot the predicate with different color and dashed line
        color = predicate_colors[color_idx % len(predicate_colors)]
        ax2.plot(time_steps[:len(robs)], robs,
                 color=color, linewidth=1.2, linestyle='--',  # All predicates use dashed lines
                 label=pred_display, zorder=1)

        # Add state markers for this predicate line
        add_state_markers(ax2, time_steps, robs, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)

        color_idx += 1

    # Professional layout
    ax2.spines['top'].set_visible(True)
    ax2.spines['right'].set_visible(True)
    ax2.set_xlim(START_TIME_SECONDS, END_TIME_SECONDS)
    ax2.grid(False)  # No grid for professional appearance

    # Add main legend for lines
    legend1 = ax2.legend(loc='lower right', fontsize=9, frameon=True,
                         fancybox=False, shadow=False, framealpha=1.0,
                         edgecolor='black', facecolor='white')

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

    # Add second legend for state transitions
    legend2 = ax2.legend(handles=legend_elements, loc='lower left',
                         title='State Transitions', fontsize=8,
                         frameon=True, fancybox=False, shadow=False,
                         framealpha=1.0, edgecolor='black', facecolor='white')
    ax2.add_artist(legend1)  # Keep the first legend

    plt.tight_layout()
    fig2.savefig('RB2_robustness_with_states.pdf', dpi=300, bbox_inches='tight')
    print("Robustness plot with state markers saved")
else:
    print("Warning: No robustness data to plot")

# Distance d figure with state markers
if d:
    fig3, ax3 = plt.subplots(figsize=(8, 5))

    # Calculate appropriate y-axis limits
    if len(d) > 0:
        d_max = max(d) + 0.5
        d_max = min(d_max, 7)  # Cap at 7 as in original
    else:
        d_max = 7

    ax3.set_ylim(0, d_max)
    ax3.tick_params(axis='both', which='major', direction='in', top=True, right=True)
    ax3.set_xlabel("Time (s)", fontsize=10)
    ax3.set_ylabel("lateral deviation (m)", fontsize=10)

    # Plot d line
    ax3.plot(time_steps[:len(d)], d, color='#0066CC', linewidth=1.2, zorder=1)

    # Add state transition markers
    add_state_markers(ax3, time_steps, d, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)

    # Add threshold line
    ax3.axhline(y=1, linestyle="--", color="black", alpha=0.5, linewidth=1.0)
    # ax3.text(START_TIME_SECONDS + (END_TIME_SECONDS - START_TIME_SECONDS) * 0.7,
    #          1.2, 'Threshold for d(d_parking)', color="black", fontsize=9)
    ax3.text(
        START_TIME_SECONDS + (END_TIME_SECONDS - START_TIME_SECONDS) * 0.7,
        1.2,
        r'Threshold for $d(d_{\mathrm{parking}})$',
        color="black",
        fontsize=9
    )

    ax3.set_xlim(START_TIME_SECONDS, END_TIME_SECONDS)
    ax3.grid(False)  # No grid for professional appearance

    # Add spines on all sides
    ax3.spines['top'].set_visible(True)
    ax3.spines['right'].set_visible(True)
    ax3.spines['bottom'].set_visible(True)
    ax3.spines['left'].set_visible(True)

    # Create legend for state transitions
    legend_elements = []
    for time_sec, (label, color) in STATE_TRANSITIONS.items():
        legend_elements.append(
            Line2D([0], [0], marker='o', color='w',
                   markerfacecolor=color, markersize=8,
                   markeredgecolor='black', markeredgewidth=1,
                   label=f'{time_sec}s: {label}')
        )

    ax3.legend(handles=legend_elements, loc='best', fontsize=8, frameon=True,
               fancybox=False, shadow=False, framealpha=1.0,
               edgecolor='black', facecolor='white')

    plt.tight_layout()
    fig3.savefig('RB2_distance_d_with_states.pdf', dpi=300, bbox_inches='tight')
    print("Distance d plot with state markers saved")
else:
    print("Warning: No distance d data to plot")

# Theta figure with state markers
if theta:
    fig4, ax4 = plt.subplots(figsize=(8, 5))

    # Calculate appropriate y-axis limits
    if len(theta) > 0:
        theta_min = min(theta) - 0.02
        theta_max = max(theta) + 0.02
        # Ensure we can see the threshold
        theta_max = max(theta_max, 0.12)
    else:
        theta_min, theta_max = -0.02, 0.12

    ax4.set_ylim(theta_min, theta_max)
    ax4.tick_params(axis='both', which='major', direction='in', top=True, right=True)
    ax4.set_xlabel("Time (s)", fontsize=10)
    ax4.set_ylabel("Theta (rad)", fontsize=10)

    # Plot theta line
    ax4.plot(time_steps[:len(theta)], theta, color='#009900', linewidth=1.2, zorder=1)

    # Add state transition markers
    add_state_markers(ax4, time_steps, theta, STATE_TRANSITIONS, START_TIME_SECONDS, END_TIME_SECONDS)

    # Add threshold line
    ax4.axhline(y=0.09, linestyle="--", color="black", alpha=0.5, linewidth=1.0)
    ax4.text(START_TIME_SECONDS + (END_TIME_SECONDS - START_TIME_SECONDS) * 0.7,
             0.095, 'Threshold for theta', color="black", fontsize=9)

    ax4.set_xlim(START_TIME_SECONDS, END_TIME_SECONDS)
    ax4.grid(False)  # No grid for professional appearance

    # Add spines on all sides
    ax4.spines['top'].set_visible(True)
    ax4.spines['right'].set_visible(True)
    ax4.spines['bottom'].set_visible(True)
    ax4.spines['left'].set_visible(True)

    # Create legend for state transitions
    legend_elements = []
    for time_sec, (label, color) in STATE_TRANSITIONS.items():
        legend_elements.append(
            Line2D([0], [0], marker='o', color='w',
                   markerfacecolor=color, markersize=8,
                   markeredgecolor='black', markeredgewidth=1,
                   label=f'{time_sec}s: {label}')
        )

    ax4.legend(handles=legend_elements, loc='best', fontsize=8, frameon=True,
               fancybox=False, shadow=False, framealpha=1.0,
               edgecolor='black', facecolor='white')

    plt.tight_layout()
    fig4.savefig('RB2_theta_with_states.pdf', dpi=300, bbox_inches='tight')
    print("Theta plot with state markers saved")
else:
    print("Warning: No theta data to plot")

plt.show()

print(f"\nProcessing completed. Displayed data from {START_TIME_SECONDS:.1f}s to {END_TIME_SECONDS:.1f}s")
print(f"Total data points processed: {len(time_steps)}")