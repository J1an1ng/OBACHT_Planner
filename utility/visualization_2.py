# third party
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import os
import pandas as pd
from typing import Optional
import yaml
# commonroad

from commonroad.scenario.trajectory import Trajectory
path_root = Path(__file__).resolve().parent.parent
solutions_dir = path_root / "experiments" / "output_result"

def plot_ksstate_trajectory(trajectory, filename: str):
    # Set Seaborn style for better aesthetics
    # sns.set(style="whitegrid")
    path_root = Path(__file__).resolve().parent.parent

    save_dir = path_root / "experiments" / "output_result"

    save_dir.mkdir(parents=True, exist_ok=True)


    base = os.path.basename(filename)
    if not os.path.splitext(base)[1]:
        base += ".png"

    filename = str(save_dir / base)
    # Set global font sizes
    plt.rcParams.update(
        {
            "font.size": 14,
            "axes.titlesize": 16,
            "axes.labelsize": 14,
            "legend.fontsize": 12,
            "xtick.labelsize": 12,
            "ytick.labelsize": 12,
        }
    )

    # Time steps
    k = np.arange(1, len(trajectory.state_list) + 1)
    dt = 0.1  # Time step duration

    # Extract parameters from each state
    x_positions = np.array([state.position[0] for state in trajectory.state_list])
    y_positions = np.array([state.position[1] for state in trajectory.state_list])
    velocities = np.array([state.velocity for state in trajectory.state_list])
    orientations = np.array([state.orientation for state in trajectory.state_list])
    deltas = np.array([state.steering_angle for state in trajectory.state_list])

    # Calculate acceleration and prepend the first element as 0
    accelerations = np.zeros(len(velocities))
    # accelerations[1:] = np.diff(velocities) / dt
    accelerations[0:] = np.array([state.acceleration for state in trajectory.state_list])
    # Calculate jerk and prepend the first element as 0
    jerks = np.zeros(len(velocities))
    jerks[1:] = np.diff(accelerations) / dt

    # Calculate orientation rate of change
    orientation_rates = np.zeros(len(orientations))
    orientation_rates[1:] = np.diff(orientations) / dt

    # Calculate steering rate of change
    steering_rates = np.zeros(len(deltas))
    steering_rates[1:] = np.diff(deltas) / dt

    # Create 8 subplots arranged in 2 columns and 4 rows
    fig, axs = plt.subplots(4, 2, figsize=(18, 24), constrained_layout=True)

    # Flatten the axs array for easy indexing
    axs = axs.flatten()

    # Define a list of plot configurations
    plot_configs = [
        {
            "plot_func": axs[0].plot,
            "data": [x_positions, y_positions],
            "color": "#1f77b4",
            "label": "Trajectory Path",
            "xlabel": "x (m)",
            "ylabel": "y (m)",
            "title": "Position Trajectory (x, y)",
            "scatter": True,
        },
        {
            "plot_func": axs[1].plot,
            "data": [k, velocities],
            "color": "#2ca02c",
            "ylabel": "Velocity (m/s)",
            "title": "Velocity Over Time Steps",
            "ylim": [0, max(velocities) * 1.1],
            "annotation": {
                "text": "Max Velocity",
                "xy": (k[np.argmax(velocities)], max(velocities)),
                "xytext": (k[np.argmax(velocities)] + 10, max(velocities) + 1),
            },
        },
        {
            "plot_func": axs[2].plot,
            "data": [k, accelerations],
            "color": "#d62728",
            "ylabel": "Acceleration (m/s²)",
            "title": "Acceleration Over Time Steps",
        },
        {
            "plot_func": axs[3].plot,
            "data": [k, jerks],
            "color": "#9467bd",
            "ylabel": "Jerk (m/s³)",
            "title": "Jerk Over Time Steps",
        },
        {
            "plot_func": axs[4].plot,
            "data": [k, orientations],
            "color": "#8c564b",
            "ylabel": "Orientation (rad)",
            "title": "Orientation Over Time Steps",
        },
        {
            "plot_func": axs[5].plot,
            "data": [k, orientation_rates],
            "color": "#e377c2",
            "ylabel": "Orientation Rate (rad/s)",
            "title": "Orientation Rate of Change Over Time Steps",
        },
        {
            "plot_func": axs[6].plot,
            "data": [k, deltas],
            "color": "#17becf",
            "ylabel": "Steering Angle (rad)",
            "title": "Steering Angle Over Time Steps",
        },
        {
            "plot_func": axs[7].plot,
            "data": [k, steering_rates],
            "color": "#ff7f0e",
            "ylabel": "Steering Rate (rad/s)",
            "xlabel": "Time Step (k)",
            "title": "Steering Rate of Change Over Time Steps",
        },
    ]

    # Iterate over each plot configurations and apply settings
    for i, config in enumerate(plot_configs):
        ax = axs[i]
        plot_func = config["plot_func"]
        data = config["data"]
        color = config.get("color", "#000000")  # Default to black if not specified
        label = config.get("label", None)

        # Plot the data
        plot_func(*data, color=color, linewidth=2)

        # If scatter is True, add scatter points
        if config.get("scatter", False):
            ax.scatter(
                x_positions, y_positions, color=color, s=15, alpha=0.6, label=label
            )
            if label:
                ax.legend()

        # Set labels and title
        ax.set_xlabel(config.get("xlabel", ""), fontsize=14)
        ax.set_ylabel(config.get("ylabel", ""), fontsize=14)
        ax.set_title(config.get("title", ""), fontsize=16)

        # Set y-axis limits if specified
        if "ylim" in config:
            ax.set_ylim(config["ylim"])

        # Add grid
        ax.grid(True, linestyle="--", alpha=0.5)

        # Add annotation if specified
        if "annotation" in config:
            ann = config["annotation"]
            ax.annotate(
                ann["text"],
                xy=ann["xy"],
                xytext=ann["xytext"],
                arrowprops=dict(facecolor="black", arrowstyle="->"),
                fontsize=12,
            )

    # Remove any unused subplots (if any)
    for j in range(len(plot_configs), len(axs)):
        fig.delaxes(axs[j])

    # Save and close the figure with tight layout
    plt.savefig(filename, dpi=300, bbox_inches="tight")
    plt.close()




def  to_dataframe(traj, save_path: Optional[str] = None) -> pd.DataFrame:
    """
    Convert traj.state_list to a pandas DataFrame, display all rows/columns,
    and optionally save to CSV.

    :param traj: CommonRoad Trajectory instance
    :param save_path: if provided, save the DataFrame to this CSV file path
    :return: pandas DataFrame of trajectory states
    """
    # Ensure full display of DataFrame in console/notebook
    pd.set_option('display.max_rows', None)
    pd.set_option('display.max_columns', None)

    # Build data rows
    records = []
    attrs = traj.state_list[0].attributes
    for s in traj.state_list:
        row = {'time_step': s.time_step}
        for a in attrs:
            val = getattr(s, a)
            # Flatten sequences (e.g., position vectors) into multiple columns
            if hasattr(val, '__len__') and not isinstance(val, (str, bytes)):
                for idx, comp in enumerate(val):
                    row[f"{a}_{idx}"] = comp
            else:
                row[a] = val
        records.append(row)

    df = pd.DataFrame.from_records(records)


    # Save if path provided
    if save_path:
        df.to_csv(save_path, index=False)
        print(f"Trajectory saved to {save_path}")

    return df





def load_plot_config():
    """Load plotting configuration from YAML file"""
    config_path = Path(__file__).parent / "plot_config.yaml"
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        # Return default config if file not found
        return {
            'font': {'family': 'Times New Roman', 'size': 12},
            'axes': {'labelsize': 12, 'titlesize': 14, 'linewidth': 1.0},
            'ticks': {'labelsize': 10},
            'legend': {'fontsize': 11},
            'figure': {'titlesize': 16, 'dpi': 300},
            'lines': {'linewidth': 1.5},
            'grid': {'alpha': 0.3},
            'mathtext': {'fontset': 'stix'},
            'savefig': {'dpi': 300, 'bbox': 'tight', 'format': 'pdf'},
            'colors': {
                'velocity': '#1f77b4',
                'acceleration': '#ff7f0e',
                'jerk': '#2ca02c'
            },
            'linestyles': {
                'velocity': '-',
                'acceleration': '--',
                'jerk': '-.'
            }
        }


def apply_plot_config(config):
    """Apply configuration to matplotlib"""
    # Increase all font sizes by 1.5x
    font_multiplier = 1.5
    plt.rcParams.update({
        'font.family': config['font']['family'],
        'font.size': config['font']['size'] * font_multiplier,
        'axes.labelsize': config['axes']['labelsize'] * font_multiplier,
        'axes.titlesize': config['axes']['titlesize'] * font_multiplier,
        'axes.linewidth': config['axes']['linewidth'],
        'xtick.labelsize': config['ticks']['labelsize'] * font_multiplier,
        'ytick.labelsize': config['ticks']['labelsize'] * font_multiplier,
        'legend.fontsize': config['legend']['fontsize'] * font_multiplier,
        'figure.titlesize': config['figure']['titlesize'] * font_multiplier,
        'lines.linewidth': config['lines']['linewidth'],
        'mathtext.fontset': config['mathtext']['fontset'],
        'savefig.dpi': config['savefig']['dpi'],
        'savefig.bbox': config['savefig']['bbox'],
        'savefig.format': config['savefig']['format']
    })


# 新增的 generate_v_des 函数
def generate_v_des(k, bus_stop_type='bus_bulb'):
    """
    Generate desired velocity (v_des) based on bus stop type

    Args:
        k: Array of time steps
        bus_stop_type: 'bus_bulb' or 'bus_bay'

    Returns:
        Array of desired velocities
    """
    v_des = np.zeros_like(k, dtype=float)

    if bus_stop_type == 'bus_bulb':
        # Bus bulb scenario
        for i, step in enumerate(k):
            if step <= 46:
                v_des[i] = 10
            elif step <= 331:
                v_des[i] = 3
            elif step <= 345:
                v_des[i] = 0
            elif step <= 471:
                v_des[i] = 2.5
            else:
                v_des[i] = 10
    else:  # bus_bay
        # Bus bay scenario
        for i, step in enumerate(k):
            if step <= 46:
                v_des[i] = 10
            elif step <= 264:
                v_des[i] = 3
            elif step <= 286:
                v_des[i] = 0
            elif step <= 431:
                v_des[i] = 2.5
            else:
                v_des[i] = 10

    return v_des

def save_individual_plot(x_data, y_data, title, xlabel, ylabel, filename, config,
                         color_key=None, linestyle_key=None, annotation=None, ylim=None):
    """Save individual plot for a single variable"""
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))

    # Get color and linestyle from config
    color = config['colors'].get(color_key, '#1f77b4') if color_key else '#1f77b4'
    linestyle = config['linestyles'].get(linestyle_key, '-') if linestyle_key else '-'

    # Plot the data
    ax.plot(x_data, y_data, color=color, linestyle=linestyle,
            linewidth=config['lines']['linewidth'])

    # Set labels and title
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)

    # Set y-axis limits if specified
    if ylim:
        ax.set_ylim(ylim)

    # Remove grid (as requested)
    ax.grid(False)

    # Add annotation if specified
    if annotation:
        ax.annotate(
            annotation['text'],
            xy=annotation['xy'],
            xytext=annotation['xytext'],
            arrowprops=dict(facecolor='black', arrowstyle='->'),
            fontsize=config['font']['size']
        )

    # Save the figure
    plt.savefig(filename, dpi=config['savefig']['dpi'],
                bbox_inches=config['savefig']['bbox'])
    plt.close()


def plot_state(trajectory, filename: str):
    """
    Plot trajectory state variables as separate individual plots

    Args:
        trajectory: CommonRoad trajectory object
        filename: Base filename for saving plots
    """
    # Load and apply configuration
    config = load_plot_config()
    apply_plot_config(config)

    # Create output directory
    path_root = Path(__file__).resolve().parent.parent
    save_dir = path_root / "experiments" / "output_result"
    save_dir.mkdir(parents=True, exist_ok=True)

    # Get base filename without extension
    base = os.path.splitext(os.path.basename(filename))[0]

    # Time steps
    k = np.arange(1, len(trajectory.state_list) + 1)
    dt = 0.1  # Time step duration

    # Extract parameters from each state
    x_positions = np.array([state.position[0] for state in trajectory.state_list])
    y_positions = np.array([state.position[1] for state in trajectory.state_list])
    velocities = np.array([state.velocity for state in trajectory.state_list])
    orientations = np.array([state.orientation for state in trajectory.state_list])
    deltas = np.array([state.steering_angle for state in trajectory.state_list])

    # Calculate acceleration
    accelerations = np.zeros(len(velocities))
    accelerations[0:] = np.array([state.acceleration for state in trajectory.state_list])

    # Calculate jerk
    jerks = np.zeros(len(velocities))
    jerks[1:] = np.diff(accelerations) / dt

    # Define plot configurations for individual plots
    plot_configs = [
        {
            'x_data': x_positions,
            'y_data': y_positions,
            'title': 'Position Trajectory',
            'xlabel': 'x (m)',
            'ylabel': 'y (m)',
            'filename': f"{base}_position_trajectory.{config['savefig']['format']}",
            'color_key': None,
            'linestyle_key': None
        },
        {
            'x_data': k,
            'y_data': velocities,
            'title': 'Velocity Over Time Steps',
            'xlabel': 'Time Step (k)',
            'ylabel': 'Velocity (m/s)',
            'filename': f"{base}_velocity.{config['savefig']['format']}",
            'color_key': 'velocity',
            'linestyle_key': 'velocity',
            'ylim': [0, max(velocities) * 1.1] if len(velocities) > 0 else None,
            'annotation': {
                'text': 'Max Velocity',
                'xy': (k[np.argmax(velocities)], max(velocities)),
                'xytext': (k[np.argmax(velocities)] + len(k) * 0.1, max(velocities) + max(velocities) * 0.1)
            } if len(velocities) > 0 else None
        },
        {
            'x_data': k,
            'y_data': accelerations,
            'title': 'Acceleration Over Time Steps',
            'xlabel': 'Time Step (k)',
            'ylabel': 'Acceleration (m/s²)',
            'filename': f"{base}_acceleration.{config['savefig']['format']}",
            'color_key': 'acceleration',
            'linestyle_key': 'acceleration'
        },
        {
            'x_data': k,
            'y_data': jerks,
            'title': 'Jerk Over Time Steps',
            'xlabel': 'Time Step (k)',
            'ylabel': 'Jerk (m/s³)',
            'filename': f"{base}_jerk.{config['savefig']['format']}",
            'color_key': 'jerk',
            'linestyle_key': 'jerk'
        },
        {
            'x_data': k,
            'y_data': orientations,
            'title': 'Orientation Over Time Steps',
            'xlabel': 'Time Step (k)',
            'ylabel': 'Orientation (rad)',
            'filename': f"{base}_orientation.{config['savefig']['format']}"
        },

    ]

    # Generate individual plots
    for plot_config in plot_configs:
        filename_full = str(save_dir / plot_config['filename'])
        save_individual_plot(
            x_data=plot_config['x_data'],
            y_data=plot_config['y_data'],
            title=plot_config['title'],
            xlabel=plot_config['xlabel'],
            ylabel=plot_config['ylabel'],
            filename=filename_full,
            config=config,
            color_key=plot_config.get('color_key'),
            linestyle_key=plot_config.get('linestyle_key'),
            annotation=plot_config.get('annotation'),
            ylim=plot_config.get('ylim')
        )

    print(f"Individual plots saved to: {save_dir}")




# visualization.py 文件的修改部分
def plot_state_dual_config(trajectory1, trajectory2, filename: str, labels=None,
                           show_state_markers=True, state_transitions=None, bus_stop_type='bus_bulb'):
    """
    Plot two trajectory state variables using configuration-based styling with state transition markers

    Args:
        trajectory1: First CommonRoad trajectory object
        trajectory2: Second CommonRoad trajectory object
        filename: Base filename for saving plots
        labels: List of two strings for legend labels, if None uses config default
        show_state_markers: Boolean to show state transition markers
        state_transitions: Dict containing transition points for both trajectories
        bus_stop_type: 'bus_bulb' or 'bus_bay' for v_des calculation
    """
    # Load and apply configuration
    config = load_plot_config()
    apply_plot_config(config)

    # Create output directory
    path_root = Path(__file__).resolve().parent.parent
    save_dir = path_root / "experiments" / "output_result"
    save_dir.mkdir(parents=True, exist_ok=True)

    # Get base filename without extension
    base = os.path.splitext(os.path.basename(filename))[0]

    # Get labels from config if not provided
    if labels is None:
        labels = config.get('dual_labels', {}).get('default', ['Trajectory 1', 'Trajectory 2'])

    # 定义默认的状态转换点
    if state_transitions is None:
        if bus_stop_type == 'bus_bulb':
            state_transitions = {
                'post_opt': {
                    46: ('H→A', '#FF9800'),  # Heading to Arriving (橙色)
                    313: ('A→B', '#9E9E9E'),  # Arriving to Boarding (灰色)
                    334: ('B→D', '#2196F3'),  # Boarding to Departure (蓝色)
                    447: ('D→H', '#4CAF50')  # Departure to Heading (绿色)
                },
                'original': {
                    46: ('H→A', '#FF9800'),
                    331: ('A→B', '#9E9E9E'),
                    345: ('B→D', '#2196F3'),
                    471: ('D→H', '#4CAF50')
                }
            }
        else:  # bus_bay
            state_transitions = {
                'post_opt': {
                    46: ('H→A', '#FF9800'),
                    264: ('A→B', '#9E9E9E'),
                    276: ('B→D', '#2196F3'),
                    410: ('D→H', '#4CAF50')
                },
                'original': {
                    46: ('H→A', '#FF9800'),
                    274: ('A→B', '#9E9E9E'),
                    286: ('B→D', '#2196F3'),
                    431: ('D→H', '#4CAF50')
                }
            }

    # Extract trajectory configurations
    traj1_config = config.get('dual_trajectories', {}).get('trajectory1', {})
    traj2_config = config.get('dual_trajectories', {}).get('trajectory2', {})

    # Process first trajectory
    k1 = np.arange(1, len(trajectory1.state_list) + 1)
    dt = 0.1  # Time step duration

    x_positions1 = np.array([state.position[0] for state in trajectory1.state_list])
    y_positions1 = np.array([state.position[1] for state in trajectory1.state_list])
    velocities1 = np.array([state.velocity for state in trajectory1.state_list])
    orientations1 = np.array([state.orientation for state in trajectory1.state_list])
    deltas1 = np.array([state.steering_angle for state in trajectory1.state_list])

    # Calculate acceleration from velocity differences
    accelerations1 = np.zeros(len(velocities1))
    accelerations1[1:] = np.diff(velocities1) / dt

    jerks1 = np.zeros(len(velocities1))
    jerks1[1:] = np.diff(accelerations1) / dt

    # Process second trajectory
    k2 = np.arange(1, len(trajectory2.state_list) + 1)

    x_positions2 = np.array([state.position[0] for state in trajectory2.state_list])
    y_positions2 = np.array([state.position[1] for state in trajectory2.state_list])
    velocities2 = np.array([state.velocity for state in trajectory2.state_list])
    orientations2 = np.array([state.orientation for state in trajectory2.state_list])
    deltas2 = np.array([state.steering_angle for state in trajectory2.state_list])

    # Calculate acceleration from velocity differences
    accelerations2 = np.zeros(len(velocities2))
    accelerations2[1:] = np.diff(velocities2) / dt

    jerks2 = np.zeros(len(velocities2))
    jerks2[1:] = np.diff(accelerations2) / dt

    # Generate v_des for velocity plots
    max_k = max(len(k1), len(k2))
    k_full = np.arange(1, max_k + 1)
    v_des = generate_v_des(k_full, bus_stop_type)

    # Define plot configurations for dual trajectory plots
    plot_configs = [
        {
            'data': [(x_positions1, y_positions1), (x_positions2, y_positions2)],
            'title': 'Position Trajectory Comparison',
            'xlabel': 'x (m)',
            'ylabel': 'y (m)',
            'filename': f"{base}_position_trajectory_comparison.{config['savefig']['format']}",
            'variable_key': 'position',
            'plot_type': 'scatter',
            'show_markers': False,  # 位置图不需要状态标记
            'legend_position': 'upper right'
        },
        {
            'data': [(k1, velocities1), (k2, velocities2)],
            'data_raw': [velocities1, velocities2],  # 用于标记点
            'title': 'Velocity Comparison Over Time Steps',
            'xlabel': 'Time Step',
            'ylabel': 'Velocity (m/s)',
            'filename': f"{base}_velocity_comparison.{config['savefig']['format']}",
            'variable_key': 'velocity',
            'plot_type': 'line',
            'show_markers': True,  # 速度图显示状态标记
            'show_v_des': True,  # 显示v_des
            'v_des_data': (k_full, v_des),
            'legend_position': 'upper center'  # State transitions legend position for velocity
        },
        {
            'data': [(k1, accelerations1), (k2, accelerations2)],
            'data_raw': [accelerations1, accelerations2],
            'title': 'Acceleration Comparison Over Time Steps',
            'xlabel': 'Time Step',
            'ylabel': 'Acceleration (m/s²)',
            'filename': f"{base}_acceleration_comparison.{config['savefig']['format']}",
            'variable_key': 'acceleration',
            'plot_type': 'line',
            'show_markers': True,  # 加速度图显示状态标记
            'legend_position': 'upper left'  # State transitions legend position for acceleration
        },
        {
            'data': [(k1, jerks1), (k2, jerks2)],
            'data_raw': [jerks1, jerks2],
            'title': 'Jerk Comparison Over Time Steps',
            'xlabel': 'Time Step',
            'ylabel': 'Jerk (m/s³)',
            'filename': f"{base}_jerk_comparison.{config['savefig']['format']}",
            'variable_key': 'jerk',
            'plot_type': 'line',
            'show_markers': True,  # Jerk图显示状态标记
            'legend_position': 'upper left'  # State transitions legend position for jerk
        },
        {
            'data': [(k1, orientations1), (k2, orientations2)],
            'data_raw': [orientations1, orientations2],
            'title': 'Orientation Comparison Over Time Steps',
            'xlabel': 'Time Step',
            'ylabel': 'Orientation (rad)',
            'filename': f"{base}_orientation_comparison.{config['savefig']['format']}",
            'variable_key': 'orientation',
            'plot_type': 'line',
            'show_markers': True,  # 方向角图显示状态标记
            'legend_position': 'lower right'  # Keep original position for orientation
        },
        {
            'data': [(k1, deltas1), (k2, deltas2)],
            'data_raw': [deltas1, deltas2],
            'title': 'Steering Angle Comparison Over Time Steps',
            'xlabel': 'Time Step',
            'ylabel': 'Steering Angle (rad)',
            'filename': f"{base}_steering_comparison.{config['savefig']['format']}",
            'variable_key': 'steering',
            'plot_type': 'line',
            'show_markers': False,  # 转向角图不显示状态标记
            'legend_position': 'upper right'
        }
    ]

    # Generate comparison plots
    for plot_config in plot_configs:
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))

        variable_key = plot_config['variable_key']

        # Plot both trajectories with config-based styling
        trajectory_configs = [traj1_config, traj2_config]

        for i, (x_data, y_data) in enumerate(plot_config['data']):
            traj_config = trajectory_configs[i]
            label = labels[i]

            # Get styling from config
            color = traj_config.get('colors', {}).get(variable_key, '#1f77b4')
            linestyle = traj_config.get('linestyles', {}).get(variable_key, '-')
            marker = traj_config.get('markers', {}).get(variable_key, 'None')
            alpha = traj_config.get('alpha', 0.8)
            linewidth = traj_config.get('linewidth', 1.5)
            markersize = traj_config.get('markersize', 4)

            # Handle marker 'None' string
            if marker == 'None':
                marker = None

            # Plot based on type
            if plot_config['plot_type'] == 'scatter':
                ax.plot(x_data, y_data, color=color, linestyle=linestyle,
                        linewidth=linewidth, alpha=alpha, label=label)
                if marker is not None:
                    ax.scatter(x_data, y_data, color=color, marker=marker,
                               s=markersize ** 2, alpha=alpha * 0.8)
            else:
                ax.plot(x_data, y_data, color=color, linestyle=linestyle,
                        linewidth=linewidth, alpha=alpha, marker=marker,
                        markersize=markersize, label=label, zorder=1)

        # Add v_des for velocity plots
        if plot_config.get('show_v_des', False) and 'v_des_data' in plot_config:
            k_des, v_des_values = plot_config['v_des_data']
            # Plot v_des as a step function with markers
            ax.step(k_des, v_des_values, where='post', color='#808080',
                    linestyle=':', linewidth=2, alpha=0.7, zorder=0)
            # Add markers at transition points for visibility
            # v_des_transitions = []
            # prev_v = v_des_values[0]
            # for i, v in enumerate(v_des_values):
            #     if v != prev_v or i == 0:
            #         v_des_transitions.append(i)
            #     prev_v = v
            #
            # # Plot markers at transitions
            # for idx in v_des_transitions:
            #     if idx < len(k_des):
            #         ax.scatter(k_des[idx], v_des_values[idx], color='#808080',
            #                    marker='D', s=80, edgecolors='black', linewidth=1,
            #                    zorder=2, alpha=0.8)

        # 添加状态转换标记 - 两个planner都用圆形
        if show_state_markers and plot_config.get('show_markers', False) and 'data_raw' in plot_config:
            # 为Post-optimization添加标记 (trajectory1) - 圆形
            for step, (trans_name, color) in state_transitions['post_opt'].items():
                # 调整索引（因为时间步从1开始，但数组索引从0开始）
                idx = step - 1 if step > 0 else step
                if idx < len(plot_config['data_raw'][0]):
                    ax.scatter(step, plot_config['data_raw'][0][idx],
                               color=color, s=120, marker='o',  # 圆形
                               edgecolors='black', linewidth=1.5,
                               zorder=3, alpha=0.9)

            # 为Original planner添加标记 (trajectory2) - 也用圆形
            for step, (trans_name, color) in state_transitions['original'].items():
                idx = step - 1 if step > 0 else step
                if idx < len(plot_config['data_raw'][1]):
                    ax.scatter(step, plot_config['data_raw'][1][idx],
                               color=color, s=120, marker='o',  # 圆形
                               edgecolors='black', linewidth=1.5,
                               zorder=3, alpha=0.9)

        # Set labels and title with increased font size
        ax.set_xlabel(plot_config['xlabel'], fontsize=plt.rcParams['axes.labelsize'])
        ax.set_ylabel(plot_config['ylabel'], fontsize=plt.rcParams['axes.labelsize'])
        ax.set_title(plot_config['title'], fontsize=plt.rcParams['axes.titlesize'])

        # Add legend
        legend_handles = []
        legend_labels = []

        # Add main trajectory lines to legend
        for i, label in enumerate(labels):
            traj_config = trajectory_configs[i]
            color = traj_config.get('colors', {}).get(variable_key, '#1f77b4')
            linestyle = traj_config.get('linestyles', {}).get(variable_key, '-')
            from matplotlib.lines import Line2D
            line = Line2D([0], [0], color=color, linestyle=linestyle,
                          linewidth=traj_config.get('linewidth', 1.5), label=label)
            legend_handles.append(line)
            legend_labels.append(label)

        # Add v_des to legend if shown
        if plot_config.get('show_v_des', False):
            from matplotlib.lines import Line2D
            # v_des_line = Line2D([0], [0], color='#808080', linestyle=':',
            #                     linewidth=2, marker='D', markersize=6,
            #                     markerfacecolor='#808080', markeredgecolor='black',
            #                     label=r'$v_{\mathrm{des}}$')
            v_des_line = Line2D([0], [0], color='#808080', linestyle=':',
                                linewidth=2, label=r'$v_{\mathrm{des}}$')
            legend_handles.append(v_des_line)
            legend_labels.append(r'desired velocity $v_{\mathrm{des}}$')

        # Create main legend with increased font size
        # Determine main legend position based on variable type
        if plot_config['variable_key'] == 'acceleration':
            main_legend_pos = 'lower right'
        elif plot_config['variable_key'] == 'orientation':
            main_legend_pos = 'upper left'
        elif plot_config['variable_key'] == 'jerk':
            main_legend_pos = 'upper right'
        else:  # velocity, position, steering
            main_legend_pos = 'lower left'

        legend1 = ax.legend(handles=legend_handles, labels=legend_labels,
                            loc=main_legend_pos, fontsize=plt.rcParams['legend.fontsize'])

        # 如果显示了状态标记，添加状态转换图例
        if show_state_markers and plot_config.get('show_markers', False):
            # 创建状态转换图例元素
            from matplotlib.lines import Line2D
            legend_elements = []

            # 添加转换类型的颜色说明
            unique_transitions = {}
            for _, (trans_name, color) in state_transitions['post_opt'].items():
                if trans_name not in unique_transitions:
                    unique_transitions[trans_name] = color

            for trans_name, color in unique_transitions.items():
                legend_elements.append(
                    Line2D([0], [0], marker='o', color='w',
                           markerfacecolor=color, markersize=10,
                           markeredgecolor='black', markeredgewidth=1,
                           label=trans_name)
                )

            # Determine legend position based on plot type
            state_legend_pos = plot_config.get('legend_position', 'lower right')

            # 添加第二个图例（状态转换）with increased font size
            legend2 = ax.legend(handles=legend_elements, loc=state_legend_pos,
                                title='State Transitions',
                                fontsize=plt.rcParams['legend.fontsize'] * 0.9,
                                title_fontsize=plt.rcParams['legend.fontsize'])
            ax.add_artist(legend1)  # 保留第一个图例

        # 强制关闭网格
        ax.grid(False)

        # Save the figure
        filename_full = str(save_dir / plot_config['filename'])
        plt.savefig(filename_full, dpi=config['savefig']['dpi'],
                    bbox_inches=config['savefig']['bbox'])
        plt.close()

    print(f"Config-based dual trajectory comparison plots with state markers saved to: {save_dir}")

def save_config_dual_plot(x_data1, y_data1, x_data2, y_data2, title, xlabel, ylabel,
                          filename, config, labels, variable_key, plot_type='line'):
    """Save individual comparison plot using configuration-based styling"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    # Get trajectory configurations
    traj1_config = config.get('dual_trajectories', {}).get('trajectory1', {})
    traj2_config = config.get('dual_trajectories', {}).get('trajectory2', {})

    trajectory_configs = [traj1_config, traj2_config]
    data_pairs = [(x_data1, y_data1), (x_data2, y_data2)]

    # Plot both trajectories
    for i, (x_data, y_data) in enumerate(data_pairs):
        traj_config = trajectory_configs[i]
        label = labels[i]

        # Get styling from config
        color = traj_config.get('colors', {}).get(variable_key, '#1f77b4')
        linestyle = traj_config.get('linestyles', {}).get(variable_key, '-')
        marker = traj_config.get('markers', {}).get(variable_key, 'None')
        alpha = traj_config.get('alpha', 0.8)
        linewidth = traj_config.get('linewidth', 1.5)
        markersize = traj_config.get('markersize', 4)

        # Handle marker 'None' string
        if marker == 'None':
            marker = None

        # Plot based on type
        if plot_type == 'scatter':
            ax.plot(x_data, y_data, color=color, linestyle=linestyle,
                    linewidth=linewidth, alpha=alpha, label=label)
            if marker is not None:
                ax.scatter(x_data, y_data, color=color, marker=marker,
                           s=markersize ** 2, alpha=alpha * 0.8)
        else:
            ax.plot(x_data, y_data, color=color, linestyle=linestyle,
                    linewidth=linewidth, alpha=alpha, marker=marker,
                    markersize=markersize, label=label)

    # Set labels and title
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)

    # Add legend
    ax.legend()

    # Add grid based on config
    if config.get('grid', {}).get('alpha', 0) > 0:
        ax.grid(True, alpha=config['grid']['alpha'])
    else:
        ax.grid(False)

    # Save the figure
    plt.savefig(filename, dpi=config['savefig']['dpi'],
                bbox_inches=config['savefig']['bbox'])
    plt.close()