import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# Upright fonts and compact layout
mpl.rcParams.update({
    "font.family": ["Times New Roman"],
    "font.style": "normal",
    "mathtext.default": "regular",
    "font.size": 10,
    "axes.labelsize": 11,
    "pdf.fonttype": 42, "ps.fonttype": 42,  # Embed TrueType fonts
    "axes.linewidth": 0.8,
})

# Data
scenarios = ['Bus bulb', 'Bus bay']
original_cost = np.array([1.00, 1.00])
optimized_cost = np.array([0.76, 0.77])

x = np.arange(len(scenarios))
width = 0.36

# Wider landscape layout (increase further if needed)
fig, ax = plt.subplots(figsize=(7.0, 2.6), dpi=300)

# Two-color bars with outlines for grayscale legibility
color1 = "#4C78A8"   # Blue
color2 = "#F58518"   # Orange
b1 = ax.bar(x - width/2, original_cost, width,
            label='Cost of post-optimized planner',
            edgecolor='black', linewidth=0.8, color=color1)
b2 = ax.bar(x + width/2, optimized_cost, width,
            label='Cost of CommonRoad Reactive Planner',
            edgecolor='black', linewidth=0.8, color=color2)

# Axis labels and ticks
ax.set_xlabel('Scenario')
ax.set_ylabel('Normalized Cost')
ax.set_xticks(x, scenarios)

# Leave a compact space for the legend at the top of the axes
ymax = float(max(np.max(original_cost), np.max(optimized_cost)))
ax.set_ylim(0, ymax * 1.24)

# Center value labels inside bars to avoid the legend and top margin
for bars in (b1, b2):
    texts = ax.bar_label(bars, fmt='%.2f', label_type='center', padding=0)
    for t in texts:
        t.set_fontstyle('normal')
        t.set_color('white')
        t.set_weight('semibold')

# Disable the grid
ax.grid(False)

# Center a single-row legend at the top without covering the bars
ax.legend(
    loc='upper center',
    bbox_to_anchor=(0.5, 0.98),
    ncol=2,
    frameon=True, framealpha=0.95, facecolor='white',
    borderaxespad=0.3, handlelength=1.2, handletextpad=0.6
)

# Compact margins
fig.tight_layout(pad=0.2)
fig.subplots_adjust(top=0.90, bottom=0.18, left=0.11, right=0.99)

# Export as vector PDF
fig.savefig('cost_function_comparison.pdf', format='pdf', bbox_inches='tight')
plt.close(fig)
