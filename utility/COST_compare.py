import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# —— 正体字体 & 紧凑排版 ——
mpl.rcParams.update({
    "font.family": ["Times New Roman"],
    "font.style": "normal",
    "mathtext.default": "regular",
    "font.size": 10,
    "axes.labelsize": 11,
    "pdf.fonttype": 42, "ps.fonttype": 42,  # 嵌入 TrueType
    "axes.linewidth": 0.8,
})

# 数据
scenarios = ['Bus bulb', 'Bus bay']
original_cost = np.array([1.00, 1.00])
optimized_cost = np.array([0.76, 0.77])

x = np.arange(len(scenarios))
width = 0.36

# 更宽的横向尺寸（按需再调大）
fig, ax = plt.subplots(figsize=(7.0, 2.6), dpi=300)

# 双色柱（黑白打印仍可区分，保留描边）
color1 = "#4C78A8"   # 蓝
color2 = "#F58518"   # 橙
b1 = ax.bar(x - width/2, original_cost, width,
            label='Cost of post-optimized planner',
            edgecolor='black', linewidth=0.8, color=color1)
b2 = ax.bar(x + width/2, optimized_cost, width,
            label='Cost of CommonRoad Reactive Planner',
            edgecolor='black', linewidth=0.8, color=color2)

# 轴标签与刻度
ax.set_xlabel('Scenario')
ax.set_ylabel('Normalized Cost')
ax.set_xticks(x, scenarios)

# 顶部为图内图例留空间（不至于空太多）
ymax = float(max(np.max(original_cost), np.max(optimized_cost)))
ax.set_ylim(0, ymax * 1.24)

# 数值标注：柱内居中，避免与图例/顶部冲突
for bars in (b1, b2):
    texts = ax.bar_label(bars, fmt='%.2f', label_type='center', padding=0)
    for t in texts:
        t.set_fontstyle('normal')
        t.set_color('white')
        t.set_weight('semibold')

# 关闭网格
ax.grid(False)

# 图例放图内顶部居中（单行），不压柱子
ax.legend(
    loc='upper center',
    bbox_to_anchor=(0.5, 0.98),
    ncol=2,
    frameon=True, framealpha=0.95, facecolor='white',
    borderaxespad=0.3, handlelength=1.2, handletextpad=0.6
)

# 紧凑边距
fig.tight_layout(pad=0.2)
fig.subplots_adjust(top=0.90, bottom=0.18, left=0.11, right=0.99)

# 导出 PDF（矢量）
fig.savefig('cost_function_comparison.pdf', format='pdf', bbox_inches='tight')
plt.close(fig)
