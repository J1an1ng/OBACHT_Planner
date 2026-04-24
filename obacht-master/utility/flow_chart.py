import matplotlib.pyplot as plt

# 统一使用Times New Roman字体
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman"],
    "mathtext.fontset": "stix",
    "mathtext.rm": "Times New Roman",
    "font.monospace": ["Times New Roman"]
})

SCALE = 1.3  # 除 Arriving 内部 & 状态名称外，其他文字放大倍数

# —— 调整参数 —— #
SHIFT_BA_DEP = 0.6       # BA 与 Departure 整体向左移动的距离
ARROW_SHORTEN_BT = 0.6   # 缩短 BA→Terminal 水平箭头（终点向右移）
LABEL_LEFT_SHIFT = 0.4   # 将 "final_stop = True" 再向左偏的量

def draw_state_diagram():

    fig, ax = plt.subplots(figsize=(12.5, 5))
    ax.set_ylim(0.6, 6.5)
    ax.axis('off')

    # —— 可调公共边距（左右相等）——
    MARGIN = 0.5

    boxes = {
        "Terminal": {"xy": (-6.50, 4.1), "w": 3.0, "h": 1.8},
        "Boarding and Alighting": {"xy": (0.25, 4.0), "w": 3.5, "h": 2.0},
        "Departure": {"xy": (0.25, 0.7), "w": 3.5, "h": 2.0},
        "Heading to Next Station": {"xy": (7.75, 0.7), "w": 3.5, "h": 2.0},
        "Arriving": {"xy": (7.75, 4.0), "w": 3.5, "h": 2.0},
    }

    # —— 面积 +25%（宽度×1.25），右边框保持不变 ——
    grow_factor = 1.25
    to_grow = ["Boarding and Alighting", "Departure", "Heading to Next Station", "Arriving"]
    for name in to_grow:
        x, y = boxes[name]["xy"]
        w, h = boxes[name]["w"], boxes[name]["h"]
        right = x + w
        new_w = w * grow_factor
        boxes[name]["w"] = new_w
        boxes[name]["xy"] = (right - new_w, y)

    # —— 将 BA 与 Departure 整体左移 —— #
    for name in ["Boarding and Alighting", "Departure"]:
        x, y = boxes[name]["xy"]
        boxes[name]["xy"] = (x - SHIFT_BA_DEP, y)

    # —— 依据“等边距”设定画布左右边界 ——
    h2n_right = boxes["Heading to Next Station"]["xy"][0] + boxes["Heading to Next Station"]["w"]
    xmax = h2n_right + MARGIN
    terminal_left = boxes["Terminal"]["xy"][0]
    xmin = terminal_left - MARGIN
    ax.set_xlim(xmin, xmax)

    def bx(name):
        info = boxes[name]
        x, y, w, h = info["xy"][0], info["xy"][1], info["w"], info["h"]
        return {"x": x, "y": y, "w": w, "h": h,
                "left": x, "right": x + w, "top": y + h, "bottom": y,
                "cx": x + w/2, "cy": y + h/2}

    # 绘制盒子
    for name, info in boxes.items():
        x, y, w, h = info["xy"][0], info["xy"][1], info["w"], info["h"]
        ax.add_patch(plt.Rectangle((x, y), w, h, fill=False, linewidth=1.7))
        # 状态名称：保持不变（字号 13）
        ax.text(x + w / 2, y + h - 0.18, name, ha='center', va='top',
                fontsize=13, fontweight='bold', fontfamily='Times New Roman')

        if name == "Boarding and Alighting":
            ax.text(x + 1.35, y + h - 0.68,
                    "\n".join([
                        r"$\mathbf{v}_{\mathrm{des}}$: None",
                        r"$\mathbf{d}_{\mathrm{des}}$: None",
                        r"$\mathbf{s}_{\mathrm{des}}$: None",
                    ]),
                    ha='left', va='top', fontsize=10.5 * SCALE, fontfamily='Times New Roman', linespacing=1.1)
            ax.text(x + 0.15, y + h - 1.42,
                    "\n".join(["entry action:", "start timer C"]),
                    ha='left', va='top', fontsize=10.5 * SCALE, fontfamily='Times New Roman', linespacing=1.1)

        elif name == "Terminal":
            ax.text(x + 1.15, y + h - 0.62,
                    "\n".join([
                        r"$\mathbf{v}_{\mathrm{des}}$: None",
                        r"$\mathbf{d}_{\mathrm{des}}$: None",
                        r"$\mathbf{s}_{\mathrm{des}}$: None",
                    ]),
                    ha='left', va='top', fontsize=10.5 * SCALE, fontfamily='Times New Roman', linespacing=1.1)

        elif name == "Departure":
            ax.text(x + 0.85, y + h - 0.58,
                    "\n".join([
                        r"$\mathbf{v}_{\mathrm{des}} = 5\,\mathrm{m/s}$",
                        "",
                        r"$\mathbf{d}_{\mathrm{des}}$: center line of the road",
                        "",
                        r"$\mathbf{s}_{\mathrm{des}}$: None"
                    ]),
                    ha='left', va='top', fontsize=10.5 * SCALE, fontfamily='Times New Roman', linespacing=1.1)

        elif name == "Heading to Next Station":
            ax.text(x + 0.85, y + h - 0.58,
                    "\n".join([
                        r"$\mathbf{v}_{\mathrm{des}} = 10\,\mathrm{m/s}$",
                        "",
                        r"$\mathbf{d}_{\mathrm{des}}$: center line of the road",
                        "",
                        r"$\mathbf{s}_{\mathrm{des}}$: None"
                    ]),
                    ha='left', va='top', fontsize=10.5 * SCALE, fontfamily='Times New Roman', linespacing=1.1)

        elif name == "Arriving":
            # Arriving 内部：全部保持原字号（10.5 / 9）
            block_top = [
                r"$\mathbf{v}_{\mathrm{des}} = 3\,\mathrm{m/s}$",
                r"$\mathbf{d}_{\mathrm{des}}$: center line of the road",
                r"$\mathbf{s}_{\mathrm{des}}$: None"
            ]
            block_bottom = [
                r"$\mathbf{v}_{\mathrm{des}} = 0$",
                r"$\mathbf{d}_{\mathrm{des}}$: goal region",
                r"$\mathbf{s}_{\mathrm{des}}$: goal region"
            ]
            # 左列文本
            ax.text(x + 0.15, y + h - 0.65-0.15, "\n".join(block_top),
                    ha='left', va='top', fontsize=10.5, fontfamily='Times New Roman', linespacing=1.1)
            # 右列文本
            ax.text(x + 2.8, y + h - 0.65-0.15, "\n".join(block_bottom),
                    ha='left', va='top', fontsize=10.5, fontfamily='Times New Roman', linespacing=1.1)

            rect_top = y + h - 0.40      # 顶部略高于文字
            rect_h  = 0.90               # 底线上抬 → 框更矮

            center = x + w / 2
            pad_x  = 0.10                # 两侧内边距
            GAP    = 0.18                # 中间空隙，确保不相连

            # block_top（左块）：右边界向右扩一些（更宽）
            left_rect_x = x + pad_x
            desired_left_right = center + 0.50
            max_left_right = (x + w - pad_x) - GAP - 0.10
            left_rect_right = min(desired_left_right, max_left_right)
            left_rect_w = max(0.1, left_rect_right - left_rect_x)
            ax.add_patch(plt.Rectangle((left_rect_x, rect_top - rect_h-0.15),
                                       left_rect_w, rect_h,
                                       fill=False, linestyle=(0, (3, 3)), linewidth=0.8, zorder=2))

            # block_bottom（右块）：左边界往右收（更窄）
            right_rect_x = left_rect_right + GAP-0.1
            right_rect_right = x + w - pad_x
            right_rect_w = max(0.1, right_rect_right - right_rect_x)
            ax.add_patch(plt.Rectangle((right_rect_x, rect_top - rect_h-0.15),
                                       right_rect_w, rect_h,
                                       fill=False, linestyle=(0, (3, 3)), linewidth=0.8, zorder=2))

            # —— 计算两个方框的底边 y 值（相同）——
            rect_bottom_y = rect_top - rect_h - 0.15  # NEW: 虚线方框底边 y

            # 状态内部的标签位置（保持不变）
            mid_y = y + h - 1.8

            # 原“水平箭头”改造：
            # 1) 定义左右两端点（左点 / 右点）
            left_pt  = (x + 1.0,     mid_y)
            right_pt = (x + w - 1.0, mid_y)

            # 2) 先从左点作垂线到“左方框底边”
            ax.plot([left_pt[0], left_pt[0]],
                    [left_pt[1], rect_bottom_y],
                    linewidth=1.2, zorder=3,color = "black")  # NEW: 左点垂线

            # 3) 用“无箭头”的线段连接左点与右点（取代原水平箭头）
            ax.plot([left_pt[0], right_pt[0]],
                    [left_pt[1], right_pt[1]],
                    linewidth=1.2, zorder=3,color ="black" )  # NEW: 左右点连线（无箭头）

            # 4) 以右点为起点，画新箭头，指向“右方框底边”
            ax.annotate("",
                        xy=(right_pt[0], rect_bottom_y),  # 箭头尖到达右方框底边
                        xytext=(right_pt[0], right_pt[1]-0.01),
                        arrowprops=dict(arrowstyle="->", linewidth=1.2),
                        zorder=3)  # NEW: 右点→右方框底边的新箭头

            # --- NEW: 粗略取 A 的位置，画一条水平线 + 一个箭头（全黑） ---
            A_mid_y = y + h - 0.30  # 近似当作“Arriving”标题的中线 y
            A_left_x_approx = x + w / 2 - 0.35  # 近似当作字母 A 的左沿 x
            seg_right = (A_left_x_approx - 0.15, A_mid_y)  # 右端点：在 A 左边再左 0.15
            seg_left = (left_pt[0], A_mid_y)  # 左端点：x 与左点一致

            # 新水平线段（黑色）
            # ax.plot([seg_left[0], seg_right[0]],
            #         [seg_left[1], seg_right[1]],
            #         linewidth=1.2, zorder=3, color="black")

            # 新箭头：尾部在这条线段左端点；尖端在左虚线方框顶边上（黑色）
            top_y = rect_top - 0.15
            tip_x = min(max(seg_left[0], left_rect_x), left_rect_x + left_rect_w)
            ax.annotate("", xy=(tip_x, top_y-0.02), xytext=(seg_left[0], seg_left[1]+0.02),
                        arrowprops=dict(arrowstyle="->", linewidth=1.2, color="black"),
                        zorder=3)

            # 条件文本（保持不变）
            ax.text(x + w / 2, mid_y + 0.0, r"$d_{\mathrm{goal}} < 15\,\mathrm{m}$",
                    ha='center', va='bottom', fontsize=10.5, fontfamily='Times New Roman',
                    backgroundcolor='white', zorder=1)

            # 小标题
            left_label_x = left_rect_x + left_rect_w / 2 - 0.75
            right_label_x = right_rect_x + right_rect_w / 2 - 0.40
            label_y = rect_top -0.19  # 标题离框顶稍微留点间距

            ax.text(left_label_x, label_y-0.15, "Decelerating",
                    ha='center', va='bottom',
                    fontsize=9, fontweight='bold',
                    fontfamily='Times New Roman', backgroundcolor='white', zorder=0)

            ax.text(right_label_x, label_y-0.15, "Parking",
                    ha='center', va='bottom',
                    fontsize=9, fontweight='bold',
                    fontfamily='Times New Roman', backgroundcolor='white', zorder=0)

    # 水平箭头的上下偏移
    LABEL_UP = 0.26
    LABEL_DOWN = 0.20

    def arrow(start, end, label=None, label_pos=None, text_below=None):
        ax.annotate("", xy=end, xytext=start,
                    arrowprops=dict(arrowstyle="->", linewidth=1.5))
        if label:
            if label_pos:
                lx, ly = label_pos
            else:
                lx = (start[0] + end[0]) / 2
                ly = (start[1] + end[1]) / 2 + LABEL_UP
            ax.text(lx, ly, label, ha='center', va='center',
                    fontsize=10.5 * SCALE, fontfamily='Times New Roman', backgroundcolor='white')
        if text_below:
            bx = (start[0] + end[0]) / 2
            by = (start[1] + end[1]) / 2 - LABEL_DOWN
            ax.text(bx, by, text_below, ha='center', va='center',
                    fontsize=10.5 * SCALE, fontfamily='Times New Roman', backgroundcolor='white')

    # ----- 箭头（自动基于盒子边缘） -----
    BA  = bx("Boarding and Alighting")
    DEP = bx("Departure")
    H2N = bx("Heading to Next Station")
    ARR = bx("Arriving")
    TER = bx("Terminal")

    # BA -> Terminal（水平，缩短箭头 + 标签左移）
    start_bt = (BA["left"], BA["cy"])
    end_bt   = (TER["right"], TER["cy"])  # 终点右移 → 箭头更短
    midx_bt  = (start_bt[0] + end_bt[0]) / 2 - LABEL_LEFT_SHIFT
    midy_bt  = (start_bt[1] + end_bt[1]) / 2 + LABEL_UP
    arrow(start_bt, end_bt, label="final_stop = True", label_pos=(midx_bt+0.4, midy_bt))

    # BA -> Departure（竖直，中点放标签）
    start_bd = (BA["cx"], BA["bottom"])
    end_bd   = (DEP["cx"], DEP["top"])
    mid_bd   = ((start_bd[0] + end_bd[0]) / 2, (start_bd[1] + end_bd[1]) / 2)
    arrow(start_bd, end_bd,
          label=r"$\mathrm{door\_closed} = \mathrm{True} \wedge C > T_{\mathrm{threshold}}$",
          label_pos=mid_bd)

    # Departure -> H2N（水平）
    start_dh = (DEP["right"], DEP["cy"])
    end_dh   = (H2N["left"], H2N["cy"])
    arrow(start_dh, end_dh)
    midx_dh = (start_dh[0] + end_dh[0]) / 2
    midy_dh = (start_dh[1] + end_dh[1]) / 2
    ax.text(midx_dh, midy_dh + LABEL_UP,
            r"$|v_{\mathrm{cur}} - v_{\mathrm{des}}|<10^{-1}$",
            ha='center', va='center', fontsize=10.5 * SCALE, fontfamily='Times New Roman', backgroundcolor='white')
    ax.text(midx_dh, midy_dh - LABEL_DOWN,
            r"merge_into_main_traffic = True",
            ha='center', va='center', fontsize=10.5 * SCALE, fontfamily='Times New Roman', backgroundcolor='white',zorder=0)

    # H2N -> Arriving（竖直，中点放标签）
    start_ha = (H2N["cx"], H2N["top"])
    end_ha   = (ARR["cx"], ARR["bottom"])
    mid_ha   = ((start_ha[0] + end_ha[0]) / 2, (start_ha[1] + end_ha[1]) / 2)
    arrow(start_ha, end_ha,
          label=r"$d_{\mathrm{goal}} < 100\,\mathrm{m}$",
          label_pos=mid_ha)

    # Arriving -> BA（水平）
    start_ab = (ARR["left"], ARR["cy"])
    end_ab   = (BA["right"], BA["cy"])
    arrow(start_ab, end_ab)
    midx_ab = (start_ab[0] + end_ab[0]) / 2
    midy_ab = (start_ab[1] + end_ab[1]) / 2
    ax.text(midx_ab, midy_ab + LABEL_UP, "fully_stop = True",
            ha='center', va='center', fontsize=10.5 * SCALE, fontfamily='Times New Roman', backgroundcolor='white')
    ax.text(midx_ab, midy_ab - LABEL_DOWN,
            r"$v_{\mathrm{cur}} \in [-10^{-3}, +10^{-3}]$",
            ha='center', va='center', fontsize=10.5 * SCALE, fontfamily='Times New Roman', backgroundcolor='white')

    plt.tight_layout(pad=0.2)
    plt.savefig('state_diagram.pdf', dpi=300, bbox_inches='tight', pad_inches=0.05)
    plt.savefig('state_diagram.png', dpi = 1200, bbox_inches='tight', pad_inches=0.05)
    plt.show()

if __name__ == "__main__":
    draw_state_diagram()
