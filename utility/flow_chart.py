"""Draw the state machine used by the current bus-stop-bay planner.

The state topology and transition guards come from
``post_optimization_planner/State.py`` and
``post_optimization_planner/state_machine.py``. Planner targets shown inside
the states come from ``configurations/bus_stop_bay/*.yaml``.
"""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch


plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 10,
    }
)


BOX_W = 3.25
BOX_H = 1.75
TOP_Y = 5.15
BOTTOM_Y = 1.55
STATE_X = {
    "HEADING": 0.0,
    "ARRIVING": 4.25,
    "ALIGN": 8.50,
    "MERGE": 12.75,
    "DEPARTING": -4.25,
    "SERVICE": 0.0,
    "STOPPING": 4.25,
    "FINAL": 12.75,
}


def _box_geometry(name):
    """Return useful anchor points for a state box."""
    x = STATE_X[name]
    y = TOP_Y if name in {"HEADING", "ARRIVING", "ALIGN", "MERGE"} else BOTTOM_Y
    return {
        "x": x,
        "y": y,
        "left": x,
        "right": x + BOX_W,
        "bottom": y,
        "top": y + BOX_H,
        "cx": x + BOX_W / 2,
        "cy": y + BOX_H / 2,
    }


def _draw_state(ax, name, title, lines, *, implicit=False):
    box = _box_geometry(name)
    title_size = 10.0 if len(title) > 20 else 11.0
    patch = FancyBboxPatch(
        (box["x"], box["y"]),
        BOX_W,
        BOX_H,
        boxstyle="round,pad=0.025,rounding_size=0.04",
        facecolor="#f7f9fb" if not implicit else "#fffaf0",
        edgecolor="black",
        linewidth=1.35,
        linestyle="--" if implicit else "-",
        zorder=2,
    )
    ax.add_patch(patch)
    ax.text(
        box["cx"],
        box["top"] - 0.17,
        title,
        ha="center",
        va="top",
        fontsize=title_size,
        fontweight="bold",
        zorder=3,
    )
    ax.text(
        box["left"] + 0.18,
        box["top"] - 0.55,
        "\n".join(lines),
        ha="left",
        va="top",
        fontsize=9.2,
        linespacing=1.25,
        zorder=3,
    )
    return box


def _arrow(ax, start, end, *, connectionstyle="arc3", linestyle="-"):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops={
            "arrowstyle": "->",
            "linewidth": 1.35,
            "color": "black",
            "linestyle": linestyle,
            "connectionstyle": connectionstyle,
            "shrinkA": 1,
            "shrinkB": 1,
        },
        zorder=1,
    )


def _label(ax, x, y, text, *, fontsize=8.5, ha="center", va="center"):
    ax.text(
        x,
        y,
        text,
        ha=ha,
        va=va,
        fontsize=fontsize,
        linespacing=1.15,
        bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.5},
        zorder=4,
    )


def draw_state_diagram(output_dir=None, show=False):
    """Generate PDF and PNG versions of the current state-machine diagram."""
    fig, ax = plt.subplots(figsize=(20, 10))
    ax.set_xlim(-5.0, 17.0)
    ax.set_ylim(-0.45, 8.75)
    ax.axis("off")

    # UML-style composite state containing the three staged bay-entry states.
    composite_x = 8.15
    composite_y = 1.18
    composite_w = 8.20
    composite_h = 6.72
    ax.add_patch(
        FancyBboxPatch(
            (composite_x, composite_y),
            composite_w,
            composite_h,
            boxstyle="round,pad=0.035,rounding_size=0.06",
            facecolor="#fbfcfe",
            edgecolor="black",
            linewidth=1.55,
            zorder=0,
        )
    )
    ax.text(
        composite_x + composite_w / 2,
        composite_y + composite_h - 0.14,
        "BEFORE STOPPING (COMPOSITE STATE)",
        ha="center",
        va="top",
        fontsize=11.2,
        fontweight="bold",
        zorder=3,
    )

    heading = _draw_state(
        ax,
        "HEADING",
        "HEADING TO NEXT STATION",
        [
            r"$v_{\mathrm{des}} = 10.0\,\mathrm{m/s}$",
            r"$d \in [-3.75,\,0]\,\mathrm{m}$",
            "reference path: lanelet 2",
        ],
    )
    arriving = _draw_state(
        ax,
        "ARRIVING",
        "ARRIVING",
        [
            r"$v_{\mathrm{des}} = 5.0\,\mathrm{m/s}$",
            r"$d \in [-1.5,\,1.5]\,\mathrm{m}$",
            "reference path: lanelet 1",
        ],
    )
    align = _draw_state(
        ax,
        "ALIGN",
        "BEFORE STOPPING: ALIGN",
        [
            r"$v_{\mathrm{des}} = 2.0\,\mathrm{m/s}$",
            r"$d \in [-0.35,\,0.35]\,\mathrm{m}$",
            "reference: current pose",
        ],
    )
    merge = _draw_state(
        ax,
        "MERGE",
        "BEFORE STOPPING: MERGE",
        [
            r"$v_{\mathrm{des}} = 1.2\,\mathrm{m/s}$",
            r"$d \in [-0.8,\,0.8]\,\mathrm{m}$",
            "smooth lateral shift into bay",
        ],
    )
    final = _draw_state(
        ax,
        "FINAL",
        "BEFORE STOPPING: FINAL",
        [
            r"$v_{\mathrm{des}} = 0$",
            "mode: longitudinal stopping",
            r"target: $(x_{\mathrm{goal}},y_{\mathrm{goal}})$",
        ],
    )
    stopping = _draw_state(
        ax,
        "STOPPING",
        "STOPPING",
        [
            r"$v_{\mathrm{des}} = 0$",
            r"$d \in [-0.8,\,0.8]\,\mathrm{m}$",
            "target: goal-region center",
        ],
    )
    service = _draw_state(
        ax,
        "SERVICE",
        "BOARDING / ALIGHTING",
        [
            "implicit stopped-state hold",
            r"$v = 0$",
            r"hold while counter $\leq 15$",
        ],
        implicit=True,
    )
    departing = _draw_state(
        ax,
        "DEPARTING",
        "DEPARTING",
        [
            r"$v_{\mathrm{des}} = 2.5\,\mathrm{m/s}$",
            r"$d \in [-3.0,\,3.0]\,\mathrm{m}$",
            "smooth merge to lanelet 2",
        ],
    )

    # Nominal approach sequence.
    _arrow(ax, (heading["right"], heading["cy"]), (arriving["left"], arriving["cy"]))
    _label(
        ax,
        (heading["right"] + arriving["left"]) / 2,
        heading["top"] + 0.28,
        r"$0 < x_{\mathrm{goal}}-x < 100\,\mathrm{m}$",
    )

    _arrow(ax, (arriving["right"], arriving["cy"]), (align["left"], align["cy"]))
    _label(
        ax,
        (arriving["right"] + align["left"]) / 2,
        arriving["top"] + 0.28,
        r"$|x_{\mathrm{goal}}-x| < 45\,\mathrm{m}$",
    )

    _arrow(ax, (align["right"], align["cy"]), (merge["left"], merge["cy"]))
    _label(
        ax,
        (align["right"] + merge["left"]) / 2,
        align["top"] + 0.38,
        "\n".join(
            [
                r"$|x_{\mathrm{goal}}-x| < 36\,\mathrm{m}$",
                r"$|\delta| < 0.08\,\mathrm{rad}$",
                r"$|\dot{\psi}| < 0.06\,\mathrm{rad/s}$",
            ]
        ),
        fontsize=8.1,
    )

    # Turn down into the final stopping sequence.
    _arrow(ax, (merge["cx"], merge["bottom"]), (final["cx"], final["top"]))
    _label(
        ax,
        merge["cx"] + 0.18,
        (merge["bottom"] + final["top"]) / 2,
        r"$|x_{\mathrm{goal}}-x| < 8\,\mathrm{m}$",
        ha="left",
    )

    _arrow(ax, (final["left"], final["cy"]), (stopping["right"], stopping["cy"]))
    _label(
        ax,
        (final["left"] + stopping["right"]) / 2,
        3.05,
        "\n".join(
            [
                r"$[|x_{\mathrm{goal}}-x|<3\,\mathrm{m}$",
                r"$\ \vee\ (x>x_{\mathrm{goal}}\wedge v<0.8)]$",
                r"$v\leq0.8,\ |\delta|\leq0.12\,\mathrm{rad}$",
                r"$|y-y_{\mathrm{goal}}|\leq1.2\,\mathrm{m}$",
            ]
        ),
        fontsize=7.9,
    )

    _arrow(ax, (stopping["left"], stopping["cy"]), (service["right"], service["cy"]))
    _label(
        ax,
        (stopping["left"] + service["right"]) / 2,
        4.12,
        "inside goal rectangle\n" + r"$v < 10^{-5}\,\mathrm{m/s}$",
        fontsize=8.2,
    )

    _arrow(ax, (service["left"], service["cy"]), (departing["right"], departing["cy"]))
    _label(
        ax,
        -0.35,
        3.62,
        r"$\mathrm{stopping\ counter}>15\ \wedge$"
        "\n"
        r"$\mathrm{door\_closed}=\mathrm{True}$",
        fontsize=7.7,
    )

    # Departure returns to HEADING after the bus has merged into the main lane.
    _arrow(
        ax,
        (departing["cx"], departing["top"]),
        (heading["left"], heading["bottom"]),
        connectionstyle="angle3,angleA=90,angleB=0",
    )
    _label(
        ax,
        -2.15,
        (departing["top"] + heading["bottom"]) / 2 + 0.20,
        "\n".join(
            [
                r"$v\geq2.5\,\mathrm{m/s}$",
                r"$|\psi|<0.02\,\mathrm{rad},\ |a|<0.2\,\mathrm{m/s^2}$",
                r"offset to lanelet 2 $\leq0.6\,\mathrm{m}$",
            ]
        ),
        fontsize=7.9,
        ha="center",
    )

    # Mission completion is checked only after the stop service and departure.
    terminal_x, terminal_y = -3.55, 5.62
    ax.add_patch(
        FancyBboxPatch(
            (terminal_x, terminal_y),
            1.8,
            0.82,
            boxstyle="round,pad=0.03,rounding_size=0.15",
            facecolor="#f2f2f2",
            edgecolor="black",
            linewidth=1.35,
        )
    )
    ax.text(
        terminal_x + 0.9,
        terminal_y + 0.41,
        "TERMINAL",
        ha="center",
        va="center",
        fontsize=10.5,
        fontweight="bold",
    )
    _arrow(
        ax,
        (heading["left"], heading["cy"]),
        (terminal_x + 1.8, terminal_y + 0.41),
    )
    _label(
        ax,
        -1.72,
        heading["top"] + 0.28,
        r"$\mathrm{final\_stop}=\mathrm{True}$",
        fontsize=8.1,
    )

    # Auxiliary behavior: this does not change the active state.
    ax.text(
        8.1,
        0.32,
        "Dashed state = implicit service phase. "
        "If all trajectory-sampling attempts fail, controlled braking brings the "
        "vehicle to standstill and planning resumes in the same active state.",
        ha="center",
        va="center",
        fontsize=8.5,
        style="italic",
        color="#333333",
    )

    ax.set_title(
        "State Machine for Bus-Stop-Bay Behavior Planning",
        fontsize=15,
        fontweight="bold",
        pad=12,
    )

    output_dir = Path(output_dir) if output_dir is not None else Path(__file__).resolve().parent
    output_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = output_dir / "bus_stop_bay_state_machine.pdf"
    png_path = output_dir / "bus_stop_bay_state_machine.png"
    fig.savefig(pdf_path, dpi=300, bbox_inches="tight", pad_inches=0.08)
    fig.savefig(png_path, dpi=300, bbox_inches="tight", pad_inches=0.08)

    if show:
        plt.show()
    else:
        plt.close(fig)

    print(f"Saved: {pdf_path}")
    print(f"Saved: {png_path}")
    return pdf_path, png_path


if __name__ == "__main__":
    draw_state_diagram()
