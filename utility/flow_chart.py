"""Draw state-machine diagrams for the bus-stop planners.

State topology comes from ``post_optimization_planner/State.py`` and
``post_optimization_planner/state_machine.py``. Numeric planner targets and
distance guards are read from the scenario YAML files at render time.
"""

import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import yaml
from matplotlib.patches import FancyBboxPatch

PATH_ROOT = Path(__file__).resolve().parent.parent
CONFIG_ROOT = PATH_ROOT / "configurations"


plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 11,
    }
)


BOX_W = 3.75
BOX_H = 1.75
TOP_Y = 5.15
BOTTOM_Y = 1.55
BASE_FONT_SIZE = 11
TITLE_FONT_SIZE = 13
GROUP_TITLE_FONT_SIZE = 12


def _load_config(scenario_type: str, filename: str) -> dict[str, Any]:
    with open(CONFIG_ROOT / scenario_type / filename, "r") as fh:
        return yaml.safe_load(fh) or {}


def _cfg(config: dict[str, Any], *keys: str, default: Any = None) -> Any:
    value: Any = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _num(value: Any) -> str:
    try:
        numeric = float(value)
        if numeric != 0.0 and abs(numeric) < 1e-3:
            exponent = int(math.floor(math.log10(abs(numeric))))
            mantissa = numeric / (10**exponent)
            if math.isclose(abs(mantissa), 1.0, rel_tol=1e-9, abs_tol=1e-12):
                sign = "-" if numeric < 0 else ""
                return rf"{sign}1\times10^{{{exponent}}}"
            return rf"{mantissa:g}\times10^{{{exponent}}}"
        return f"{numeric:g}"
    except (TypeError, ValueError):
        return str(value)


def _velocity_line(config: dict[str, Any], default: Any = 0.0) -> str:
    value = _cfg(config, "sampling", "desire_velocity", default=default)
    v_min = _cfg(config, "sampling", "v_min", default=None)
    v_max = _cfg(config, "sampling", "v_max", default=None)
    if v_min is not None and v_max is not None:
        return rf"$v_{{\mathrm{{des}}}} \in [{_num(v_min)},\,{_num(v_max)}]\,\mathrm{{m/s}}$"
    return rf"$v_{{\mathrm{{des}}}} = {_num(value)}\,\mathrm{{m/s}}$"


def _d_line(config: dict[str, Any]) -> str:
    d_min = _cfg(config, "sampling", "d_min", default=None)
    d_max = _cfg(config, "sampling", "d_max", default=None)
    if d_min is None or d_max is None:
        return r"$d$: not constrained in YAML"
    return rf"$d \in [{_num(d_min)},\,{_num(d_max)}]\,\mathrm{{m}}$"


def _s_line(config: dict[str, Any]) -> str:
    s_min = _cfg(config, "sampling", "s_min", default=None)
    s_max = _cfg(config, "sampling", "s_max", default=None)
    if s_min is None or s_max is None:
        return "target: goal-region center"
    return (
        rf"$s-s_{{\mathrm{{goal}}}}\in [{_num(s_min)},\,{_num(s_max)}]\,\mathrm{{m}}$"
    )


def _box_geometry(name: str, state_x: dict[str, Any]) -> dict[str, float]:
    position = state_x[name]
    if isinstance(position, tuple):
        x, y = position
    else:
        x = position
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


def _draw_state(
    ax, state_x, name, title, lines, *, implicit=False, fontsize=BASE_FONT_SIZE
):
    box = _box_geometry(name, state_x)
    title_size = (
        fontsize - 1.0
        if len(title) > 20
        else fontsize - 0.5 if len(title) > 14 else fontsize
    )
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
        fontfamily="DejaVu Sans",
        zorder=3,
    )
    ax.text(
        box["left"] + 0.18,
        box["top"] - 0.55,
        "\n".join(lines),
        ha="left",
        va="top",
        fontsize=fontsize,
        linespacing=1.25,
        zorder=3,
    )
    return box


def _draw_safety_state(ax, x, y, *, fontsize=BASE_FONT_SIZE):
    safety_color = "#d00000"
    width = 3.95
    height = 1.92
    box = {
        "x": x,
        "y": y,
        "left": x,
        "right": x + width,
        "bottom": y,
        "top": y + height,
        "cx": x + width / 2,
        "cy": y + height / 2,
    }
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.025,rounding_size=0.04",
        facecolor="#fff7f3",
        edgecolor=safety_color,
        linewidth=1.55,
        linestyle="--",
        zorder=2,
    )
    ax.add_patch(patch)
    ax.text(
        box["cx"],
        box["top"] - 0.17,
        "EMERGENCY_BRAKE",
        ha="center",
        va="top",
        fontsize=fontsize,
        fontweight="bold",
        color=safety_color,
        fontfamily="DejaVu Sans",
        zorder=3,
    )
    ax.text(
        box["left"] + 0.18,
        box["top"] - 0.55,
        "\n".join(
            [
                "controlled braking",
                "fallback",
                r"$a_{\mathrm{brake}}=-\min(2.0,a_{\max})$",
                "brake to standstill",
            ]
        ),
        ha="left",
        va="top",
        fontsize=fontsize,
        linespacing=1.25,
        zorder=3,
    )
    return box


def _draw_normal_group(ax, x, y, width, height, *, fontsize=GROUP_TITLE_FONT_SIZE):
    box = {
        "x": x,
        "y": y,
        "left": x,
        "right": x + width,
        "bottom": y,
        "top": y + height,
        "cx": x + width / 2,
        "cy": y + height / 2,
    }
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            width,
            height,
            boxstyle="round,pad=0.04,rounding_size=0.07",
            facecolor="none",
            edgecolor="black",
            linewidth=1.55,
            zorder=-2,
        )
    )
    ax.text(
        box["left"] + 0.34,
        box["top"] + 0.08,
        "NORMAL PLANNING STATES",
        ha="left",
        va="bottom",
        fontsize=fontsize,
        fontweight="bold",
        color="white",
        fontfamily="DejaVu Sans",
        bbox={
            "boxstyle": "round,pad=0.32,rounding_size=0.12",
            "facecolor": "black",
            "edgecolor": "black",
        },
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


def _polyline_arrow(
    ax,
    points,
    *,
    color="black",
    linestyle="-",
    linewidth=1.35,
    zorder=1,
):
    if len(points) < 2:
        return

    if len(points) > 2:
        xs = [p[0] for p in points[:-1]]
        ys = [p[1] for p in points[:-1]]
        ax.plot(
            xs,
            ys,
            color=color,
            linestyle=linestyle,
            linewidth=linewidth,
            zorder=zorder,
        )

    ax.annotate(
        "",
        xy=points[-1],
        xytext=points[-2],
        arrowprops={
            "arrowstyle": "->",
            "linewidth": linewidth,
            "color": color,
            "linestyle": linestyle,
            "connectionstyle": "arc3",
            "shrinkA": 1,
            "shrinkB": 1,
        },
        zorder=zorder,
    )


def _safety_arrow(ax, start, end, *, connectionstyle="arc3"):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops={
            "arrowstyle": "->",
            "linewidth": 1.25,
            "color": "#9f1d20",
            "linestyle": "--",
            "connectionstyle": connectionstyle,
            "shrinkA": 1,
            "shrinkB": 1,
        },
        zorder=1,
    )


def _label(ax, x, y, text, *, fontsize=BASE_FONT_SIZE, ha="center", va="center"):
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


def _draw_emergency_brake_overlay(
    ax,
    emergency_box,
    normal_box,
    *,
    compact=False,
    fontsize=BASE_FONT_SIZE,
):
    entry_start = (normal_box["right"], normal_box["top"] - 0.98)
    entry_top_x = emergency_box["cx"]
    entry_label_x = 0.5 * (entry_start[0] + entry_top_x) + 0.42
    entry_label_y = entry_start[1] + 0.32
    _polyline_arrow(
        ax,
        [
            entry_start,
            (entry_top_x, entry_start[1]),
            (entry_top_x, emergency_box["top"]),
        ],
        color="#9f1d20",
        linestyle="--",
        linewidth=1.25,
    )
    _label(
        ax,
        entry_label_x,
        entry_label_y,
        (
            r"$\mathrm{planning\_failed}\ \vee\ \mathrm{collision\_risk}$"
            if compact
            else r"$\mathrm{trajectory\_planning\_failed}\ \vee\ \mathrm{collision\_risk}$"
        ),
        fontsize=fontsize,
    )

    recovery_end = (normal_box["right"], normal_box["bottom"] + 0.02)
    recovery_label_x = 0.5 * (emergency_box["cx"] + recovery_end[0])
    recovery_label_y = recovery_end[1] - 0.36
    _polyline_arrow(
        ax,
        [
            (emergency_box["cx"], emergency_box["bottom"]),
            (emergency_box["cx"], recovery_end[1]),
            recovery_end,
        ],
        color="#9f1d20",
        linestyle="--",
        linewidth=1.25,
    )
    _label(
        ax,
        recovery_label_x,
        recovery_label_y,
        (
            "\n".join(
                [
                    r"$|v|,|a|,|\delta|\leq10^{-3}$",
                    r"$\mathrm{risk\ cleared}$",
                    r"$\mathrm{restart\ from}\ \mathrm{ego\_state}_{\mathrm{stop}}$",
                ]
            )
            if compact
            else "\n".join(
                [
                    r"$|v|,|a|,|\delta|\leq10^{-3}\ \wedge\ \neg\mathrm{imminent\_collision\_risk}$",
                    r"$x_0 \leftarrow \mathrm{ego\_state}_{\mathrm{stop}}$",
                    r"$\mathrm{state}\leftarrow\mathrm{select\_normal\_state}(\mathrm{ego\_state}_{\mathrm{stop}})$",
                ]
            )
        ),
        fontsize=fontsize,
        va="top",
    )


def _save(fig, output_dir, stem: str, show: bool):
    output_dir = (
        Path(output_dir) if output_dir is not None else Path(__file__).resolve().parent
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = output_dir / f"{stem}.pdf"
    png_path = output_dir / f"{stem}.png"
    fig.savefig(pdf_path, dpi=300, bbox_inches="tight", pad_inches=0.08)
    fig.savefig(png_path, dpi=300, bbox_inches="tight", pad_inches=0.08)
    if show:
        plt.show()
    else:
        plt.close(fig)
    print(f"Saved: {pdf_path}")
    print(f"Saved: {png_path}")
    return pdf_path, png_path


def draw_bus_stop_bay_state_diagram(output_dir=None, show=False):
    """Generate PDF and PNG versions of the bus-stop-bay state machine."""
    scenario = "bus_stop_bay"
    bay_font_size = BASE_FONT_SIZE - 1
    cfg_heading = _load_config(scenario, "heading_to_next.yaml")
    cfg_arriving = _load_config(scenario, "arriving.yaml")
    cfg_align = _load_config(scenario, "before_stopping_align.yaml")
    cfg_merge = _load_config(scenario, "before_stopping_merge.yaml")
    cfg_final = _load_config(scenario, "before_stopping_final.yaml")
    cfg_departing = _load_config(scenario, "departure.yaml")

    state_x = {
        "DEPARTING": (-4.15, 1.18),
        "HEADING": (0.00, 4.72),
        "ARRIVING": (4.60, 4.72),
        "MERGE": (9.20, 4.72),
        "ALIGN": (13.80, 4.72),
        "SERVICE": (0.00, 1.18),
        "STOPPING": (4.60, 1.18),
        "FINAL": (13.80, 1.18),
    }

    fig, ax = plt.subplots(figsize=(19.0, 8.0))
    ax.set_xlim(-4.95, 22.70)
    ax.set_ylim(-0.50, 7.55)
    ax.axis("off")

    normal_group = _draw_normal_group(
        ax,
        -4.65,
        0.82,
        22.55,
        6.28,
        fontsize=GROUP_TITLE_FONT_SIZE - 1,
    )

    composite_x = 8.85
    composite_y = 0.98
    composite_w = 8.85
    composite_h = 6.02
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
        composite_x + 0.32,
        composite_y + composite_h + 0.06,
        "BEFORE STOPPING",
        ha="left",
        va="bottom",
        fontsize=GROUP_TITLE_FONT_SIZE - 1,
        fontweight="bold",
        fontfamily="DejaVu Sans",
        bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.0},
        zorder=3,
    )

    heading = _draw_state(
        ax,
        state_x,
        "HEADING",
        "HEADING TO NEXT",
        [_velocity_line(cfg_heading), _d_line(cfg_heading), "reference: lanelet 2"],
        fontsize=bay_font_size,
    )
    arriving = _draw_state(
        ax,
        state_x,
        "ARRIVING",
        "ARRIVING",
        [_velocity_line(cfg_arriving), _d_line(cfg_arriving), "reference: lanelet 1"],
        fontsize=bay_font_size,
    )
    merge = _draw_state(
        ax,
        state_x,
        "MERGE",
        "MERGE",
        [_velocity_line(cfg_merge), _d_line(cfg_merge), "curved entry path"],
        fontsize=bay_font_size,
    )
    align = _draw_state(
        ax,
        state_x,
        "ALIGN",
        "ALIGNMENT",
        [_velocity_line(cfg_align), _d_line(cfg_align), "blend to stop line"],
        fontsize=bay_font_size,
    )
    final = _draw_state(
        ax,
        state_x,
        "FINAL",
        "FINAL",
        [
            r"$v_{\mathrm{des}}=0\,\mathrm{m/s}$",
            _d_line(cfg_final),
            _s_line(cfg_final),
            "target: stop line",
        ],
        fontsize=bay_font_size,
    )
    stopping = _draw_state(
        ax,
        state_x,
        "STOPPING",
        "STOPPING",
        [
            r"$v_{\mathrm{des}}=0\,\mathrm{m/s}$",
            "monotonic braking",
            "no position target",
        ],
        fontsize=bay_font_size,
    )
    service = _draw_state(
        ax,
        state_x,
        "SERVICE",
        "BOARDING / ALIGHTING",
        ["stopped-state hold", r"$v = 0$", r"hold while counter $\leq 10$"],
        implicit=True,
        fontsize=bay_font_size,
    )
    departing = _draw_state(
        ax,
        state_x,
        "DEPARTING",
        "DEPARTING",
        [
            _velocity_line(cfg_departing),
            r"$d\in[-0.75,\,0.75]\,\mathrm{m}$",
            "merge to lanelet 2",
        ],
        fontsize=bay_font_size,
    )
    emergency = _draw_safety_state(ax, 18.30, 3.05, fontsize=bay_font_size)

    heading_to_arriving = _cfg(
        cfg_heading, "planning", "distance_heading_to_next_to_arriving"
    )
    arriving_to_merge = _cfg(
        cfg_arriving, "planning", "distance_arriving_to_next_to_before_stopping"
    )
    merge_to_align = _cfg(cfg_merge, "planning", "distance_arriving_to_stopping")
    align_to_final = _cfg(
        cfg_align, "planning", "distance_arriving_to_next_to_before_stopping"
    )
    final_to_stopping = _cfg(
        cfg_final, "planning", "distance_before_stopping_to_stopping"
    )
    departing_v = _cfg(cfg_departing, "sampling", "desire_velocity")

    _arrow(ax, (heading["right"], heading["cy"]), (arriving["left"], arriving["cy"]))
    _label(
        ax,
        (heading["right"] + arriving["left"]) / 2,
        heading["top"] + 0.25,
        rf"$0<x_{{\mathrm{{goal}}}}-x<{_num(heading_to_arriving)}\,\mathrm{{m}}$",
        fontsize=bay_font_size,
    )

    _arrow(ax, (arriving["right"], arriving["cy"]), (merge["left"], merge["cy"]))
    _label(
        ax,
        (arriving["right"] + merge["left"]) / 2,
        arriving["top"] + 0.25,
        rf"$|x_{{\mathrm{{goal}}}}-x|<{_num(arriving_to_merge)}\,\mathrm{{m}}$",
        fontsize=bay_font_size,
    )

    _arrow(ax, (merge["right"], merge["cy"]), (align["left"], align["cy"]))
    _label(
        ax,
        (merge["right"] + align["left"]) / 2,
        merge["top"] + 0.25,
        rf"$|x_{{\mathrm{{goal}}}}-x|<{_num(merge_to_align)}\,\mathrm{{m}}$",
        fontsize=bay_font_size,
    )

    _arrow(ax, (align["cx"], align["bottom"]), (final["cx"], final["top"]))
    _label(
        ax,
        align["cx"] + 0.18,
        (align["bottom"] + final["top"]) / 2,
        rf"$|x_{{\mathrm{{goal}}}}-x|<{_num(align_to_final)}\,\mathrm{{m}}$",
        fontsize=bay_font_size,
        ha="left",
    )

    _arrow(ax, (final["left"], final["cy"]), (stopping["right"], stopping["cy"]))
    _label(
        ax,
        (final["left"] + stopping["right"]) / 2,
        final["cy"] + 0.60,
        "\n".join(
            [
                rf"transition gate: $-{_num(final_to_stopping)}"
                r"<x-x_{\mathrm{goal}}\leq1.5\,\mathrm{m}$",
                r"$v\leq0.55\,\mathrm{m/s},\ |\delta|\leq0.12\,\mathrm{rad}$",
                r"$|y-y_{\mathrm{goal}}|\leq1.2\,\mathrm{m}$",
            ]
        ),
        fontsize=bay_font_size,
    )

    _arrow(ax, (stopping["left"], stopping["cy"]), (service["right"], service["cy"]))
    _label(
        ax,
        (stopping["left"] + service["right"]) / 2,
        3.52,
        "inside goal area\n" + r"$v<10^{-5}\,\mathrm{m/s}$",
        fontsize=bay_font_size,
    )

    _arrow(ax, (service["left"], service["cy"]), (departing["right"], departing["cy"]))
    _label(
        ax,
        (service["left"] + departing["right"]) / 2,
        3.38,
        r"$\mathrm{stopping\ counter}>10$",
        fontsize=bay_font_size,
    )

    _arrow(
        ax,
        (departing["cx"], departing["top"]),
        (heading["left"], heading["bottom"]),
        connectionstyle="angle3,angleA=90,angleB=0",
    )
    _label(
        ax,
        -2.10,
        (departing["top"] + heading["bottom"]) / 2,
        "\n".join(
            [
                rf"$v\geq{_num(departing_v)}\,\mathrm{{m/s}}$",
                r"$|\psi|<0.02\,\mathrm{rad},\ |a|<0.2\,\mathrm{m/s^2}$",
                r"lane offset $\leq0.6\,\mathrm{m}$",
            ]
        ),
        fontsize=bay_font_size,
    )

    terminal_x, terminal_y = -4.30, 5.00
    terminal_w, terminal_h = 2.15, 1.18
    ax.add_patch(
        FancyBboxPatch(
            (terminal_x, terminal_y),
            terminal_w,
            terminal_h,
            boxstyle="round,pad=0.03,rounding_size=0.10",
            facecolor="#f2f2f2",
            edgecolor="black",
            linewidth=1.35,
        )
    )
    ax.text(
        terminal_x + terminal_w / 2,
        terminal_y + terminal_h / 2,
        "TERMINAL",
        ha="center",
        va="center",
        fontsize=bay_font_size,
        fontweight="bold",
        fontfamily="DejaVu Sans",
    )
    _arrow(
        ax,
        (heading["left"], heading["cy"]),
        (terminal_x + terminal_w, terminal_y + terminal_h / 2),
    )
    _label(
        ax,
        -1.10,
        heading["cy"] + 0.34,
        r"$\mathrm{final\_stop}=\mathrm{True}$",
        fontsize=bay_font_size,
    )

    _draw_emergency_brake_overlay(
        ax,
        emergency,
        normal_group,
        compact=True,
        fontsize=bay_font_size,
    )

    ax.set_title(
        "Bus-Stop-Bay State Machine",
        fontsize=TITLE_FONT_SIZE,
        fontweight="bold",
        pad=12,
    )
    return _save(fig, output_dir, "bus_stop_bay_state_machine", show)


def draw_bus_stop_bulb_state_diagram(output_dir=None, show=False):
    """Generate PDF and PNG versions of the bus-stop-bulb state machine."""
    scenario = "bus_stop_bulb"
    cfg_heading = _load_config(scenario, "heading_to_next.yaml")
    cfg_arriving = _load_config(scenario, "arriving.yaml")
    cfg_stopping = _load_config(scenario, "stopping.yaml")
    cfg_departing = _load_config(scenario, "departure.yaml")

    state_x = {
        "DEPARTING": (-4.25, 1.42),
        "HEADING": (0.0, 4.90),
        "ARRIVING": (5.95, 4.90),
        "STOPPING": (9.10, 1.42),
        "SERVICE": (2.15, 1.42),
    }

    fig, ax = plt.subplots(figsize=(17.3, 7.5))
    ax.set_xlim(-4.95, 19.00)
    ax.set_ylim(-0.48, 7.15)
    ax.axis("off")

    normal_group = _draw_normal_group(ax, -4.65, 1.04, 17.60, 5.82)

    heading = _draw_state(
        ax,
        state_x,
        "HEADING",
        "HEADING TO NEXT",
        [
            _velocity_line(cfg_heading),
            _d_line(cfg_heading),
            "reference path: lanelet 1",
        ],
    )
    arriving = _draw_state(
        ax,
        state_x,
        "ARRIVING",
        "ARRIVING",
        [
            _velocity_line(cfg_arriving),
            _d_line(cfg_arriving),
            "reference path: lanelet 2",
        ],
    )
    stopping = _draw_state(
        ax,
        state_x,
        "STOPPING",
        "STOPPING",
        [_velocity_line(cfg_stopping), _d_line(cfg_stopping), _s_line(cfg_stopping)],
    )
    service = _draw_state(
        ax,
        state_x,
        "SERVICE",
        "BOARDING / ALIGHTING",
        ["stopped-state hold", r"$v = 0$", r"hold while counter $\leq 10$"],
        implicit=True,
    )
    departing = _draw_state(
        ax,
        state_x,
        "DEPARTING",
        "DEPARTING",
        [
            r"$v_{\mathrm{des}}=2.55\,\mathrm{m/s}$",
            r"$d\in[-0.45,\,0.45]\,\mathrm{m}$",
            "smooth merge to lanelet 1",
        ],
    )
    emergency = _draw_safety_state(ax, 13.60, 3.15)

    heading_to_arriving = _cfg(
        cfg_heading, "planning", "distance_heading_to_next_to_arriving"
    )
    arriving_to_stopping = _cfg(
        cfg_arriving, "planning", "distance_arriving_to_stopping"
    )
    departing_v = min(
        3.5,
        float(_cfg(cfg_departing, "sampling", "v_max", default=3.5)),
    )

    _arrow(ax, (heading["right"], heading["cy"]), (arriving["left"], arriving["cy"]))
    _label(
        ax,
        (heading["right"] + arriving["left"]) / 2,
        heading["cy"] + 0.30,
        rf"$0 < x_{{\mathrm{{goal}}}}-x < {_num(heading_to_arriving)}\,\mathrm{{m}}$",
    )

    _polyline_arrow(
        ax,
        [
            (arriving["right"], arriving["cy"]),
            (stopping["cx"], arriving["cy"]),
            (stopping["cx"], stopping["top"]),
        ],
    )
    _label(
        ax,
        stopping["cx"] - 0.78,
        4.08,
        rf"$|x_{{\mathrm{{goal}}}}-x| < {_num(arriving_to_stopping)}\,\mathrm{{m}}$",
        ha="right",
    )

    _arrow(ax, (stopping["left"], stopping["cy"]), (service["right"], service["cy"]))
    _label(
        ax,
        (stopping["left"] + service["right"]) / 2,
        stopping["cy"] + 0.42,
        r"$x \leq x_{\mathrm{goal}}+\frac{L_{\mathrm{goal}}}{2}$"
        "\n"
        r"$\wedge\ v < 10^{-5}\,\mathrm{m/s}$",
    )

    _arrow(ax, (service["left"], service["cy"]), (departing["right"], departing["cy"]))
    _label(
        ax,
        (service["left"] + departing["right"]) / 2,
        service["cy"] + 0.35,
        r"$\mathrm{stopping\ counter}>10$",
    )

    _arrow(
        ax,
        (departing["cx"], departing["top"]),
        (heading["left"], heading["bottom"]),
        connectionstyle="angle3,angleA=90,angleB=0",
    )
    _label(
        ax,
        -2.20,
        (departing["top"] + heading["bottom"]) / 2 - 0.10,
        "\n".join(
            [
                rf"$v\geq{_num(departing_v)}\,\mathrm{{m/s}}$",
                r"$|\psi|<0.02\,\mathrm{rad}$",
                r"$|a|<0.2\,\mathrm{m/s^2}$",
                r"$|d_{\mathrm{lanelet\ 1}}|\leq0.6\,\mathrm{m}$",
            ]
        ),
    )

    terminal_x, terminal_y = -4.28, 5.12
    terminal_w, terminal_h = 2.35, 1.25
    ax.add_patch(
        FancyBboxPatch(
            (terminal_x, terminal_y),
            terminal_w,
            terminal_h,
            boxstyle="round,pad=0.03,rounding_size=0.08",
            facecolor="#f2f2f2",
            edgecolor="black",
            linewidth=1.35,
        )
    )
    ax.text(
        terminal_x + terminal_w / 2,
        terminal_y + terminal_h / 2,
        "TERMINAL",
        ha="center",
        va="center",
        fontsize=BASE_FONT_SIZE,
        fontweight="bold",
        fontfamily="DejaVu Sans",
    )
    _arrow(
        ax,
        (heading["left"], heading["cy"]),
        (terminal_x + terminal_w, terminal_y + terminal_h / 2),
    )
    _label(
        ax,
        -0.98,
        heading["cy"] + 0.32,
        r"$\mathrm{final\_stop}=\mathrm{True}$",
        fontsize=BASE_FONT_SIZE - 1,
    )

    _draw_emergency_brake_overlay(
        ax,
        emergency,
        normal_group,
    )

    ax.set_title(
        "Bus-Stop-Bulb State Machine",
        fontsize=TITLE_FONT_SIZE,
        fontweight="bold",
        pad=12,
    )
    return _save(fig, output_dir, "bus_stop_bulb_state_machine", show)


def draw_state_diagram(output_dir=None, show=False):
    """Backward-compatible alias for the bay diagram."""
    return draw_bus_stop_bay_state_diagram(output_dir=output_dir, show=show)


def draw_all_state_diagrams(output_dir=None, show=False):
    bay_paths = draw_bus_stop_bay_state_diagram(output_dir=output_dir, show=show)
    bulb_paths = draw_bus_stop_bulb_state_diagram(output_dir=output_dir, show=show)
    return bay_paths, bulb_paths


if __name__ == "__main__":
    draw_all_state_diagrams()
