# OBACHT Planner

OBACHT Planner is a hybrid motion-planning and control system for autonomous
public-transport vehicles. The current runners combine a CommonRoad reactive
planner, a state-machine controller, SUMO traffic simulation, and Spatiotemporal
Logic (STL) verification. Legacy post-optimization modules remain in the
repository, but post-optimization is disabled in the current runner workflow.

The repository includes two complete bus-stop scenarios:

- `bus_stop_bay`: a recessed bus bay
- `bus_stop_bulb`: a curb-extension bus bulb

## Project structure

| Path | Purpose |
| --- | --- |
| `runner/` | Main entry points for the bus-bay and bus-bulb simulations |
| `configurations/` | Global scenario selection and scenario-specific planner states |
| `scenarios/` | CommonRoad and SUMO scenario files |
| `post_optimization_planner/` | Active state machine and legacy post-optimization logic |
| `source/` | Reactive planner, simulation, and STL-monitoring modules |
| `experiments/` | STL notebooks and generated experiment output |
| `utility/` | Plotting, conversion, comparison, and analysis scripts |

## Installation

The supported environment uses Python 3.11. Create it from the repository root:

```bash
conda env create -f environment.yaml
conda activate obacht-planner
```

The environment installs the CommonRoad and SUMO packages used by the
simulation, as well as the notebook and development tools. Confirm that SUMO is
available before starting a run:

```bash
sumo --version
```

Optional: enable the repository's formatting hooks:

```bash
pre-commit install
```

## Run the simulations

Run commands from the repository root so that project-relative imports and
configuration paths resolve consistently.

### Bus-stop bay

Set the scenario in `configurations/scenario.yaml`:

```yaml
scenario:
  type: bus_stop_bay
```

Then run:

```bash
python runner/bus_stop_bay_opt.py
```

### Bus-stop bulb

Run:

```bash
python runner/bus_stop_bulb_opt.py
```

The bulb runner uses `bus_stop_bulb` for its run without permanently changing
`configurations/scenario.yaml`.

Both runners disable planner multiprocessing so projection-domain errors remain
visible in the main process. They execute the SUMO simulation, export the driven
trajectory, generate a velocity profile and GIF, print diagnostic summaries,
and save a debug visualization.

Generated files are written to:

- `experiments/output_result/result_bay/`
- `experiments/output_result/result_bulb/`

These output directories are intentionally excluded from Git.

## STL verification

Start JupyterLab and open the RB1/RB2 experiment notebook:

```bash
jupyter lab experiments/scenario_with_RB1_RB2.ipynb
```

The notebook evaluates trajectories against the STL specifications implemented
in `source/crmonitor/`.

## Additional utilities

For example, regenerate a velocity profile from an exported trajectory:

```bash
python utility/plot_velocity_profile.py \
  --input experiments/output_result/result_bay/bus_stop_bay_opt_trajectory.csv \
  --output experiments/output_result/result_bay/bus_stop_bay_opt_velocity_profile.png
```

Run `python utility/plot_velocity_profile.py --help` for all available options.
