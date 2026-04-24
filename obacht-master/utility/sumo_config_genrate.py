import os
import pickle

from sumocr.sumo_config import DefaultConfig


def generate_simulation_config(
    interactive_folder: str, scenario_name: str, simulation_steps: int = 1000
):
    conf = DefaultConfig()
    conf.scenario_name = scenario_name
    conf.sumo_cfg_file = os.path.join(interactive_folder, f"{scenario_name}.sumo.cfg")
    conf.simulation_steps = simulation_steps

    out_path = os.path.join(interactive_folder, "simulation_config.p")
    with open(out_path, "wb") as f:
        pickle.dump(conf, f)
    print(f"Written simulation_config.p to {out_path}")


folder = "/home/guangyan/Downloads/repository/reactive-planner-development/scenarios/bus_stop_bulb"
name = "bus_stop_bulb"
steps = 550
generate_simulation_config(folder, name, steps)
