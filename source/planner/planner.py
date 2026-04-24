import logging
from typing import Optional
import yaml
from pathlib import Path
from commonroad.common.solution import (
    CommonRoadSolutionReader,
    CostFunction,
    VehicleModel,
    VehicleType,
    vehicle_parameters,
)
from commonroad.scenario.scenario import Scenario

from source.simulation.simulations import simulate_with_planner
from source.simulation.utility import save_solution

def motion_planner(scenario: Scenario, solution_dir: str):
    """
    Motion planner that takes a scenario and outputs a solution.

    Drivability Checker toolbox (`commonroad_dc`) can be used
    here to make sure the solution is feasible and does not
    collide with other scenario
    elements.
    """

    # Implement your own planner here

    # As a template, load already existing dummy solution and return
    solution_path = (
        "/commonroad/dummy_solutions/dummy_solution_KS2:SM1:CHN_Sha-4_1_T-1:2018b.xml"
    )
    return CommonRoadSolutionReader.open(solution_path)


def motion_planner_interactive(
    scenario_path: str, solution_dir: str, scenario_type: Optional[str] = None
):
    """
    Motion planner that takes the path of the folder with an interactive scenario folder and saves a solution.

    Drivability Checker toolbox (`commonroad_dc`) can be used here to make sure the solution is feasible and does not
    collide with other scenario elements.
    """
    # a sample code

    VALID_SCENARIOS = {"bus_stop_bay", "bus_stop_bulb"}

    print(f"Running interactive planner for scenario '{scenario_type}' ...")
    vehicle_parameters[VehicleType.FORD_ESCORT].l = 12.5
    vehicle_parameters[VehicleType.FORD_ESCORT].w = 2.55
    vehicle_type = VehicleType.FORD_ESCORT

    vehicle_model = VehicleModel.KS
    cost_function = CostFunction.WX1
    # cost_function = CostFunction.TR1
    # option 1: run your motion planner with closed-loop simulation
    # Implement your own planner in the simulate_scenario function within simulation/simulations.py.
    # You can Implement your own planner without saving videos

    if scenario_type in VALID_SCENARIOS:

        print(f"Using scenario: '{scenario_type}'")
        scenario_with_planner, pps, ego_vehicles_planner = simulate_with_planner(
            interactive_scenario_path=scenario_path
        )
        if scenario_with_planner:
            print("Saving solutions..")
            save_solution(
                scenario_with_planner,
                pps,
                ego_vehicles_planner,
                vehicle_type,
                vehicle_model,
                cost_function,
                solution_dir,
                overwrite=True,
            )
        yaml_path = Path(scenario_path).parent.parent / "configurations" / "scenario.yaml"
        with open(yaml_path, "r") as f:
            cfg = yaml.safe_load(f)


    else:
        logging.warning("No scenario specified—proceeding with an empty scenario name.")

    return None
