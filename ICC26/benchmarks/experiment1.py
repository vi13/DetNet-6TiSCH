import pickle
from pathlib import Path

from ICC26.benchmarks.solvers import Solver
from ICC26.environment.flow_generator import FlowGenerator
from ICC26.environment.topology_generator import TopologyGenerator


# Topology settings
DIM_X = 1000
DIM_Y = 1000
NODES = 50
MIN_PDR = 0.9
STABLE_LINKS = 2
DISTANCE = 100

# Flow settings
UTILIZATION = 100
HYPER_PERIOD = 100

ITERATIONS = [0, 60]

TIME_LIMIT = 1200
SOLVER = [Solver.separate_energy,
          Solver.separate_tx,
          Solver.jras]

FILE_NAME = 'Experiment1'


def benchmark():
    # Create directory for logging
    path_scenario = f"data/{FILE_NAME}_Scenarios"
    Path(path_scenario).mkdir(parents=True, exist_ok=True)
    path_results = f"data/{FILE_NAME}_Results"
    Path(path_results).mkdir(parents=True, exist_ok=True)

    for i in range(ITERATIONS[0], ITERATIONS[1]):
        print(f"Round {i} starts")
        file_name = f"Iter{i}"
        G, I = TopologyGenerator.random_mesh_pisterhack(DIM_X, DIM_Y, NODES, MIN_PDR, STABLE_LINKS, DISTANCE)
        F, sink = FlowGenerator.random_traffic(G, UTILIZATION)

        runs = []
        for solver in SOLVER:
            (solution1, solution2), (info1, info2), (runtime1, runtime2), progress = solver(G, F, HYPER_PERIOD, sink, TIME_LIMIT)

            runs.append({
                'Solution_Routing': solution1,
                'Solution_Scheduling': solution2,
                'Info_Routing': info1,
                'Info_Scheduling': info2,
                'Runtime_Routing': runtime1,
                'Runtime_Scheduling': runtime2,
                'Progress': progress
            })

        scenario_to_disk = {
            'G': G,
            'I': I,
            'F': F,
            'Sink': sink,
            'Hyper-Period': HYPER_PERIOD
        }

        with open(f"{path_scenario}/{file_name}.pkl", 'wb') as outp:
            pickle.dump(scenario_to_disk, outp, pickle.HIGHEST_PROTOCOL)
        # ... and run results
        with open(f"{path_results}/{file_name}.pkl", 'wb') as outp:
            pickle.dump(runs, outp, pickle.HIGHEST_PROTOCOL)


if __name__ == "__main__":
    benchmark()
