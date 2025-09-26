import collections
from typing import Dict, List, Tuple, Any, Optional
from ortools.sat.python import cp_model


Task = collections.namedtuple("Task", "start interval")


class CPLogger(cp_model.CpSolverSolutionCallback):
    """Log time/value of first and best solution found."""
    def __init__(self, has_objective: bool = True):
        super().__init__()
        self.has_objective = has_objective
        self.first_time: Optional[float] = None
        self.first_value: Optional[float] = None
        self.best_time: Optional[float] = None
        self.best_value: Optional[float] = None
        self.n_solutions: int = 0

    def on_solution_callback(self):
        t = self.WallTime()
        v = self.ObjectiveValue() if self.has_objective else None

        if self.first_time is None:
            self.first_time = t
            self.first_value = v

        # Each found solution will overwrite current best solution:
        self.best_time = t
        self.best_value = v
        self.n_solutions += 1


def solve_scheduling(
    G,
    F: List[Dict[str, Any]],
    solution_routing: List[List[Tuple[Any, Any]]],
    hyper_period: int,
    time_limit: int = None,
    workers: int = None,
    log_progress: bool = False
):

    model = cp_model.CpModel()

    n_flows = len(F)
    nodes = list(G.nodes)
    max_latency = max(f['latency'] for f in F)

    # Repeats for each flow in hyper-period, does not count number of packets per flow.
    repeats = [hyper_period // f['period'] for f in F]

    # Save variables for later use:
    all_flows: Dict[Tuple[int, int, int, int], Task] = {}
    node_to_intervals = collections.defaultdict(list)
    node_to_starts = collections.defaultdict(list)
    sample_vars = {}
    first_tx = {}
    last_tx = {}

    # The time horizon used for scheduling variables:
    horizon = hyper_period + max([f['latency'] for f in F])

    # Variablen & Constraints
    for c, path in enumerate(solution_routing):
        sample_vars[c] = model.NewIntVar(0, horizon, f"sample_{c}")

        for off in range(repeats[c]):
            base = off * F[c]['period']
            for e_idx, (u, v) in enumerate(path):
                for p in range(F[c]['packets']):
                    suf = f"_{c}_{off}_{u}_{v}_{p}"
                    start = model.NewIntVar(base, horizon, "start" + suf)
                    interval = model.NewFixedSizeIntervalVar(start, 1, "itv" + suf)
                    all_flows[(c, off, e_idx, p)] = Task(start=start, interval=interval)

                    node_to_intervals[u].append(interval)
                    node_to_intervals[v].append(interval)
                    node_to_starts[u].append(start)
                    node_to_starts[v].append(start)

                    # We save the first and the last transmission of a flow,
                    # so we can calculate the latency later on:
                    if e_idx == 0 and p == 0:
                        first_tx[(c, off)] = start
                    if e_idx == len(path) - 1 and p == F[c]['packets'] - 1:
                        last_tx[(c, off)] = start

    # Constraint 1: No-Overlap for intervals on each node
    for n in nodes:
        if node_to_intervals[n]:
            model.AddNoOverlap(node_to_intervals[n])

    # Constraint 2: The sampling period of each node is fixed,
    # and we need to wait for at least one time-period to send a new packet.
    for c in range(n_flows):
        for off in range(repeats[c]):
            model.Add(sample_vars[c] + off * F[c]['period'] <= first_tx[(c, off)])

    # Constraint 3: Precedence on path
    for c, path in enumerate(solution_routing):
        for off in range(repeats[c]):
            for e_idx in range(len(path) - 1):
                for p in range(F[c]['packets']):
                    cur = all_flows[(c, off, e_idx, p)]
                    nxt = all_flows[(c, off, e_idx + 1, p)]
                    model.Add(cur.interval.EndExpr() <= nxt.start)

    # Constraint 4: Packets are sent in correct order:
    for c, path in enumerate(solution_routing):
        for off in range(repeats[c]):
            for e_idx in range(len(path)):
                for p in range(F[c]['packets'] - 1):
                    cur = all_flows[(c, off, e_idx, p)]
                    nxt = all_flows[(c, off, e_idx, p + 1)]
                    model.Add(cur.interval.EndExpr() <= nxt.start)

    # Constraint 5: Check latency for each flow.
    # Note that the latency depends on the sample-time and not the send-time:
    for c in range(n_flows):
        for off in range(repeats[c]):
            model.Add(
                last_tx[(c, off)] - (sample_vars[c] + off * F[c]['period']) < F[c]['latency']
            )

    # Constraint 6: Phase-Shift/Capacity: (active time slots on node) < hyper_period
    # Note: Time-values for each node are relative, we only need to check if the interval
    # between the first and last transmission does not exceed the hyper-period.
    for n in nodes:
        if node_to_starts[n]:
            mx = model.NewIntVar(0, horizon, f"max_{n}")
            mn = model.NewIntVar(0, horizon, f"min_{n}")
            model.AddMaxEquality(mx, node_to_starts[n])
            model.AddMinEquality(mn, node_to_starts[n])
            model.Add(mx - mn < hyper_period)

    # Cost: Minimize the sum of all latencies per flow
    lat_terms = []
    for c in range(n_flows):
        for off in range(repeats[c]):
            lat = model.NewIntVar(0, horizon, f"lat_{c}_{off}")
            model.Add(lat == last_tx[(c, off)] - (sample_vars[c] + off * F[c]['period']))
            lat_terms.append(lat)
    model.Minimize(sum(lat_terms))

    # Solver + Logger
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit
    if workers is not None:
        solver.parameters.num_search_workers = int(workers)
    solver.parameters.log_search_progress = bool(log_progress)

    logger = CPLogger(has_objective=True)
    status = solver.Solve(model, logger)

    # Extract scheduling solution:
    solution_scheduling = []
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        for c, f in enumerate(F):
            for off in range(repeats[c]):
                for e_idx, (u, v) in enumerate(solution_routing[c]):
                    for p in range(f['packets']):
                        # A solution contains:
                        # (flow-id, offset, packet-id, source, destination, start-time)
                        start_val = solver.Value(all_flows[(c, off, e_idx, p)].start)
                        solution_scheduling.append((c, off, p, u, v, start_val))

    # Status-String
    if status == cp_model.OPTIMAL:
        info = 'optimal'
    elif status == cp_model.FEASIBLE:
        info = 'feasible'
    elif status == cp_model.INFEASIBLE:
        info = 'infeasible'
    else:
        info = 'timeout'

    # Progress/Observer-Infos
    best_bound = solver.BestObjectiveBound()
    best_val = logger.best_value
    gap = None
    if best_val is not None and best_val != 0:
        gap = abs(best_val - best_bound) / max(1e-12, abs(best_val))

    progress = {
        "first_solution_time": logger.first_time,
        "first_objective": logger.first_value,
        "best_time": logger.best_time,
        "best_objective": logger.best_value,
        "n_solutions": logger.n_solutions,
        "wall_time": solver.WallTime(),
        "best_bound": best_bound,
        "gap": (None if gap is None else 100.0 * gap),
    }

    return solution_scheduling, info, progress
