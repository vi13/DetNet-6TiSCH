import collections
import itertools
from ortools.sat.python import cp_model

from typing import Optional, Any, Dict, List, Tuple



# ---- Solution Observer ----
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


def solve_jras(
    G,
    F: List[Dict[str, Any]],
    hyper_period: int,
    sink: Any,
    bounds: Dict[str, int],
    slack_tx = 0,
    w_latency = 1,
    w_energy = 1,
    hint: Optional[List[List[Tuple[Any, Any]]]] = None,
    time_limit: int = None,
    workers: Optional[int] = None,
    log_progress: bool = False,
):
    """
    Joint Routing and Scheduling (JRaS) with CP-SAT.
    Minimizes sum of latencies with given bounds (NumTransmissions, EnergyMax).

    Returns:
      (solution_routing, solution_scheduling), info, progress
        - solution_routing: List[List[(u,v)]] for each flow
        - solution_scheduling: List[(flow, offset, packet, u, v, start)]
        - info: 'optimal'|'feasible'|'infeasible'|'timeout'
        - progress: dict(first_solution_time, first_objective, best_time, best_objective,
                         n_solutions, wall_time, best_bound, gap[%])
    """
    model = cp_model.CpModel()
    flows = range(len(F))
    edges = list(G.edges())

    repeats = [hyper_period // f['period'] for f in F]

    # ---------------- Routing-Variables ----------------
    flow_v = {(c, e): model.NewBoolVar(f"f_{c}_{e}") for c in flows for e in edges}

    # Constraint 1: Flow conseervation for each flow: Source out=1/in=0; Sink in=1/out=0; Relay-Node in=out<=1
    for c, f in enumerate(F):
        s, t = f['source'], f['destination']
        model.Add(sum(flow_v[c, e] for e in G.out_edges(s)) == 1)
        model.Add(sum(flow_v[c, e] for e in G.in_edges(s)) == 0)
        model.Add(sum(flow_v[c, e] for e in G.in_edges(t)) == 1)
        model.Add(sum(flow_v[c, e] for e in G.out_edges(t)) == 0)
        for v in G.nodes():
            if v in (s, t): continue
            in_v  = sum(flow_v[c, e] for e in G.in_edges(v))
            out_v = sum(flow_v[c, e] for e in G.out_edges(v))
            model.Add(in_v == out_v)

    # Constraint 2: We do not allow cycles
    N = G.number_of_nodes()
    u = {(c, n): model.NewIntVar(0, N, f"lvl_{c}_{n}") for c in flows for n in G.nodes()}
    for c, f in enumerate(F):
        t = f['destination']
        model.Add(u[c, t] == 0)
        M = N
        for (i, j) in edges:
            # f_ij = 1 -> u[i] >= u[j] + 1  (strictly decreasing levels => no cycles)
            model.Add(u[c, i] >= u[c, j] + 1 - M * (1 - flow_v[c, (i, j)]))

    # ---------------- Scheduling-Variables ----------------
    Task = collections.namedtuple("Task", "start interval")
    all_flows = {}
    node_to_intervals = collections.defaultdict(list)
    node_to_starts = collections.defaultdict(list)
    sample_vars = {}

    horizon = hyper_period + max(f["latency"] for f in F)

    for c, f in enumerate(F):
        sample_vars[c] = model.NewIntVar(0, horizon, f"sample_{c}")

        for offset in range(repeats[c]):
            base = offset * f["period"]
            for (n1, n2) in edges:
                for p in range(f["packets"]):
                    suf = f"_{c}_{offset}_{n1}_{n2}_{p}"
                    start = model.NewIntVar(base, horizon, "start" + suf)
                    # optional interval: only active when edge is chosen
                    interval = model.NewOptionalFixedSizeIntervalVar(start, 1, flow_v[c, (n1, n2)], "intv" + suf)
                    all_flows[c, offset, (n1, n2), p] = Task(start, interval)
                    node_to_intervals[n1].append(interval)
                    node_to_intervals[n2].append(interval)
                    node_to_starts[n1].append((start, flow_v[c, (n1, n2)]))
                    node_to_starts[n2].append((start, flow_v[c, (n1, n2)]))

    # ---------------- Scheduling-Constraints ----------------
    # Constraint 1: NoOverlap for each node
    for n in G.nodes():
        if node_to_intervals[n]:
            model.AddNoOverlap(node_to_intervals[n])

    # Constraint 2: The sampling period of each node is fixed,
    # and we need to wait for at least one time-period to send a new packet.
    for c, f in enumerate(F):
        for offset in range(repeats[c]):
            for e in G.out_edges(f["source"]):
                model.Add(sample_vars[c] + offset * f["period"] <= all_flows[c, offset, e, 0].start)\
                     .OnlyEnforceIf(flow_v[c, e])

    # Constraint 3: Precedence on path
    for c, f in enumerate(F):
        for offset in range(repeats[c]):
            for n in G.nodes():
                for p in range(f["packets"]):
                    for (i, _), (_, j) in itertools.product(G.in_edges(n), G.out_edges(n)):
                        model.Add(all_flows[c, offset, (i, n), p].interval.EndExpr()
                                  <= all_flows[c, offset, (n, j), p].start)\
                             .OnlyEnforceIf([flow_v[c, (i, n)], flow_v[c, (n, j)]])

    # Constraint 4: Packets are sent in correct order:
    for c, f in enumerate(F):
        for offset in range(repeats[c]):
            for e in edges:
                for p in range(f["packets"] - 1):
                    model.Add(all_flows[c, offset, e, p].interval.EndExpr()
                              <= all_flows[c, offset, e, p + 1].start)\
                         .OnlyEnforceIf(flow_v[c, e])

    # Constraint 5: Check latency for each flow.
    # Note that the latency depends on the sample-time and not the send-time:
    for c, f in enumerate(F):
        last_packet = f["packets"] - 1
        for offset in range(repeats[c]):
            for e in G.in_edges(f['destination']):
                model.Add(
                    all_flows[c, offset, e, last_packet].start
                    - (sample_vars[c] + offset * f["period"]) < f["latency"]
                ).OnlyEnforceIf(flow_v[c, e])

    # Constraint 6: Phase-Shift/Capacity: (active time slots on node) < hyper_period
    # Note: Time-values for each node are relative, we only need to check if the interval
    # between the first and last transmission does not exceed the hyper-period.
    for n in G.nodes():
        if node_to_starts[n]:
            mx = model.NewIntVar(0, horizon, f"max_{n}")
            mn = model.NewIntVar(0, horizon, f"min_{n}")
            for s, b in node_to_starts[n]:
                model.Add(mx >= s).OnlyEnforceIf(b)
                model.Add(mn <= s).OnlyEnforceIf(b)
            model.Add(mx - mn < hyper_period)

    # ---------------- Bounds / Objectives ----------------
    # 1) Total-Transmissions ≤ Bound
    for c, f in enumerate(F):
        path_len = sum([flow_v[c, e] for e in edges])
        model.Add(path_len <= bounds["NumTransmissionsPerFlow"][c] + slack_tx)

    # Cost 1: Min-Max Energy for each node
    c_rx, c_tx = 1, 1
    energy_vars = []
    for n in G.nodes():
        if n == sink:
            continue
        cons = 0
        for e in G.in_edges(n):
            cons += sum(c_rx * flow_v[c, e] * F[c]["packets"] * repeats[c] for c in flows)
        for e in G.out_edges(n):
            cons += sum(c_tx * flow_v[c, e] * F[c]["packets"] * repeats[c] for c in flows)
        energy_vars.append(cons)

    energy_max = model.NewIntVar(0, hyper_period, "energy_max")
    model.AddMaxEquality(energy_max, energy_vars)


    # Cost 2: Minimize sum of all latencies
    lat_terms = []
    for c, f in enumerate(F):
        last_packet = f["packets"] - 1
        for offset in range(repeats[c]):
            lat = model.NewIntVar(0, f["latency"], f"lat_{c}_{offset}")
            for e in G.in_edges(f["destination"]):
                model.Add(all_flows[c, offset, e, last_packet].start
                          - (sample_vars[c] + offset * f["period"]) == lat
                ).OnlyEnforceIf(flow_v[c, e])
            lat_terms.append(lat)

    # Combine the two objectives and solve:
    model.Minimize(w_latency * sum(lat_terms) + w_energy * energy_max)

    # ---------------- Hints (Warm-Start) ----------------
    if hint is not None:
        chosen = {(c, (i, j)) for c, path in enumerate(hint) for (i, j) in path}
        for c in flows:
            for e in edges:
                model.AddHint(flow_v[c, e], 1 if (c, e) in chosen else 0)

    # ---------------- Solver + Observer ----------------
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = int(time_limit)
    if workers is not None:
        solver.parameters.num_search_workers = int(workers)
    solver.parameters.log_search_progress = bool(log_progress)

    logger = CPLogger(has_objective=True)
    status = solver.Solve(model, logger)

    # ---------------- Extract solution ----------------
    solution_routing, solution_scheduling = [], []
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        solution_routing_temp = []
        for c in flows:
            path = []
            for (i, j) in edges:
                if solver.Value(flow_v[c, (i, j)]):
                    path.append((i, j))
                    for p in range(F[c]["packets"]):
                        for offset in range(repeats[c]):
                            # A solution contains:
                            # (flow-id, offset, packet-id, source, destination, start-time)
                            start_val = solver.Value(all_flows[c, offset, (i, j), p].start)
                            solution_scheduling.append((c, offset, p, i, j, start_val))
            solution_routing_temp.append(path)

        # Sort paths for each flow:
        for c in flows:
            s = F[c]['source']
            t = F[c]['destination']
            path = []
            cur = s
            guard = 0
            while cur != t and solution_routing_temp and guard < len(edges) + 5:
                nxt = None
                for (i, j) in list(solution_routing_temp[c]):
                    if i == cur:
                        path.append((i, j))
                        nxt = j
                        solution_routing_temp[c].remove((i, j))
                        break
                if nxt is None:
                    break
                cur = nxt
                guard += 1
            solution_routing.append(path)

    # Status
    if status == cp_model.OPTIMAL:
        info = "optimal"
    elif status == cp_model.FEASIBLE:
        info = "feasible"
    elif status == cp_model.INFEASIBLE:
        info = "infeasible"
    else:
        info = "timeout"

    # Progress-Report
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

    return (solution_routing, solution_scheduling), info, progress
