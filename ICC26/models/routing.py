import gurobipy as gp
from gurobipy import GRB


def solve_routing(G, F, hyper_period, sink, prio_energy, prio_tx, time_limit=None, log=False):
    """
    Multi-Objective ILP:
      (1) Minimize maximal node-energy-consumption (min-max)
      (2) Afterward: Minimize Number of overall radio transmissions

    G : networkx.DiGraph
    F : List of Flows; each Flow: dict with Keys 'source','destination','packets','period','latency'
    hyper_period : int
    sink : Node-ID
    """

    flows = range(len(F))
    V = list(G.nodes())
    E = list(G.edges())

    # How many packets are sent each hyper-period for each flow? Depends on flow-period and number of packets
    flow_reps = [F[c]['packets'] * (hyper_period // F[c]['period']) for c in flows]

    # Model
    m = gp.Model("netflow")
    m.Params.OutputFlag = 1 if log else 0
    if time_limit is not None:
        m.Params.TimeLimit = int(time_limit)

    # The binary variable indicates if an edge is used for routing
    flow_var = m.addVars(flows, E, vtype=GRB.BINARY, name="flow")

    # --------------------------
    # Constraint 1: Flow conservation, so that a path is established between source and destination
    # Note: Here, a flow cannot be split among multiple paths. Each flow (all packets + periods) take exactly one path.
    # --------------------------
    for c in flows:
        s = F[c]['source']
        t = F[c]['destination']

        # Source: exactly 1 outgoing, 0 incoming
        m.addConstr(gp.quicksum(flow_var[c, s, j] for j in G.successors(s)) == 1, name=f"src_out_{c}")
        m.addConstr(gp.quicksum(flow_var[c, i, s] for i in G.predecessors(s)) == 0, name=f"src_in_{c}")

        # Sink: exactly 1 incoming, 0 outgoing
        m.addConstr(gp.quicksum(flow_var[c, i, t] for i in G.predecessors(t)) == 1, name=f"sink_in_{c}")
        m.addConstr(gp.quicksum(flow_var[c, t, j] for j in G.successors(t)) == 0, name=f"sink_out_{c}")

        # Relay nodes: incoming = outgoing ∈ {0,1}
        for v in V:
            if v in (s, t):
                continue
            in_v  = gp.quicksum(flow_var[c, i, v] for i in G.predecessors(v))
            out_v = gp.quicksum(flow_var[c, v, j] for j in G.successors(v))
            m.addConstr(in_v == out_v, name=f"cont_{c}_{v}")

    # --------------------------
    # Constraint 2: Do not allow cycles.
    # Optional: This is not needed if we minimize the number of transmissions in the end,
    # because the optimal solution will remove all cycles.
    # --------------------------
    N = len(V)
    u = m.addVars(flows, V, vtype=GRB.INTEGER, lb=0, ub=N, name="u")
    for c in flows:
        t = F[c]['destination']
        m.addConstr(u[c, t] == 0, name=f"lvl_sink_{c}")
        M = N
        for (i, j) in E:
            m.addConstr(u[c, i] >= u[c, j] + 1 - M * (1 - flow_var[c, i, j]), name=f"mtz_{c}_{i}_{j}")

    # --------------------------
    # Constraint 3: Capacity per node (RX+TX Slots <= hyper_period)
    # --------------------------
    for n in V:
        loads = []
        for c in flows:
            deg_n_c = flow_var.sum(c, '*', n) + flow_var.sum(c, n, '*')  # in + out
            loads.append(deg_n_c * flow_reps[c])
        m.addConstr(gp.quicksum(loads) <= hyper_period, name=f"cap_{n}")

    # --------------------------
    # Constraint 4: Minimize path-length so that latency requirements are not violated
    # --------------------------
    path_len = {}
    h = {}
    for c, f in enumerate(F):
        # Pfadlänge als Anzahl genutzter Kanten (Integer)
        path_len[c] = m.addVar(vtype=GRB.INTEGER, lb=1, ub=len(E), name=f"pathlen_{c}")
        m.addConstr(path_len[c] == flow_var.sum(c, '*', '*'), name=f"pathlen_def_{c}")

        # h ∈ {1,2}; h = 1 if path_len==1, else 2
        # Why? Nodes between source and destination will need 2 timeslots (rx, tx) for each packet.
        h[c] = m.addVar(vtype=GRB.INTEGER, lb=1, ub=2, name=f"h_{c}")
        b = m.addVar(vtype=GRB.BINARY, name=f"is_len1_{c}")
        # b=1  <=> path_len == 1 ; b=0 => path_len >= 2
        m.addGenConstrIndicator(b, True,  path_len[c] == 1, name=f"len1_if_b1_{c}")
        m.addGenConstrIndicator(b, False, path_len[c] >= 2, name=f"len_ge2_if_b0_{c}")
        # h = 2 - b  → 1 if b=1, else 2
        m.addConstr(h[c] == 2 - b, name=f"h_link_{c}")

        # Latenz-Constraint (End-to-end): path_len + h*(packets-1) ≤ latency
        m.addConstr(
            path_len[c] + h[c] * (f['packets'] - 1) <= f['latency'],
            name=f"latency_{c}"
        )

    # --------------------------
    # Cost 1: Energy (min-max) – primary goal
    # --------------------------
    # We assume that it takes the same amount of energy to transmit and receive a packet
    c_rx, c_tx = 1, 1
    energy_max = m.addVar(lb=0, vtype=GRB.INTEGER, name="EnergyMax")
    for n in V:
        # The sink node usually has a fixed power-supply, so we exclude it:
        if n == sink:
            continue
        per_node = []
        for c in flows:
            rx = flow_var.sum(c, '*', n)
            tx = flow_var.sum(c, n, '*')
            per_node.append((c_rx * rx + c_tx * tx) * flow_reps[c])
        m.addConstr(gp.quicksum(per_node) <= energy_max, name=f"energy_node_{n}")

    # Cost 2: Reduce number of transmissions – secondary goal
    # Short routes reduce energy and increase reliability
    tx_sum = m.addVar(lb=0, vtype=GRB.INTEGER, name="HopSum")
    m.addConstr(tx_sum == gp.quicksum(path_len[c] * flow_reps[c] for c in flows), name="hop_sum_def")

    # --------------------------
    # Multi-Objective (lexikografisch)
    # --------------------------
    m.setObjectiveN(energy_max, index=0, priority=prio_energy, weight=1.0, name="MinMaxEnergy")
    m.setObjectiveN(tx_sum,   index=1, priority=prio_tx, weight=1.0, name="MinHopSum")

    # Solve
    m.optimize()

    # Status
    if m.Status == GRB.OPTIMAL:
        info = 'optimal'
    elif m.Status == GRB.TIME_LIMIT:
        info = 'timeout'
    elif m.Status == GRB.INFEASIBLE:
        info = 'infeasible'
    elif m.SolCount > 0:
        info = 'feasible'
    else:
        info = 'unknown'

    # Construct routing solutions and sort paths for each flow:
    solution_routing = []
    if m.SolCount > 0:
        val = m.getAttr("X", flow_var)
        for c in flows:
            used = {(i, j) for (i, j) in E if val[c, i, j]}
            s = F[c]['source']
            t = F[c]['destination']
            path = []
            cur = s
            guard = 0
            while cur != t and used and guard < len(E) + 5:
                nxt = None
                for (i, j) in list(used):
                    if i == cur:
                        path.append((i, j))
                        nxt = j
                        used.remove((i, j))
                        break
                if nxt is None:
                    break
                cur = nxt
                guard += 1
            solution_routing.append(path)

    # Results
    bounds = None
    if m.SolCount > 0:
        bounds = {
            'NumTransmissions': int(tx_sum.X),
            'NumTransmissionsPerFlow': [len(p) for p in solution_routing],
            'EnergyMax': int(energy_max.X)
        }

    return solution_routing, info, bounds
