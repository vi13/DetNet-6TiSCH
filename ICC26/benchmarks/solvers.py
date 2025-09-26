import time

from ICC26.optimizers.routing import solve_routing
from ICC26.optimizers.scheduler import solve_scheduling
from ICC26.optimizers.JRaS import solve_jras


class Solver:
    @staticmethod
    def separate_energy(G, F, hyper_period, sink, time_limit):
        start_routing = time.time()
        solution_routing, info_routing, bounds = solve_routing(G, F, hyper_period, sink, prio_energy=2, prio_tx=1, time_limit=time_limit)
        runtime_routing = time.time() - start_routing
        runtime_scheduling = 0
        solution_scheduling, info_scheduling, progress = None, None, None
        if len(solution_routing) > 0:
            start_scheduling = time.time()
            solution_scheduling, info_scheduling, progress = solve_scheduling(G, F, solution_routing, hyper_period, time_limit=time_limit)
            runtime_scheduling = time.time() - start_scheduling
        return (solution_routing, solution_scheduling), (info_routing, info_scheduling), (runtime_routing, runtime_scheduling), progress

    @staticmethod
    def separate_tx(G, F, hyper_period, sink, time_limit):
        start_routing = time.time()
        solution_routing, info_routing, bounds = solve_routing(G, F, hyper_period, sink, prio_energy=1, prio_tx=2, time_limit=time_limit)
        runtime_routing = time.time() - start_routing
        runtime_scheduling = 0
        solution_scheduling, info_scheduling, progress = None, None, None
        if len(solution_routing) > 0:
            start_scheduling = time.time()
            solution_scheduling, info_scheduling, progress = solve_scheduling(G, F, solution_routing, hyper_period, time_limit=time_limit)
            runtime_scheduling = time.time() - start_scheduling
        return (solution_routing, solution_scheduling), (info_routing, info_scheduling), (runtime_routing, runtime_scheduling), progress

    @staticmethod
    def jras(G, F, hyper_period, sink, time_limit):
        start_routing = time.time()
        solution_routing_temp, info_routing, bounds = solve_routing(G, F, hyper_period, sink, prio_energy=2, prio_tx=1, time_limit=time_limit)
        runtime_routing = time.time() - start_routing
        runtime_scheduling = 0
        solution_scheduling, solution_routing, info_scheduling, progress = None, None, None, None
        if len(solution_routing_temp) > 0:
            start = time.time()
            (solution_routing, solution_scheduling), info_scheduling, progress = solve_jras(G, F, hyper_period, sink, bounds, hint=solution_routing_temp, time_limit=time_limit, w_latency=1, w_energy=1)
            runtime_scheduling = time.time() - start
        return (solution_routing, solution_scheduling), (info_routing, info_scheduling), (runtime_routing, runtime_scheduling), progress
