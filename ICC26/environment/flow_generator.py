import random
import networkx as nx


"""
The traffic classes and hyper-period used in the experiments.
"""
HYPER_PERIOD = 100
FLOW_SETTINGS = [
    # Critical: Control or alarms
    {
        'id': 'Class_1',
        'period': 10,
        'latency': 5,
        'packets': [1]
    },
    # Fast: Open-Loop Control, fast telemetry
    {
        'id': 'Class_2',
        'period': 25,
        'latency': 10,
        'packets': [1]
    },
    # normal: Monitoring, standard telemetry
    {
        'id': 'Class_3',
        'period': 50,
        'latency': 20,
        'packets': [1, 2]
    },
    # Bulk: Monitoring, diagnosis
    {
        'id': 'Class_4',
        'period': 100,
        'latency': 30,
        'packets': [1, 2, 3, 4]
    }
]


class FlowGenerator:
    """
    Used for experiment 1: source nodes are placed randomly in the network.
    """
    @staticmethod
    def random_traffic(G, utilization):
        F = []
        node_list = list(G)
        sink = random.sample(node_list, 1)[0]
        node_list.remove(sink)

        flow_id = -1
        packet_counter = 0

        while packet_counter < utilization:
            if len(node_list) == 0:
                # Did not work: reset
                F = []
                node_list = list(G)
                sink = random.sample(node_list, 1)[0]
                node_list.remove(sink)
                flow_id = -1
                packet_counter = 0

            # This is hard-coded from settings above
            # If utilization is almost max, just fill remaining packets.
            if packet_counter + 4 >= utilization:
                id = FLOW_SETTINGS[3]['id']
                period = FLOW_SETTINGS[3]['period']
                latency = FLOW_SETTINGS[3]['latency']
                random_packets = utilization - packet_counter
            else:
                priority_class = random.sample(FLOW_SETTINGS, 1)[0]
                id = priority_class['id']
                period = priority_class['period']
                latency = priority_class['latency']
                packets = priority_class['packets']
                random_packets = random.sample(packets, 1)[0]

            additional_packets = random_packets * (HYPER_PERIOD // period)
            if packet_counter + additional_packets > utilization:
                continue

            temp_nodes = []
            for node in node_list:
                if node == sink:
                    continue
                sp = nx.shortest_path_length(G, node, sink)
                lower_bound = sp + min(2, sp) * (random_packets - 1)
                if lower_bound <= latency:
                    temp_nodes.append(node)

            if len(temp_nodes) == 0:
                continue

            flow_id += 1
            packet_counter += additional_packets
            source = random.sample(temp_nodes, 1)[0]

            if source in node_list:
                node_list.remove(source)

            F.append(
                {'id': str(flow_id),
                 'source': source,
                 'destination': sink,
                 'priority_id': id,
                 'packets': random_packets,
                 'period': period,
                 'latency': latency})
        return F, sink


    """
    Used for experiment 2: Stress-test, place source nodes as far away from the sink as possible.
    """
    @staticmethod
    def random_traffic_stress(G, utilization):
        F = []
        sink = random.sample(list(G), 1)[0]

        flow_id = -1
        packet_counter = 0

        # Here we sort nodes, depending on how far they are away from the sink
        H = G.reverse(copy=False)
        lengths = nx.single_source_shortest_path_length(H, sink)  # {node: hops}
        # Sink selbst rausnehmen
        lengths.pop(sink, None)
        # top-k nach Distanz (absteigend)
        top = sorted(lengths.items(), key=lambda x: x[1], reverse=True)
        nodes_sorted = [n for n, _ in top]

        while packet_counter < utilization:
            if len(nodes_sorted) == 0:
                # Did not work: reset
                F = []
                sink = random.sample(list(G), 1)[0]
                nodes_sorted = [n for n, _ in top]
                flow_id = -1
                packet_counter = 0

            # This is hard-coded from settings above
            if packet_counter + 4 >= utilization:
                id = FLOW_SETTINGS[3]['id']
                period = FLOW_SETTINGS[3]['period']
                latency = FLOW_SETTINGS[3]['latency']
                random_packets = utilization - packet_counter
            else:
                priority_class = random.sample(FLOW_SETTINGS, 1)[0]
                id = priority_class['id']
                period = priority_class['period']
                latency = priority_class['latency']
                packets = priority_class['packets']
                random_packets = random.sample(packets, 1)[0]

            additional_packets = random_packets * (HYPER_PERIOD // period)
            if packet_counter + additional_packets > utilization:
                continue

            source = None
            for node in nodes_sorted:
                if node == sink:
                    continue
                sp = nx.shortest_path_length(G, node, sink)
                lower_bound = sp + min(2, sp) * (random_packets - 1)
                if lower_bound <= latency:
                    source = node
                    break

            if source is None:
                continue

            flow_id += 1
            packet_counter += additional_packets

            if source in nodes_sorted:
                nodes_sorted.remove(source)

            F.append(
                {'id': str(flow_id),
                 'source': source,
                 'destination': sink,
                 'priority_id': id,
                 'packets': random_packets,
                 'period': period,
                 'latency': latency})
        return F, sink
