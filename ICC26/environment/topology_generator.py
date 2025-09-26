import math
import random
import networkx as nx

from ICC26.environment.propagation_model import PisterHackModel


class TopologyGenerator:
    @staticmethod
    def random_mesh_pisterhack(dim_x, dim_y, nodes, min_pdr, num_stable_links, min_distance):
        G_temp = nx.DiGraph()
        # Add random init node
        x = random.randint(0, dim_x)
        y = random.randint(0, dim_y)
        G_temp.add_node(0, pos=(x, y))

        # Add remaining nodes
        id = 1
        while len(G_temp.nodes()) < nodes:
            x = random.randint(0, dim_x)
            y = random.randint(0, dim_y)
            node_to_pos = nx.get_node_attributes(G_temp, 'pos')

            stable_counter = 0
            edges_l, rssi_d, pdr_d = [], {}, {}
            for id2, pos2 in node_to_pos.items():
                # compute RSSI and PDR values for each link
                euc_distance = math.sqrt((x - pos2[0]) ** 2 + (y - pos2[1]) ** 2)

                if euc_distance < min_distance:
                    stable_counter = 0
                    break

                rssi = PisterHackModel.compute_rssi(euc_distance)
                pdr = PisterHackModel.convert_rssi_to_pdr(rssi)

                edges_l.append((id, id2))
                rssi_d[(id, id2)] = rssi
                pdr_d[(id, id2)] = pdr

                edges_l.append((id2, id))
                rssi_d[(id2, id)] = rssi
                pdr_d[(id2, id)] = pdr

                if min_pdr <= pdr:
                    stable_counter += 1

            if stable_counter == len(G_temp.nodes()) or num_stable_links <= stable_counter:
                G_temp.add_node(id, pos=(x, y))
                for (id1, id2) in edges_l:
                    if pdr_d[(id1, id2)] > 0:
                        G_temp.add_edge(id1, id2, RSSI=rssi_d[(id1, id2)], PDR=pdr_d[(id1, id2)], ETX=1/pdr_d[(id1, id2)])
                id += 1

        G = nx.DiGraph()
        G.add_nodes_from(G_temp.nodes(data=True))
        edges_routing = [(u, v, d) for u, v, d in G_temp.edges(data=True) if d['PDR'] >= min_pdr]
        G.add_edges_from(edges_routing)

        I = nx.DiGraph()
        I.add_nodes_from(G_temp.nodes(data=True))
        edges_interference = [(u, v, d) for u, v, d in G_temp.edges(data=True) if d['RSSI'] >= -95]
        I.add_edges_from(edges_interference)

        return G, I
