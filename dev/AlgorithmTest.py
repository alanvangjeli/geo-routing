import pickle
import random
import time

import networkx as nx
import numpy as np
from matplotlib import pyplot as plt

from GraphGenerator import random_planar_graph
from RoutingAlgos.GeometricRouting.GOAFR import GOAFR
from RoutingAlgos.GeometricRouting.GOAFRPlus import GOAFRPlus
from RoutingAlgos.GeometricRouting.GR import GR
from RoutingAlgos.GeometricRouting.OAFR import OAFR
from RoutingAlgos.GeometricRouting.OFR import OFR

# Graph parameters
number_nodes = 2000
radius_lower_bound = 0.5
radius_upper_bound = 1.5
position_lower_bound = 0
position_upper_bound = 20

# Generate random planar graph which has a route from s to d
while True:
    graph_generation_start = time.process_time_ns()
    planar_graph = random_planar_graph(
        number_nodes,
        radius_lower_bound=radius_lower_bound,
        radius_upper_bound=radius_upper_bound,
        position_lower_bound=position_lower_bound,
        position_upper_bound=position_upper_bound,
        dim=2,
        p=2,
        seed=None,
        pos_name="pos",
    )
    graph_generation_time = (time.process_time_ns() - graph_generation_start) / 10**9
    print("Graph generation time: " + str(graph_generation_time) + " seconds")
    if len(planar_graph.edges) != 0:
        nodes_data = planar_graph.nodes(data=True)
        nodes, data = list(zip(*nodes_data))
        # random.seed(7)
        # Randomly pick s and d
        while True:
            s = random.choice(nodes)
            tmp_nodes = list(nodes)
            tmp_nodes.remove(s)
            tmp_nodes = tuple(tmp_nodes)
            d = random.choice(tmp_nodes)
            # If there is a path from s to d, break the loop
            if nx.has_path(planar_graph, s, d):
                # print('s: ' + str(s) + ', d: ' + str(d))
                # Shortest path needed for mean performance
                shortest_path = nx.shortest_path(planar_graph, s, d)
                s_and_d_connected = True
                break
        if s_and_d_connected:
            break

# Save graph object to file
pickle.dump(
    planar_graph, open("test_planar_graph_s" + str(s) + "_d" + str(d) + ".pickle", "wb")
)
print("Is the graph planar: " + str(nx.is_planar(planar_graph)))

_, planar_embedding = nx.check_planarity(planar_graph)
planar_embedding.check_structure()
# print('Planar embedding: ' + str(planar_embedding.get_data()))

positions = nx.get_node_attributes(planar_graph, "pos")
print("Source: " + str(s) + ", Destination: " + str(d))

# Plot the graph
color_map = ["yellow" if node == s else "#1f78b4" for node in planar_graph]
color_map[d] = "green"
print(len(planar_graph.nodes))
print(len(color_map))
nx.draw_networkx(planar_graph, pos=positions, node_color=color_map)
plt.show()

# Algorithm test
gr = GR(planar_graph, s, d, positions)
ofr_new = OFR(planar_graph, s, d, positions)
oafr_new = OAFR(planar_graph, s, d, positions)
goafr_new = GOAFR(planar_graph, s, d, positions)
goafr_plus_new = GOAFRPlus(planar_graph, s, d, positions, np.sqrt(2), 0.01, 1.4)
print("Route from " + str(s) + " to " + str(d) + " exists")
algorithm_execution_start = time.process_time_ns()
success, route, resultTag = goafr_plus_new.find_route()
algorithm_execution_time = (time.process_time_ns() - algorithm_execution_start) / 10**9
print("Algorithm execution time: " + str(algorithm_execution_time) + " seconds")
if success:
    print("Result tag: " + str(resultTag))
    print("Route: " + str(route))
    print("Number of hops: " + str(len(route) - 1))
else:
    print("No route found")
    print("Result tag: " + str(resultTag))
    print("Route: " + str(route))
