import pickle

import networkx as nx
import numpy as np
from matplotlib import pyplot as plt

from RoutingAlgos.GeometricRouting.GOAFR import GOAFR
from RoutingAlgos.GeometricRouting.GOAFRPlus import GOAFRPlus
from RoutingAlgos.GeometricRouting.OAFR import OAFR
from RoutingAlgos.GeometricRouting.OFR import OFR

# Load graph object from file
planar_graph = pickle.load(open("challengeExamples/dead_end_ofr_s14_d19.pickle", "rb"))
nodes_data = planar_graph.nodes(data=True)
nodes, data = list(zip(*nodes_data))
s = 14
d = 19
# s = random.choice(nodes)
# tmp_nodes = list(nodes)
# tmp_nodes.remove(s)
# tmp_nodes = tuple(tmp_nodes)
# d = random.choice(tmp_nodes)
# #random.seed(7)

print("Is the graph planar: " + str(nx.is_planar(planar_graph)))

_, planar_embedding = nx.check_planarity(planar_graph)
planar_embedding.check_structure()
# print('Planar embedding: ' + str(planar_embedding.get_data()))

positions = nx.get_node_attributes(planar_graph, "pos")
print("Source: " + str(s) + ", Destination: " + str(d))
color_map = ["yellow" if node == s else "#1f78b4" for node in planar_graph]
color_map[d] = "green"
color_map[32] = "red"
nx.draw_networkx(planar_graph, pos=positions, node_color=color_map)
plt.show()

# Algorithm test
print("Route from " + str(s) + " to " + str(d) + " exists")
ofr_new = OFR(planar_graph, s, d, positions)
oafr_new = OAFR(planar_graph, s, d, positions)
goafr_new = GOAFR(planar_graph, s, d, positions)
goafr_plus_new = GOAFRPlus(planar_graph, s, d, positions, np.sqrt(2), 0.01, 1.4)
result, route, resultTag = ofr_new.find_route()
if result:
    print("Result tag: " + str(resultTag))
    print("Route: " + str(route))
    print("Number of hops: " + str(len(route) - 1))
else:
    print("No route found")
    print("Result tag: " + str(resultTag))
    print("Route: " + str(route))
