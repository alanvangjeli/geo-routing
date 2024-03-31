import random
import pickle
import networkx as nx
import numpy as np
from matplotlib import pyplot as plt
from GraphGenerators import random_planar_graph
from RoutingAlgos.geometricRouting.GOAFRPlus import GOAFRPlus
from RoutingAlgos.geometricRouting.GOAFR import GreedyOtherAdaptiveFaceRouting
from RoutingAlgos.geometricRouting.OAFR import OtherBoundedFaceRouting, OtherAdaptiveFaceRouting
from RoutingAlgos.geometricRouting.OFR import OtherFaceRouting

# Generate random planar graph which has a route from s to d
while True:
    planar_graph = random_planar_graph(
        50, radius_lower_bound=0.5, radius_upper_bound=2.5, position_lower_bound=0, position_upper_bound=10, dim=2,
        pos=None, p=2, seed=None, pos_name="pos")
    nodes_data = planar_graph.nodes(data=True)
    nodes, data = list(zip(*nodes_data))
    #random.seed(7)
    s = random.choice(nodes)
    tmp_nodes = list(nodes)
    tmp_nodes.remove(s)
    tmp_nodes = tuple(tmp_nodes)
    d = random.choice(tmp_nodes)
    if nx.has_path(planar_graph, s, d):
        shortest_path = nx.shortest_path(planar_graph, s, d)
        break

# Save graph object to file
pickle.dump(planar_graph, open('test_planar_graph.pickle', 'wb'))
print('Is the graph planar: ' + str(nx.is_planar(planar_graph)))

_, planar_embedding = nx.check_planarity(planar_graph)
planar_embedding.check_structure()
#print('Planar embedding: ' + str(planar_embedding.get_data()))

positions = nx.get_node_attributes(planar_graph, "pos")
print ('Source: ' + str(s) + ', Destination: ' + str(d))

# Plot the graph
color_map = ['yellow' if node == s else '#1f78b4' for node in planar_graph]
color_map[d] = 'green'
nx.draw_networkx(planar_graph, pos=positions, node_color=color_map)
plt.show()

# Algorithm test
ofr = OtherFaceRouting()
obfr = OtherBoundedFaceRouting()
oafr = OtherAdaptiveFaceRouting()
goafr = GreedyOtherAdaptiveFaceRouting()
goafr_plus = GOAFRPlus()
print('Route from ' + str(s) + ' to ' + str(d) + ' exists')
# result, route, resultTag = goafr.route(planar_graph, s, d, positions)
success, route, resultTag = goafr_plus.find_route(planar_graph, s, d, positions, 1.4, np.sqrt(2), 0.01)
if success:
    print('Result tag: ' + str(resultTag))
    print('Route: ' + str(route))
    print('Number of hops: ' + str(len(route)-1))
else:
    print('No route found')
    print('Result tag: ' + str(resultTag))
    print('Route: ' + str(route))