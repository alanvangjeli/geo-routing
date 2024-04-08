import pickle
import networkx as nx
import numpy as np
from matplotlib import pyplot as plt

from RoutingAlgos.geometricRouting.GOAFRPlus import GOAFRPlus
from RoutingAlgos.geometricRouting.OFR import OtherFaceRouting

# Load graph object from file
planar_graph = pickle.load(open('challengeExamples/double_face_loop_s26_d16.pickle', 'rb'))
nodes_data = planar_graph.nodes(data=True)
nodes, data = list(zip(*nodes_data))
s = 26
d = 16
# #random.seed(7)

print('Is the graph planar: ' + str(nx.is_planar(planar_graph)))

_, planar_embedding = nx.check_planarity(planar_graph)
planar_embedding.check_structure()
#print('Planar embedding: ' + str(planar_embedding.get_data()))

positions = nx.get_node_attributes(planar_graph, "pos")
print ('Source: ' + str(s) + ', Destination: ' + str(d))
color_map = ['yellow' if node == s else '#1f78b4' for node in planar_graph]
color_map[d] = 'green'
nx.draw_networkx(planar_graph, pos=positions, node_color=color_map)
plt.show()

# Algorithm test
print('Route from ' + str(s) + ' to ' + str(d) + ' exists')
ofr = OtherFaceRouting(planar_graph, s, d, positions)
goafr_plus = GOAFRPlus(planar_graph, s, d, positions, np.sqrt(2), 0.01, 1.4)
result, route, resultTag = ofr.find_route_ofr()
if result:
    print('Result tag: ' + str(resultTag))
    print('Route: ' + str(route))
    print('Number of hops: ' + str(len(route)-1))
else:
    print('No route found')
    print('Result tag: ' + str(resultTag))
    print('Route: ' + str(route))