import random
import pickle
from matplotlib import pyplot as plt
from GraphGenerators import random_planar_graph
from RoutingAlgos.geomentricRouting.FaceRouting import *
from RoutingAlgos.geomentricRouting.GreedyRouting import *

# Load graph object from file
#planar_graph = pickle.load(open('test_planar_graph.pickle', 'rb'))

# Generate random planar graph
planar_graph = random_planar_graph(
    50, radius_lower_bound=0.5, radius_upper_bound=2, position_lower_bound=0, position_upper_bound=5, dim=2,
    pos=None, p=2, seed=None, pos_name="pos")
# Save graph object to file
pickle.dump(planar_graph, open('double_face_loop_s26_d16.pickle', 'wb'))
print('Is the graph planar: ' + str(nx.is_planar(planar_graph)))
_, planar_embedding = nx.check_planarity(planar_graph)
#print('Planar embedding: ' + str(planar_embedding.get_data()))
nodes_data = planar_graph.nodes(data=True)
nodes, data = list(zip(*nodes_data))
positions = {n: data[n]['pos'] for n in nodes}
nx.draw_networkx(planar_graph, pos=positions)
plt.show()

# Algorithm test
s = random.choice(nodes)
tmp_nodes = list(nodes)
tmp_nodes.remove(s)
tmp_nodes = tuple(tmp_nodes)
d = random.choice(tmp_nodes)
print ('Source: ' + str(s) + ', Destination: ' + str(d))
if check_route(planar_graph, s, d):
    print('Route from ' + str(s) + ' to ' + str(d) + ' exists')
    result, route = other_face_route(planar_graph, planar_embedding, s, d)
    if result:
        print('Route was found successfully!')
        print('Route: ' + str(route))
        print('Number of hops: ' + str(len(route)-1))
    else:
        print('No route found')
else:
    print('No route from ' + str(s) + ' to ' + str(d) + ' exists')