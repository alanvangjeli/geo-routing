import json
import random
import time

import networkx as nx
import numpy as np
from matplotlib import pyplot as plt

from GraphGenerators import random_planar_graph
from RoutingAlgos.geometricRouting.GOAFRPlus import GOAFRPlus
from RoutingAlgos.geometricRouting.GreedyOtherAdaptiveFaceRouting import GreedyOtherAdaptiveFaceRouting
from RoutingAlgos.geometricRouting.OAFR import OtherBoundedFaceRouting, OtherAdaptiveFaceRouting
from RoutingAlgos.geometricRouting.OFR import OtherFaceRouting
from RoutingAlgos.geometricRouting.GreedyRouting import GreedyRouting

# Graph parameters
number_nodes = 0
radius_lower_bound = 0.5
radius_upper_bound = 1.5
position_lower_bound = 0
position_upper_bound = 20

# Initialize metrics
results = []
total_runtime = 0
performance_cumulative = 0
success_count = 0
iteration_time_list = []
mean_performance_list = []
success_rate_list = []
network_density_list = []

# Number of generated triples (g, s, d) was 2000 in GOAFR+
k = 100
interval_length = 10
node_number_factor = 10

# Initialize geometric routing classes
#greedy = GreedyRouting()
#ofr = OtherFaceRouting()
#obfr = OtherBoundedFaceRouting()
#oafr = OtherAdaptiveFaceRouting()
#goafr = GreedyOtherAdaptiveFaceRouting()
#goafr_plus = GOAFRPlus()

# Choose algorithm
#algorithm = goafr

# 0 to 1999
for i in range(k):
    # Increase number of nodes by 10 every 10 iterations
    if i % interval_length == 0:
        number_nodes += node_number_factor
        network_density = number_nodes * np.pi / position_upper_bound ** 2
        network_density_list[i:i+interval_length] = [network_density] * interval_length
        print("Iteration: " + str(i))

    # Start measuring iteration time
    iteration_start = time.process_time_ns()
    # Generate random planar graphs until a route from s to d exists
    # Network density = number of nodes * π  / surface of plane
    while True:
        start = time.process_time()
        planar_graph = random_planar_graph(
            number_nodes, radius_lower_bound=radius_lower_bound, radius_upper_bound=radius_upper_bound, position_lower_bound=position_lower_bound, position_upper_bound=position_upper_bound, dim=2, pos=None, p=2, seed=None, pos_name="pos")
        nodes_data = planar_graph.nodes(data=True)
        nodes, data = list(zip(*nodes_data))
        #random.seed(7)
        s = random.choice(nodes)
        tmp_nodes = list(nodes)
        tmp_nodes.remove(s)
        tmp_nodes = tuple(tmp_nodes)
        d = random.choice(tmp_nodes)
        if nx.has_path(planar_graph, s, d):
            graph_generation_time = time.process_time() - start
            print("Graph generated in " + str(graph_generation_time) + " seconds")
            start = time.process_time()
            shortest_path = nx.shortest_path(planar_graph, s, d)
            shortest_path_time = time.process_time() - start
            print("Shortest path found in " + str(shortest_path_time) + " seconds")
            break

    # Planarity checks
    start = time.process_time()
    if not nx.is_planar(planar_graph):
        break
    _, planar_embedding = nx.check_planarity(planar_graph)
    planar_embedding.check_structure()
    planarity_check_time = time.process_time() - start
    print("Planarity checks done in " + str(planarity_check_time) + " seconds")

    # Data preprocessing
    positions_start = time.process_time()
    positions = nx.get_node_attributes(planar_graph, "pos")
    positions_time = time.process_time() - positions_start
    print("Positions extracted in " + str(positions_time) + " seconds")

    # Strongly connected components
    # contains_sd = False
    # for c in nx.strongly_connected_components(planar_graph):
    #     if s in c and d in c:
    #         contains_sd = True
    #         break

    # if contains_sd:
    #    planar_graph.subgraph(c)

    algorithm = GreedyRouting(planar_graph, s, d, positions)
    # algorithm = OtherFaceRouting(planar_graph, s, d, positions)
    # algorithm = OtherBoundedFaceRouting(planar_graph, s, d, positions)
    # algorithm = OtherAdaptiveFaceRouting(planar_graph, s, d, positions)
    # algorithm = GreedyOtherAdaptiveFaceRouting(planar_graph, s, d, positions)
    # algorithm = GOAFRPlus(planar_graph, s, d, positions)

    # Execute algorithm
    start = time.process_time()
    # GOAFR params: 1.4, np.sqrt(2), 0.01
    #success, route, resultTag = algorithm.find_route(planar_graph, s, d, positions)
    success, route, resultTag = algorithm.find_route()

    # Measure metrics
    # Success
    success_count += int(success)
    print("Success count: " + str(success_count))
    success_rate_list.append(success_count/(i+1))
    print("Success rate: " + str(success_count/(i+1)))

    # Mean performance
    if success:
        performance = len(route) / len(shortest_path)
        performance_cumulative += performance
        #mean_performance_list.append(performance_cumulative/(i+1))
        mean_performance_list.append(performance_cumulative/success_count)
    else:
        mean_performance_list.append(mean_performance_list[i-1])

    # Runtime
    iteration_time = (time.process_time_ns() - iteration_start) / 10 ** 9
    iteration_time_list.append(iteration_time)
    total_runtime += iteration_time
    print("Iteration #" + str(i) + " time: " + str(iteration_time) + " seconds")

    results.append({
        'iteration': i,
        'success': success,
        'result_tag': resultTag,
        'network_density': network_density_list[i],
        'success_rate': success_rate_list[i],
        'mean_performance': mean_performance_list[i],
        'iteration_time': iteration_time_list[i],
    })

# Average metrics
success_rate = success_count / k
runtime_avg = total_runtime / k
#mean_performance = performance_cumulative / k
mean_performance = performance_cumulative / success_count

# Print results
print('Success rate: ' + str(success_rate))
print('Total runtime: ' + str(total_runtime) + ' seconds')
print('Mean performance: ' + str(mean_performance))

# Export results as JSON
FILE_PATH = 'out/gr/results.json'
with open(FILE_PATH, 'w') as output_file:
    json.dump(results, output_file, indent=2)

# Plot results
# Mean performance
fig, ax = plt.subplots()
ax.plot(network_density_list, mean_performance_list)
ax.set(xlabel='Network Density', ylabel='Mean Performance',
       title='Greedy Routing Mean Performance')
fig.savefig("out/gr/mean_performance.png")
plt.show()

# Success rate
fig, ax = plt.subplots()
ax.plot(network_density_list, success_rate_list)
ax.set(xlabel='Network Density', ylabel='Success',
       title='Greedy Routing Success Rate')
fig.savefig("out/gr/success_rate.png")
plt.show()


