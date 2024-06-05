import json
import math
import random
import time

import networkx as nx
import numpy as np
import scipy
from matplotlib import pyplot as plt

from GraphGenerator import random_planar_graph
from RoutingAlgos.GeometricRouting.GOAFR import GOAFR
from RoutingAlgos.GeometricRouting.GOAFRPlus import GOAFRPlus
from RoutingAlgos.GeometricRouting.GR import GR
from RoutingAlgos.GeometricRouting.OAFR import OAFR
from RoutingAlgos.GeometricRouting.OFR import OFR
from RoutingAlgos.GeometricRouting.util import RECURSION_DEPTH_LIMIT, ResultTag

# Graph parameters
number_nodes = 2
radius_lower_bound = 0.75
radius_upper_bound = 1.25
position_lower_bound = 0
position_upper_bound = 20

scc_test_mapping = ["GOAFR+", "GOAFR+SCC"]
normal_test_mapping = ["GR", "OFR", "OAFR", "GOAFR", "GOAFR+"]
face_routing_mapping = ["OFR", "OAFR", "GOAFR", "GOAFR+"]
face_routing_mapping_scc = ["OFR", "OAFR", "GOAFR", "GOAFR+", "GOAFR+SCC"]
all_algos_mapping = ["GR", "OFR", "OAFR", "GOAFR", "GOAFR+", "GOAFR+SCC"]

# Initialize metrics
(
    max_edge_list_length_all_succeeded,
    max_edge_list_length_all_succeeded_scc,
    max_edge_list_lengths,
    min_edge_list_lengths,
    mean_edge_list_lengths,
) = (
    {
        "OFR": [],
        "OAFR": [],
        "GOAFR": [],
        "GOAFR+": [],
        "GOAFR+SCC": [],
    }
    for _ in range(5)
)
(
    results,
    mean_performance,
    mean_performance_all_succeeded,
    mean_performance_all_succeeded_scc,
    success_rate,
) = (
    {"GR": [], "OFR": [], "OAFR": [], "GOAFR": [], "GOAFR+": [], "GOAFR+SCC": []}
    for _ in range(5)
)
(
    performance_cumulative,
    performance_cumulative_all_succeeded,
    performance_cumulative_all_succeeded_scc,
    success_count,
    total_runtime,
) = (
    {"GR": 0, "OFR": 0, "OAFR": 0, "GOAFR": 0, "GOAFR+": 0, "GOAFR+SCC": 0}
    for _ in range(5)
)
network_density_list = []
total_preprocessing_time = 0
iteration_count = 0
success_count_all_succeeded = 0
success_count_all_succeeded_scc = 0

# Number of generated triples (g, s, d) was 2000 per network density in GOAFR+
# k = 100800
# interval_length = 360
# node_increase_factor = 10

# k = 56040  # 2802 * 20 # 2802 is the number of nodes needed for network density 22, 20 is the interval length
# interval_length = 20
# network_density_number = int(k / interval_length)
# node_increase_factor = 1

# k = 5604
# interval_length = 2
# network_density_number = int(k / interval_length)
# node_increase_factor = 1

k = 44000
interval_length = 2000
number_of_network_densities = int(k / interval_length)
network_density_limit = 22
node_increase_factor = math.ceil(
    position_upper_bound**2
    * network_density_limit
    / np.pi
    / number_of_network_densities
)

# k = 440
# interval_length = 20
# number_of_network_densities = int(k / interval_length)
# network_density_limit = 22
# node_increase_factor = math.ceil(
#     position_upper_bound**2
#     * network_density_limit
#     / np.pi
#     / number_of_network_densities
# )

iteration_parameters = [
    k,
    interval_length,
    number_of_network_densities,
    node_increase_factor,
]
for parameter in iteration_parameters:
    if not float(parameter).is_integer():
        raise ValueError("Invalid parameters")

# Export parameters
parameters = {
    "radius_lower_bound": radius_lower_bound,
    "radius_upper_bound": radius_upper_bound,
    "position_lower_bound": position_lower_bound,
    "position_upper_bound": position_upper_bound,
    "iterations": k,
    "network_density_number": number_of_network_densities,
    "interval_length": interval_length,
    "node_increase_factor": node_increase_factor,
    "recursion_depth_limit": RECURSION_DEPTH_LIMIT,
}
with open("results/parameters.json", "w") as output_file:
    json.dump(parameters, output_file, indent=2)

total_execution_start = time.process_time_ns()

break_outer_loop = False
# 0 to network_density_number-1
for i in range(number_of_network_densities):
    # Increase number of nodes by node_increase_factor every interval_length iterations
    number_nodes += node_increase_factor
    # Network density = number of nodes * π  / surface of plane
    network_density = number_nodes * np.pi / position_upper_bound**2
    network_density_list.append(network_density)

    (
        interval_edge_list_lengths,
        interval_edge_list_lengths_all_succeeded,
        interval_edge_list_lengths_all_succeeded_scc,
    ) = (
        {
            "OFR": [],
            "OAFR": [],
            "GOAFR": [],
            "GOAFR+": [],
            "GOAFR+SCC": [],
        }
        for _ in range(3)
    )

    for j in range(interval_length):
        iteration_count += 1
        iteration_edge_list_lengths = {
            "GR": [],
            "OFR": [],
            "OAFR": [],
            "GOAFR": [],
            "GOAFR+": [],
            "GOAFR+SCC": [],
        }
        iteration_performances = {
            "GR": 0,
            "OFR": 0,
            "OAFR": 0,
            "GOAFR": 0,
            "GOAFR+": 0,
            "GOAFR+SCC": 0,
        }
        print("Iteration: #" + str(iteration_count))

        # Generate a random planar graph where s and d are connected
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
            graph_generation_time = (
                time.process_time_ns() - graph_generation_start
            ) / 10**9
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

        positions = nx.get_node_attributes(planar_graph, "pos")

        # # Planarity checks
        # start = time.process_time()
        # if not nx.is_planar(planar_graph):
        #     break
        # _, planar_embedding = nx.check_planarity(planar_graph)
        # planar_embedding.check_structure()
        # planarity_check_time = time.process_time() - start
        # print("Planarity checks done in " + str(planarity_check_time) + " seconds")

        # Strongly connected components
        scc_subgraph = nx.empty_graph()
        for c in sorted(
            nx.strongly_connected_components(planar_graph), key=len, reverse=True
        ):
            if s in c and d in c:
                scc_subgraph = planar_graph.subgraph(c)
                break

        algorithm_mapping = {
            "GR": GR(planar_graph, s, d, positions),
            "OFR": OFR(planar_graph, s, d, positions),
            "OAFR": OAFR(planar_graph, s, d, positions),
            "GOAFR": GOAFR(planar_graph, s, d, positions),
            "GOAFR+": GOAFRPlus(planar_graph, s, d, positions, np.sqrt(2), 0.01, 1.4),
            "GOAFR+SCC": GOAFRPlus(
                scc_subgraph, s, d, positions, np.sqrt(2), 0.01, 1.4
            ),
        }

        one_algo_normal_test_failed = False
        one_algo_scc_test_failed = False
        for algorithm in algorithm_mapping:
            # print("Algorithm: " + algorithm)

            # Start measuring iteration time
            iteration_start = time.process_time_ns()

            if algorithm == "GOAFR+SCC" and nx.is_empty(scc_subgraph):
                success, route, resultTag = False, [], ResultTag.NO_SCC_WITH_S_D
            else:
                # Execute algorithm
                success, route, resultTag, edge_list_lengths = algorithm_mapping[
                    algorithm
                ].find_route()

            # Measure metrics

            # Runtime
            iteration_time = (time.process_time_ns() - iteration_start) / 10**6
            total_runtime[algorithm] += iteration_time
            # print(
            #     "Iteration #"
            #     + str(i)
            #     + " time: "
            #     + str(iteration_time)
            #     + " milliseconds"
            # )

            # Success
            success_count[algorithm] += int(success)
            # print("Success count: " + str(success_count))

            if success:
                # Performance
                performance = len(route) / len(shortest_path)
                iteration_performances[algorithm] = performance
                performance_cumulative[algorithm] += performance
                # Edge list lengths
                if algorithm != "GR":
                    iteration_edge_list_lengths[algorithm] = edge_list_lengths
                    interval_edge_list_lengths[algorithm].extend(edge_list_lengths)
            else:
                if algorithm in scc_test_mapping:
                    one_algo_scc_test_failed = True
                if algorithm in face_routing_mapping:
                    one_algo_normal_test_failed = True

            results[algorithm].append(
                {
                    "iteration": iteration_count,
                    "success": success,
                    "result_tag": resultTag,
                    "network_density": network_density,
                    "performance": performance,
                    "iteration_time_ms": iteration_time,
                }
            )

        # Fill all succeeded lists
        if not one_algo_normal_test_failed:
            success_count_all_succeeded += 1
            for algorithm in face_routing_mapping:
                # Add performances of algorithms in last iteration
                performance_cumulative_all_succeeded[algorithm] += (
                    iteration_performances[algorithm]
                )
                # Add edge list lengths of algorithms in last iteration
                interval_edge_list_lengths_all_succeeded[algorithm].extend(
                    iteration_edge_list_lengths[algorithm]
                )
        if not one_algo_scc_test_failed:
            success_count_all_succeeded_scc += 1
            for algorithm in scc_test_mapping:
                # Add performances of algorithms in last iteration
                performance_cumulative_all_succeeded_scc[algorithm] += (
                    iteration_performances[algorithm]
                )
                # Add edge list lengths of algorithms in last iteration
                interval_edge_list_lengths_all_succeeded_scc[algorithm].extend(
                    iteration_edge_list_lengths[algorithm]
                )

    # Add metrics to lists for all succeeded plots
    for algorithm in face_routing_mapping:
        # Mean performance
        if performance_cumulative_all_succeeded[algorithm] == 0:
            mean_performance_all_succeeded[algorithm].append(float("NaN"))
        else:
            mean_performance_all_succeeded[algorithm].append(
                performance_cumulative_all_succeeded[algorithm]
                / success_count_all_succeeded
            )
        # Max edge list length
        if len(interval_edge_list_lengths_all_succeeded[algorithm]) == 0:
            max_edge_list_length_all_succeeded[algorithm].append(float("NaN"))
        else:
            max_edge_list_length_all_succeeded[algorithm].append(
                max(interval_edge_list_lengths_all_succeeded[algorithm])
            )
    for algorithm in scc_test_mapping:
        # Mean performance
        if performance_cumulative_all_succeeded_scc[algorithm] == 0:
            mean_performance_all_succeeded_scc[algorithm].append(float("NaN"))
        else:
            mean_performance_all_succeeded_scc[algorithm].append(
                performance_cumulative_all_succeeded_scc[algorithm]
                / success_count_all_succeeded_scc
            )
        # Max edge list length
        if len(interval_edge_list_lengths_all_succeeded_scc[algorithm]) == 0:
            max_edge_list_length_all_succeeded_scc[algorithm].append(float("NaN"))
        else:
            max_edge_list_length_all_succeeded_scc[algorithm].append(
                max(interval_edge_list_lengths_all_succeeded_scc[algorithm])
            )

    # Add metrics to lists for single algo plots
    for algorithm in all_algos_mapping:
        # Success rate
        success_rate[algorithm].append(success_count[algorithm] / iteration_count)
        # print("Success rate: " + str(success_rate[algorithm][-1]))

        # Mean performance
        if performance_cumulative[algorithm] == 0:
            mean_performance[algorithm].append(float("NaN"))
        else:
            mean_performance[algorithm].append(
                performance_cumulative[algorithm] / success_count[algorithm]
            )
        # print("Mean performance: " + str(mean_performance[algorithm][-1]))

        # Max, min, mean edge list length
        if algorithm != "GR":
            if len(interval_edge_list_lengths[algorithm]) == 0:
                max_edge_list_lengths[algorithm].append(float("NaN"))
                min_edge_list_lengths[algorithm].append(float("NaN"))
                mean_edge_list_lengths[algorithm].append(float("NaN"))
            else:
                max_edge_list_lengths[algorithm].append(
                    max(interval_edge_list_lengths[algorithm])
                )
                min_edge_list_lengths[algorithm].append(
                    min(interval_edge_list_lengths[algorithm])
                )
                mean_edge_list_lengths[algorithm].append(
                    np.mean(interval_edge_list_lengths[algorithm])
                )


########################################################################################
################################### JSON EXPORTS #######################################
########################################################################################

total_execution_time = (time.process_time_ns() - total_execution_start) / 10**9
print("Total execution time: " + str(total_execution_time) + " seconds")

# End results
results_summary = {}
for algorithm in all_algos_mapping:
    results_summary[algorithm] = {
        "success_rate": success_rate[algorithm][-1],
        "mean_performance": performance_cumulative[algorithm]
        / success_count[algorithm],
        "average_runtime_ms": total_runtime[algorithm] / k,
    }
results_summary["total_execution_time_s"] = total_execution_time
print(results_summary)
with open("results/results_summary.json", "w") as output_file:
    json.dump(results_summary, output_file, indent=2)

# Export results as JSON
for algorithm_directory in all_algos_mapping:
    FILE_PATH = f"results/{algorithm_directory}/results.json"
    with open(FILE_PATH, "w") as output_file:
        json.dump(results[algorithm_directory], output_file, indent=2)

########################################################################################
####################################### PLOTS ##########################################
########################################################################################

# Mean performance of each algorithm
for algorithm in mean_performance:
    ci = scipy.stats.norm.interval(
        0.95,
        loc=np.nanmean(mean_performance[algorithm]),
        scale=scipy.stats.sem(mean_performance[algorithm], nan_policy="omit"),
    )
    margin_of_error = (ci[1] - ci[0]) / 2
    fig, ax = plt.subplots()
    # mean = np.mean(mean_performance[algorithm])
    # stdev = np.std(mean_performance[algorithm])
    # confidence_interval = 1.96 * stdev / np.sqrt(len(mean_performance[algorithm]))
    ax.plot(
        network_density_list,
        mean_performance[algorithm],
        label=algorithm,
    )
    ax.fill_between(
        network_density_list,
        mean_performance[algorithm] - margin_of_error,
        mean_performance[algorithm] + margin_of_error,
        alpha=0.1,
    )
    ax.set(
        xlabel="Network Density",
        ylabel="Mean Performance",
        title=f"Mean Performance - {algorithm}",
    )
    fig.savefig(f"results/{algorithm}/mean_performance.png")

# Mean performance where all algos succeeded
fig, ax = plt.subplots()
for algorithm in face_routing_mapping:
    ci = scipy.stats.norm.interval(
        0.95,
        loc=np.nanmean(mean_performance_all_succeeded[algorithm]),
        scale=scipy.stats.sem(
            mean_performance_all_succeeded[algorithm], nan_policy="omit"
        ),
    )
    margin_of_error = (ci[1] - ci[0]) / 2
    ax.plot(
        network_density_list,
        mean_performance_all_succeeded[algorithm],
        label=algorithm,
    )
    # ax.errorbar(
    #     network_density_list, mean_performance_all_succeeded[algorithm], yerr=margin_of_error, fmt="none", alpha=0.5
    # )
    ax.fill_between(
        network_density_list,
        (mean_performance_all_succeeded[algorithm] - margin_of_error),
        (mean_performance_all_succeeded[algorithm] + margin_of_error),
        alpha=0.1,
    )
ax.set(
    xlabel="Network Density",
    ylabel="Mean Performance",
    title="Mean Performance - All Succeeded",
)
ax.legend()
fig.savefig("results/all_algos_test/mean_performance_all_succeeded.png")

# Mean performance where all algos succeeded scc
fig, ax = plt.subplots()
for algorithm in scc_test_mapping:
    ci = scipy.stats.norm.interval(
        0.95,
        loc=np.nanmean(mean_performance_all_succeeded_scc[algorithm]),
        scale=scipy.stats.sem(
            mean_performance_all_succeeded_scc[algorithm], nan_policy="omit"
        ),
    )
    margin_of_error = (ci[1] - ci[0]) / 2
    ax.plot(
        network_density_list,
        mean_performance_all_succeeded_scc[algorithm],
        label=algorithm,
    )
    # ax.errorbar(
    #     network_density_list, mean_performance_all_succeeded[algorithm], yerr=margin_of_error, fmt="none", alpha=0.5
    # )
    ax.fill_between(
        network_density_list,
        (mean_performance_all_succeeded_scc[algorithm] - margin_of_error),
        (mean_performance_all_succeeded_scc[algorithm] + margin_of_error),
        alpha=0.1,
    )
ax.set(
    xlabel="Network Density",
    ylabel="Mean Performance",
    title="Mean Performance - All Succeeded",
)
ax.legend()
fig.savefig("results/scc_test/mean_performance_all_succeeded_scc.png")

# Max edge list length where all algos succeeded
fig, ax = plt.subplots()
for algorithm in face_routing_mapping:
    ax.plot(
        network_density_list,
        max_edge_list_length_all_succeeded[algorithm],
        label=algorithm,
    )
    if not np.isnan(max_edge_list_length_all_succeeded[algorithm]).any():
        ci = scipy.stats.norm.interval(
            0.99,
            loc=np.nanmean(max_edge_list_length_all_succeeded[algorithm]),
            scale=scipy.stats.sem(
                max_edge_list_length_all_succeeded[algorithm], nan_policy="omit"
            ),
        )
        margin_of_error = (ci[1] - ci[0]) / 2
        ax.fill_between(
            network_density_list,
            max_edge_list_length_all_succeeded[algorithm] - margin_of_error,
            max_edge_list_length_all_succeeded[algorithm] + margin_of_error,
            alpha=0.1,
        )
ax.set(
    xlabel="Network Density",
    ylabel="Max Edge List Length",
    title="Max Edge List Length - All Succeeded",
)
ax.legend()
fig.savefig("results/all_algos_test/max_edge_list_length_all_succeeded.png")

# Max edge list length where all algos succeeded scc
fig, ax = plt.subplots()
for algorithm in scc_test_mapping:
    ax.plot(
        network_density_list,
        max_edge_list_length_all_succeeded_scc[algorithm],
        label=algorithm,
    )
    if not np.isnan(max_edge_list_length_all_succeeded_scc[algorithm]).any():
        ci = scipy.stats.norm.interval(
            0.99,
            loc=np.nanmean(max_edge_list_length_all_succeeded_scc[algorithm]),
            scale=scipy.stats.sem(
                max_edge_list_length_all_succeeded_scc[algorithm], nan_policy="omit"
            ),
        )
        margin_of_error = (ci[1] - ci[0]) / 2
        ax.fill_between(
            network_density_list,
            max_edge_list_length_all_succeeded_scc[algorithm] - margin_of_error,
            max_edge_list_length_all_succeeded_scc[algorithm] + margin_of_error,
            alpha=0.1,
        )
ax.set(
    xlabel="Network Density",
    ylabel="Max Edge List Length",
    title="Max Edge List Length - All Succeeded",
)
ax.legend()
fig.savefig("results/scc_test/max_edge_list_length_all_succeeded_scc.png")

# Max, min, mean edge list length of each algorithm
for algorithm in face_routing_mapping_scc:
    fig, ax = plt.subplots()
    ax.plot(
        network_density_list,
        max_edge_list_lengths[algorithm],
        label="Maximum",
    )
    ax.plot(
        network_density_list,
        mean_edge_list_lengths[algorithm],
        label="Mean",
    )
    ax.plot(
        network_density_list,
        min_edge_list_lengths[algorithm],
        label="Minimum",
    )
    ax.fill_between(
        network_density_list,
        max_edge_list_lengths[algorithm],
        min_edge_list_lengths[algorithm],
        alpha=0.1,
    )
    ax.set(
        xlabel="Network Density",
        ylabel="Edge List Length",
        title=f"Edge List Length - {algorithm}",
    )
    ax.legend()
    fig.savefig(f"results/{algorithm}/edge_list_length.png")

# Success rate
fig, ax = plt.subplots()
for algorithm in normal_test_mapping:
    ax.plot(network_density_list, success_rate[algorithm], label=algorithm)
ax.set(xlabel="Network Density", ylabel="Success Rate", title="Success Rate")
ax.legend()
fig.savefig("results/all_algos_test/success_rate.png")

# Success rate scc
fig, ax = plt.subplots()
for algorithm in scc_test_mapping:
    ax.plot(network_density_list, success_rate[algorithm], label=algorithm)
ax.set(xlabel="Network Density", ylabel="Success Rate", title="Success Rate")
ax.legend()
fig.savefig("results/scc_test/success_rate_scc.png")

# Mean performance/success rate (efficiency)
efficiency = {
    algorithm: [
        mean_performance[algorithm][i] / success_rate[algorithm][i]
        if not math.isnan(mean_performance[algorithm][i])
        else float("NaN")
        for i in range(len(mean_performance[algorithm]))
    ]
    for algorithm in normal_test_mapping
}
fig, ax = plt.subplots()
for algorithm in normal_test_mapping:
    ax.plot(
        network_density_list,
        efficiency[algorithm],
        label=algorithm,
    )
ax.set(
    xlabel="Network Density",
    ylabel="Efficiency",
    title="Efficiency",
)
ax.legend()
fig.savefig("results/all_algos_test/efficiency.png")

# Mean performance/success rate (efficiency) scc
efficiency_scc = {
    algorithm: [
        mean_performance[algorithm][i] / success_rate[algorithm][i]
        if not math.isnan(mean_performance[algorithm][i])
        else float("NaN")
        for i in range(len(mean_performance[algorithm]))
    ]
    for algorithm in scc_test_mapping
}
fig, ax = plt.subplots()
for algorithm in scc_test_mapping:
    ax.plot(
        network_density_list,
        efficiency_scc[algorithm],
        label=algorithm,
    )
ax.set(
    xlabel="Network Density",
    ylabel="Efficiency",
    title="Efficiency",
)
ax.legend()
fig.savefig("results/scc_test/efficiency_scc.png")

# Export plot data as JSON
for algorithm in all_algos_mapping:
    plot_data = {
        "algorithm": algorithm,
        "network_density_list": network_density_list,
        "mean_performance": mean_performance[algorithm]
        if algorithm in mean_performance
        else [],
        "mean_performance_all_succeeded": mean_performance_all_succeeded[algorithm]
        if algorithm in mean_performance_all_succeeded
        else [],
        "mean_performance_all_succeeded_scc": mean_performance_all_succeeded_scc[
            algorithm
        ]
        if algorithm in mean_performance_all_succeeded_scc
        else [],
        "max_edge_list_length_all_succeeded": max_edge_list_length_all_succeeded[
            algorithm
        ]
        if algorithm in max_edge_list_length_all_succeeded
        else [],
        "max_edge_list_length_all_succeeded_scc": max_edge_list_length_all_succeeded_scc[
            algorithm
        ]
        if algorithm in max_edge_list_length_all_succeeded_scc
        else [],
        "max_edge_list_lengths": max_edge_list_lengths[algorithm]
        if algorithm in max_edge_list_lengths
        else [],
        "mean_edge_list_lengths": mean_edge_list_lengths[algorithm]
        if algorithm in mean_edge_list_lengths
        else [],
        "min_edge_list_lengths": min_edge_list_lengths[algorithm]
        if algorithm in min_edge_list_lengths
        else [],
        "success_rate": success_rate[algorithm] if algorithm in success_rate else [],
        "efficiency": efficiency[algorithm] if algorithm in efficiency else [],
        "efficiency_scc": efficiency_scc[algorithm]
        if algorithm in efficiency_scc
        else [],
    }
    FILE_PATH = f"results/{algorithm}/plot_data.json"
    with open(FILE_PATH, "w") as output_file:
        json.dump(plot_data, output_file, indent=2)
