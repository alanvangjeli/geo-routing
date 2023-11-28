import GraphGenerators as gg
import FailureGenerators as fg
import RoutingAlgorithms as ra
import Evaluation

f_05 = [.0, .1, .2, .3, .4, .5]
f_08 = [.0, .1, .2, .3, .4, .5, .6, .7, .8]
f_09 = [.0, .1, .2, .3, .4, .5, .6, .7, .8, .9]
dir_er_30_35 = gg.erdos_renyi(n=50, p=0.35, directed=True)
dir_er_50_35 = gg.erdos_renyi(n=50, p=0.35, directed=True)
dir_er_50_60 = gg.erdos_renyi(n=50, p=0.6, directed=True)
undir_er_50_35 = gg.erdos_renyi(n=50, p=0.35, directed=False)


# Kapitel 5.3.2
def plot_tree():
    # Abb 5.4
    Evaluation.eval(100, "failure_rate", f_08, [Evaluation.Packetloss, Evaluation.Stretch],
                    undir_er_50_35, fg.random_failure(), [ra.TREE("single"), ra.TREE("multiple")])   
    # Abb 5.5
    Evaluation.eval(100, "p", f_09, [Evaluation.EdgesUsable], dir_er_50_35, fg.no_failure(), [
                    ra.TREE("single"), ra.TREE("multiple"), ra.Bonsai("greedy")])
    # Abb 5.6
    Evaluation.eval_fr(100, "failure_rate", f_05, [Evaluation.Packetloss, Evaluation.Stretch], dir_er_50_35, fg.random_failure(), [
                       ra.TREE("single"), ra.TREE("multiple"), ra.TREE("multiple", switching=True)])
    # Abb 5.7
    Evaluation.eval_fr(100, "failure_rate", f_08, [Evaluation.Packetloss], dir_er_50_60, fg.random_failure(), [
                       ra.TREE("single"), ra.TREE("multiple"), ra.TREE("multiple", switching=True)])
    # Abb 5.8
    Evaluation.eval_fr(500, "failure_rate", f_05, [Evaluation.Packetloss], gg.random_directed(
        n=50, d=8, p=0.4), fg.random_failure(), [ra.TREE("multiple"), ra.TREE("multiple", tree_choice="undirected")])
    Evaluation.eval_fr(500, "failure_rate", f_05, [Evaluation.Packetloss], gg.random_directed(
        n=50, d=8, p=0.4), fg.random_failure(undir_fail=True), [ra.TREE("multiple"), ra.TREE("multiple", tree_choice="undirected")])


# Kapitel 5.3.3
def plot_bonsai():
    # Abb 5.9
    Evaluation.eval_fr(500, "failure_rate", f_08, [Evaluation.Packetloss, Evaluation.Stretch], dir_er_50_35, fg.random_failure(), [
        ra.Bonsai(), ra.Bonsai("round-robin"), ra.Bonsai("random")])
    # Abb 5.10
    Evaluation.eval_fr(500, "failure_rate", f_08, [Evaluation.Packetloss, Evaluation.Stretch], dir_er_30_35, fg.random_failure(), [
        ra.Bonsai(), ra.Bonsai("round-robin"), ra.Bonsai("random")])


# Kapitel 5.3.4
def plot_grafting():
    # Abb 5.11
    Evaluation.eval(100, "p", f_05, [Evaluation.EdgesUsable], gg.erdos_renyi(
        p=0, n=30, directed=True), fg.no_failure(), [ra.EDP(), ra.TREE("single"), ra.Bonsai(), ra.Grafting()])
    # Abb 5.12
    Evaluation.eval_fr(100, "failure_rate", f_08, [Evaluation.Packetloss, Evaluation.Stretch], gg.erdos_renyi(
        p=0.4, n=20, directed=True), fg.random_failure(), [ra.Bonsai(), ra.Grafting(), ra.Grafting(method="Cluster")])


# Kapitel 5.3.5
def plot_kf():
    # Abb. 5.13
    Evaluation.eval_fr(500, "failure_rate", f_08, [Evaluation.Packetloss, Evaluation.Stretch], gg.erdos_renyi(
        n=50, p=0.35, directed=True), fg.random_failure(), [ra.KeepForwarding(), ra.Bonsai("round-robin"), ra.TREE("multiple")])


# Kapitel 5.3.6
def plot_fcp():
    # Abb 5.14
    g_generator = gg.random_directed_var_degree(
        n=10, degree_dist=[(1, 0.3), (2, 0.05), (3, 0.05), (4, 0.5), (5, 0.1)])
    Evaluation.eval(1000, "failure_rate", f_05, [Evaluation.Packetloss, Evaluation.Stretch, Evaluation.Runtime], g_generator, fg.random_failure(), [ra.FailureCarrying(), ra.FailureCarrying(
        measure="path_div", ranking="max_min"), ra.FailureCarrying(measure="path_div", ranking="greedy_max"), ra.FailureCarrying(measure="degree", ranking="greedy_max")])
    # Abb 5.15
    Evaluation.eval_fr(1000, "failure_rate", f_05, [Evaluation.Packetloss, Evaluation.Stretch], gg.heathland(), fg.random_failure(), [ra.FailureCarrying(), ra.FailureCarrying(
        measure="path_div", ranking="max_min"), ra.FailureCarrying(measure="path_div", ranking="greedy_max"), ra.FailureCarrying(measure="degree", ranking="greedy_max")])
    # Abb 5.16
    Evaluation.eval(10, "n", [5, 7, 9, 11, 13, 15, 17, 19], [Evaluation.Runtime], gg.erdos_renyi(n=0, p=0.15, directed=True), fg.random_failure(
        failure_rate=0.5), [ra.EDP(), ra.Bonsai(), ra.FailureCarrying(measure="path_div", ranking="max_min")])


# Kapitel 5.3.7
def plot_all():
    iterations = 500
    algos = [ra.TREE("multiple"), ra.Bonsai("round-robin"),
             ra.Grafting(), ra.KeepForwarding()]
    hard_algos = [ra.FailureCarrying(), ra.TREE("multiple", switching=True)]
    failure = fg.random_failure()
    undir_failure = fg.random_failure(undir_fail=True)
    graphs_dir = [gg.random_directed(n=25, d=4, p=0.1), gg.random_directed(n=25, d=4, p=0.3), gg.random_directed(n=25, d=4, p=0.75), gg.erdos_renyi(
        n=25, p=0.35, directed=True), gg.erdos_renyi(n=25, p=0.6, directed=True), gg.erdos_renyi(n=50, p=0.35, directed=True), gg.heathland()]
    graphs_dir_names = ["rnd_d_25_4_10", "rnd_d_25_4_30", "rnd_d_25_4_75",
                        "er_25_35_d", "er_25_60_d", "er_50_35_d", "heatland"]

    aggs = [Evaluation.Packetloss, Evaluation.Runtime,
            Evaluation.Stretch, Evaluation.Loops, Evaluation.EdgesUsable]

    for graph, name in zip(graphs_dir, graphs_dir_names):
        # To measure runtime switch to eval()!
        Evaluation.eval_fr(iterations, "failure_rate", f_05, aggs,
                           graph, failure, algos, filename="results/a.fg-d.gg-" + name)
        Evaluation.eval_fr(iterations, "failure_rate", f_05, aggs, graph,
                           undir_failure, algos, filename="results/a.fg-ud.gg-" + name)
        Evaluation.eval_fr(iterations, "failure_rate", f_05, aggs, graph,
                           failure, hard_algos, filename="results/ha.fg-d.gg-" + name)
        Evaluation.eval_fr(iterations, "failure_rate", f_05, aggs, graph,
                           undir_failure, hard_algos, filename="results/ha.fg-ud.gg-" + name)


if __name__ == "__main__":
    plot_tree()
    plot_bonsai()
    plot_grafting()
    plot_kf()
    plot_fcp()
    plot_all()
