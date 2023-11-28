import abc
import networkx as nx
from Info import *

from RoutingAlgos import BonsaiImpl, TREEImpl, PATHImpl, KeepForwardingImpl, FailureCarryingImpl, GraftingImpl


class RoutingAlgo:
    """
    Base class for routing algorithms.
    """

    def __init__(self):
        self.kwargs = {}

    def set_kwarg(self, **kwargs):
        """
        Update the kwargs supplied to method in every generate() call
        @param kwargs - Argument(s) to update + new values
        """
        self.kwargs.update(kwargs)

    def build_routes(self, g: nx.Graph, s: int, d: int) -> RouteBuildInfo:
        """
        Build routes for destination d
        @param g - Graph to prepare for routing
        @param s - Source node
        @param d - Destination node
        @return RouteBuildInfo
        """
        pass

    @abc.abstractmethod
    def route(self, marked_g: nx.Graph, s: int, d: int) -> RoutingInfo:
        """
        Simulate routing from s to d
        @param marked_g - Graph to route on, with markings generated by build_routes()
        @param s - Source node
        @param d - Destination node
        @return RoutingInfo
        """
        pass

    @abc.abstractmethod
    def get_name(self) -> str:
        """
        @return Name of the routing algorithm to be shown in the plots.
        """
        pass


def has_loop(path: list[tuple[int, int]]) -> bool:
    return path.index(path[-1]) != len(path) - 1 if len(path) > 0 else False


class EDP(RoutingAlgo):
    """
    Edge Disjoint Paths. Try to route on EDPs from s to d. Upon encountering an error, send the packet back to s and try another EDP.
    """

    def build_routes(self, g: nx.Graph, s: int, d: int) -> RouteBuildInfo:
        """
        Build routes, marked with edge["attr"] in the returned graph
        @param g - graph to build the routes on
        @param s - source node
        @param d - destination node
        @return RouteBuildInfo
        """
        g = PATHImpl.kResiliencePaths(g, s, d)
        return RouteBuildInfo(True, g)

    def route(self, marked_g: nx.Graph, s: int, d: int) -> RoutingInfo:
        """
        Simulate routing from s to d
        @param marked_g - Graph to route on, with markings generated by build_routes()
        @param s - source node
        @param d - destination node
        @return RoutingInfo
        """
        found, hops, route = PATHImpl.routing_kResiliencePaths(marked_g, s, d)
        return RoutingInfo(found, hops, route)

    def get_name(self) -> str:
        """
        @return Name of the routing algorithm to be shown in the plots.
        """
        return "EDPs"


class TREE(RoutingAlgo):
    """
    Extend EDPs to trees to route on
    """

    def __init__(self, method="multiple", tree_choice="shortest", switching=False):
        """
        init
        @param method - "single"/"multiple"
        @param tree_choice - "shortest"/"undirected"
        @param switching - switch trees at nodes other than s
        """
        self.method = method
        self.tree_choice = tree_choice
        self.switching = switching
        super().__init__()

    def build_routes(self, g: nx.Graph, s: int, d: int) -> RouteBuildInfo:
        """
        Build routes, marked with edge["attr"] in the returned graph
        @param g - graph to build the routes on
        @param s - source node
        @param d - destination node
        @return RouteBuildInfo
        """
        _g = TREEImpl.build_routes(
            g.to_directed(), s, d, version=self.method, treeChoice=self.tree_choice)
        return RouteBuildInfo(True, _g)

    def route(self, marked_g: nx.Graph, s: int, d: int) -> RoutingInfo:
        """
        Simulate routing from s to d
        @param marked_g - Graph to route on, with markings generated by build_routes()
        @param s - Source node
        @param d - Destination node
        @return RoutingInfo
        """
        found, hops, route, additional = TREEImpl.route(
            marked_g, s, d, self.switching, self.tree_choice, version=self.method)

        return RoutingInfo(found, hops, route, additional=additional)

    def get_name(self) -> str:
        """
        @return Name of the routing algorithm to be shown in the plots.
        """
        name = ""
        if self.tree_choice == "shortest":
            name = f"TREE - {self.method}"
        else:
            name = f"TREE - {self.method}, ranking: undir"
        if self.switching:
            name += " switched"
        return name


class Bonsai(RoutingAlgo):
    """
    Bonsai. Routing using edge disjoint, spanning arborescences.
    """

    def __init__(self, method="greedy", cut=False, swap=False):
        """
        @param method - "greedy"/"random"/"round-robin"
        @param cut - Only round-robin: Check edge connectivity on unsused graph after every arb construction
        @param swap - Only round-robin: Swap edges already used in arbs to possibly fix deadlocks in construction
        """
        super().__init__()
        self.method = method
        self.cut = cut
        self.swap = swap

    def build_routes(self, g: nx.Graph, s: int, d: int) -> RouteBuildInfo:
        """
        Build routes, marked with edge["attr"] in the returned graph
        @param g - graph to build the routes on
        @param s - source node
        @param d - destination node
        @return RouteBuildInfo
        """
        g.graph['root'] = d
        g = g.to_directed()
        if self.method == "greedy":
            arbs = BonsaiImpl.GreedyArborescenceDecomposition(g)
        elif self.method == "random":
            arbs = BonsaiImpl.RandomTrees(g)
        elif self.method == "round-robin":
            arbs = BonsaiImpl.round_robin(g, cut=self.cut, swap=self.swap)
        for edge in g.edges:
            g[edge[0]][edge[1]]["attr"] += 1
        return RouteBuildInfo(True, g, additional={"arbs": arbs})

    def route(self, marked_g: nx.Graph, s: int, d: int) -> RoutingInfo:
        """
        Simulate routing from s to d
        @param marked_g - Graph to route on, with markings generated by build_routes()
        @param s - Source node
        @param d - Destination node
        @return RoutingInfo
        """
        fail, hops, _, detour_edges = BonsaiImpl.RouteDetCirc(s, d, marked_g)
        loop = has_loop(detour_edges)
        return RoutingInfo(not fail, hops, detour_edges, loop=loop)

    def get_name(self) -> str:
        """
        @return Name of the routing algorithm to be shown in the plots.
        """
        return f"Bonsai - {self.method}"


class KeepForwarding(RoutingAlgo):
    """
    Greedy routing in the direction of d.
    """

    def build_routes(self, g: nx.Graph, s: int, d: int) -> RouteBuildInfo:
        """
        Build routes, marked with edge["attr"] in the returned graph
        @param g - graph to build the routes on
        @param s - source node
        @param d - destination node
        @return RouteBuildInfo
        """
        g = g.to_directed()  # Always use directed graphs with KF
        g.graph['root'] = d
        KeepForwardingImpl.KeepForwardingPrecomputation(g)
        return RouteBuildInfo(True, g)

    def route(self, marked_g: nx.Graph, s: int, d: int) -> RoutingInfo:
        """
        Simulate routing from s to d
        @param marked_g - Graph to route on, with markings generated by build_routes()
        @param s - Source node
        @param d - Destination node
        @return RoutingInfo
        """
        result = KeepForwardingImpl.KeepForwardingRouting(
            s, d, marked_g)
        detour_edges = result[2]
        loop = has_loop(detour_edges)
        return RoutingInfo(*result, loop=loop)

    def get_name(self) -> str:
        """
        @return Name of the routing algorithm to be shown in the plots.
        """
        return "KeepForwarding"


class FailureCarrying(RoutingAlgo):
    """
    Failure Carrying Packets. Every node has a full network map. Failures get saved to the packet. Route gets regenerated at every node.
    """

    def __init__(self, measure="length", ranking="greedy_max"):
        """
        Init
        @param measure - "length"/"path_div"/"degree"
        @param ranking - "greedy_max"/"max_min"
        """
        super().__init__()
        self.measure = measure
        self.ranking = ranking

    def build_routes(self, g: nx.Graph, s: int, d: int) -> RouteBuildInfo:
        """
        Build routes (actually do nothing, because FailureCarrying builds the routes while routing)
        @param g - graph to build the routes on
        @param s - source node
        @param d - destination node
        @return RouteBuildInfo
        """
        return RouteBuildInfo(True, g)

    def route(self, marked_g: nx.Graph, s: int, d: int) -> RoutingInfo:
        """
        Simulate routing from s to d
        @param marked_g - Graph to route on, with markings generated by build_routes()
        @param s - Source node
        @param d - Destination node
        @return RoutingInfo
        """
        found, hops, detour_edges = FailureCarryingImpl.route(
            marked_g, s, d, self.ranking, self.measure)
        detour_edges = list(zip(detour_edges, detour_edges[1:]))
        loop = has_loop(detour_edges)
        return RoutingInfo(found, hops, detour_edges, loop=loop)

    def get_name(self) -> str:
        """
        @return Name of the routing algorithm to be shown in the plots.
        """
        if self.measure == "length":
            return "FailureCarrying"
        else:
            return f"FailureCarrying {self.ranking} {self.measure}"


class Grafting(RoutingAlgo):
    """
    Grafting. Based on Bonsai, supplies three different strategies to use more of the available edges than just spanning arbs.
    """

    def __init__(self, method="DAG-FRR"):
        """
        Constructor.
        @param method - "DAG-FRR": (non-spanning arbs expanded to DAGs); "Cluster": (clusters get their own local arbs);
        """
        super().__init__()
        self.method = method

    def build_routes(self, g: nx.Graph, s: int, d: int) -> RouteBuildInfo:
        """
        Build routes, marked with edge["attr"] in the returned graph
        @param g - graph to build the routes on
        @param s - source node
        @param d - destination node
        @return RouteBuildInfo
        """
        g.graph['root'] = d
        g = g.to_directed()
        if self.method == "DAG-FRR":
            GraftingImpl.DegreeMaxDAG(g)
        elif self.method == "Cluster":
            GraftingImpl.MaximizeFindClusters(g)
        for edge in g.edges:
            g[edge[0]][edge[1]]["attr"] += 1
        return RouteBuildInfo(True, g)

    def route(self, marked_g: nx.Graph, s: int, d: int) -> RoutingInfo:
        """
        Simulate routing from s to d
        @param marked_g - Graph to route on, with markings generated by build_routes()
        @param s - Source node
        @param d - Destination node
        @return RoutingInfo
        """
        fail, hops, detour_edges = GraftingImpl.RouteDetCircSkip(
            s, d, marked_g)
        loop = has_loop(detour_edges)
        return RoutingInfo(not fail, hops, detour_edges, loop=loop)

    def get_name(self) -> str:
        """
        @return Name of the routing algorithm to be shown in the plots.
        """
        return f"Grafting - {self.method}"
