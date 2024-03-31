from typing import Any, Callable, Dict
import random
import networkx as nx
import numpy as np
import scipy as sp
from itertools import permutations
import fnss
from util import require_args


class GraphGenerator:
    """
    This class generates a graph using a specified method and keyword arguments.
    """

    def __init__(self, method: Callable[[dict[str, Any]], tuple[int, int, nx.Graph | nx.DiGraph]], **kwargs):
        """
        Constructor
        @param method - the method to be used for graph generation
        @param kwargs - arguments that are inserted into method with each call to generate(). They can be modified with set_kwarg()
        """
        self.method = method
        self.kwargs = kwargs

    def set_kwarg(self, **kwargs):
        """
        Update the kwargs supplied to method in every generate() call
        @param kwargs - Argument(s) to update + new values
        """
        self.kwargs.update(kwargs)

    def generate(self) -> tuple[int, int, nx.Graph | nx.DiGraph]:
        """
        Generates a graph using method and kwargs. Initialises node["attr"] and edge["attr"] with 0 and edge["failed"] with False
        @return A tuple containing the start node, end node, and the generated graph.
        """
        s, d, g = self.method(**self.kwargs)
        nx.set_edge_attributes(g, "0", "attr")
        nx.set_node_attributes(g, "0", "attr")
        nx.set_edge_attributes(g, False, "failed")
        g.nodes[s]["attr"] = "s"
        g.nodes[d]["attr"] = "d"
        g.graph["root"] = d
        return s, d, g


def graph_gen(func: Callable[[dict[str, Any]], tuple[int, int, nx.Graph | nx.DiGraph]]) -> Callable[[dict[str, Any]], GraphGenerator]:
    """
    Decorator. Embeds the function into a GraphGenerator-object for use in Evaluation.eval()
    @param func - the function to generate a graph with
    @return a new function that returns a GraphGenerator-object
    """
    def wrapper(**kwargs):
        return GraphGenerator(func, **kwargs)
    return wrapper


@graph_gen
@require_args(["n", "p", "directed"])
def erdos_renyi(**kwargs) -> tuple[int, int, nx.DiGraph]:
    """
    Generates a random directed Erdos-Renyi graph with a given number of nodes and probability of edge creation.
    @param **kwargs - n: number of nodes, p: probability of creation for each possible edge, directed: output directed/undirected graph
    @return a tuple containing the source node, destination node, and the generated graph.
    """
    n = kwargs.get("n")
    p = kwargs.get("p")
    g = nx.erdos_renyi_graph(n, p, directed=kwargs.get("directed"))
    nodes = list(g.nodes)
    s, d = random.sample(nodes, 2)
    return s, d, g


@graph_gen
@require_args(["n", "d"])
def random_undirected(**kwargs) -> tuple[int, int, nx.Graph]:
    """
    Generates a random graph with a given number of nodes and node degrees.
    @param **kwargs - n: number of nodes, d: degree of each node. n*d must be even!
    @return a tuple containing the source node, destination node, and the generated graph.
    """
    n = kwargs.get("n")
    d = kwargs.get("d")
    g = nx.random_regular_graph(d, n, seed=None)
    nodes = list(g.nodes)
    s, d = random.sample(nodes, 2)
    return s, d, g


@graph_gen
@require_args(["n", "d", "p"])
def random_directed(**kwargs) -> tuple[int, int, nx.DiGraph]:
    """
    Generates a random directed graph with a given number of nodes and node degrees.
    @param **kwargs - n: number of nodes, d: degree of each node. n*d must be even!, p: fraction of edges to be directed in a random direction
    @return a tuple containing the source node, destination node, and the generated graph.
    """
    n = kwargs.get("n")
    d = kwargs.get("d")
    factor = kwargs.get("p")
    s, d, g = random_undirected(d=d, n=n).generate()
    l = list(g.edges)
    dir_edges = random.sample(l, int(len(l)*factor))
    g_dir = g.to_directed()
    for edge in dir_edges:
        g_dir.remove_edge(edge[0], edge[1])
    return s, d, g_dir

def validate_input(G, pos_name):
    # Input validation - every node must have a "pos" attribute
    for n, pos in G.nodes(data=pos_name):
        if pos is None:
            raise nx.NetworkXError(
                f"Node {n} (and all nodes) must have a '{pos_name}' attribute."
            )

def directed_rdg_edges(G, p=2, *, pos_name="pos") -> list:
    """
    Returns directional edge list of node pairs, if v is within radius of u.

    Parameters
    ----------
    G : networkx graph
        The graph from which to generate the edge list. The nodes in `G` should
        have an attribute ``pos`` corresponding to the node position, which is
        used to compute the distance to other nodes.
    pos_name : string, default="pos"
        The name of the node attribute which represents the position of each
        node in 2D coordinates. Every node in the Graph must have this attribute.
    p : scalar, default=2
        The `Minkowski distance metric
        <https://en.wikipedia.org/wiki/Minkowski_distance>`_ used to compute
        distances. The default value is 2, i.e. Euclidean distance.

    Returns
    -------
    edges : list
        List of edges whose distances are less than radius.
    """

    nodes_data = G.nodes(data=True)

    try:
        import scipy as sp
    except ImportError:
        print("scipy is not available, cannot use directed_rdg_edges")

    # Parse node positions and radii
    node_list, data = list(zip(*nodes_data))
    radius_list = [sample['rad'] for sample in data]
    position_list = [sample['pos'] for sample in data]

    kdtree = sp.spatial.KDTree(position_list)

    edges = []
    for node in node_list:
        edge_indexes = kdtree.query_ball_point(position_list[node], radius_list[node], p)
        # Filter out node itself from edge_indexes
        edge_indexes.remove(node)
        for neighbor in edge_indexes:
            # Check if directed edge already exists
            if (node, neighbor) not in edges:
                edges.append((node, neighbor))

    return edges

def directed_random_disk_graph(
    n, radius_lower_bound=0.1, radius_upper_bound=1, position_lower_bound=0, position_upper_bound=5, dim=2, pos=None,
    p=2, seed=None, *, pos_name="pos") -> nx.DiGraph:
    """
    Returns a directed random disk graph in the cube of dimensions 'dim'.

    The random geometric graph model places 'n' nodes uniformly at
    random in the unit cube. Two nodes are joined by an edge if the
    distance between the nodes is at most 'radius'.

    Parameters
    ----------
    n : int or iterable
        Number of nodes or iterable of nodes.
    radius_lower_bound : float, optional
        Lower bound of the transfer radius of each node.
    radius_upper_bound : float, optional
        Upper bound of the transfer radius of each node.
    position_lower_bound : float, optional
        Lower bound of the position of each node.
    position_upper_bound : float, optional
        Upper bound of the position of each node.
    dim : int, optional
        Dimension of graph.
    pos : dict, optional
        A dictionary keyed by node with node positions as values.
    p : float, optional
        Which Minkowski distance metric to use.  `p` has to meet the condition
        ``1 <= p <= infinity``.

        If this argument is not specified, the :math:`L^2` metric
        (the Euclidean distance metric), p = 2 is used.
        This should not be confused with the `p` of an Erdős-Rényi random
        graph, which represents probability.
    seed : integer, random_state, or None (default)
        Indicator of random number generation state.
    pos_name : string, default="pos"
        The name of the node attribute which represents the position
        in 2D coordinates of the node in the returned graph.

    Returns
    -------
    DiGraph
        A random disk graph, directed and without self-loops.
        Each node has a node attribute 'pos' that stores the
        position of that node in Euclidean space as provided by the
        'pos' keyword argument or, if 'pos' was not provided, as
        generated by this function.
    """

    G = nx.empty_graph(n, create_using=nx.DiGraph)

    # Set a random transfer radius for each node
    radii = {v: random.uniform(radius_lower_bound, radius_upper_bound) for v in G}
    nx.set_node_attributes(G, radii, name="rad")

    # If no positions are provided, choose uniformly random vectors in Euclidean space of the specified dimension.
    random.seed(seed)
    if pos is None:
        pos = {v: [random.uniform(position_lower_bound, position_upper_bound) for i in range(dim)] for v in G}
    nx.set_node_attributes(G, pos, pos_name)

    G.add_edges_from(directed_rdg_edges(G, p, pos_name=pos_name))
    return G

def delaunay_triangulation_edges(rdg: nx.DiGraph) -> list:
    """
    Returns directional edge list of the Delaunay triangulation of the nodes in `rdg`.

    Parameters
    ----------
    rdg : nx.DiGraph
        Random disk graph generated from directed_random_disk_graph().

    Returns
    -------
    edge_list : list
        List of edges computed from the Delaunay triangulation of the nodes in `rdg`.
    """

    nodes_data = rdg.nodes(data=True)
    nodes, data = list(zip(*nodes_data))
    coords = [list(data[n]['pos']) for n in nodes]
    tri = sp.spatial.Delaunay(points=coords)
    # Create edge list from the simplices
    edge_list = []
    for simplex in tri.simplices:
        for edge in permutations(simplex, 2):
            edge_list.append(edge)
    return edge_list

def random_planar_graph(
    n, radius_lower_bound=0.1, radius_upper_bound=1, position_lower_bound=0, position_upper_bound=5, dim=2, pos=None,
    p=2, seed=None, *, pos_name="pos") -> nx.DiGraph:
    """
    Returns a random planar graph by computing the Delaunay triangulation from a directed random disk graph.

    Parameters
    ----------
    See directed_random_disk_graph().

    Returns
    -------
    directed_planar_rdg : nx.DiGraph
        A random planar graph, based on a random disk graph and made planar by computing the Delaunay triangulation.
    """

    # Generate a directed rdg
    directed_rdg = directed_random_disk_graph(
        n, radius_lower_bound, radius_upper_bound, position_lower_bound, position_upper_bound, dim, pos, p, seed,
        pos_name=pos_name)
    # Compute the Delaunay triangulation of the nodes
    delaunay_edges = delaunay_triangulation_edges(directed_rdg)
    # Filter out all edges that are not part of the triangulation
    for edge in list(directed_rdg.edges):
        if edge not in delaunay_edges:
            directed_rdg.remove_edge(edge[0], edge[1])

    # Return the resulting graph
    directed_planar_rdg = directed_rdg
    return directed_planar_rdg

@graph_gen
@require_args(["filename"])
def rocketfuel(**kwargs) -> tuple[int, int, nx.Graph]:
    """
    Loads a graph from the rocketfuel project and chooses two random nodes as source and destination.
    @param **kwargs - filename: full path to the graph file to be loaded
    @return a tuple containing the source node, destination node, and the loaded graph.
    """
    g = nx.Graph()
    g.add_edges_from(fnss.parse_rocketfuel_isp_map(
        kwargs.get("filename")).edges())
    nx.convert_node_labels_to_integers(g)
    s, d = random.sample(list(g.nodes), 2)
    return s, d, g


@graph_gen
@require_args(["filename"])
def topology_zoo(**kwargs) -> tuple[int, int, nx.Graph]:
    """
    Loads a graph from the topology zoo project and chooses two random nodes as source and destination.
    @param **kwargs - filename: full path to the graph file to be loaded
    @return a tuple containing the source node, destination node, and the loaded graph.
    """
    g = nx.read_graphml(kwargs.get("filename"))
    s, d = random.sample(list(g.nodes), 2)
    return s, d, g


@graph_gen
@require_args(["n", "degree_dist"])
def random_directed_var_degree(**kwargs) -> tuple[int, int, nx.DiGraph]:
    """
    Generates a random directed graph with a given number of nodes and a probability distribution of node degrees.
    @param **kwargs - n: number of nodes, degree_dist: distribution of node degrees [(degree, prob), ...]
    @return a tuple containing the source node, destination node, and the loaded graph.
    """
    n = kwargs.get("n")
    options, dist = list(zip(*kwargs.get("degree_dist")))
    g = nx.DiGraph()
    nodes = list(range(n))
    g.add_nodes_from(nodes)
    random.shuffle(nodes)
    s = nodes[0]
    d = nodes[-1]
    # Add a random path from s to d visiting all nodes
    g.add_edges_from(zip(nodes, nodes[1:]))
    # Add random outgoing edges to every node, in a number according to prob. dist.
    for node in nodes:
        node_c = nodes.copy()
        node_c.remove(node)  # No self-loops
        n_edges = np.random.choice(options, 1, dist)[0]
        if node != d:
            # This node already has an outgoing edge
            # No second edge to neighbor
            node_c.remove(list(g.neighbors(node))[0])
            n_edges -= 1
        for other in random.sample(node_c, n_edges):
            g.add_edge(node, other)
    return s, d, g


@graph_gen
def heathland_undir(**kwargs) -> tuple[int, int, nx.Graph]:
    """
    Get the graph from the heatland experiment (see Thesis). The graph contains only undirected edges.
    @return a tuple containing the source node, destination node, and the graph.
    """
    g = nx.Graph()
    g.add_edges_from([(0, 1), (0, 7), (0, 13), (2, 13), (3, 13), (2, 3), (2, 6),
                     (3, 4), (4, 6), (4, 5), (2, 12), (7, 8), (7, 11), (8, 9), (9, 10), (10, 11)])
    s = random.randint(0, 12)
    return s, 13, g


@graph_gen
def heathland(**kwargs) -> tuple[int, int, nx.DiGraph]:
    """
    Get the graph from the heatland experiment (see Thesis). The graph contains directed and undirected edges.
    @return a tuple containing the source node, destination node, and the graph.
    """
    g = nx.Graph()
    g.add_edges_from([(0, 1), (0, 7), (0, 13), (2, 13), (3, 13), (2, 3), (2, 6),
                     (3, 4), (4, 6), (4, 5), (2, 12), (7, 8), (7, 11), (8, 9), (9, 10), (10, 11)])
    g = g.to_directed()
    g.add_edges_from([(1, 7), (7, 2), (13, 5), (4, 1), (3, 6),
                     (6, 5), (5, 11), (11, 6), (7, 6), (12, 8), (12, 9), (12, 10)])
    s = random.randint(0, 12)
    return s, 13, g


@graph_gen
@require_args(["s", "d", "edges"])
def fixed_graph(**kwargs) -> tuple[int, int, nx.Graph]:
    """
    Generate a graph with edges 
    @param kwargs - s: source node, d: destination node, edges: list of edges, graph_type (optional,default="dir"): "dir" or "undir"
    @return a tuple containing the source node, destination node, and the graph.
    """
    if kwargs.get("graph_type", "dir") == "dir":
        g = nx.DiGraph()
    else:
        g = nx.Graph()
    g.add_edges_from(kwargs.get("edges"))
    return kwargs.get("s"), kwargs.get("d"), g
