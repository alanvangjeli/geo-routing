# Based on https://gitlab.cs.univie.ac.at/ct-papers/fast-failover/-/blob/master/extra_links.py
# - Modified to work on directed graphs (DFS-Traversal, routing assumes no existence of reverse edge)

import sys
import networkx as nx
from RoutingAlgos.BonsaiImpl import *


def KeepForwardingPrecomputation(g: nx.Graph) -> None:
    """
    Precomputations for Keep Forwarding Routing. Precomputation is saved in g.graph["precomp"]
    @param g: Graph to build routes on
    """
    nx.set_edge_attributes(g, False, "is_a_link")
    nx.set_edge_attributes(g, False, "used_a_link")
    nx.set_edge_attributes(g, 1, "attr")  # So every edge gets drawn as used
    d = g.graph['root']
    n = len(g.nodes())
    dist = nx.shortest_path_length(g, target=d)
    edge_weight = {}
    node_weight = {v: 0 for v in g.nodes()}
    dist_nodes = {i: set() for i in range(n)}  # nodes with distance i
    dist_nodes[sys.maxsize] = set()  # nodes that can't reach destination
    down_links = {v: set() for v in g.nodes()}
    A_links = {v: set() for v in g.nodes()}
    up_links = {v: set() for v in g.nodes()}
    # Annotate with distances
    for (u, v) in g.edges():
        if not u in dist:
            # Destination not reachable, set high dist to avoid this node being used
            dist[u] = sys.maxsize
        if not v in dist:
            # Destination not reachable, set high dist to avoid this node being used
            dist[v] = sys.maxsize
        if u == v:
            continue
        if dist[u] > dist[v]:
            edge_weight[(u, v)] = n*n
            down_links[u].add(v)
        elif dist[u] == dist[v]:
            edge_weight[(u, v)] = n
            A_links[u].add(v)
            g[u][v]["is_a_link"] = True
        elif dist[u] < dist[v]:
            edge_weight[(u, v)] = 1
            up_links[u].add(v)
        node_weight[u] += edge_weight[(u, v)]
        dist_nodes[dist[u]].add(u)
        dist_nodes[dist[v]].add(v)
    label = {}  # eulerian label
    label_size = {}  # eulerian size
    for k, v in dist_nodes.items():  # distance, all nodes with equal distance
        if len(v) < 2:
            continue
        subgraph = g.subgraph(v)
        for component in nx.strongly_connected_components(subgraph):
            count = 0
            try:
                for (u, v) in nx.eulerian_circuit(g.subgraph(component)):
                    if not g.has_edge(u, v):
                        continue
                    g[u][v]["used_a_link"] = True
                    # Just to highlight a-links when drawing the graph
                    g[u][v]["attr"] = 2
                    label[(u, v)] = count
                    count += 1
                    label_size[(u, v)] = g.subgraph(
                        component).number_of_edges()
            except:  # No eulerian circuit available. Should only happen on directed graphs
                # DFS
                dfs_edges = [(u, v) if label == "forward" else (v, u) for (
                    u, v, label) in nx.dfs_labeled_edges(g.subgraph(component)) if label != 'nontree' and u != v]
                for (u, v) in dfs_edges:
                    if not g.has_edge(u, v):
                        continue
                    try:  # In case DFS uses non-existent edges
                        g[u][v]["used_a_link"] = True
                        # Just to highlight a-links when drawing the graph
                        g[u][v]["attr"] = 2
                    except:
                        pass
                    label[(u, v)] = count
                    count += 1
                    label_size[(u, v)] = g.subgraph(
                        component).number_of_edges()

    # Undirected graphs use every a-link. On directed graphs, delete all unused
    for (key, val) in A_links.items():
        for v in val:
            if not g[key][v]["used_a_link"]:
                g[key][v]["attr"] = 0  # Mark unused edges with attr = 0
    A_links = {key: [v for v in val if g[key][v]["used_a_link"]]
               for (key, val) in A_links.items()}
    g.graph["precomp"] = [label_size, label, edge_weight,
                          node_weight, down_links, A_links, up_links]


def KeepForwardingRouting(s: int, d: int, g: nx.Graph) -> tuple[bool, int, list[tuple[int, int]]]:
    """
    Route
    @param s - source node
    @param d - destination node
    @param g - graph to route on
    @return (found, hops, route (as edge list))
    """
    fails = []
    for edge in g.edges:
        data = g.get_edge_data(*edge)
        if data['failed']:
            fails.append((edge[0], edge[1]))

    [label_size, label, edge_weight, node_weight,
        down_links, A_links, up_links] = g.graph["precomp"]

    hops = 0

    # add edges taken to this list when the first failure has been encountered...
    detour_edges = []
    n = len(g.nodes())
    incoming_link = (s, s)
    incoming_node = s
    while (s != d):
        # remove incoming node from all link lists
        curr_dl = list(down_links[s])
        if incoming_node in curr_dl:
            curr_dl.remove(incoming_node)
        curr_al = list(A_links[s])
        if incoming_node in curr_al:
            curr_al.remove(incoming_node)
        curr_ul = list(up_links[s])
        if incoming_node in curr_ul:
            curr_ul.remove(incoming_node)

        # sort up/down according to weights (higher->earlier) and a-list according to labels (lower->earlier)  #maybe refactor to only sort if there is a failure to speed  up...
        curr_dl = sorted(curr_dl, key=lambda x: int(
            node_weight[x]), reverse=True)
        curr_al = sorted(curr_al, key=lambda x: int(
            label[(s, x)]), reverse=False)
        curr_ul = sorted(curr_ul, key=lambda x: int(
            node_weight[x]), reverse=True)
        # init for a links
        a_overflow = 0  # counter, to try all a-links only once
        a_count = -1  # init counter for label of link (for safety)
        # if incoming was a-link, get correct counter
        if incoming_link in list(A_links[incoming_node]):
            a_count = label[incoming_link]  # get label from incoming link
        elif len(curr_al) > 0:
            a_count = label[(s, curr_al[0])]
        if len(curr_dl) > 0:  # if down list is not empty, set nxt as first element of down list
            curr_list = curr_dl
            nxt = curr_list[0]
            curr_index = 0  # 0 for down, 1 for A, 2 for up
        elif len(curr_al) > 0:  # if a list is not empty, set nxt as next a link: if incoming is a-link, then next, else, as first element of down list
            curr_list = curr_al
            # if incoming was a link
            if incoming_link in list(A_links[incoming_node]):
                # increase counter by 1
                a_count = (a_count+1) % label_size[incoming_link]
                nxt = next(i for i in list(
                    A_links[s]) if label[(s, i)] == a_count)
            else:  # if incoming was not a-link
                nxt = curr_list[0]
                a_count = label[(s, curr_list[0])]
            curr_index = 1  # 0 for down, 1 for A, 2 for up
        elif len(curr_ul) > 0:  # if a list is not empty, set nxt as first element of down list
            curr_list = curr_ul
            nxt = curr_list[0]
            curr_index = 2  # 0 for down, 1 for A, 2 for up
        else:  # note: this should not happen, as we did not yet check if the next link is failed, but added for good measure...
            if g.has_edge(s, incoming_node):
                nxt = incoming_node
            else:
                nxt = s
            curr_index = 3

        while (s, nxt) in fails:  # or (nxt, s) in fails:
            if curr_index == 0:  # down_links usage
                if curr_list.index(nxt) < len(curr_list)-1:  # are there elements left?
                    if incoming_node != curr_list[curr_list.index(nxt)+1]:
                        # next item from down_links
                        nxt = curr_list[curr_list.index(nxt)+1]
                elif a_count > -1:
                    curr_index = 1
                    curr_list = curr_al
                    if a_count == label[(s, curr_al[0])]:
                        nxt = next(i for i in list(
                            A_links[s]) if label[(s, i)] == a_count)
                    else:
                        # increase counter by 1
                        a_count = (a_count+1) % label_size[incoming_link]
                        nxt = next(i for i in list(
                            A_links[s]) if label[(s, i)] == a_count)
                else:
                    curr_index = 2
                    curr_list = curr_ul  # list(up_links[s])
                    if len(curr_list) > 0:
                        nxt = curr_list[0]
                    else:
                        if g.has_edge(s, incoming_node):
                            nxt = incoming_node
                        else:
                            nxt = s
                        curr_index = 3
            elif curr_index == 1:
                if a_overflow < label_size[(s, curr_list[0])]:
                    a_overflow = a_overflow+1
                    if curr_list.index(nxt) < len(curr_list)-1:
                        nxt = next(i for i in list(curr_list)
                                   if label[(s, i)] > a_count)
                        a_count = label[(s, nxt)]
                    else:
                        nxt = curr_list[0]
                        a_count = label[(s, nxt)]
                else:
                    curr_index = 2
                    curr_list = curr_ul  # list(up_links[s])
                    if len(curr_list) > 0:
                        nxt = curr_list[0]
                    else:
                        if g.has_edge(s, incoming_node):
                            nxt = incoming_node
                        else:
                            nxt = s
                        curr_index = 3
            elif curr_index == 2:  # up_links usage
                if curr_list.index(nxt) < len(curr_list)-1:  # are there elements left?
                    if incoming_node != curr_list[curr_list.index(nxt)+1]:
                        # next item from down_links
                        nxt = curr_list[curr_list.index(nxt)+1]
                    else:
                        curr_index = 3
                        nxt = incoming_node
                else:
                    if g.has_edge(s, incoming_node):
                        nxt = incoming_node
                    else:
                        nxt = s
                    curr_index = 3
            else:
                return (False, hops, detour_edges)

        if s == nxt:  # Self loop: Found no valid edge to send the packet
            return (False, hops, detour_edges)
        detour_edges.append((s, nxt))
        hops += 1
        n_end = n*n+20
        if hops > n_end:  # n*n*n:  #to kill early, later set back to n*n*n
            # probably a loop, return
            return (False, hops, detour_edges)
        incoming_link = (s, nxt)
        incoming_node = s
        s = nxt
    return (True, hops, detour_edges)
