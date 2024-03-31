import matplotlib
from matplotlib.patches import Ellipse

from RoutingAlgos.geomentricRouting.FaceRouting import calculate_angle_ccw
from RoutingAlgos.geomentricRouting.archive.OFR_archived import *
from util import ResultTag

def create_ellipse(pos_s: tuple, pos_d: tuple) -> matplotlib.patches.Ellipse:
    distance_s_d = distance.euclidean(pos_s, pos_d)
    width = 2 * distance_s_d
    height = 2 * np.sqrt(width ** 2 - distance_s_d ** 2 / 4)
    centre = ((pos_s[0] + pos_d[0]) / 2, (pos_s[1] + pos_d[1]) / 2)

    vector_a = (pos_d[0] - pos_s[0], 0)
    vector_b = (pos_d[0] - pos_s[0], pos_d[1] - pos_s[1])
    angle = calculate_angle_ccw(vector_a, vector_b)

    ellipse = Ellipse(centre, width, height, angle=angle)
    return ellipse

def other_adaptive_face_route(g: nx.DiGraph, s: int, d: int, route: list,
                              positions: dict) -> tuple[bool, list[int], str]:

    ellipse = create_ellipse(positions[s], positions[d])
    result, route, ellipse, result_tag = _other_adaptive_face_route(g, s, d, route, positions, [], ellipse, False)
    return result, route, result_tag

def _other_adaptive_face_route(g: nx.DiGraph, s: int, d: int, route: list, positions: dict, previous_face: list,
                               ellipse: Ellipse, traverse_one_face=False) -> tuple[bool, list[int], Ellipse, str]:

    result, route, result_tag = other_bounded_face_route(g, s, d, route, positions, previous_face, None, ellipse, traverse_one_face)
    if result:
        return result, route, ellipse, result_tag
    else:
        if result_tag == ResultTag.DEAD_END or result_tag == ResultTag.LOOP:
            return False, route, ellipse, result_tag
        node_positions = list(positions.values())
        # If any of the nodes is not inside the ellipse, then double the major axis (width) and continue search
        if not any(ellipse.contains_points(node_positions)) and not traverse_one_face:
            ellipse.set_width(ellipse.width * 2)
            return _other_adaptive_face_route(g, s, d, route, positions, previous_face, ellipse, traverse_one_face)
        # Otherwise all nodes are inside ellipse and still route not discovered
        else:
            # OBFR delivers ResultTag.NO_PROGRESS even though all nodes are inside ellipse
            return False, route, ellipse, ResultTag.LOOP

def other_bounded_face_route(g: nx.DiGraph, s: int, d: int, route: list, positions: dict, previous_face: list, previous_closest_node, ellipse: Ellipse, traverse_one_face=False) -> tuple[bool, list, str]:
    """
    Other Face Routing OFR
    @param g - Graph to route on
    @param s - Source node
    @param d - Destination node
    @param route - Route to destination
    @param positions - Positions of nodes
    @param previous_face - Previous face
    @param previous_closest_node - Previous closest node
    @param ellipse - Ellipse that bounds face traversal
    @param traverse_one_face - Whether to traverse only one face
    """
    if s == d:
        return True, route, ResultTag.SUCCESS

    # Initialize values
    half_edges = []
    face_starting_node = s
    if len(route) != 0:
        if route[-1] != face_starting_node:
            route.append(face_starting_node)
    else:
        route.append(face_starting_node)
    neighbors = [node for node in g.neighbors(face_starting_node)]

    # Dead end
    if len(neighbors) == 0:
        print('Dead end')
        return False, route, ResultTag.DEAD_END
    # Only one neighbor
    if len(neighbors) == 1:
        print('Only neighbor: ' + str(neighbors[0]))
        return other_face_route(g, neighbors[0], d, route, positions, previous_face)
    # Multiple neighbors, take first edge ccw of line sd
    else:
        # TODO: Put all this code in traverse_face
        ######################################################################################################################################################
        neighbors = [node for node in g.neighbors(face_starting_node)]
        angles = {node: calculate_angle_ccw(np.subtract(positions[d], positions[s]),
                                            np.subtract(positions[node], positions[s])) for node in neighbors}
        sorted_angles = sorted(angles.items(), key=itemgetter(1))
        print('Angles: ' + str(angles))
        min_angle_neighbor, _ = sorted_angles[0]
        print('Min angle neighbor: ' + str(min_angle_neighbor))
        current_face = traverse_face(g, face_starting_node, min_angle_neighbor, d, positions,
                                     mark_half_edges=half_edges)
        for edge in half_edges:
            route.append(edge[1])
        #######################################################################################################################################################
        if half_edges[-1][1] == d:
            print('Destination was reached')
            return True, route, ResultTag.SUCCESS

    print('Current face: ' + str(current_face))
    print('Half edges: ' + str(half_edges))
    # Detect if stuck in a loop
    if current_face == previous_face:
        print('Stuck in a loop')
        return False, route, ResultTag.LOOP
    else:
        previous_face = current_face

    # Route back to node closest to destination
    distances = {node: distance.euclidean(positions[node], positions[d]) for node in current_face}
    closest_node, _ = min(distances.items(), key=itemgetter(1))
    print('Closest node: ' + str(closest_node))
    last_node_reached, route_back = face_route(g, half_edges, face_starting_node, closest_node, ellipse, positions)
    # OBFR
    if last_node_reached == closest_node:
        if closest_node == previous_closest_node:
            print('Closest node was already encountered')
            return False, route, ResultTag.NO_PROGRESS
        else:
            previous_closest_node = closest_node
    route.extend(route_back)
    if not traverse_one_face:
        return other_bounded_face_route(g, last_node_reached, d, route, positions, previous_face,
                                        previous_closest_node, ellipse)
    else:
        return False, route, ResultTag.FACE

def traverse_face(g, v, w, positions, ellipse, mark_half_edges=None) -> list:
    """Returns nodes on the face that belong to the half-edge (v, w).

    The face that is traversed lies to the right of the half-edge (in an
    orientation where v is below w).

    Optionally it is possible to pass a list to which all encountered half
    edges are added.

    Parameters
    ----------
    g : DiGraph
    v : node
        Start node of half-edge. Face starting node.
    w : node
        End node of half-edge.
    positions : dict
        Node positions.
    ellipse : matplotlib.patches.Ellipse
        Ellipse that bounds face traversal.
    mark_half_edges: list, optional
        List to which all encountered half-edges are added.

    Returns
    -------
    face : list
        A list of nodes that lie on this face.
    """
    if mark_half_edges is None:
        mark_half_edges = []

    face_nodes = [v, w]
    prev_node = v
    cur_node = w
    mark_half_edges.append((prev_node, cur_node))
    print('Half edge: ' + str((prev_node, cur_node)))

    ellipse_hit, prev_node, cur_node = next_face_half_edge(g, prev_node, cur_node, positions, ellipse, 'ccw')
    while not ellipse_hit:
        if (prev_node, cur_node) in mark_half_edges:
            print('Half edge: ' + str((prev_node, cur_node)))
            print('Half-edge was already encountered')
            break
        if cur_node is None:
            # Dead end
            print('Face could not be traversed completely due to a dead end')
            break
        if cur_node not in face_nodes:
            face_nodes.append(cur_node)
        # mark_half_edges is a list which preserves order of how face was traversed
        mark_half_edges.append((prev_node, cur_node))
        print('Half edge: ' + str((prev_node, cur_node)))
        if cur_node == v:
            # Face traversal is complete
            break
        ellipse_hit, prev_node, cur_node = next_face_half_edge(g, prev_node, cur_node, positions, ellipse, 'ccw')
    if ellipse_hit:
        print('Ellipse was hit for the first time by this edge: ' + str((cur_node, prev_node)))
        mark_half_edges.append((cur_node, prev_node))
        mark_half_edges.append((prev_node, cur_node))
        # Continue exploration in opposite direction
        ellipse_hit, prev_node, cur_node = next_face_half_edge(g, prev_node, cur_node, positions, ellipse, 'cw')
        while not ellipse_hit:
            if (prev_node, cur_node) in mark_half_edges:
                print('Half-edge was already encountered')
                break
            if cur_node is None:
                # Dead end
                print('Face could not be traversed completely due to a dead end')
                break
            if cur_node not in face_nodes:
                face_nodes.append(cur_node)
            # mark_half_edges is a list which preserves order of how face was traversed
            mark_half_edges.append((prev_node, cur_node))
            print('Half edge: ' + str((prev_node, cur_node)))
            ellipse_hit, prev_node, cur_node = next_face_half_edge(g, prev_node, cur_node, positions, ellipse, 'cw')
        if ellipse_hit:
            print('Ellipse was hit for the second time by this edge: ' + str((cur_node, prev_node)))
            mark_half_edges.append((cur_node, prev_node))
            mark_half_edges.append((prev_node, cur_node))

    return face_nodes

def next_face_half_edge(g, v, w, positions, ellipse, order) -> tuple[bool, int, None]:
    """Returns the following half-edge ccw of (v, w).

    Parameters
    ----------
    g : DiGraph
    v : node
        Start node of half-edge.
    w : node
        End node of half-edge.
    positions : dict
        Node positions.
    ellipse : matplotlib.patches.Ellipse
        Ellipse that bounds face traversal.
    order : str
        'ccw' or 'cw'

    Returns
    -------
    half-edge : tuple
    """
    neighbors_directed = [node for node in g.neighbors(w)]
    # Filter out previous node from neighbors_directed, otherwise this is the smallest angle 0
    if v in neighbors_directed:
        neighbors_directed.remove(v)
    if len(neighbors_directed) == 0:
        # Dead end
        return False, w, None
    elif len(neighbors_directed) == 1:
        new_node = neighbors_directed[0]
    else:
        # Calculate inner angles ccw
        if order == 'ccw':
            angles = {node: calculate_angle_ccw(np.subtract(positions[w], positions[v]), np.subtract(positions[w], positions[node])) for node in neighbors_directed}
        else:
            angles = {node: calculate_angle_cw(np.subtract(positions[w], positions[v]), np.subtract(positions[w], positions[node])) for node in neighbors_directed}
        sorted_angles = sorted(angles.items(), key=itemgetter(1))
        #print('Sorted angles: ' + str(sorted_angles))
        min_angle_neighbor, min_angle = sorted_angles[0]
        new_node = min_angle_neighbor
    if ellipse.contains_point(positions[new_node]):
        return False, w, new_node
    else:
        # Ellipse was hit
        return True, new_node, w

def face_route(g, half_edges: list, face_starting_node: int, route_to_node: int, ellipse: matplotlib.patches.Ellipse, positions: dict) -> tuple[int, list]:
    """
    Route back from face_starting_node to route_to_node after face traversal.
    @param g - Graph to route on
    @param half_edges - Half edges of face traversal
    @param face_starting_node - Face starting node
    @param route_to_node - Node to route to
    @param ellipse - Ellipse that bounds face traversal
    @param positions - Positions of nodes

    Returns last node reached and route back to route_to_node
    """
    route = []
    current_node = half_edges[-1][1]
    if half_edges[-1][1] == face_starting_node:
        # Face was traversed completely
        for edge in half_edges:
            current_node = edge[1]
            if ellipse.contains_point(positions[edge[1]]):
                if edge[1] == route_to_node:
                    break
                route.append(edge[1])
    else:
        # Route back from current node to closest node
        for edge in reversed(half_edges):
            if edge[0] in g.neighbors(edge[1]):
                if ellipse.contains_point(positions[edge[0]]):
                    current_node = edge[0]
                    if edge[0] == route_to_node:
                        break
                    route.append(edge[0])
            else:
                print('No route back to route_to_node.')
                break
    return current_node, route
