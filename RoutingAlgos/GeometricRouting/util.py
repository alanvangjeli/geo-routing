import numpy as np


def check_last_edge(prev_node, cur_node, face_nodes, half_edges, s, d):
    result_tag = ResultTag.DEFAULT
    if cur_node is None:
        # Dead end
        # print('Face could not be traversed completely due to a dead end in ' + str(prev_node))
        result_tag = ResultTag.DEAD_END
    else:
        if cur_node in face_nodes and cur_node != s:
            # Node was already visited
            # print('Node ' + str(cur_node) + ' was already encountered')
            half_edges.append((prev_node, cur_node))
            result_tag = ResultTag.NODE_ENCOUNTERED
        else:
            face_nodes.add(cur_node)
            half_edges.append((prev_node, cur_node))
            # print('Half edge: ' + str((prev_node, cur_node)))
            if cur_node == d:
                # Destination was reached
                result_tag = ResultTag.SUCCESS
    return face_nodes, half_edges, result_tag


def forward_search(half_edges, current_node, closest_node):
    # Iterate until current node is encountered again
    current_node_index = len(half_edges) - 1
    for i in range(len(half_edges)):
        if half_edges[i][0] == current_node:
            current_node_index = i
    # Iterate until closest node is reached
    route = []
    for i in range(current_node_index, len(half_edges)):
        route.append(half_edges[i][1])
        if half_edges[i][1] == closest_node:
            return closest_node, route
    # Closest node not reached
    return current_node, route


def backward_search(half_edges, current_node, closest_node, g):
    # Iterate until current node is encountered again
    current_node_index = len(half_edges) - 1
    for i in range(len(half_edges)):
        if half_edges[i][0] == current_node:
            current_node_index = i
    # Iterate until closest node is reached
    route = []
    for edge in reversed(half_edges[0:current_node_index]):
        if edge[0] in g.neighbors(edge[1]):
            current_node = edge[0]
            route.append(current_node)
            if current_node == closest_node:
                break
        else:
            # print('No directed edge from ' + str(edge[1]) + ' to ' + str(edge[0]) + '.')
            break
    # Closest node not reached
    return current_node, route


def append_to_edges_and_face(prev_node, cur_node, face_nodes, half_edges):
    face_nodes.add(cur_node)
    half_edges.append((prev_node, cur_node))
    return prev_node, cur_node


def calculate_angle_ccw(vector_a, vector_b):
    """Calculate angle of vector_b counterclockwise from vector_a"""
    # Measures angle of first vector point_a clockwise from the y-axis at coordinate x of vector point_a
    ang_a = np.arctan2(*vector_a[0::])
    # Measures angle of vector point_b clockwise from the y-axis at coordinate x of vector point_a
    ang_b = np.arctan2(*vector_b[0::])
    # Sum up angles and convert to degrees
    return np.rad2deg((ang_a - ang_b) % (2 * np.pi))


def calculate_angle_cw(vector_a, vector_b):
    """Calculate angle of vector_b clockwise from vector_a"""
    # Measures angle of first vector point_a counterclockwise from the y-axis at coordinate x of vector point_a
    ang_a = np.arctan2(*vector_a[::-1])
    # Measures angle of vector point_b counterclockwise from the y-axis at coordinate x of vector point_a
    ang_b = np.arctan2(*vector_b[::-1])
    # Sum up angles and convert to degrees
    return np.rad2deg((ang_a - ang_b) % (2 * np.pi))


class ResultTag:
    DEAD_END: str = "Dead-end"
    LOOP: str = "Stuck in a loop"
    LOCAL_MINIMUM: str = "Local minimum"
    SUCCESS: str = "Destination was reached"
    RECURSION_LIMIT: str = "Recursion limit was reached"
    FACE: str = "Face was traversed"
    NO_PROGRESS: str = "Current closest node is the same as in the previous iteration"
    NODE_ENCOUNTERED: str = "Node was already encountered"
    EDGE_ENCOUNTERED: str = "Edge was already traversed two times"
    DEFAULT: str = "Catch-all for other cases"
    SECOND_BOUND_HIT: str = "Bound was hit for the second time"
    CONDITION_2B: str = "Condition 2b is true"
    NO_SCC_WITH_S_D: str = "There is no SCC containing s and d"


RECURSION_DEPTH_LIMIT = 50
