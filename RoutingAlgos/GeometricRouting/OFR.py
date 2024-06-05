from operator import itemgetter

import networkx as nx
import numpy as np
from scipy.spatial import distance

from RoutingAlgos.GeometricRouting.util import (
    RECURSION_DEPTH_LIMIT,
    ResultTag,
    append_to_edges_and_face,
    backward_search,
    calculate_angle_ccw,
    calculate_angle_cw,
    forward_search,
)


class OFR:
    def __init__(
        self, graph: nx.DiGraph, start: int, destination: int, positions: dict
    ):
        """
        @param graph - Graph to route on
        @param start - Source node
        @param destination - Destination node
        @param positions - Positions of nodes
        """
        self.g = graph
        self.s = start
        self.d = destination
        self.positions = positions
        self.route = [start]
        self.previous_face = set()
        self.previous_closest_node = start
        self.recursion_depth = 0
        self.edge_list_lengths = []

    def find_route(self) -> tuple[bool, list[int], str, int]:
        """
        Other Face Routing OFR
        """
        if self.s == self.d:
            return True, self.route, ResultTag.SUCCESS, self.edge_list_lengths

        neighbors = [node for node in self.g.neighbors(self.s)]

        # Dead end
        if len(neighbors) == 0:
            return False, self.route, ResultTag.DEAD_END, self.edge_list_lengths

        # Multiple neighbors, take first edge ccw of line sd
        else:
            # Only one neighbor
            if len(neighbors) == 1:
                # print('Only neighbor: ' + str(neighbors[0]))
                pass
            current_face, half_edges, result_tag = self.traverse_face()
            self.edge_list_lengths.append(len(half_edges))
            # Edge case, bound was hit twice from s
            if len(half_edges) == 0:
                current_node = self.s
            else:
                current_node = half_edges[-1][1]
            for edge in half_edges:
                self.route.append(edge[1])
            if result_tag == ResultTag.DEAD_END:
                return False, self.route, ResultTag.DEAD_END, self.edge_list_lengths
            if current_node == self.d:
                return True, self.route, ResultTag.SUCCESS, self.edge_list_lengths

        # print('Current face: ' + str(current_face))
        # print('Half edges: ' + str(half_edges))
        # Loop detection
        if current_face == self.previous_face:
            return False, self.route, ResultTag.LOOP, self.edge_list_lengths
        else:
            self.previous_face = current_face

        # Route to node closest to destination
        last_node_reached, path_last_node_reached = self.route_to_closest_node(
            current_node, current_face, half_edges
        )
        # print('Last node reached: ' + str(last_node_reached))
        self.s = last_node_reached
        self.route.extend(path_last_node_reached)
        if self.recursion_depth < RECURSION_DEPTH_LIMIT:
            self.recursion_depth += 1
            return self.find_route()
        else:
            return False, self.route, ResultTag.RECURSION_LIMIT, self.edge_list_lengths

    def traverse_face(self) -> tuple[set, list, str]:
        """Returns nodes on the face that is intersected by sd.

        Returns
        -------
        face : set
            A set of nodes that lie on this face.
        """
        half_edges = []
        face_nodes = set([self.s])
        result_tag = ResultTag.DEFAULT

        # Find first neighbor ccw of line sd
        min_angle_neighbor = self.get_first_neighbor_ccw()
        prev_node, cur_node = append_to_edges_and_face(
            self.s, min_angle_neighbor, face_nodes, half_edges
        )
        # print('Half edge: ' + str((prev_node, cur_node)))
        if cur_node == self.d:
            # Destination was reached
            return face_nodes, half_edges, ResultTag.SUCCESS

        # Iterate until face starting node (self.s) is reached
        while cur_node != self.s:
            prev_node, cur_node = self.next_face_half_edge(prev_node, cur_node, "ccw")
            # print('Current node Face Routing: ' + str(cur_node))
            face_nodes, half_edges, result_tag = self.check_last_edge(
                prev_node, cur_node, face_nodes, half_edges
            )
            if result_tag != ResultTag.DEFAULT:
                break

        return face_nodes, half_edges, result_tag

    def check_last_edge(self, prev_node, cur_node, face_nodes, half_edges):
        result_tag = ResultTag.DEFAULT
        if cur_node is None:
            # Dead end
            # print('Face could not be traversed completely due to a dead end in ' + str(prev_node))
            result_tag = ResultTag.DEAD_END
        else:
            if cur_node in face_nodes and cur_node != self.s:
                # Node was already visited
                # print('Node ' + str(cur_node) + ' was already encountered')
                half_edges.append((prev_node, cur_node))
                result_tag = ResultTag.NODE_ENCOUNTERED
            else:
                face_nodes.add(cur_node)
                half_edges.append((prev_node, cur_node))
                # print('Half edge: ' + str((prev_node, cur_node)))
                if cur_node == self.d:
                    # Destination was reached
                    result_tag = ResultTag.SUCCESS
        return face_nodes, half_edges, result_tag

    def get_first_neighbor_ccw(self):
        neighbors = [node for node in self.g.neighbors(self.s)]
        angles = {
            node: calculate_angle_ccw(
                np.subtract(self.positions[self.d], self.positions[self.s]),
                np.subtract(self.positions[node], self.positions[self.s]),
            )
            for node in neighbors
        }
        sorted_angles = sorted(angles.items(), key=itemgetter(1))
        min_angle_neighbor, _ = sorted_angles[0]
        return min_angle_neighbor

    def next_face_half_edge(self, v, w, order) -> tuple[int, int | None]:
        """Returns the following half-edge ccw of (v, w).

        Parameters
        ----------
        v : node
            Start node of half-edge.
        w : node
            End node of half-edge.
        order : str
            'ccw' or 'cw'

        Returns
        -------
        half-edge : tuple
        """
        neighbors_directed = [node for node in self.g.neighbors(w)]
        if len(neighbors_directed) == 0:
            # Dead end
            return w, None
        else:
            # Calculate inner angles ccw or cw
            if order == "ccw":
                angles = {
                    node: calculate_angle_ccw(
                        np.subtract(self.positions[w], self.positions[v]),
                        np.subtract(self.positions[w], self.positions[node]),
                    )
                    for node in neighbors_directed
                }
            else:
                angles = {
                    node: calculate_angle_cw(
                        np.subtract(self.positions[w], self.positions[v]),
                        np.subtract(self.positions[w], self.positions[node]),
                    )
                    for node in neighbors_directed
                }
            # Make angle for incoming edge 360, otherwise this is the smallest angle 0
            if v in angles:
                angles[v] = 360
            sorted_angles = sorted(angles.items(), key=itemgetter(1))
            # print('Sorted angles: ' + str(sorted_angles))
            min_angle_neighbor, _ = sorted_angles[0]
            new_node = min_angle_neighbor
            return w, new_node

    def route_to_closest_node(
        self, current_node: int, current_face: set, half_edges
    ) -> tuple[int, list[int]]:
        """
        Route to the closest node to d after face traversal.
        @param current_node - Source node
        @param current_face - Current face
        @param half_edges - Half edges of face traversal
        """
        # Find closest node to destination
        distances = {
            node: distance.euclidean(self.positions[node], self.positions[self.d])
            for node in current_face
        }
        closest_node, _ = min(distances.items(), key=itemgetter(1))
        # print('Closest node: ' + str(closest_node))

        last_node_reached, route = self.search_half_edges(
            current_node, half_edges, closest_node
        )
        return last_node_reached, route

    def search_half_edges(
        self, current_node: int, half_edges: list, closest_node: int
    ) -> tuple[int, list]:
        """
        Route back from current_node to closest_node after face traversal.
        @param current_node - Current node
        @param half_edges - Half edges of face traversal
        @param closest_node - Node to route to

        Returns last node reached and route back to route_to_node
        """

        route = []
        # print('Current node: ' + str(current_node))
        if current_node != closest_node:
            if current_node == self.s:
                # Face was traversed completely, repeat edges taken during face traversal
                for edge in half_edges:
                    current_node = edge[1]
                    route.append(current_node)
                    if current_node == closest_node:
                        return current_node, route
            else:
                # NOT ORIGINALLY IN OFR
                # Case: node was already visited
                forward_search_node, forward_search_route = forward_search(
                    half_edges, current_node, closest_node
                )
                if forward_search_node == closest_node:
                    return closest_node, forward_search_route
                else:
                    # print('Forward search failed.')
                    return backward_search(
                        half_edges, current_node, closest_node, self.g
                    )
        else:
            return current_node, route
