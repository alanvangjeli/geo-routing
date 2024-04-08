from operator import itemgetter
import networkx as nx
import numpy as np
from scipy.spatial import distance
from util import ResultTag

class OtherFaceRouting:
    def __init__(self, graph: nx.DiGraph, start: int, destination: int, positions: dict):
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

    def find_route_ofr(self) -> tuple[bool, list, str]:
        """
        Other Face Routing OFR
        """
        if self.route[-1] != self.s:
            self.route.append(self.s)

        if self.s == self.d:
            return True, self.route, ResultTag.SUCCESS

        # Initialize values
        half_edges = []
        neighbors = [node for node in self.g.neighbors(self.s)]

        # Dead end
        if len(neighbors) == 0:
            return False, self.route, ResultTag.DEAD_END
        # Multiple neighbors, take first edge ccw of line sd
        else:
            # Only one neighbor
            if len(neighbors) == 1:
                print('Only neighbor: ' + str(neighbors[0]))
            current_face, half_edges, result_tag, condition_2c = self.traverse_face(half_edges=half_edges)
            current_node = half_edges[-1][1]
            for edge in half_edges:
                self.route.append(edge[1])
            # Condition 2b in GOAFR+
            if result_tag == ResultTag.SECOND_BOUND_HIT and self.is_goafr_plus():
                return self.goafr_plus_2b(current_node, half_edges, current_face)
            if condition_2c and self.is_goafr_plus():
                return self.goafr_plus_greedy_mode()
            if result_tag == ResultTag.DEAD_END:
                return False, self.route, ResultTag.DEAD_END
            if current_node == self.d:
                # Destination was reached
                return True, self.route, ResultTag.SUCCESS

        print('Current face: ' + str(current_face))
        print('Half edges: ' + str(half_edges))
        # Detect if stuck in a loop
        if current_face == self.previous_face:
            return False, self.route, ResultTag.LOOP
        else:
            self.previous_face = current_face

        # Route to node closest to destination
        last_node_reached, closest_node, path_last_node_reached = self.route_to_closest_node(current_node, current_face,
                                                                                             half_edges)
        self.route.extend(path_last_node_reached)

        # Check if last_node_reached is not the same as previous_closest_node in OBFR
        print("Previous closest node: " + str(self.previous_closest_node))
        if self.check_closest_node_progress(last_node_reached, closest_node):
            self.previous_closest_node = closest_node
        else:
            return False, self.route, ResultTag.NO_PROGRESS

        # In GOAFR only one face is traversed
        return self.terminate_face_routing_mode()

    def traverse_face(self, half_edges=None) -> tuple[set, list, str, bool]:
        """Returns nodes on the face that is intersected by sd.

        Parameters
        ----------
        half_edges: list, optional
            List to which all encountered half-edges are added.

        Returns
        -------
        face : set
            A set of nodes that lie on this face.
        """
        condition_2c = False
        if half_edges is None:
            half_edges = []
        result_tag = ''

        # Find first neighbor ccw of line sd
        neighbors = [node for node in self.g.neighbors(self.s)]
        angles = {node: calculate_angle_ccw(np.subtract(self.positions[self.d], self.positions[self.s]),
                                            np.subtract(self.positions[node], self.positions[self.s])) for node in neighbors}
        sorted_angles = sorted(angles.items(), key=itemgetter(1))
        min_angle_neighbor, _ = sorted_angles[0]
        face_nodes = set([self.s, min_angle_neighbor])

        # If bound was hit by first edge (s, min_angle_neighbor)
        bound_hit = not self.inside_bound(min_angle_neighbor)
        if bound_hit:
            face_nodes, half_edges, result_tag, condition_2c = self.traverse_opposite_direction(face_nodes, half_edges,
                                                                                  min_angle_neighbor, self.s)
        else:
            # Bound was not hit
            prev_node, cur_node = self.s, min_angle_neighbor
            half_edges.append((prev_node, cur_node))
            print('Half edge: ' + str((prev_node, cur_node)))
            if cur_node == self.d:
                # Destination was reached
                return face_nodes, half_edges, ResultTag.SUCCESS, False
            condition_2c = self.check_counters(cur_node, face_nodes, half_edges)

            # Iterate until face starting node is reached
            while cur_node != self.s and not condition_2c:
                bound_hit, prev_node, cur_node = self.next_face_half_edge(prev_node, cur_node, 'ccw')
                #print('Current node Face Routing: ' + str(cur_node))
                condition_2c = self.check_counters(cur_node, face_nodes, half_edges)
                if condition_2c:
                    break
                if bound_hit:
                    face_nodes, half_edges, result_tag, condition_2c = self.traverse_opposite_direction(face_nodes, half_edges,
                                                                                          cur_node, prev_node)
                    break
                else:
                    face_nodes, half_edges, result_tag = check_last_edge(prev_node, cur_node, face_nodes, half_edges, self.d)
                    if result_tag != ResultTag.DEFAULT:
                        break

        return face_nodes, half_edges, result_tag, condition_2c

    def next_face_half_edge(self, v, w, order) -> tuple[bool, int, None]:
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
            return False, w, None
        else:
            # Calculate inner angles ccw or cw
            if order == 'ccw':
                angles = {node: calculate_angle_ccw(np.subtract(self.positions[w], self.positions[v]), np.subtract(self.positions[w], self.positions[node])) for node in neighbors_directed}
            else:
                angles = {node: calculate_angle_cw(np.subtract(self.positions[w], self.positions[v]), np.subtract(self.positions[w], self.positions[node])) for node in neighbors_directed}
            # Make angle for incoming edge 360, otherwise this is the smallest angle 0
            if v in angles:
                angles[v] = 360
            sorted_angles = sorted(angles.items(), key=itemgetter(1))
            #print('Sorted angles: ' + str(sorted_angles))
            min_angle_neighbor, _ = sorted_angles[0]
            new_node = min_angle_neighbor

        if self.inside_bound(new_node):
            return False, w, new_node
        else:
            # Ellipse was hit
            return True, w, new_node

    def route_to_closest_node(self, current_node: int, current_face: set, half_edges) -> tuple[int, int,  list[int]]:
        """
        Route to the closest node to d after face traversal.
        @param current_node - Source node
        @param current_face - Current face
        @param half_edges - Half edges of face traversal
        """
        # Find closest node to destination
        distances = {node: distance.euclidean(self.positions[node], self.positions[self.d]) for node in current_face}
        closest_node, _ = min(distances.items(), key=itemgetter(1))
        print('Closest node: ' + str(closest_node))

        last_node_reached, route = self.search_half_edges(half_edges, current_node, closest_node)
        return last_node_reached, closest_node, route

    def search_half_edges(self, half_edges: list, face_starting_node: int, route_to_node: int) -> tuple[int, list]:
        """
        Route back from half_edges[-1][1] to route_to_node after face traversal.
        @param half_edges - Half edges of face traversal
        @param face_starting_node - Face starting node
        @param route_to_node - Node to route to

        Returns last node reached and route back to route_to_node
        """

        route = []
        current_node = half_edges[-1][1]
        if current_node != route_to_node:
            if current_node == face_starting_node:
                # Face was traversed completely, repeat edges taken during face traversal
                for edge in half_edges:
                    current_node = edge[1]
                    if self.inside_bound(edge[1]):
                        if edge[1] == route_to_node:
                            break
                        route.append(edge[1])
            else:
                # TODO: Search path also forwards
                # Closest node: 41
                # [(20, 25), (25, 48), (48, 41), (41, 25)] there is no edge (25, 41) but 41 can be reached.
                # Iterate over half edges in reverse
                for edge in reversed(half_edges):
                    if edge[0] in self.g.neighbors(edge[1]):
                        if self.inside_bound(edge[0]):
                            current_node = edge[0]
                            route.append(current_node)
                            if current_node == route_to_node:
                                break
                    else:
                        print('No route back to route_to_node.')
                        break
        return current_node, route

    ###################################################################################################################
    # Overridden in OBFR, GOAFR and GOAFRPlus
    ###################################################################################################################
    def goafr_plus_2b(self, cur_node, half_edges, current_face):
        # Condition 2b in GOAFR+
        return False, self.route, ResultTag.DEFAULT
    def check_counters(self, cur_node, current_face, half_edges):
        # Check condition 2c in GOAFR+
        return False
    def terminate_face_routing_mode(self):
        return self.find_route_ofr()
    def check_closest_node_progress(self, last_node_reached, closest_node) -> bool:
        return True
    def is_goafr_plus(self):
        return False
    def goafr_plus_greedy_mode(self):
        return False, self.route, ResultTag.DEFAULT
    ###################################################################################################################
    # Methods for face traversal in opposite direction
    ###################################################################################################################
    def inside_bound(self, node):
        return True
    def traverse_opposite_direction(self, face_nodes, half_edges, cur_node, prev_node):
        # Explore face in opposite direction after bound is hit
        return face_nodes, half_edges, ResultTag.DEFAULT, False

def check_last_edge(prev_node, cur_node, face_nodes, half_edges, d):
    result_tag = ResultTag.DEFAULT
    if cur_node is None:
        # Dead end
        print('Face could not be traversed completely due to a dead end in ' + str(prev_node))
        result_tag = ResultTag.DEAD_END
    else:
        face_nodes.add(cur_node)
        # mark_half_edges is a list which preserves order of how face was traversed
        if (prev_node, cur_node) in half_edges:
            print('Half-edge ' + str((prev_node, cur_node)) + ' was already encountered')
            result_tag = ResultTag.EDGE_ENCOUNTERED
        else:
            half_edges.append((prev_node, cur_node))
            print('Half edge: ' + str((prev_node, cur_node)))
            if cur_node == d:
                # Destination was reached
                result_tag = ResultTag.SUCCESS
    return face_nodes, half_edges, result_tag

def calculate_angle_ccw(vector_a, vector_b):
    """ Calculate angle of vector_b counterclockwise from vector_a """
    # Measures angle of first vector point_a clockwise from the y-axis at coordinate x of vector point_a
    ang_a = np.arctan2(*vector_a[0::])
    # Measures angle of vector point_b clockwise from the y-axis at coordinate x of vector point_a
    ang_b = np.arctan2(*vector_b[0::])
    # Sum up angles and convert to degrees
    return np.rad2deg((ang_a - ang_b) % (2 * np.pi))

def calculate_angle_cw(vector_a, vector_b):
    """ Calculate angle of vector_b clockwise from vector_a """
    # Measures angle of first vector point_a counterclockwise from the y-axis at coordinate x of vector point_a
    ang_a = np.arctan2(*vector_a[::-1])
    # Measures angle of vector point_b counterclockwise from the y-axis at coordinate x of vector point_a
    ang_b = np.arctan2(*vector_b[::-1])
    # Sum up angles and convert to degrees
    return np.rad2deg((ang_a - ang_b) % (2 * np.pi))