from operator import itemgetter
import matplotlib
import networkx as nx
import numpy as np
from matplotlib.patches import Ellipse, Circle
from scipy.spatial import distance
from util import ResultTag

class FaceRouting:

    # HELPER FUNCTIONS
    def check_closest_node_progress(self, last_node_reached, closest_node, route, previous_closest_node=None) -> bool:
        return True

    def terminate_goafr(self, g, s, d, route, positions, previous_face, previous_closest_node, searchable_area, sigma, rho):
        return self.face_routing(g, s, d, route, positions, previous_face, previous_closest_node,
                                 searchable_area, sigma, rho)

    def invert_direction(self, g, bound_hit, half_edges, searchable_area, positions, prev_node, cur_node, v, d,
                         face_nodes, prev_face, p, q, sigma, rho, route):
        # Prepare to traverse face in opposite direction in OBFR
        pass

    def inside_bound(self, searchable_area, positions, node, rho):
        return True

    def traverse_opposite_direction(self, g, v, d, positions, face_nodes, prev_face, searchable_area, prev_node,
                                    cur_node, p, q, sigma, rho, route, mark_half_edges=None):
        # Explore face in opposite direction after bound is hit
        pass

    def check_counters(self, g, route, p, q, sigma, rho, positions, cur_node, v, d, current_face, half_edges, circle):
        # Check condition 2c in GOAFR+
        pass

    def next_face_half_edge(self, g, v, w, positions, searchable_area, order) -> tuple[bool, int, None]:
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
        searchable_area : matplotlib.patches.Ellipse | matplotlib.patches.Circle
            Area that bounds face traversal.
        order : str
            'ccw' or 'cw'

        Returns
        -------
        half-edge : tuple
        """
        neighbors_directed = [node for node in g.neighbors(w)]
        if len(neighbors_directed) == 0:
            # Dead end
            return False, w, None
        else:
            # Calculate inner angles ccw or cw
            if order == 'ccw':
                angles = {node: calculate_angle_ccw(np.subtract(positions[w], positions[v]), np.subtract(positions[w], positions[node])) for node in neighbors_directed}
            else:
                angles = {node: calculate_angle_cw(np.subtract(positions[w], positions[v]), np.subtract(positions[w], positions[node])) for node in neighbors_directed}
            # Make angle for incoming edge 360, otherwise this is the smallest angle 0
            if v in angles:
                angles[v] = 360
            sorted_angles = sorted(angles.items(), key=itemgetter(1))
            #print('Sorted angles: ' + str(sorted_angles))
            min_angle_neighbor, _ = sorted_angles[0]
            new_node = min_angle_neighbor

        if self.inside_bound(searchable_area, positions, new_node, 0):
            return False, w, new_node
        else:
            # Ellipse was hit
            return True, w, new_node

    def search_half_edges(self, g, half_edges: list, face_starting_node: int, route_to_node: int,
                          searchable_area: matplotlib.patches.Ellipse | matplotlib.patches.Circle, positions: dict) -> tuple[int, list]:
        """
        Route back from half_edges[-1][1] to route_to_node after face traversal.
        @param g - Graph to route on
        @param half_edges - Half edges of face traversal
        @param face_starting_node - Face starting node
        @param route_to_node - Node to route to
        @param searchable_area - Ellipse that bounds face traversal
        @param positions - Positions of nodes
        @param protocol - Routing protocol

        Returns last node reached and route back to route_to_node
        """

        route = []
        current_node = half_edges[-1][1]
        if current_node != route_to_node:
            if current_node == face_starting_node:
                # Face was traversed completely, repeat edges taken during face traversal
                for edge in half_edges:
                    current_node = edge[1]
                    if self.inside_bound(searchable_area, positions, edge[1], 0):
                        if edge[1] == route_to_node:
                            break
                        route.append(edge[1])
            else:
                # TODO: Search path also forwards
                # Closest node: 41
                # [(20, 25), (25, 48), (48, 41), (41, 25)] there is no edge (25, 41) but 41 can be reached.
                # Iterate over half edges in reverse
                for edge in reversed(half_edges):
                    if edge[0] in g.neighbors(edge[1]):
                        if self.inside_bound(searchable_area, positions, edge[0], 0):
                            current_node = edge[0]
                            route.append(current_node)
                            if current_node == route_to_node:
                                break
                    else:
                        print('No route back to route_to_node.')
                        break
        return current_node, route

    def route_to_closest_node(self, g: nx.DiGraph, current_node: int, d: int, positions: dict, current_face: set,
                              half_edges, searchable_area) -> tuple[int, int,  list[int]]:
        """
        Route to closest node after face traversal.
        @param g - Graph to route on
        @param current_node - Source node
        @param d - Destination node
        @param positions - Positions of nodes
        @param current_face - Current face
        @param half_edges - Half edges of face traversal
        @param searchable_area - Area that bounds face traversal
        """
        # Find closest node to destination
        distances = {node: distance.euclidean(positions[node], positions[d]) for node in current_face}
        closest_node, _ = min(distances.items(), key=itemgetter(1))
        print('Closest node: ' + str(closest_node))

        last_node_reached, route = self.search_half_edges(g, half_edges, current_node, closest_node, searchable_area, positions)
        return last_node_reached, closest_node, route

    def traverse_face(self, g, v, d, positions, searchable_area, sigma, rho, route,
                      mark_half_edges=None) -> tuple[set, bool]:
        """Returns nodes on the face that is intersected by vd.

        Parameters
        ----------
        g : DiGraph
        v : node
            Start node of half-edge. Face starting node.
        d : node
            Destination
        positions : dict
            Node positions.
        searchable_area : matplotlib.patches.Ellipse | matplotlib.patches.Circle
        sigma: float
            Threshold for condition 2c in GOAFR+
        rho: float
            Value to divide circle radius by in GOAFR+
        mark_half_edges: list, optional
            List to which all encountered half-edges are added.

        Returns
        -------
        face : set
            A set of nodes that lie on this face.
        """
        if mark_half_edges is None:
            mark_half_edges = []
        dead_end = False

        # Initialize counters for GOAFR+
        p = 0  # counts the nodes closer to d than face_starting_node
        q = 0  # counts the nodes not located closer to d than face_starting_node

        # Find first neighbor ccw of line sd
        neighbors = [node for node in g.neighbors(v)]
        angles = {node: calculate_angle_ccw(np.subtract(positions[d], positions[v]),
                                            np.subtract(positions[node], positions[v])) for node in neighbors}
        sorted_angles = sorted(angles.items(), key=itemgetter(1))
        min_angle_neighbor, _ = sorted_angles[0]
        face_nodes = set([v, min_angle_neighbor])

        # If bound was hit by edge (v, min_angle_neighbor)
        bound_hit = not self.inside_bound(searchable_area, positions, min_angle_neighbor, rho)
        if bound_hit:
            self.invert_direction(g, bound_hit, mark_half_edges, searchable_area, positions, v, min_angle_neighbor, v,
                                  d, face_nodes, set(), p, q, sigma, rho, route)
        else:
            # Bound was not hit
            prev_node, cur_node = v, min_angle_neighbor
            mark_half_edges.append((prev_node, cur_node))
            print('Half edge: ' + str((prev_node, cur_node)))
            if cur_node == d:
                # Destination was reached
                return face_nodes, dead_end
            self.check_counters(g, route, p, q, sigma, rho, positions, cur_node, v, d, face_nodes, mark_half_edges,
                                searchable_area)

            # Iterate until face starting node is reached
            while cur_node != v:
                bound_hit, prev_node, cur_node = self.next_face_half_edge(g, prev_node, cur_node, positions, searchable_area, 'ccw')
                print('Current node Face Routing: ' + str(cur_node))
                self.check_counters(g, route, p, q, sigma, rho, positions, cur_node, v, d, face_nodes, mark_half_edges, searchable_area)
                if bound_hit:
                    self.invert_direction(g, bound_hit, mark_half_edges, searchable_area, positions, prev_node,
                                          cur_node, v, d, face_nodes, set(), p, q, sigma, rho, route)
                    break
                else:
                    face_nodes, mark_half_edges, dead_end, interrupt = check_last_edge(prev_node, cur_node, face_nodes, mark_half_edges, d)
                    if interrupt:
                        break

        return face_nodes, dead_end

    def face_routing(self, g: nx.DiGraph, s: int, d: int, route: list, positions: dict, previous_face: set,
                     previous_closest_node: int | None, searchable_area: Ellipse | Circle | None, sigma, rho) -> tuple[bool, list, str]:
        """
        Other Face Routing OFR
        @param g - Graph to route on
        @param s - Source node
        @param d - Destination node
        @param route - Route to destination
        @param positions - Positions of nodes
        @param previous_face - Previous face
        @param previous_closest_node - Previous closest node
        @param searchable_area - Area that bounds face traversal
        @param sigma - Threshold
        @param rho - Value to divide circle radius by
        """
        if s == d:
            return True, route, ResultTag.SUCCESS

        # Initialize values
        half_edges = []
        neighbors = [node for node in g.neighbors(s)]

        if route[-1] != s:
            route.append(s)

        # Dead end
        if len(neighbors) == 0:
            return False, route, ResultTag.DEAD_END
        # Multiple neighbors, take first edge ccw of line sd
        else:
            # Only one neighbor
            if len(neighbors) == 1:
                print('Only neighbor: ' + str(neighbors[0]))
            current_face, dead_end_encountered = self.traverse_face(g, s, d, positions, searchable_area, sigma, rho, route, mark_half_edges=half_edges)
            current_node = half_edges[-1][1]
            for edge in half_edges:
                route.append(edge[1])
            if dead_end_encountered:
                return False, route, ResultTag.DEAD_END
            if current_node == d:
                # Destination was reached
                return True, route, ResultTag.SUCCESS

        print('Current face: ' + str(current_face))
        print('Half edges: ' + str(half_edges))
        # Detect if stuck in a loop
        if current_face == previous_face:
            return False, route, ResultTag.LOOP
        else:
            previous_face = current_face

        # Route to node closest to destination
        last_node_reached, closest_node, path_last_node_reached = self.route_to_closest_node(g, current_node, d,
                                                                                             positions, current_face,
                                                                                             half_edges,
                                                                                             searchable_area)
        route.extend(path_last_node_reached)

        # Check if last_node_reached is not the same as previous_closest_node in OBFR
        print("Previous closest node: " + str(previous_closest_node))
        if self.check_closest_node_progress(last_node_reached, closest_node, route, previous_closest_node=previous_closest_node):
            previous_closest_node = closest_node
        else:
            return False, route, ResultTag.NO_PROGRESS

        # In GOAFR only one face is traversed
        return self.terminate_goafr(g, last_node_reached, d, route, positions, previous_face, previous_closest_node, searchable_area, sigma, rho)

def check_last_edge(prev_node, cur_node, face_nodes, half_edges, d):
    interrupt = False
    dead_end = False
    if cur_node is None:
        # Dead end
        print('Face could not be traversed completely due to a dead end in ' + str(prev_node))
        dead_end = True
        interrupt = True
    else:
        face_nodes.add(cur_node)
        # mark_half_edges is a list which preserves order of how face was traversed
        if (prev_node, cur_node) in half_edges:
            print('Half-edge ' + str((prev_node, cur_node)) + ' was already encountered')
            interrupt = True
        else:
            half_edges.append((prev_node, cur_node))
            print('Half edge: ' + str((prev_node, cur_node)))
            if cur_node == d:
                # Destination was reached
                interrupt = True
    return face_nodes, half_edges, dead_end, interrupt

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