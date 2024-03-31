import matplotlib
import networkx as nx
import numpy as np
from matplotlib.patches import Ellipse, Circle
from scipy.spatial import distance
from RoutingAlgos.geometricRouting.FaceRouting import FaceRouting, calculate_angle_ccw, check_last_edge
from util import ResultTag

class OtherBoundedFaceRouting(FaceRouting):

    def inside_bound(self, searchable_area, positions, node, rho):
        return searchable_area.contains_point(positions[node])

    def traverse_opposite_direction(self, g, v, d, positions, face_nodes, prev_face, searchable_area, prev_node,
                                    cur_node, p, q, sigma, rho, route, mark_half_edges=None):
        # Explore face in opposite direction
        bound_hit = False
        while not bound_hit:
            bound_hit, prev_node, cur_node = self.next_face_half_edge(g, prev_node, cur_node, positions, searchable_area, 'cw')
            self.check_counters(g, route, p, q, sigma, rho, positions, cur_node, v, d, face_nodes, mark_half_edges,
                                searchable_area)
            face_nodes, mark_half_edges, dead_end, interrupt = check_last_edge(prev_node, cur_node, face_nodes, mark_half_edges, d)
            if interrupt:
                break
            self.goafr_plus_2b(g, cur_node, d, positions, mark_half_edges, face_nodes, route, bound_hit, p, sigma, rho, searchable_area)
            #print('Bound was hit for the second time by this edge: ' + str((prev_node, cur_node)))

    def goafr_plus_2b(self, g, cur_node, d, positions, half_edges, current_face, route, bound_hit, p, sigma, rho, circle):
        # Condition 2b in GOAFR+
        pass

    def invert_direction(self, g, bound_hit, half_edges, searchable_area, positions, prev_node, cur_node, v, d,
                         face_nodes, prev_face, p, q, sigma, rho, route):
        if bound_hit:
            print('Bound was hit for the first time by this edge: ' + str((prev_node, cur_node)))
            half_edges.extend([(prev_node, cur_node), (cur_node, prev_node)])
            prev_node, cur_node = cur_node, prev_node
            self.traverse_opposite_direction(g, prev_node, cur_node, positions, face_nodes, prev_face, searchable_area,
                                             prev_node, cur_node, p, q, sigma, rho, route, mark_half_edges=half_edges)

    def check_closest_node_progress(self, last_node_reached, closest_node, route, previous_closest_node=None) -> bool:
        if last_node_reached == closest_node and closest_node == previous_closest_node:
            print('Closest node was already encountered')
            return False
        else:
            return True

    def route(self, g: nx.DiGraph, s: int, d: int, positions: dict) -> tuple[bool, list, str]:
        return self._other_bounded_face_route(g, s, d, [s], positions, set(), s,
                                              create_ellipse(positions[s], positions[d]))

    def _other_bounded_face_route(self, g: nx.DiGraph, s: int, d: int, route: list, positions: dict, previous_face: set, previous_closest_node,
                                  ellipse: Ellipse) -> tuple[bool, list, str]:
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
        """
        return super().face_routing(g, s, d, route, positions, previous_face, previous_closest_node, ellipse, 0, 0)

class OtherAdaptiveFaceRouting(OtherBoundedFaceRouting):

    def route(self, g: nx.DiGraph, s: int, d: int, positions: dict) -> tuple[bool, list[int], str]:
        """
        Wrapper method for _other_adaptive_face_route, initializes ellipse.
        """
        ellipse = create_ellipse(positions[s], positions[d])
        return self._other_adaptive_face_route(g, s, d, [s], positions, set(), ellipse, 0, 0)

    def _other_adaptive_face_route(self, g: nx.DiGraph, s: int, d: int, route: list, positions: dict,
                                   previous_face: set, searchable_area: Ellipse | Circle, sigma,
                                   rho) -> tuple[bool, list[int], str]:

        result, route, result_tag = self._other_bounded_face_route(g, s, d, route, positions, previous_face, s, searchable_area)
        if result:
            # Destination was reached
            return result, route, result_tag
        else:
            if result_tag == ResultTag.NO_PROGRESS:
                node_positions = list(positions.values())
                # OBFR delivers ResultTag.NO_PROGRESS even though all nodes are inside ellipse
                if all(searchable_area.contains_points(node_positions)):
                    return False, route, ResultTag.LOOP
                else:
                    # If any of the nodes is not inside the ellipse, then double the major axis (width) and continue search
                    searchable_area.set_width(searchable_area.width * 2)
                    print('Ellipse width doubled')
                    return self._other_adaptive_face_route(g, s, d, route, positions, previous_face, searchable_area, 0,
                                                           0)
            else:
                # ResultTag.LOOP or ResultTag.DEAD_END or ResultTag.FACE
                return result, route, result_tag

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
