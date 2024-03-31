import matplotlib
import networkx as nx
import numpy as np
from matplotlib.patches import Ellipse, Circle
from scipy.spatial import distance
from RoutingAlgos.geometricRouting.OFR import OtherFaceRouting, calculate_angle_ccw, check_last_edge
from util import ResultTag

class OtherBoundedFaceRouting(OtherFaceRouting):
    # TODO: The Euclidian length of the optimal path from s to d is required to create the ellipse
    def __init__(self, graph: nx.DiGraph, start: int, destination: int, positions: dict, searchable_area: Ellipse | Circle):
        self.searchable_area = searchable_area
        super().__init__(graph, start, destination, positions)

    def inside_bound(self, node):
        return self.searchable_area.contains_point(self.positions[node])

    def traverse_opposite_direction(self, v, face_nodes, prev_node, cur_node, mark_half_edges=None):
        # Explore face in opposite direction
        bound_hit = False
        while not bound_hit:
            bound_hit, prev_node, cur_node = self.next_face_half_edge(prev_node, cur_node, 'cw')
            self.check_counters(cur_node, face_nodes, mark_half_edges)
            face_nodes, mark_half_edges, dead_end, interrupt = check_last_edge(prev_node, cur_node, face_nodes, mark_half_edges, self.d)
            if interrupt:
                break
            self.goafr_plus_2b(cur_node, mark_half_edges, face_nodes, bound_hit)
            #print('Bound was hit for the second time by this edge: ' + str((prev_node, cur_node)))

    def goafr_plus_2b(self, cur_node, half_edges, current_face, bound_hit):
        # Condition 2b in GOAFR+
        pass

    def face_routing_mode(self):
        self.find_route()

    def invert_direction(self, bound_hit, half_edges, prev_node, cur_node, v, face_nodes):
        if bound_hit:
            print('Bound was hit for the first time by this edge: ' + str((prev_node, cur_node)))
            half_edges.extend([(prev_node, cur_node), (cur_node, prev_node)])
            prev_node, cur_node = cur_node, prev_node
            self.traverse_opposite_direction(prev_node, face_nodes, prev_node, cur_node, mark_half_edges=half_edges)

    def check_closest_node_progress(self, last_node_reached, closest_node) -> bool:
        if last_node_reached == closest_node and closest_node == self.previous_closest_node:
            print('Closest node was already encountered')
            return False
        else:
            return True

class OtherAdaptiveFaceRouting(OtherBoundedFaceRouting):
    def __init__(self, graph: nx.DiGraph, start: int, destination: int, positions: dict):
        ellipse = create_ellipse(positions[start], positions[destination])
        super().__init__(graph, start, destination, positions, ellipse)

    def find_route(self) -> tuple[bool, list[int], str]:
        result_obfr, route_obfr, result_tag_obfr = super().find_route()
        self.route.extend(route_obfr)
        self.s = self.route[-1]
        if result_obfr:
            # Destination was reached
            return True, self.route, result_tag_obfr
        else:
            if result_tag_obfr == ResultTag.NO_PROGRESS:
                node_positions_list = list(self.positions.values())
                # OBFR delivers ResultTag.NO_PROGRESS even though all nodes are inside ellipse
                if all(self.searchable_area.contains_points(node_positions_list)):
                    return False, self.route, ResultTag.LOOP
                else:
                    # If any of the nodes is not inside the ellipse, then double the major axis (width) and continue search
                    self.searchable_area.set_width(self.searchable_area.width * 2)
                    print('Ellipse width doubled')
                    return self.find_route()
            else:
                # ResultTag.LOOP or ResultTag.DEAD_END or ResultTag.FACE
                return result_obfr, self.route, result_tag_obfr

    def oafr_routing_mode(self):
        self.find_route()

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
