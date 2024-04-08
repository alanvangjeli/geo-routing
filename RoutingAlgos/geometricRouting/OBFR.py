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

    def traverse_opposite_direction(self, face_nodes, half_edges, cur_node, prev_node):
        condition_2c = False
        result_tag = ResultTag.DEFAULT
        # Switch direction
        print('Bound was hit for the first time by this edge: ' + str((prev_node, cur_node)))
        half_edges.extend([(prev_node, cur_node), (cur_node, prev_node)])
        prev_node, cur_node = cur_node, prev_node
        # Traverse until bound is hit again
        bound_hit = False
        while not bound_hit:
            bound_hit, prev_node, cur_node = self.next_face_half_edge(prev_node, cur_node, 'cw')
            # goafr_plus_2c
            condition_2c = self.check_counters(cur_node, face_nodes, half_edges)
            face_nodes, half_edges, result_tag = check_last_edge(prev_node, cur_node, face_nodes, half_edges, self.d)
            if result_tag != ResultTag.DEFAULT:
                break
            if bound_hit:
                print('Bound was hit for the second time by this edge: ' + str((prev_node, cur_node)))
                result_tag = ResultTag.SECOND_BOUND_HIT
            if condition_2c:
                break
        return face_nodes, half_edges, result_tag, condition_2c

    def find_route_obfr(self) -> tuple[bool, list[int], str]:
        return super().find_route_ofr()

    def check_closest_node_progress(self, last_node_reached, closest_node) -> bool:
        if last_node_reached == closest_node and closest_node == self.previous_closest_node:
            print('Closest node was already encountered')
            return False
        else:
            return True