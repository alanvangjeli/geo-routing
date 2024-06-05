import networkx as nx
from matplotlib.patches import Circle
from scipy.spatial import distance

from RoutingAlgos.GeometricRouting.GR import GR
from RoutingAlgos.GeometricRouting.OBFR import OBFR
from RoutingAlgos.GeometricRouting.util import (
    RECURSION_DEPTH_LIMIT,
    ResultTag,
    append_to_edges_and_face,
)


class GOAFRPlus(GR, OBFR):
    def __init__(
        self,
        graph: nx.DiGraph,
        start: int,
        destination: int,
        positions: dict,
        rho: float = 0.0,
        sigma: float = 0.0,
        rho_0: float = 0.0,
    ):
        if sigma > 0 and rho > rho_0 >= 1:
            self.sigma = sigma
            self.rho = rho
            self.p = 0  # counts the nodes closer to d than face_starting_node
            self.q = (
                0  # counts the nodes not located closer to d than face_starting_node
            )
            distance_s_d = distance.euclidean(positions[start], positions[destination])
            circle = Circle(positions[destination], rho_0 * distance_s_d)
            GR.__init__(self, graph, start, destination, positions)
            OBFR.__init__(self, graph, start, destination, positions, circle)
        else:
            raise ValueError("Invalid parameters")

    def find_route(self) -> tuple[bool, list[int], str]:
        return self.greedy_routing_mode()

    def greedy_routing_mode(self):
        result_greedy, route_greedy, result_tag_greedy = self.find_route_greedy()
        self.s = self.route[-1]
        if result_greedy:
            return True, self.route, result_tag_greedy, self.edge_list_lengths
        # If local minimum was encountered -> Go into Face Routing Mode
        elif result_tag_greedy == ResultTag.LOCAL_MINIMUM:
            # print("Switched to Face Routing Mode")
            return self.face_routing_mode()
        else:
            # Dead end was encountered in greedy mode
            return False, self.route, result_tag_greedy, self.edge_list_lengths

    def face_routing_mode(self):
        if self.recursion_depth < RECURSION_DEPTH_LIMIT:
            self.recursion_depth += 1
            self.p = 0
            self.q = 0
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

            # Condition 2b
            if self.p == 0:
                self.searchable_area.set_radius(self.searchable_area.radius * self.rho)
                return self.face_routing_mode()

            # Route to node closest to destination
            last_node_reached, path_last_node_reached = self.route_to_closest_node(
                current_node, current_face, half_edges
            )
            # print('Last node reached: ' + str(last_node_reached))
            self.s = last_node_reached
            self.route.extend(path_last_node_reached)
            return self.greedy_routing_mode()
        else:
            return False, self.route, ResultTag.RECURSION_LIMIT, self.edge_list_lengths

    def traverse_face(self) -> tuple[set, list, str]:
        half_edges = []
        face_nodes = set([self.s])
        result_tag = ResultTag.DEFAULT

        # Find first neighbor ccw of line sd
        min_angle_neighbor = self.get_first_neighbor_ccw()

        # If bound was hit by first edge (s, min_angle_neighbor)
        if not self.inside_bound(min_angle_neighbor):
            return self.traverse_opposite_direction(
                face_nodes, half_edges, self.s, min_angle_neighbor
            )
        else:
            # min_angle_neighbor gets added to face_nodes and half_edges only if it is inside the bound
            prev_node, cur_node = append_to_edges_and_face(
                self.s, min_angle_neighbor, face_nodes, half_edges
            )
            # print('Half edge: ' + str((prev_node, cur_node)))
            if cur_node == self.d:
                # Destination was reached
                return face_nodes, half_edges, ResultTag.SUCCESS
            self.increment_counters(cur_node)
            # Condition 2c
            if self.p > self.sigma * self.q:
                return face_nodes, half_edges, result_tag

            # Iterate until face starting node (self.s) is reached
            while cur_node != self.s:
                prev_node, cur_node = self.next_face_half_edge(
                    prev_node, cur_node, "ccw"
                )
                if cur_node is not None and not self.inside_bound(cur_node):
                    return self.traverse_opposite_direction(
                        face_nodes, half_edges, prev_node, cur_node
                    )
                else:
                    face_nodes, half_edges, result_tag = self.check_last_edge(
                        prev_node, cur_node, face_nodes, half_edges
                    )
                    if result_tag != ResultTag.DEFAULT:
                        return face_nodes, half_edges, result_tag
                    self.increment_counters(cur_node)
                    # Condition 2c
                    if self.p > self.sigma * self.q:
                        return face_nodes, half_edges, result_tag

            return face_nodes, half_edges, result_tag

    def traverse_opposite_direction(self, face_nodes, half_edges, prev_node, cur_node):
        result_tag = ResultTag.DEFAULT
        # Switch direction
        # print('Bound was hit for the first time by this edge: ' + str((prev_node, cur_node)))
        prev_node, cur_node = cur_node, prev_node
        # Traverse until bound is hit again
        while self.inside_bound(cur_node):
            prev_node, cur_node = self.next_face_half_edge(prev_node, cur_node, "cw")
            face_nodes, half_edges, result_tag = (
                self.check_last_edge_opposite_direction(
                    prev_node, cur_node, face_nodes, half_edges
                )
            )
            if result_tag != ResultTag.DEFAULT:
                return face_nodes, half_edges, result_tag
            if cur_node not in face_nodes:
                self.increment_counters(cur_node)
                # Condition 2c
                if self.p > self.sigma * self.q:
                    return face_nodes, half_edges, result_tag
        # print('Bound was hit for the second time by this edge: ' + str((prev_node, cur_node)))
        return face_nodes, half_edges, result_tag

    def increment_counters(self, cur_node):
        # if cur_node is closer to d than local minimum v
        if distance.euclidean(
            self.positions[cur_node], self.positions[self.d]
        ) < distance.euclidean(self.positions[self.s], self.positions[self.d]):
            self.p += 1
        else:
            self.q += 1

    ################################################################################################################################################
    # GreedyRouting overrides
    ################################################################################################################################################
    def circle_bound_check(self):
        self.searchable_area.set_radius(self.searchable_area.radius / self.rho)
        if self.searchable_area.contains_point(self.positions[self.s]):
            # print('Circle radius divided by ' + str(self.rho))
            pass
        else:
            self.searchable_area.set_radius(self.searchable_area.radius * self.rho)
