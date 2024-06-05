import networkx as nx

from RoutingAlgos.GeometricRouting.GR import GR
from RoutingAlgos.GeometricRouting.OAFR import OAFR
from RoutingAlgos.GeometricRouting.util import RECURSION_DEPTH_LIMIT, ResultTag


class GOAFR(GR, OAFR):
    def __init__(
        self, graph: nx.DiGraph, start: int, destination: int, positions: dict
    ):
        GR.__init__(self, graph, start, destination, positions)
        OAFR.__init__(self, graph, start, destination, positions)

    def find_route(self) -> tuple[bool, list[int], str, int]:
        """
        Greedy other adaptive face routing GOAFR
        """

        # Greedy mode
        result_greedy, route_greedy, result_tag_greedy = self.find_route_greedy()
        if result_greedy:
            return True, self.route, result_tag_greedy, self.edge_list_lengths
        self.s = self.route[-1]

        # If greedy mode failed (local minimum) -> Traverse one face in OAFR mode
        if result_tag_greedy == ResultTag.LOCAL_MINIMUM:
            # print("Switched to Face Routing Mode")
            # Execute OAFR only on the first face
            current_face, half_edges, result_tag = self.traverse_face()
            print("GOAFR half edges: " + str(half_edges))
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
                return (
                    False,
                    self.route,
                    ResultTag.RECURSION_LIMIT,
                    self.edge_list_lengths,
                )
        else:
            # Dead end was encountered in greedy mode
            return False, self.route, result_tag_greedy, self.edge_list_lengths

    ###################################################################################################################
    # GreedyRouting overrides
    ###################################################################################################################
    # Double ellipse major axis if bound is hit
    def ellipse_bound_check(self):
        if not self.searchable_area.contains_point(self.positions[self.s]):
            self.searchable_area.set_width(self.searchable_area.get_width() * 2)
            # print('Ellipse width doubled')
