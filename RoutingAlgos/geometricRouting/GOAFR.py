import networkx as nx
from RoutingAlgos.geometricRouting.GreedyRouting import GreedyRouting
from RoutingAlgos.geometricRouting.OAFR import OtherAdaptiveFaceRouting
from util import ResultTag

class GreedyOtherAdaptiveFaceRouting(GreedyRouting, OtherAdaptiveFaceRouting):
    def __init__(self, graph: nx.DiGraph, start: int, destination: int, positions: dict):
        GreedyRouting.__init__(self, graph, start, destination, positions)
        OtherAdaptiveFaceRouting.__init__(self, graph, start, destination, positions)

    def find_route_goafr(self) -> tuple[bool, list[int], str]:
        """
        Greedy other adaptive face routing GOAFR
        """

        # Greedy mode
        result_greedy, route_greedy, result_tag_greedy = self.find_route_greedy()
        self.route.extend(route_greedy)
        if result_greedy:
            return True, self.route, result_tag_greedy
        self.s = self.route[-1]

        # If greedy mode failed (local minimum) -> Traverse one face in OAFR mode
        if result_tag_greedy == ResultTag.LOCAL_MINIMUM:
            print("Switched to Face Routing Mode")
            result_oafr, route_oafr, result_tag_oafr = super().find_route_oafr()
            self.route.extend(route_oafr)
            if result_oafr:
                return True, self.route, result_tag_oafr
            else:
                if result_tag_oafr == ResultTag.DEAD_END or result_tag_oafr == ResultTag.LOOP:
                    return False, self.route, result_tag_oafr
                else:
                    self.s = self.route[-1]
                    return self.find_route_goafr()
        else:
            # Dead end was encountered in greedy mode
            return False, self.route, result_tag_greedy

    ###################################################################################################################
    # GreedyRouting overrides
    ###################################################################################################################
    # Double ellipse major axis if bound is hit
    def ellipse_bound_check(self):
        if not self.searchable_area.contains_point(self.positions[self.s]):
            self.searchable_area.set_width(self.searchable_area.get_width() * 2)
            print('Ellipse width doubled')

    ###################################################################################################################
    # FaceRouting overrides
    ###################################################################################################################
    # Stops face routing mode after one face traversal
    def terminate_face_routing_mode(self):
        return False, self.route, ResultTag.FACE
