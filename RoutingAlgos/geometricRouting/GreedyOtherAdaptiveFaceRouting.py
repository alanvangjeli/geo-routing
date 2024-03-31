import networkx as nx
from matplotlib.patches import Ellipse
from RoutingAlgos.geometricRouting.GreedyRouting import GreedyRouting
from RoutingAlgos.geometricRouting.OAFR import create_ellipse, OtherAdaptiveFaceRouting
from util import ResultTag

class GreedyOtherAdaptiveFaceRouting(GreedyRouting, OtherAdaptiveFaceRouting):
    def __init__(self, graph: nx.DiGraph = None, start: int = None, destination: int = None, positions: dict = None,
                 rho: float = 0.0):
        ellipse = create_ellipse(positions[self.s], positions[self.d])
        GreedyRouting.__init__(self, graph, start, destination, positions, rho, ellipse)

    def find_route(self) -> tuple[bool, list[int], str]:
        """
        Greedy other adaptive face routing GOAFR
        """

        # Greedy mode
        result_greedy, route_greedy, result_tag_greedy = self.greedy_routing_mode()
        self.route.extend(route_greedy)
        if result_greedy:
            return True, self.route, result_tag_greedy
        self.s = self.route[-1]

        # If greedy mode failed (local minimum) -> Traverse one face in OAFR mode
        if result_tag_greedy == ResultTag.LOCAL_MINIMUM:
            print("Switched to Face Routing Mode")
            result_oafr, route_oafr, result_tag_oafr = super()._other_adaptive_face_route(g, current_node, d,
                                                                                          route_greedy, positions,
                                                                                          set(), ellipse, 0, 0)
            self.route.extend(route_oafr)
            if result_oafr:
                return True, self.route, result_tag_oafr
            else:
                if result_tag_oafr == ResultTag.DEAD_END or result_tag_oafr == ResultTag.LOOP:
                    return False, self.route, result_tag_oafr
                else:
                    self.s = self.route[-1]
                    return self.find_route()
        else:
            # Dead end was encountered in greedy mode
            return False, self.route, result_tag_greedy

    ###################################################################################################################
    # GreedyRouting overrides
    ###################################################################################################################
    def ellipse_bound_check(self):
        if not self.searchable_area.contains_point(self.positions[self.s]):
            self.searchable_area.set_width(self.searchable_area.get_width() * 2)
            print('Ellipse width doubled')

    ###################################################################################################################
    # FaceRouting overrides
    ###################################################################################################################
    def terminate_goafr(self, g, s, d, route, positions, previous_face, previous_closest_node, searchable_area, sigma, rho):
        return False, route, ResultTag.FACE
