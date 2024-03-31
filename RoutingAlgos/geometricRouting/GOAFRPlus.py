import networkx as nx
from matplotlib.patches import Circle
from scipy.spatial import distance
from RoutingAlgos.geometricRouting.GreedyRouting import GreedyRouting
from RoutingAlgos.geometricRouting.OAFR import OtherAdaptiveFaceRouting
from util import ResultTag

class GOAFRPlus(GreedyRouting, OtherAdaptiveFaceRouting):
    def __init__(self, graph: nx.DiGraph = None, start: int = None, destination: int = None, positions: dict = None,
                 rho: float = 0.0, sigma: float = 0.0, rho_0: float = 0.0):
        self.sigma = sigma
        if self.sigma > 0 and rho > rho_0 >= 1:
            distance_s_d = distance.euclidean(positions[start], positions[destination])
            circle = Circle(positions[destination], rho_0 * distance_s_d)
        else:
            raise ValueError('Invalid parameters')
        GreedyRouting.__init__(self, graph, start, destination, positions, rho, circle)

    def find_route(self) -> tuple[bool, list[int], str]:
        # TODO: This is identical to GOAFR apart from calling FaceRouting instead of OAFR
        # Greedy Routing Mode
        result_greedy, route_greedy, result_tag_greedy = self.greedy_routing_mode()
        self.route.extend(route_greedy)
        if result_greedy:
            return True, self.route, result_tag_greedy
        self.s = self.route[-1]

        # If greedy mode failed (local minimum) -> Go into Face Routing Mode
        if result_tag_greedy == ResultTag.LOCAL_MINIMUM:
            print("Switched to Face Routing Mode")
            result_face, route_face, result_tag_face = self.face_routing_mode(g, current_node, d, route, positions, circle, sigma, rho)
            self.route.extend(route_face)
            if result_face:
                return True, self.route, result_tag_face
            else:
                if result_tag_face == ResultTag.DEAD_END or result_tag_face == ResultTag.LOOP:
                    return False, self.route, result_tag_face
                else:
                    self.s = self.route[-1]
                    return self.find_route()
        else:
            # Dead end was encountered in greedy mode
            return False, self.route, result_tag_greedy

    def face_routing_mode(self, g: nx.DiGraph, s: int, d: int, route: list, positions: dict, circle: Circle,
                          sigma: float, rho):
        """
        Face routing mode
        @param g - Graph to route on
        @param s - Source node
        @param d - Destination node
        @param route - Route so far
        @param positions - Positions of nodes
        @param circle - Circle that bounds face traversal
        @param rho - Value to divide circle radius by
        @param sigma - Threshold
        """

        return super().face_routing(g, s, d, route, positions, set(), s, circle, sigma, rho)

    ################################################################################################################################################
    # GreedyRouting overrides
    ################################################################################################################################################
    def circle_bound_check(self):
        self.searchable_area.set_radius(self.searchable_area.radius / self.rho)
        if self.searchable_area.contains_point(self.positions[self.s]):
            print('Circle radius divided by ' + str(self.rho))
        else:
            self.searchable_area.set_radius(self.searchable_area.radius * self.rho)

    ###################################################################################################################################################
    # OtherAdaptiveFaceRouting overrides
    ###################################################################################################################################################
    def goafr_plus_2b(self, g, cur_node, d, positions, half_edges, current_face, route, bound_hit, p, sigma, rho, circle):
        if bound_hit:
            if p == 0:
                # Enlarge circle and continue in Face Routing Mode
                circle.set_radius(circle.radius * rho)
            else:
                last_node_reached, closest_node, path_last_node_reached = super().route_to_closest_node(g, cur_node, d, positions, current_face, half_edges, circle)
                route.extend(path_last_node_reached)
                self.find_route(g, last_node_reached, d, route, positions, circle, sigma, rho)

    # Condition 2c
    def check_counters(self, g, route, p, q, sigma, rho, positions, cur_node, v, d, current_face, half_edges, circle):
        # if cur_node is closer to d than local minimum v
        if distance.euclidean(positions[cur_node], positions[d]) < distance.euclidean(positions[v], positions[d]):
            p += 1
        else:
            q += 1
        if p > sigma * q:
            last_node_reached, closest_node, path_last_node_reached = super().route_to_closest_node(g, cur_node, d, positions, current_face, half_edges, circle)
            route.extend(path_last_node_reached)
            self.find_route(g, last_node_reached, d, route, positions, circle, sigma, rho)
