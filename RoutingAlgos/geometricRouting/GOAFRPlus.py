import networkx as nx
from matplotlib.patches import Circle
from scipy.spatial import distance
from RoutingAlgos.geometricRouting.GreedyRouting import GreedyRouting
from RoutingAlgos.geometricRouting.OBFR import OtherBoundedFaceRouting
from util import ResultTag

class GOAFRPlus(GreedyRouting, OtherBoundedFaceRouting):
    def __init__(self, graph: nx.DiGraph, start: int, destination: int, positions: dict, rho: float = 0.0, sigma: float = 0.0, rho_0: float = 0.0):
        if sigma > 0 and rho > rho_0 >= 1:
            self.sigma = sigma
            self.rho = rho
            self.p = 0  # counts the nodes closer to d than face_starting_node
            self.q = 0  # counts the nodes not located closer to d than face_starting_node
            distance_s_d = distance.euclidean(positions[start], positions[destination])
            circle = Circle(positions[destination], rho_0 * distance_s_d)
            GreedyRouting.__init__(self, graph, start, destination, positions)
            OtherBoundedFaceRouting.__init__(self, graph, start, destination, positions, circle)
        else:
            raise ValueError('Invalid parameters')

    def greedy_routing_mode(self):
        result_greedy, route_greedy, result_tag_greedy = self.find_route_greedy()
        self.route.extend(route_greedy)
        self.s = self.route[-1]
        if result_greedy:
            return True, self.route, result_tag_greedy
        # If local minimum was encountered -> Go into Face Routing Mode
        elif result_tag_greedy == ResultTag.LOCAL_MINIMUM:
            print("Switched to Face Routing Mode")
            return self.face_routing_mode()
        else:
            # Dead end was encountered in greedy mode
            return False, self.route, result_tag_greedy

    def face_routing_mode(self):
        result_face, route_face, result_tag_face = self.find_route_obfr()
        self.route.extend(route_face)
        self.s = self.route[-1]
        if result_face:
            return True, self.route, result_tag_face
        else:
            if result_tag_face == ResultTag.DEAD_END or result_tag_face == ResultTag.LOOP:
                return False, self.route, result_tag_face
            # Face F boundary exploration was complete
            else:
                return self.greedy_routing_mode()

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
    # Condition 2b
    def goafr_plus_2b(self, cur_node, half_edges, current_face):
        # None of the visited nodes are closer to d than local minimum
        if self.p == 0:
            # Enlarge circle and continue in Face Routing Mode
            self.searchable_area.set_radius(self.searchable_area.radius * self.rho)
            self.face_routing_mode()
        else:
            last_node_reached, closest_node, path_last_node_reached = super().route_to_closest_node(cur_node, current_face, half_edges)
            self.route.extend(path_last_node_reached)
            self.s = last_node_reached
            return self.greedy_routing_mode()

    # Condition 2c
    def check_counters(self, cur_node, current_face, half_edges):
        # if cur_node is closer to d than local minimum v
        if distance.euclidean(self.positions[cur_node], self.positions[self.d]) < distance.euclidean(self.positions[self.s], self.positions[self.d]):
            self.p += 1
        else:
            self.q += 1
        if self.p > self.sigma * self.q:
            last_node_reached, closest_node, path_last_node_reached = super().route_to_closest_node(cur_node, current_face, half_edges)
            self.route.extend(path_last_node_reached)
            self.s = last_node_reached
            return True
        else:
            return False

    def is_goafr_plus(self):
        return True
    def goafr_plus_greedy_mode(self):
        return self.greedy_routing_mode()
