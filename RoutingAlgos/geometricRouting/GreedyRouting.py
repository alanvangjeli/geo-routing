import networkx as nx
from scipy.spatial import distance
from operator import itemgetter
from util import ResultTag

class GreedyRouting:
    def __init__(self, graph: nx.DiGraph, start: int, destination: int, positions: dict):
        self.g = graph
        self.s = start
        self.d = destination
        self.positions = positions
        self.route = []

    def find_route_greedy(self) -> [bool, list[int], str]:
        self.route.append(self.s)
        if self.s == self.d:
            return True, self.route, ResultTag.SUCCESS
        else:
            neighbors = [node for node in self.g.neighbors(self.s)]
            if len(neighbors) == 0:
                return False, self.route, ResultTag.DEAD_END
            else:
                distances = {node: distance.euclidean(self.positions[node], self.positions[self.d]) for node in neighbors}
                min_distance_neighbor, min_distance = min(distances.items(), key=itemgetter(1))
                current_node_distance = distance.euclidean(self.positions[self.s], self.positions[self.d])
                if min_distance < current_node_distance:
                    self.s = min_distance_neighbor
                    # GOAFR and GOAFR+ checks
                    ##########################################################################################
                    self.ellipse_bound_check()
                    self.circle_bound_check()
                    ##########################################################################################
                    print('Next node greedy: ' + str(self.s))
                    return self.find_route_greedy()
                else:
                    return False, self.route, ResultTag.LOCAL_MINIMUM

    def ellipse_bound_check(self):
        # Double ellipse major axis if bound is hit in GOAFR
        pass

    def circle_bound_check(self):
        # Divide circle radius by rho in GOAFR+
        pass