import matplotlib
import networkx as nx
import numpy as np
from matplotlib.patches import Ellipse
from scipy.spatial import distance

from RoutingAlgos.GeometricRouting.OBFR import OBFR
from RoutingAlgos.GeometricRouting.util import (
    RECURSION_DEPTH_LIMIT,
    ResultTag,
    calculate_angle_ccw,
)


class OAFR(OBFR):
    def __init__(
        self, graph: nx.DiGraph, start: int, destination: int, positions: dict
    ):
        ellipse = create_ellipse(positions[start], positions[destination])
        super().__init__(graph, start, destination, positions, ellipse)

    def find_route(self) -> tuple[bool, list[int], str, int]:
        if self.recursion_depth < RECURSION_DEPTH_LIMIT:
            self.recursion_depth += 1
            # print("Recursion depth: " + str(self.recursion_depth))
            result_obfr, route_obfr, result_tag_obfr, edge_list_lengths_obfr = (
                super().find_route()
            )
            self.s = self.route[-1]
            if result_obfr:
                # Destination was reached
                return True, self.route, result_tag_obfr, self.edge_list_lengths
            else:
                if (
                    result_tag_obfr == ResultTag.DEAD_END
                    or result_tag_obfr == ResultTag.LOOP
                ):
                    return False, self.route, result_tag_obfr, self.edge_list_lengths
                # node_positions_list = list(self.positions.values())
                # OBFR is unsuccessful even though all nodes are inside ellipse
                # if all(self.searchable_area.contains_points(node_positions_list)):
                #    return False, self.route, result_tag_obfr
                else:
                    # If any of the nodes is not inside the ellipse, then double the major axis (width) and continue search
                    self.searchable_area.set_width(self.searchable_area.width * 2)
                    # print("Ellipse width doubled")
                    return self.find_route()
        else:
            return False, self.route, ResultTag.RECURSION_LIMIT, self.edge_list_lengths


def create_ellipse(pos_s: tuple, pos_d: tuple) -> matplotlib.patches.Ellipse:
    distance_s_d = distance.euclidean(pos_s, pos_d)
    width = 2 * distance_s_d
    height = 2 * np.sqrt(width**2 - distance_s_d**2 / 4)
    centre = ((pos_s[0] + pos_d[0]) / 2, (pos_s[1] + pos_d[1]) / 2)

    vector_a = (pos_d[0] - pos_s[0], 0)
    vector_b = (pos_d[0] - pos_s[0], pos_d[1] - pos_s[1])
    angle = calculate_angle_ccw(vector_a, vector_b)

    ellipse = Ellipse(centre, width, height, angle=angle)
    return ellipse
