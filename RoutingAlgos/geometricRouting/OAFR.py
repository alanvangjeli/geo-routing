import matplotlib
import networkx as nx
import numpy as np
from matplotlib.patches import Ellipse, Circle
from scipy.spatial import distance

from RoutingAlgos.geometricRouting.OBFR import OtherBoundedFaceRouting
from RoutingAlgos.geometricRouting.OFR import OtherFaceRouting, calculate_angle_ccw, check_last_edge
from util import ResultTag

class OtherAdaptiveFaceRouting(OtherBoundedFaceRouting):
    def __init__(self, graph: nx.DiGraph, start: int, destination: int, positions: dict):
        ellipse = create_ellipse(positions[start], positions[destination])
        super().__init__(graph, start, destination, positions, ellipse)

    def find_route_oafr(self) -> tuple[bool, list[int], str]:
        result_obfr, route_obfr, result_tag_obfr = super().find_route_ofr()
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
                    return self.find_route_oafr()
            else:
                # ResultTag.LOOP or ResultTag.DEAD_END or ResultTag.FACE
                return result_obfr, self.route, result_tag_obfr

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
