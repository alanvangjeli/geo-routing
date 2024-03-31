import networkx as nx
from RoutingAlgos.geometricRouting.FaceRouting import FaceRouting

class OtherFaceRouting(FaceRouting):
    def route(self, g: nx.DiGraph, s: int, d: int, positions: dict) -> tuple[bool, list, str]:
        """
        Other Face Routing OFR
        @param g - Graph to route on
        @param s - Source node
        @param d - Destination node
        @param positions - Positions of nodes
        """
        return super().face_routing(g, s, d, [s], positions, set(), s, None, 0, 0)