import networkx as nx
from matplotlib.patches import Circle, Ellipse

from RoutingAlgos.GeometricRouting.OFR import OFR
from RoutingAlgos.GeometricRouting.util import ResultTag, append_to_edges_and_face


class OBFR(OFR):
    # The Euclidian length of the optimal path from s to d is required to create the ellipse
    def __init__(
        self,
        graph: nx.DiGraph,
        start: int,
        destination: int,
        positions: dict,
        searchable_area: Ellipse | Circle,
    ):
        self.searchable_area = searchable_area
        super().__init__(graph, start, destination, positions)

    def traverse_face(self) -> tuple[set, list, str]:
        """
        Returns nodes on the face that is intersected by sd.

        Returns
        -------
        face : set
            A set of nodes that lie on this face.
        """
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

            return face_nodes, half_edges, result_tag

    def check_last_edge_opposite_direction(
        self, prev_node, cur_node, face_nodes, half_edges
    ):
        result_tag = ResultTag.DEFAULT
        if cur_node is None:
            # Dead end
            # print('Face could not be traversed completely due to a dead end in ' + str(prev_node))
            result_tag = ResultTag.DEAD_END
        else:
            if half_edges.count((prev_node, cur_node)) == 2:
                # print('Edge ' + str((prev_node, cur_node)) + ' was already traversed two times')
                half_edges.append((prev_node, cur_node))
                result_tag = ResultTag.EDGE_ENCOUNTERED
            else:
                face_nodes.add(cur_node)
                half_edges.append((prev_node, cur_node))
                # print('Half edge: ' + str((prev_node, cur_node)))
                if cur_node == self.d:
                    # Destination was reached
                    result_tag = ResultTag.SUCCESS
        return face_nodes, half_edges, result_tag

    def inside_bound(self, node):
        return self.searchable_area.contains_point(self.positions[node])

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
        # print('Bound was hit for the second time by this edge: ' + str((prev_node, cur_node)))
        return face_nodes, half_edges, result_tag
