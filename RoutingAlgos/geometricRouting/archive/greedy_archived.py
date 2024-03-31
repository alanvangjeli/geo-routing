def base_greedy_route(self, g: nx.DiGraph, s: int, d: int, route: list, positions: dict, ellipse: Ellipse | None,
                      circle: Circle | None, rho: float | None,
                      protocol: str) -> tuple[bool, list[int], Ellipse, Circle, str]:
    """
    Greedy route
    @param g - Graph to route on
    @param s - Source node
    @param d - Destination node
    @param route - Route so far
    @param positions - Positions of nodes
    @param ellipse - Ellipse that bounds face traversal
    @param protocol - Protocol to use
    @param circle - Circle that bounds face traversal
    @param rho - Value to divide circle radius by
    """
    current_node = s
    if current_node != d:
        route.append(current_node)

    while current_node != d:
        neighbors = [node for node in g.neighbors(current_node)]
        if len(neighbors) == 0:
            return False, route, ellipse, circle, ResultTag.DEAD_END
        else:
            distances = {node: distance.euclidean(positions[node], positions[d]) for node in neighbors}
            min_distance_neighbor, min_distance = min(distances.items(), key=itemgetter(1))
            current_node_distance = distance.euclidean(positions[current_node], positions[d])
            if min_distance < current_node_distance:
                # GOAFR
                if protocol == 'goafr' and not ellipse.contains_point(positions[min_distance_neighbor]):
                    ellipse.set_width(ellipse.width * 2)
                    print('Ellipse width doubled')
                current_node = min_distance_neighbor
                # GOAFR+
                if protocol == 'goafr+':
                    current_radius = circle.radius
                    circle.set_radius(circle.radius / rho)
                    if not circle.contains_point(positions[current_node]):
                        circle.set_radius(current_radius)
                    print('Circle radius divided by p')
                print('Next node: ' + str(current_node))
                route.append(current_node)
            else:
                return False, route, ellipse, circle, ResultTag.LOCAL_MINIMUM

    return True, route, ellipse, circle, ResultTag.SUCCESS