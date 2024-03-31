from scipy.spatial import distance

from RoutingAlgos.geomentricRouting.OAFR import create_ellipse


def direct_greedy_routing_mode (g, s, d, route, positions, ellipse, L, N, R, PpCurrent, PpNext, rho):
    if N != 0:
        # forward the message to Node the N-field indicated
        pass
    else:
        #if ID == t:
        #    return True
        #elif local minimum:
        #    direct_adaptive_face_routing_mode()
        #else:
            # Forward to the adjacent node i geographically closest to t
            # if crossed ellipse
                # Increase width to rho * width
        #N = ID
        #L = last node's ID
        direct_greedy_routing_mode()

def direct_greedy_adaptive_face_routing (g, s, d, route, positions, previous_face, ellipse, rho_0):

    ellipse = create_ellipse(positions[s], positions[d], width= rho_0 * distance.euclidean(positions[s], positions[d]))

    direct_greedy_routing_mode(g, s, d, [], positions, ellipse, rho_0)

def direct_adaptive_face_routing_mode (g, s, d, route, positions, previous_face, ellipse, sigma):
    pass