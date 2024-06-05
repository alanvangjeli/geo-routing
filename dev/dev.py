import math
import numpy as np

position_upper_bound = 20
k = 44000
interval_length = 2000
number_of_network_densities = int(k / interval_length)
network_density_limit = 22
node_increase_factor = math.ceil(
    position_upper_bound**2
    * network_density_limit
    / np.pi
    / number_of_network_densities
)
iteration_parameters = [
    k,
    interval_length,
    number_of_network_densities,
    node_increase_factor,
]
for parameter in iteration_parameters:
    if not float(parameter).is_integer() and not isinstance(parameter, int):
        print(parameter)
        raise ValueError("Invalid parameters")
    
print(node_increase_factor)
