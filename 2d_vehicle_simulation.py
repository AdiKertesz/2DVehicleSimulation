import sys
import pandas as pd
from simulation_utils import handle_input_parameters, Vehicle

if __name__ == '__main__':

    # simulation parameters
    dt = 0.1  # [sec]
    t_max = 300  # [sec]

    # initialize state vector
    t = 0
    x0, y0, psi0, v, path = handle_input_parameters(sys.argv)

    vehicle = Vehicle(x0, y0, psi0, v)

    while t < t_max:

        t += dt


