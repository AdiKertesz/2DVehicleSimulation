import sys
from simulation_utils import handle_input_parameters, Vehicle

if __name__ == '__main__':

    # simulation parameters
    dt = 0.1  # [sec]
    t_max = 300  # [sec]
    vehicle_length = 2.728  # [m] Ford Fusion Wheelbase
    vehicle_cg = vehicle_length / 2.0  # [m] didn't find the data, try again late

    # initialize state vector
    t = 0
    x0, y0, psi0, v, path = handle_input_parameters(sys.argv)

    vehicle = Vehicle(vehicle_length, vehicle_cg, x0, y0, psi0, v, dt)

    while t < t_max:
        vehicle.advance()
        vehicle.estimate_position()
        delta_command = vehicle.track_path(path)
        vehicle.servo.angle_command(delta_command)

        t += dt


