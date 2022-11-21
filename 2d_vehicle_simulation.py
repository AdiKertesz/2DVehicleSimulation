import sys
from simulation_utils import handle_input_parameters, Vehicle
from matplotlib import pyplot as plt

if __name__ == '__main__':

    # simulation parameters
    dt = 0.1  # [sec]
    t_max = 300  # [sec]
    vehicle_length = 2.728  # [m] Ford Fusion Wheelbase

    # initialize vehicle object
    x0, y0, psi0, v, vehicle_path = handle_input_parameters(sys.argv)
    vehicle = Vehicle(vehicle_length, x0, y0, psi0, v, dt)

    # vectors for plotting
    x = [x0]
    y = [y0]
    time = [0]

    while time[-1] < t_max:
        # kinematics
        vehicle.advance()
        # measurement / estimation
        vehicle.estimate_position()
        # tracking logic
        delta_command = vehicle.track_path(vehicle_path)
        # servo dynamics
        vehicle.delta = vehicle.servo.angle_command(delta_command)
        # position in path coordinates
        s, t = vehicle_path.transform_to_path_coordinates((vehicle.x, vehicle.y))

        x.append(vehicle.x)
        y.append(vehicle.y)
        time.append(time[-1] + dt)
        if s >= vehicle_path.total_length:
            break

    plt.plot(vehicle_path.waypoints[:, 0], vehicle_path.waypoints[:, 1])
    plt.plot(x, y)
    plt.xlabel('Xg')
    plt.xlabel('Yg')
    plt.legend(['Desired path', 'Vehicle path'])
    plt.grid()
    plt.show()
