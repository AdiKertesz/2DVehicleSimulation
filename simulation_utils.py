from typing import List, Tuple
import numpy as np
from random import random

deg2rad = np.pi / 180


class PathSegment:
    """
    linear segment between two points
    """
    def __init__(self, start_point: np.ndarray, finish_point: np.ndarray):
        """

        :param start_point: in global coordinates
        :param finish_point: in global coordinates
        """
        self.start_point = start_point
        self.finish_point = finish_point
        self.direction = self.finish_point - self.start_point
        self.length = np.linalg.norm(self.direction)
        self.direction = self.direction / self.length

    def project_point(self, point: np.ndarray) -> Tuple[float, float]:
        """
        project a point on the segment
        :param point: ndarray with the global coordinates of the point
        :return: (parallel, normal) coordinates of the point
        """
        v = point - self.start_point
        v_parallel = np.dot(v, self.direction)
        v_normal = np.linalg.norm(v - v_parallel * self.direction)
        if v_parallel < 0:  # the point is before the segment starting point
            return 0, np.linalg.norm(point - self.start_point)
        else:  # the point is after the segment finishing point
            if v_parallel > self.length:
                return self.length, np.linalg.norm(point - self.finish_point)
            else:
                return v_parallel, v_normal


class Path:
    """
    a path connecting a series of waypoints
    """
    def __init__(self, waypoints: np.ndarray):
        """

        :param waypoints: 2d array of waypoint in global coordinates
        """
        self.origin = waypoints[0]
        self.waypoints = waypoints
        self.segment_list = []
        self.total_length = 0
        # create a list of segments between the waypoints
        for i in range(self.waypoints.shape[0] - 1):
            self.segment_list.append(PathSegment(self.waypoints[i], self.waypoints[i+1]))
            self.total_length += self.segment_list[-1].length

    def transform_to_path_coordinates(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """

        :param point: ndarray with the global coordinates of the point
        :return: (parallel, normal) coordinates of the point in path coordinates
        """
        p = np.array(point)
        minimal_distance = 1e100
        closest_segment_index = 0
        s = 0
        # find closest segment
        for idx, segment in enumerate(self.segment_list):
            s_segment, t_segment = segment.project_point(p)
            if t_segment < minimal_distance:
                closest_segment_index = idx
                minimal_distance = t_segment
                s = s_segment
        # account for the lengths of all previous segments
        for idx in range(closest_segment_index):
            s += self.segment_list[idx].length
        return s, minimal_distance

    def transform_to_global_coordinates(self, s: float) -> np.ndarray:
        """
        transformation from path coordinates to global coordinates
        :param s: path parallel coordinate
        :return: ndarray with the global coordinates of the point
        """
        if s < 0:
            return self.origin
        else:
            if s > self.total_length:
                return self.waypoints[self.waypoints.shape[0] - 1]
            else:
                segment_idx = 0
                # find the segment the point is on, and the parallel coordinate along that segment
                while s > self.segment_list[segment_idx].length:
                    s -= self.segment_list[segment_idx].length
                    segment_idx += 1
                return self.segment_list[segment_idx].start_point + s * self.segment_list[segment_idx].direction


class Servo:
    """
    actuator controlling the front wheel, 2nd order dynamics
    """
    def __init__(self, initial_angle: float, max_angle: float, max_rate: float, bandwidth: float, damping: float, dt: float):
        """

        :param initial_angle: [rad]
        :param max_angle: [rad]
        :param max_rate: [rad / sec]
        :param bandwidth: servo bandwidth
        :param damping: servo damping
        :param dt: simulated time interval
        """
        self.angle = initial_angle
        self.angle_rate = 0
        self.max_angle = max_angle
        self.max_rate = max_rate
        self.bandwidth = bandwidth
        self.damping = damping
        self.dt = dt

    def angle_command(self, angle_command: float) -> float:
        """
        command a new angle
        :param angle_command: [rad]
        :return: actual servo angle
        """
        # transfer function: angle_dot_dot / angle_command = w^2 / (s^2 + 2*zeta*s + w^2)
        w2 = self.bandwidth ** 2
        z2 = 2 * self.damping * self.bandwidth
        self.angle_rate += (w2 * (angle_command - self.angle) - z2 * self.angle_rate) * self.dt
        # rate limiter
        if np.abs(self.angle_rate) > self.max_rate:
            self.angle_rate = self.max_rate * np.sign(self.angle_rate)
        self.angle += self.angle_rate * self.dt
        # state limiter
        if np.abs(self.angle) > self.max_angle:
            self.angle = self.max_angle * np.sign(self.angle)

        return self.angle


class Vehicle:
    """
    encapsulating the kinematics and the tracking algorithm of the vehicle
    """
    def __init__(self, length: float, x0: float, y0: float, psi0: float, v: float, dt: float):
        """

        :param length: vehicle length [m]
        :param x0: initial position, x global coordinate
        :param y0: initial position, y global coordinate
        :param psi0: initial heading relative to global coordinates [rad]
        :param v: constant velocity [m/sec]
        :param dt: simulation time interval
        """
        self.length = length
        self.v = v
        # kinematic state
        self.x = x0
        self.y = y0
        self.psi = psi0
        self.delta = 0
        self.dt = dt
        # vehicle servo with damping of 0.7 and bandwidth corresponding to rise time of 0.2 sec
        self.servo = Servo(self.delta, 45 * deg2rad, 20 * deg2rad, 16.5, 0.7, dt)
        # aiming for a point 10 sec ahead
        self.dlook_ahead = v * 10
        # estimated kinematic state
        self.x_measured = self.x
        self.y_measured = self.y
        self.psi_measured = self.psi

    def advance(self):
        """
        advance the vehicle kinematics one time step
        """
        x_dot = self.v * np.cos(self.psi)
        y_dot = self.v * np.sin(self.psi)
        psi_dot = self.v * np.tan(self.delta) / self.length
        # euler method
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.psi += psi_dot * self.dt

    def transform_to_ego_coordinates(self, point: np.ndarray) -> np.ndarray:
        """
        transformation from global to ego coordinates
        :param point: ndarray with the global coordinates of the point
        :return: ndarray with the ego coordinates of the point
        """
        # use measured position
        xg = point[0] - self.x_measured
        yg = point[1] - self.y_measured
        # rotate by psi
        xe = np.cos(self.psi_measured) * xg + np.sin(self.psi_measured) * yg
        ye = -np.sin(self.psi_measured) * xg + np.cos(self.psi_measured) * yg
        return np.array([xe, ye])

    def intersect_ye_and_path(self, desired_path: Path) -> float:
        """
        find the closest intersection between ego y coordinate and a path
        :param desired_path: Path object to intersect
        :return: the path parallel coordinate of the intersection
        """
        minimum_ye_distance = 1e100
        s_ref_point = 0
        s_total = 0
        for segment in desired_path.segment_list:
            # rotate to ego
            start_point = self.transform_to_ego_coordinates(segment.start_point)
            finish_point = self.transform_to_ego_coordinates(segment.finish_point)
            if start_point[0] <= 0 and finish_point[0] >= 0:  # there is intersection
                rotated_segment = PathSegment(start_point, finish_point)
                if rotated_segment.direction[0] < 1e-10:  # segment direction is ye
                    if np.linalg.norm(start_point) < np.linalg.norm(finish_point):
                        ye_distance = np.linalg.norm(start_point)
                        s = 0
                    else:
                        ye_distance = np.linalg.norm(finish_point)
                        s = rotated_segment.length
                else:
                    s = -start_point[0] / rotated_segment.direction[0]
                    ye_distance = start_point[1] + s * rotated_segment.direction[1]
                if np.abs(ye_distance) < minimum_ye_distance:  # this is the closest intersection so far
                    minimum_ye_distance = np.abs(ye_distance)
                    s_ref_point = s_total + s
            s_total += segment.length
        return s_ref_point

    def track_path(self, desired_path: Path) -> float:
        """
        pure persuit tracking algorithm
        :param desired_path: path object to track
        :return: wheel angle command
        """
        s = self.intersect_ye_and_path(desired_path)
        ref_point = desired_path.transform_to_global_coordinates(s + self.dlook_ahead)
        ref_point_ego = self.transform_to_ego_coordinates(ref_point)
        rho = ref_point_ego[1] / (0.5 * np.linalg.norm(ref_point_ego) ** 2)  # 1 / R
        return np.arctan(self.length * rho)

    def estimate_position(self):
        """
        add estimation noise to measured position
        1 [m] in position, 0.5 [deg] in heading
        """
        self.x_measured = self.x + (1-2*random()) * 1
        self.y_measured = self.y + (1-2*random()) * 1
        self.psi_measured = self.psi + (1-2*random()) * 0.5 * deg2rad


def handle_input_parameters(input_list: List[str]) -> Tuple[float, float, float, float, Path]:
    """
    handle command line arguments
    :param input_list: command line input
    :return: (x0, y0, psi, v, path)
    """
    if not input_list:  # it's empty
        raise IOError('no inputs')
    if '-x0' not in input_list or '-y0' not in input_list or '-psi' not in input_list \
            or '-v' not in input_list or '-path' not in input_list:
        raise IOError('command line should be: python3 <pyfile> -x0 <x0> -y0 <y0 -psi <psi> -v <v> -path <desired_path_directory>')
    x0 = float(input_list[input_list.index('-x0') + 1])
    y0 = float(input_list[input_list.index('-y0') + 1])
    psi = float(input_list[input_list.index('-psi') + 1])
    v = float(input_list[input_list.index('-v') + 1])
    desired_path_directory = input_list[input_list.index('-path') + 1]
    # read csv file
    with open(desired_path_directory, 'r') as f:
        lines = f.readlines()
        f.close()
    waypoints = np.zeros((len(lines), 2))
    for idx, line in enumerate(lines):
        s = line[:-1].split(',')
        if len(s) < 2:
            break
        waypoints[idx][0] = s[0]
        waypoints[idx][1] = s[1]
    desired_path = Path(waypoints)
    return x0, y0, psi, v, desired_path
