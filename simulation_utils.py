from typing import List, Tuple
import numpy as np

deg2rad = np.pi / 180


class PathSegment:
    def __init__(self, start_point: np.ndarray, finish_point: np.ndarray):
        self.start_point = start_point
        self.finish_point = finish_point
        self.direction = self.finish_point - self.start_point
        self.length = np.linalg.norm(self.direction)
        self.direction = self.direction / self.length

    def project_point(self, point: np.ndarray) -> Tuple[float, float]:
        v = point - self.start_point
        v_parallel = np.dot(v, self.direction)
        v_normal = np.linalg.norm(v - v_parallel * self.direction)
        return v_parallel, v_normal

    def distance_from_point(self, point: np.ndarray):
        parallel, normal = self.project_point(point)
        if parallel < 0:
            return np.linalg.norm(point - self.start_point)
        else:
            if parallel > self.length:
                return np.linalg.norm(point - self.finish_point)
            else:
                return normal


class Path:
    def __init__(self, origin: Tuple[float, float], waypoints: np.ndarray):
        self.waypoints = waypoints

    def transform_to_path_coordinates(self, point: Tuple[float, float]) -> Tuple[float, float]:
        pass


class Servo:
    def __init__(self, initial_angle: float, max_angle: float, max_rate: float, time_const: float):
        self.angle = initial_angle
        self.max_angle = max_angle
        self.max_rate = max_rate
        self.time_const = time_const

    def angle_command(self, angle_command: float) -> float:
        pass


class Vehicle:
    def __init__(self, length: float, cg: float, x0: float, y0: float, psi0: float, v: float, dt: float):
        self.length = length
        self.lr = cg
        self.v = v

        self.x = x0
        self.y = y0
        self.psi = psi0
        self.delta = 0
        self.slip = np.arctan2(self.lr * np.tan(self.delta), self.length)

        self.x_dot = self.v * np.cos(self.psi)
        self.y_dot = self.v * np.cos(self.psi)
        self.psi_dot = self.v * np.tan(self.delta) * np.cos(self.slip) / self.length

        self.servo = Servo(self.delta, 45 * deg2rad, 20 * deg2rad, 0.2)

        self.dt = dt
        self.dlook_ahead = v*dt

        self.x_measured = self.x
        self.y_measured = self.y
        self.psi_measured = self.psi

    def advance(self):
        self.x_dot = self.v * np.cos(self.psi)
        self.y_dot = self.v * np.cos(self.psi)
        self.psi_dot = self.v * np.tan(self.delta) * np.cos(self.slip) / self.length
        self.x = self.x_dot * self.dt
        self.y = self.y_dot * self.dt
        self.psi = self.psi_dot * self.dt

    def transform_to_ego_coordinates(self, point: Tuple[float, float]) -> Tuple[float, float]:
        xg = point[0] - self.x
        yg = point[1] - self.y
        xe = np.cos(self.psi) * xg + np.sin(self.psi) * yg
        ye = -np.sin(self.psi) * xg + np.cos(self.psi) * yg
        return xe, ye

    def track_path(self, vehicle_path: Path) -> float:
        pass

    def estimate_position(self):
        # perfect estimation, for now
        self.x_measured = self.x
        self.y_measured = self.y
        self.psi_measured = self.psi


def handle_input_parameters(input_list: List[str]) -> Tuple[float, float, float, float, Path]:
    pass
