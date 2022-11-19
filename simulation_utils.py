from typing import List, Tuple
import numpy as np

deg2rad = np.pi / 180


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
        pass

    def track_path(self, path: Path) -> float:
        pass

    def estimate_position(self):
        # perfect estimation, for now
        self.x_measured = self.x
        self.y_measured = self.y
        self.psi_measured = self.psi


def handle_input_parameters(input_list: List[str]) -> Tuple[float, float, float, float, Path]:
    pass
