from typing import List, Tuple
import numpy as np


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
    def __init__(self, x0: float, y0: float, psi0: float, v: float):
        self.v = v

        self.x = x0
        self.y = y0
        self.psi = psi0
        self.delta = 0

        self.x_dot = self.v * np.cos(self.psi)
        self.y_dot = self.v * np.cos(self.psi)
        self.psi_dot = 0
        self.delta_dot = 0

        self.servo = Servo(self.delta, 45, 20, 0.2)

    def advance(self, dt: float):
        pass

    def transform_to_ego_coordinates(self, point: Tuple[float, float]) -> Tuple[float, float]:
        pass


def handle_input_parameters(input_list: List[str]) -> Tuple[float, float, float, float, Path]:
    pass
