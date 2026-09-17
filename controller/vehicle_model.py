"""Kinematic bicycle model for a differential-drive-like mobile robot."""

import numpy as np


class VehicleModel:
    """Discrete kinematic bicycle model.

    State: [x, y, yaw, v]
    Control: [steer, accel]

    Dynamics (Euler forward):
        x_{t+1}    = x_t + v_t * cos(yaw_t) * dt
        y_{t+1}    = y_t + v_t * sin(yaw_t) * dt
        yaw_{t+1}  = yaw_t + (v_t / L) * tan(steer_t) * dt
        v_{t+1}    = v_t + accel_t * dt
    """

    def __init__(self, dt: float = 0.1, wheelbase: float = 0.3):
        self.dt = dt
        self.L = wheelbase

    def step(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """Advance state by one time step.

        Args:
            state: [x, y, yaw, v]
            control: [steer, accel]

        Returns:
            New state [x, y, yaw, v].
        """
        x, y, yaw, v = state
        steer, accel = control

        x_new = x + v * np.cos(yaw) * self.dt
        y_new = y + v * np.sin(yaw) * self.dt
        yaw_new = yaw + (v / self.L) * np.tan(steer) * self.dt
        v_new = v + accel * self.dt

        return np.array([x_new, y_new, yaw_new, v_new])

    def linearize(self, state: np.ndarray, control: np.ndarray):
        """Compute Jacobians A, B for linearization around (state, control).

        Returns:
            (A, B) matrices for x_{t+1} ≈ A * x_t + B * u_t + c
        """
        x, y, yaw, v = state
        steer, accel = control
        dt = self.dt
        L = self.L

        A = np.array([
            [1, 0, -v * np.sin(yaw) * dt, np.cos(yaw) * dt],
            [0, 1,  v * np.cos(yaw) * dt, np.sin(yaw) * dt],
            [0, 0,  1,                      np.tan(steer) / L * dt],
            [0, 0,  0,                      1],
        ])

        B = np.array([
            [0,                          0],
            [0,                          0],
            [v / (L * np.cos(steer) ** 2) * dt, 0],
            [0,                          dt],
        ])

        # Constant term (affine part)
        c = self.step(state, control) - A @ state - B @ control

        return A, B, c
