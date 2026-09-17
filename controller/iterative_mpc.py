"""Iterative Model Predictive Control using CVXPY."""

import numpy as np
import cvxpy as cp

from .vehicle_model import VehicleModel


class IterativeMPC:
    """Iterative Linear MPC for trajectory tracking.

    At each solve call, the nonlinear dynamics are linearized around
    a nominal trajectory (initialized from the reference), then the
    convex QP is solved. This repeats for `num_iterations` rounds.
    """

    def __init__(self, config: dict):
        self.dt = config.get("dt", 0.1)
        self.N = config.get("horizon", 10)
        self.Q = np.diag(config.get("Q", [1.0, 1.0, 0.5, 0.1]))
        self.R = np.diag(config.get("R", [0.1, 0.1]))
        self.max_steer = config.get("max_steer", 0.5)
        self.max_accel = config.get("max_accel", 2.0)
        self.max_speed = config.get("max_speed", 3.0)
        self.L = config.get("vehicle_length", 0.3)
        self.num_iter = config.get("num_iterations", 3)

        self.vehicle = VehicleModel(dt=self.dt, wheelbase=self.L)

    def solve(
        self,
        state: np.ndarray,
        ref_trajectory: np.ndarray,
    ) -> tuple[np.ndarray, str]:
        """Solve MPC for current state against reference trajectory.

        Args:
            state: Current state [x, y, yaw, v].
            ref_trajectory: Reference states, shape (N+1, 4) or (N, 4).
                            If (N, 4), the last reference is repeated.

        Returns:
            (control, status) where control = [steer, accel] and
            status is "optimal", "infeasible", or "solver_error".
        """
        ref = np.array(ref_trajectory, dtype=float)
        if ref.shape[0] < self.N + 1:
            # Pad with last row
            pad = np.tile(ref[-1:], (self.N + 1 - ref.shape[0], 1))
            ref = np.vstack([ref, pad])
        ref = ref[: self.N + 1]

        # Iterative linearization
        x_nom = np.zeros((self.N + 1, 4))
        x_nom[0] = state
        u_nom = np.zeros((self.N, 2))

        for iteration in range(self.num_iter):
            x_opt, u_opt, status = self._solve_qp(state, ref, x_nom, u_nom)
            if status == "optimal":
                x_nom = x_opt
                u_nom = u_opt
            else:
                break

        if status == "optimal":
            return u_opt[0], status
        else:
            # Fallback: proportional controller toward first reference point
            return self._fallback_control(state, ref[0]), "fallback"

    def _solve_qp(
        self,
        x0: np.ndarray,
        ref: np.ndarray,
        x_nom: np.ndarray,
        u_nom: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, str]:
        """Solve the convex QP around nominal trajectory."""
        N = self.N
        n_x, n_u = 4, 2

        # Linearize around nominal
        A_list, B_list, c_list = [], [], []
        for k in range(N):
            A, B, c = self.vehicle.linearize(x_nom[k], u_nom[k])
            A_list.append(A)
            B_list.append(B)
            c_list.append(c)

        # Decision variables
        x = cp.Variable((N + 1, n_x))
        u = cp.Variable((N, n_u))

        # Initial state constraint
        constraints = [x[0] == x0]

        # Dynamics constraints (linearized)
        for k in range(N):
            constraints.append(
                x[k + 1] == A_list[k] @ x[k] + B_list[k] @ u[k] + c_list[k]
            )

        # Input constraints
        for k in range(N):
            constraints += [
                cp.abs(u[k, 0]) <= self.max_steer,
                cp.abs(u[k, 1]) <= self.max_accel,
            ]

        # Speed constraint
        for k in range(N + 1):
            constraints += [
                x[k, 3] >= -0.5,
                x[k, 3] <= self.max_speed,
            ]

        # Cost
        cost = 0
        for k in range(N):
            err = x[k] - ref[k]
            cost += cp.quad_form(err, self.Q)
            cost += cp.quad_form(u[k], self.R)
        # Terminal cost
        err_terminal = x[N] - ref[N]
        cost += 2.0 * cp.quad_form(err_terminal, self.Q)

        prob = cp.Problem(cp.Minimize(cost), constraints)

        try:
            prob.solve(solver=cp.OSQP, warm_start=True, verbose=False,
                       max_iter=2000, eps_abs=1e-4, eps_rel=1e-4)
        except cp.SolverError:
            try:
                prob.solve(solver=cp.SCS, verbose=False)
            except cp.SolverError:
                return x_nom, u_nom, "solver_error"

        if prob.status in ("optimal", "optimal_inaccurate"):
            return x.value, u.value, "optimal"
        else:
            return x_nom, u_nom, "infeasible"

    def _fallback_control(self, state: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Simple proportional controller as fallback."""
        dx = target[0] - state[0]
        dy = target[1] - state[1]
        target_yaw = np.arctan2(dy, dx)
        yaw_err = target_yaw - state[2]
        # Normalize to [-pi, pi]
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi

        steer = np.clip(yaw_err * 2.0, -self.max_steer, self.max_steer)
        dist = (dx ** 2 + dy ** 2) ** 0.5
        accel = np.clip(dist * 1.0, -self.max_accel, self.max_accel)

        return np.array([steer, accel])
