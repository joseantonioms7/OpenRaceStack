"""
System Identification Engine for OpenRaceStack.

Provides Recursive Least Squares (RLS) with exponential forgetting and Batch Least Squares
for estimating dynamic vehicle parameters from sensor telemetry streams:
- Front and rear tire cornering stiffness (C_alpha_f, C_alpha_r)
- Road surface adhesion friction coefficient (mu)
- Vehicle mass and rotational inertia consistency
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .dynamics import F1TENTH_PARAMS, VehicleParameters


@dataclass
class SysIdEstimate:
    """Estimated physical vehicle parameters and statistical confidence."""
    c_alpha_f: float                     # Front tire cornering stiffness (N/rad)
    c_alpha_r: float                     # Rear tire cornering stiffness (N/rad)
    mu_peak: float                       # Estimated peak surface friction coefficient
    r_squared: float = 0.0               # Coefficient of determination R^2 [0.0, 1.0]
    covariance: np.ndarray = field(default_factory=lambda: np.eye(2))
    sample_count: int = 0


class RecursiveLeastSquares:
    """
    Multi-parameter Recursive Least Squares (RLS) with exponential forgetting factor.
    
    Update equations:
        K_k = P_{k-1} phi_k / (lambda + phi_k^T P_{k-1} phi_k)
        theta_k = theta_{k-1} + K_k * (y_k - phi_k^T theta_{k-1})
        P_k = (I - K_k phi_k^T) P_{k-1} / lambda
    """

    def __init__(self, n_params: int = 2, forgetting_factor: float = 0.98, initial_covariance: float = 1000.0):
        self.n_params = int(n_params)
        self.lambda_f = float(forgetting_factor)
        self.theta = np.zeros(self.n_params, dtype=np.float64)
        self.P = np.eye(self.n_params, dtype=np.float64) * initial_covariance
        self.update_count = 0

    def initialize(self, initial_theta: np.ndarray, initial_covariance: float | None = None) -> None:
        """Sets prior parameter vector."""
        self.theta = np.asarray(initial_theta, dtype=np.float64).copy()
        if initial_covariance is not None:
            self.P = np.eye(self.n_params, dtype=np.float64) * initial_covariance

    def update(self, phi: np.ndarray, y: float) -> np.ndarray:
        """
        Ingests regressor vector phi and scalar observation y to update theta.

        Args:
            phi: Regressor vector of shape (n_params,)
            y: Observed target scalar

        Returns:
            Updated parameter vector theta.
        """
        phi_vec = np.asarray(phi, dtype=np.float64).reshape(-1, 1)
        y_val = float(y)

        # Innovation residual
        y_hat = float((phi_vec.T @ self.theta).item())
        error = y_val - y_hat

        # Kalman gain vector
        P_phi = self.P @ phi_vec
        denom = self.lambda_f + float((phi_vec.T @ P_phi).item())
        K = P_phi / max(denom, 1e-12)

        # Parameter state update
        self.theta += (K * error).flatten()

        # Covariance update (Joseph-stabilized form)
        I_KH = np.eye(self.n_params) - K @ phi_vec.T
        self.P = (I_KH @ self.P @ I_KH.T + (1e-6 * np.eye(self.n_params))) / self.lambda_f

        self.update_count += 1
        return self.theta.copy()


class OnlineVehicleSystemIdentifier:
    """
    Online Recursive Estimator tracking tire cornering stiffnesses (C_alpha_f, C_alpha_r)
    and road friction coefficient (mu) at high rates from telemetry.
    """

    def __init__(
        self,
        params: VehicleParameters = F1TENTH_PARAMS,
        forgetting_factor: float = 0.985,
        min_excitation_speed_mps: float = 1.0,
        min_slip_angle_rad: float = 0.005,
    ):
        self.params = params
        self.min_speed = float(min_excitation_speed_mps)
        self.min_slip = float(min_slip_angle_rad)

        # 2-parameter RLS: theta = [C_alpha_f, C_alpha_r]
        self.rls = RecursiveLeastSquares(
            n_params=2,
            forgetting_factor=forgetting_factor,
            initial_covariance=500.0,
        )
        # Initialize with nominal values from vehicle parameters
        nominal_c_f = self.params.mass * 9.81 * (self.params.lr / self.params.wheelbase) * 0.5
        nominal_c_r = self.params.mass * 9.81 * (self.params.lf / self.params.wheelbase) * 0.5
        self.rls.initialize(np.array([nominal_c_f, nominal_c_r]))

        self.estimated_mu: float = 1.0
        self._mu_window: list[float] = []

    def update_from_telemetry(
        self,
        vx: float,
        vy: float,
        omega_z: float,
        steer_angle: float,
        ay_measured: float,
        ax_measured: float = 0.0,
        dt: float = 0.05,
    ) -> SysIdEstimate:
        """
        Updates parameter estimates using single-track lateral bicycle dynamics:
            m * a_y = C_alpha_f * alpha_f + C_alpha_r * alpha_r
        """
        vx_safe = max(abs(vx), 0.1)

        # Calculate slip angles
        alpha_f = steer_angle - math.atan2(vy + self.params.lf * omega_z, vx_safe)
        alpha_r = -math.atan2(vy - self.params.lr * omega_z, vx_safe)

        # Update RLS only if vehicle is in motion and exhibits persistent excitation
        if vx >= self.min_speed and (abs(alpha_f) >= self.min_slip or abs(alpha_r) >= self.min_slip):
            # Regressor phi = [alpha_f * cos(delta), alpha_r]
            phi = np.array([alpha_f * math.cos(steer_angle), alpha_r], dtype=np.float64)
            y_obs = self.params.mass * ay_measured
            self.rls.update(phi, y_obs)

            # Enforce physical positivity on cornering stiffness
            self.rls.theta[0] = max(1.0, self.rls.theta[0])
            self.rls.theta[1] = max(1.0, self.rls.theta[1])

            # Surface friction observation: total acceleration norm relative to gravity
            total_g = math.hypot(ax_measured, ay_measured) / 9.81
            if abs(alpha_f) > 0.05 or abs(alpha_r) > 0.05:  # high-slip regime
                self._mu_window.append(total_g)
                if len(self._mu_window) > 50:
                    self._mu_window.pop(0)
                self.estimated_mu = float(np.clip(np.percentile(self._mu_window, 90), 0.2, 2.0))

        return SysIdEstimate(
            c_alpha_f=float(self.rls.theta[0]),
            c_alpha_r=float(self.rls.theta[1]),
            mu_peak=float(self.estimated_mu),
            covariance=self.rls.P.copy(),
            sample_count=self.rls.update_count,
        )


class BatchSystemIdentifier:
    """
    Offline batch parameter identification using constrained multivariate least squares.
    """

    def __init__(self, params: VehicleParameters = F1TENTH_PARAMS):
        self.params = params

    def identify_lateral_parameters(
        self,
        vx_arr: np.ndarray,
        vy_arr: np.ndarray,
        omega_arr: np.ndarray,
        delta_arr: np.ndarray,
        ay_arr: np.ndarray,
    ) -> SysIdEstimate:
        """
        Fits tire stiffnesses C_alpha_f and C_alpha_r across recorded batch telemetry.
        """
        vx = np.asarray(vx_arr, dtype=np.float64)
        vy = np.asarray(vy_arr, dtype=np.float64)
        omega = np.asarray(omega_arr, dtype=np.float64)
        delta = np.asarray(delta_arr, dtype=np.float64)
        ay = np.asarray(ay_arr, dtype=np.float64)

        mask = vx > 1.0
        if np.sum(mask) < 10:
            raise ValueError("Insufficient dynamic excitation samples for batch identification.")

        vx_m = vx[mask]
        vy_m = vy[mask]
        om_m = omega[mask]
        del_m = delta[mask]
        ay_m = ay[mask]

        # Compute tire slip angles
        alpha_f = del_m - np.arctan2(vy_m + self.params.lf * om_m, vx_m)
        alpha_r = -np.arctan2(vy_m - self.params.lr * om_m, vx_m)

        # Regressor matrix Phi = [alpha_f * cos(delta), alpha_r]
        Phi = np.column_stack((alpha_f * np.cos(del_m), alpha_r))
        Y = self.params.mass * ay_m

        # Regularized Ordinary Least Squares
        reg = 1e-3 * np.eye(2)
        theta, _, _, _ = np.linalg.lstsq(Phi.T @ Phi + reg, Phi.T @ Y, rcond=None)

        c_f = float(max(1.0, theta[0]))
        c_r = float(max(1.0, theta[1]))

        # Calculate goodness of fit R^2
        y_pred = Phi @ np.array([c_f, c_r])
        ss_res = np.sum((Y - y_pred) ** 2)
        ss_tot = np.sum((Y - np.mean(Y)) ** 2)
        r2 = float(1.0 - (ss_res / max(ss_tot, 1e-6)))

        # Estimate peak friction from top 5% lateral accelerations
        mu_est = float(np.percentile(np.abs(ay_m), 95) / 9.81)

        cov = np.linalg.pinv(Phi.T @ Phi + reg)

        return SysIdEstimate(
            c_alpha_f=c_f,
            c_alpha_r=c_r,
            mu_peak=float(np.clip(mu_est, 0.2, 2.5)),
            r_squared=float(np.clip(r2, 0.0, 1.0)),
            covariance=cov,
            sample_count=len(ay_m),
        )
