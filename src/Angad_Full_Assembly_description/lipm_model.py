#!/usr/bin/env python3
"""
lipm_model.py — Linear Inverted Pendulum Model for Angad Humanoid
=================================================================
Implements the 3D Linear Inverted Pendulum Model (LIPM) to generate
Center of Mass (CoM) trajectories for bipedal walking.

The LIPM constrains the CoM to move on a horizontal plane at height z_com,
and the dynamics in X and Y decouple:

    x(t) = x0 * cosh(ω*t) + (ẋ0/ω) * sinh(ω*t)
    ẋ(t) = x0 * ω * sinh(ω*t) + ẋ0 * cosh(ω*t)

where ω = sqrt(g / z_com) is the natural frequency.
"""

import numpy as np
from typing import Tuple


class LIPMModel:
    """
    Linear Inverted Pendulum Model for CoM trajectory generation.

    Parameters:
        z_com:    height of the CoM plane [m] (default: 0.75, slightly crouched)
        gravity:  gravitational acceleration [m/s²] (default: 9.81)
        dt:       time step for trajectory generation [s] (default: 0.01 = 100 Hz)
    """

    def __init__(
        self,
        z_com: float = 0.75,
        gravity: float = 9.81,
        dt: float = 0.01,
    ):
        self.z_com = z_com
        self.gravity = gravity
        self.dt = dt

        # Natural frequency of the inverted pendulum
        self.omega = np.sqrt(gravity / z_com)  # ω = sqrt(g/z_com)

        # Time constant for exponential divergence
        self.T_c = 1.0 / self.omega

    def propagate_1d(
        self,
        x0: float,
        xdot0: float,
        t: float,
    ) -> Tuple[float, float]:
        """
        Propagate 1D LIPM state forward by time t.

        The LIPM solution for a given support point at origin is:
            x(t) = x0 * cosh(ω*t) + (ẋ0/ω) * sinh(ω*t)
            ẋ(t) = x0 * ω * sinh(ω*t) + ẋ0 * cosh(ω*t)

        Here x0 and xdot0 are measured relative to the center of pressure (CoP).

        Args:
            x0:    initial position relative to CoP [m]
            xdot0: initial velocity [m/s]
            t:     time to propagate [s]

        Returns:
            (x, xdot) at time t, relative to CoP
        """
        cosh_wt = np.cosh(self.omega * t)
        sinh_wt = np.sinh(self.omega * t)

        x = x0 * cosh_wt + (xdot0 / self.omega) * sinh_wt
        xdot = x0 * self.omega * sinh_wt + xdot0 * cosh_wt

        return x, xdot

    def compute_initial_velocity(
        self,
        x0: float,
        x_end: float,
        T: float,
    ) -> float:
        """
        Compute the initial velocity needed to reach x_end from x0 in time T.

        From the LIPM solution:
            x_end = x0 * cosh(ωT) + (ẋ0/ω) * sinh(ωT)
            => ẋ0 = ω * (x_end - x0 * cosh(ωT)) / sinh(ωT)

        Both positions are relative to the CoP (support foot).

        Args:
            x0:    initial position relative to CoP [m]
            x_end: desired final position relative to CoP [m]
            T:     total time for the transfer [s]

        Returns:
            required initial velocity [m/s]
        """
        cosh_wT = np.cosh(self.omega * T)
        sinh_wT = np.sinh(self.omega * T)

        if abs(sinh_wT) < 1e-10:
            return 0.0

        xdot0 = self.omega * (x_end - x0 * cosh_wT) / sinh_wT
        return xdot0

    def generate_com_trajectory(
        self,
        cop_position: np.ndarray,
        com_start: np.ndarray,
        com_vel_start: np.ndarray,
        duration: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate a CoM trajectory over a single support phase.

        The trajectory is computed by propagating the LIPM in X and Y independently,
        relative to the center of pressure (CoP = support foot position).

        Args:
            cop_position:  (x, y) position of the center of pressure [m]
            com_start:     (x, y) initial CoM position in world frame [m]
            com_vel_start: (vx, vy) initial CoM velocity [m/s]
            duration:      duration of this support phase [s]

        Returns:
            times:    array of time stamps [s], shape (N,)
            com_pos:  CoM positions over time, shape (N, 2) — (x, y)
            com_vel:  CoM velocities over time, shape (N, 2) — (vx, vy)
        """
        num_steps = int(duration / self.dt) + 1
        times = np.linspace(0.0, duration, num_steps)

        com_pos = np.zeros((num_steps, 2))
        com_vel = np.zeros((num_steps, 2))

        # Relative initial conditions (CoM w.r.t. CoP)
        x0_rel = com_start[0] - cop_position[0]
        y0_rel = com_start[1] - cop_position[1]
        vx0 = com_vel_start[0]
        vy0 = com_vel_start[1]

        for i, t in enumerate(times):
            # Propagate X
            x_rel, vx = self.propagate_1d(x0_rel, vx0, t)
            # Propagate Y
            y_rel, vy = self.propagate_1d(y0_rel, vy0, t)

            # Convert back to world frame
            com_pos[i, 0] = x_rel + cop_position[0]
            com_pos[i, 1] = y_rel + cop_position[1]
            com_vel[i, 0] = vx
            com_vel[i, 1] = vy

        return times, com_pos, com_vel

    def compute_orbital_energy(self, x: float, xdot: float) -> float:
        """
        Compute the orbital energy of the LIPM.

        E = 0.5 * ẋ² - 0.5 * ω² * x²

        Positive energy => the pendulum diverges (walking).
        Zero energy => the pendulum converges to the support point (standing).

        Args:
            x:    position relative to CoP [m]
            xdot: velocity [m/s]

        Returns:
            orbital energy [m²/s²]
        """
        return 0.5 * xdot**2 - 0.5 * self.omega**2 * x**2


if __name__ == "__main__":
    # ── Quick test: single support phase ──
    lipm = LIPMModel(z_com=0.75)
    print(f"LIPM natural frequency: ω = {lipm.omega:.3f} rad/s")
    print(f"Time constant: T_c = {lipm.T_c:.3f} s")

    # Simulate CoM moving from above left foot to above right foot
    cop = np.array([0.0, -0.064])      # support at right foot
    com0 = np.array([0.0, 0.0])         # CoM starts at center
    vel0 = np.array([0.03, -0.05])      # slight forward + lateral velocity

    times, pos, vel = lipm.generate_com_trajectory(cop, com0, vel0, duration=0.8)

    print(f"\nCoM trajectory over 0.8s ({len(times)} samples):")
    print(f"  Start: ({pos[0,0]:.4f}, {pos[0,1]:.4f})")
    print(f"  End:   ({pos[-1,0]:.4f}, {pos[-1,1]:.4f})")
    print(f"  Vel start: ({vel[0,0]:.4f}, {vel[0,1]:.4f})")
    print(f"  Vel end:   ({vel[-1,0]:.4f}, {vel[-1,1]:.4f})")
