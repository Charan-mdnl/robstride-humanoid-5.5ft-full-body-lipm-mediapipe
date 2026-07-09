#!/usr/bin/env python3
"""
foot_trajectory.py — Swing Foot Trajectory Generator for Angad Humanoid
========================================================================
Generates smooth 3D foot trajectories for the swing leg during walking.

The trajectory uses:
  - Linear interpolation in X and Y (from start to end position)
  - Parabolic (quadratic) arc in Z (lift and land smoothly)

The phase φ ∈ [0, 1] parameterizes the trajectory over the swing duration:
  - φ = 0: foot at start position (lift-off)
  - φ = 0.5: foot at maximum height (mid-swing)
  - φ = 1: foot at end position (touch-down)
"""

import numpy as np
from typing import Tuple


class FootTrajectoryGenerator:
    """
    Generates swing foot trajectories with a parabolic height profile.

    Parameters:
        step_height: maximum foot clearance during swing [m] (default: 0.04)
        land_offset: small negative Z offset at touchdown for ground contact [m] (default: -0.005)
    """

    def __init__(
        self,
        step_height: float = 0.04,
        land_offset: float = -0.005,
    ):
        self.step_height = step_height
        self.land_offset = land_offset

    def compute_foot_position(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        phase: float,
    ) -> np.ndarray:
        """
        Compute the 3D foot position at a given phase of the swing.

        Args:
            start_pos: (x, y, z) foot position at lift-off [m]
            end_pos:   (x, y, z) target foot position at touch-down [m]
            phase:     swing phase φ ∈ [0, 1]

        Returns:
            foot_pos: (x, y, z) foot position at the given phase [m]
        """
        phase = np.clip(phase, 0.0, 1.0)

        # ── X and Y: linear interpolation ──
        x = start_pos[0] + (end_pos[0] - start_pos[0]) * phase
        y = start_pos[1] + (end_pos[1] - start_pos[1]) * phase

        # ── Z: parabolic arc ──
        # z(φ) = z_start + 4*h*φ*(1-φ)
        # This gives z(0) = z_start, z(0.5) = z_start + h, z(1) = z_start
        z_ground = start_pos[2] + (end_pos[2] - start_pos[2]) * phase
        z_swing = 4.0 * self.step_height * phase * (1.0 - phase)

        # Near touchdown (φ > 0.9), add slight downward offset for firm contact
        if phase > 0.95:
            z_landing = self.land_offset * (phase - 0.95) / 0.05
        else:
            z_landing = 0.0

        z = z_ground + z_swing + z_landing

        return np.array([x, y, z])

    def compute_foot_velocity(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        phase: float,
        swing_duration: float,
    ) -> np.ndarray:
        """
        Compute the 3D foot velocity at a given phase (for smooth interpolation).

        Args:
            start_pos:      (x, y, z) foot position at lift-off [m]
            end_pos:        (x, y, z) target foot position at touch-down [m]
            phase:          swing phase φ ∈ [0, 1]
            swing_duration: total duration of the swing phase [s]

        Returns:
            foot_vel: (vx, vy, vz) foot velocity [m/s]
        """
        phase = np.clip(phase, 0.0, 1.0)
        dphi_dt = 1.0 / swing_duration if swing_duration > 0 else 0.0

        # dX/dt = (x_end - x_start) / T_swing
        vx = (end_pos[0] - start_pos[0]) * dphi_dt
        vy = (end_pos[1] - start_pos[1]) * dphi_dt

        # dZ/dφ = 4*h*(1 - 2φ), then dZ/dt = dZ/dφ * dφ/dt
        dz_dphi = 4.0 * self.step_height * (1.0 - 2.0 * phase)
        vz = (end_pos[2] - start_pos[2]) * dphi_dt + dz_dphi * dphi_dt

        return np.array([vx, vy, vz])

    def generate_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        swing_duration: float,
        dt: float = 0.01,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate the full swing foot trajectory as arrays.

        Args:
            start_pos:      (x, y, z) lift-off position [m]
            end_pos:        (x, y, z) touch-down position [m]
            swing_duration: duration of the swing in seconds [s]
            dt:             time step [s] (default: 0.01)

        Returns:
            times:     time stamps array [s], shape (N,)
            positions: foot positions over time, shape (N, 3) — (x, y, z)
            velocities: foot velocities over time, shape (N, 3) — (vx, vy, vz)
        """
        num_samples = int(swing_duration / dt) + 1
        times = np.linspace(0.0, swing_duration, num_samples)
        positions = np.zeros((num_samples, 3))
        velocities = np.zeros((num_samples, 3))

        for i, t in enumerate(times):
            phase = t / swing_duration if swing_duration > 0 else 1.0
            positions[i] = self.compute_foot_position(start_pos, end_pos, phase)
            velocities[i] = self.compute_foot_velocity(start_pos, end_pos, phase, swing_duration)

        return times, positions, velocities

    def compute_stance_foot_position(
        self,
        foot_pos: np.ndarray,
    ) -> np.ndarray:
        """
        Return the position of the stance foot (stays on the ground).

        The stance foot doesn't move during the step; it stays planted.

        Args:
            foot_pos: (x, y, z) position where the foot was placed [m]

        Returns:
            stance_pos: (x, y, z) same position on the ground [m]
        """
        return np.array([foot_pos[0], foot_pos[1], 0.0])


if __name__ == "__main__":
    # ── Quick test ──
    gen = FootTrajectoryGenerator(step_height=0.04)

    start = np.array([0.0, 0.064, 0.0])
    end = np.array([0.06, 0.064, 0.0])

    times, pos, vel = gen.generate_trajectory(start, end, swing_duration=0.6)

    print("Swing foot trajectory:")
    print(f"{'T':>6}  {'X':>8}  {'Y':>8}  {'Z':>8}")
    print("-" * 36)
    for i in range(0, len(times), 10):
        print(f"{times[i]:6.2f}  {pos[i,0]:8.4f}  {pos[i,1]:8.4f}  {pos[i,2]:8.4f}")
    # Print last point
    print(f"{times[-1]:6.2f}  {pos[-1,0]:8.4f}  {pos[-1,1]:8.4f}  {pos[-1,2]:8.4f}")
    print(f"\nMax height: {pos[:,2].max():.4f} m (expected ~{gen.step_height:.4f})")
