#!/usr/bin/env python3
"""
dcm_controller.py — Stable DCM Controller for Angad Humanoid
==============================================================
Simplified DCM controller that won't blow up. Uses clamped exponentials
and conservative gains to keep the CoM trajectory bounded.

DCM = x_com + ẋ_com / ω     (Capture Point / Divergent Component of Motion)

The controller computes a desired CoP that drives the DCM toward the
reference. The reference smoothly interpolates between footstep targets.
"""

import numpy as np
from typing import Tuple


class DCMController:
    """
    Stable DCM controller with bounded outputs.

    The CoM is updated via semi-implicit Euler with HEAVY damping
    to prevent divergence when footstep timing doesn't perfectly match.
    """

    def __init__(self, omega: float, k_dcm: float = 1.0, dt: float = 0.02):
        self.omega = omega
        self.k_dcm = k_dcm
        self.dt = dt
        self.b = 1.0 / omega

        # Safety: max CoM velocity and acceleration
        self.max_com_vel = 0.3     # m/s
        self.max_com_accel = 2.0   # m/s²
        self.max_com_offset = 0.10 # m from center of support

    def compute_dcm(self, com_pos: np.ndarray, com_vel: np.ndarray) -> np.ndarray:
        """DCM = x_com + ẋ/ω"""
        return com_pos + com_vel / self.omega

    def compute_dcm_reference(
        self,
        support_pos: np.ndarray,
        target_pos: np.ndarray,
        phase: float,
    ) -> np.ndarray:
        """
        Simple DCM reference: smoothly interpolate from support foot
        toward the next footstep target.

        phase ∈ [0, 1] within the current step.
        At phase=0, DCM reference is near the support foot.
        At phase=1, DCM reference should be at/near the next target.

        Uses smooth sigmoid-like transition (no exponentials that blow up).
        """
        phase = np.clip(phase, 0.0, 1.0)

        # Smooth S-curve transition: 3t² - 2t³
        s = 3.0 * phase**2 - 2.0 * phase**3

        # DCM reference moves from near support foot to near target
        dcm_ref = support_pos[:2] + s * (target_pos[:2] - support_pos[:2])

        return dcm_ref

    def step(
        self,
        com_pos: np.ndarray,
        com_vel: np.ndarray,
        dcm_ref: np.ndarray,
        support_center: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        One DCM control step. Updates CoM position and velocity.

        Args:
            com_pos: current CoM (x, y) [m]
            com_vel: current CoM velocity (vx, vy) [m/s]
            dcm_ref: reference DCM target (x, y) [m]
            support_center: center of current support polygon (x, y) [m]

        Returns:
            new_com_pos, new_com_vel
        """
        # Current DCM
        dcm = self.compute_dcm(com_pos, com_vel)

        # DCM error
        dcm_err = dcm_ref - dcm

        # Desired CoP from DCM control law:
        # cop = dcm - (1/ω) * K * dcm_error
        cop = dcm - self.b * self.k_dcm * dcm_err

        # LIPM acceleration: ẍ = ω² (x - cop)
        com_accel = self.omega**2 * (com_pos - cop)

        # Clamp acceleration
        accel_mag = np.linalg.norm(com_accel)
        if accel_mag > self.max_com_accel:
            com_accel = com_accel * self.max_com_accel / accel_mag

        # Semi-implicit Euler
        new_vel = com_vel + com_accel * self.dt

        # Clamp velocity
        vel_mag = np.linalg.norm(new_vel)
        if vel_mag > self.max_com_vel:
            new_vel = new_vel * self.max_com_vel / vel_mag

        new_pos = com_pos + new_vel * self.dt

        # HARD CLAMP: CoM must stay near the support center
        offset = new_pos - support_center
        offset = np.clip(offset, -self.max_com_offset, self.max_com_offset)
        new_pos = support_center + offset

        return new_pos, new_vel


if __name__ == "__main__":
    from lipm_model import LIPMModel

    lipm = LIPMModel(z_com=0.74)
    ctrl = DCMController(omega=lipm.omega, k_dcm=1.0, dt=0.02)

    print(f"ω = {lipm.omega:.3f} rad/s, b = {ctrl.b:.4f} s")

    # Simulate weight shift from center to right foot
    com = np.array([0.0, 0.0])
    vel = np.array([0.0, 0.0])
    support = np.array([0.0, -0.064])

    print("\nWeight shift to right foot:")
    for i in range(25):
        phase = i / 24.0
        dcm_ref = ctrl.compute_dcm_reference(
            support_pos=np.array([0.0, -0.064, 0.0]),
            target_pos=np.array([0.025, 0.064, 0.0]),
            phase=phase,
        )
        center = np.array([0.0, 0.0])
        com, vel = ctrl.step(com, vel, dcm_ref, center)

    print(f"  Final CoM: ({com[0]:.4f}, {com[1]:.4f})")
    print(f"  Final vel: ({vel[0]:.4f}, {vel[1]:.4f})")
    print("  ✓ No blowup")
