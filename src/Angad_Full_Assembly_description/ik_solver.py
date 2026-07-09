#!/usr/bin/env python3
"""
ik_solver.py — Analytical Leg IK for Angad Humanoid (FIXED)
=============================================================
Properly accounts for the NON-STANDARD hip_pitch axis:
  hip_pitch_l axis = (-0.939693, 0, 0.34202)  ← tilted 20° from -X toward +Z
  hip_pitch_r axis = (+0.939693, 0, 0.34202)

This means the hip only delivers 94% of its commanded angle as actual
sagittal-plane pitch. The IK compensates for this with a projection factor.

VERIFIED SIGN CONVENTIONS (from URDF + crouch_only.py):
  hip_pitch_l:   NEGATIVE → thigh swings FORWARD
  hip_pitch_r:   POSITIVE → thigh swings FORWARD
  knee_pitch:    NEGATIVE → knee BENDS (flexion)
  ankle_pitch:   POSITIVE → foot stays FLAT (compensates hip+knee)
  thigh_roll:    positive  → lean outward
  ankle_roll:    negative of thigh_roll → keep foot flat laterally
"""

import numpy as np
from typing import Dict, Optional


# ═══════════════════════════════════════════════════════════════
# ROBOT DIMENSIONS from URDF
# ═══════════════════════════════════════════════════════════════
HIP_OFFSET_Y = 0.064       # lateral offset base_link → hip [m]
HIP_OFFSET_Z = 0.804       # vertical offset base_link → hip [m]

# Hip pitch axis projection factor (|X component| of axis vector)
# The axis is (-0.939693, 0, 0.34202), so only 93.97% is sagittal pitch
HIP_AXIS_FACTOR = 0.939693

# Leg segment lengths [m] (from URDF joint Z-offsets)
L_UPPER = 0.027 + 0.157 + 0.210   # hip→thigh_roll→thigh_yaw→knee = 0.394 m
L_LOWER = 0.363                     # knee→ankle = 0.363 m
L_FULL  = L_UPPER + L_LOWER         # total = 0.757 m

# HARD joint limits [rad] — enforced strictly
LIM_HIP   = 0.50    # ±0.50 for walking (well within ±1.57 URDF limit)
LIM_KNEE  = 1.20    # max knee bend (within ±2.0)
LIM_ANKLE = 0.80    # ±0.80 (within ±1.0)
LIM_ROLL  = 0.15    # ±0.15 for walking (within ±0.5)


class AngadLegIK:
    """
    Analytical IK for Angad's 6-DOF legs.

    Key feature: compensates for the tilted hip_pitch axis so that the
    foot actually reaches the intended position.
    """

    def __init__(self):
        self.l_upper = L_UPPER   # 0.394 m
        self.l_lower = L_LOWER   # 0.363 m
        self.l_max   = L_FULL    # 0.757 m

    def solve(self, foot_pos: np.ndarray, side: str = 'left') -> Optional[Dict[str, float]]:
        """
        Solve IK for one leg.

        foot_pos: (x, y, z) relative to HIP JOINT (NOT base_link).
          x = forward, y = lateral, z = NEGATIVE (foot below hip)

        Returns dict of 6 joint angles with URDF-correct names & signs.
        """
        x = foot_pos[0]
        y = foot_pos[1]
        z = foot_pos[2]

        # ── Sagittal plane distance (X-Z) ──
        leg_len = np.sqrt(x**2 + z**2)

        # Clamp to avoid singularity at full extension
        if leg_len > self.l_max * 0.99:
            scale = (self.l_max * 0.99) / leg_len
            x *= scale
            z *= scale
            leg_len = self.l_max * 0.99

        if leg_len < 0.2:
            return None

        # ═══════════════════════════════════════════════════════════
        # KNEE: law of cosines
        # ═══════════════════════════════════════════════════════════
        cos_k = (self.l_upper**2 + self.l_lower**2 - leg_len**2) / \
                (2.0 * self.l_upper * self.l_lower)
        cos_k = np.clip(cos_k, -1.0, 1.0)
        knee_interior = np.arccos(cos_k)
        # NEGATIVE = bend (verified from crouch_only.py)
        knee_pitch = -(np.pi - knee_interior)

        # ═══════════════════════════════════════════════════════════
        # HIP PITCH: desired sagittal angle
        # ═══════════════════════════════════════════════════════════
        # Angle of the leg from vertical (positive = forward)
        alpha = np.arctan2(x, -z)

        # Triangle angle at hip
        sin_beta = self.l_lower * np.sin(knee_interior) / leg_len
        sin_beta = np.clip(sin_beta, -1.0, 1.0)
        beta = np.arcsin(sin_beta)

        # Desired pitch angle (how much the thigh must tilt forward)
        desired_pitch = alpha + beta

        # Command: divide by axis factor because hip only gives 94% as pitch
        # Left: NEGATIVE = forward;  Right: POSITIVE = forward
        if side == 'left':
            hip_pitch = -desired_pitch / HIP_AXIS_FACTOR
        else:
            hip_pitch = desired_pitch / HIP_AXIS_FACTOR

        # ═══════════════════════════════════════════════════════════
        # ANKLE PITCH: keep foot flat on ground
        # ═══════════════════════════════════════════════════════════
        # The kinematic chain accumulates pitch:
        #   effective_hip_pitch = hip_cmd × axis_factor = desired_pitch
        #   knee_pitch          = knee_pitch (full axis)
        #   ankle must cancel:  desired_pitch + knee_pitch + ankle = 0
        ankle_pitch = -(desired_pitch + knee_pitch)

        # ═══════════════════════════════════════════════════════════
        # ROLL: lateral foot offset → thigh roll + ankle roll
        # ═══════════════════════════════════════════════════════════
        if abs(z) > 0.05:
            thigh_roll = np.arctan2(y, -z)
        else:
            thigh_roll = 0.0
        ankle_roll = -thigh_roll

        thigh_yaw = 0.0

        # ═══════════════════════════════════════════════════════════
        # HARD CLAMP all joint angles for safety
        # ═══════════════════════════════════════════════════════════
        hip_pitch   = np.clip(hip_pitch,   -LIM_HIP,   LIM_HIP)
        knee_pitch  = np.clip(knee_pitch,  -LIM_KNEE,  0.0)  # ONLY negative!
        ankle_pitch = np.clip(ankle_pitch, -LIM_ANKLE, LIM_ANKLE)
        thigh_roll  = np.clip(thigh_roll,  -LIM_ROLL,  LIM_ROLL)
        ankle_roll  = np.clip(ankle_roll,  -LIM_ROLL,  LIM_ROLL)

        s = '_l' if side == 'left' else '_r'
        return {
            f'hip_pitch{s}':   float(hip_pitch),
            f'thigh_roll{s}':  float(thigh_roll),
            f'thigh_yaw{s}':   float(thigh_yaw),
            f'knee_pitch{s}':  float(knee_pitch),
            f'ankle_pitch{s}': float(ankle_pitch),
            f'ankle_roll{s}':  float(ankle_roll),
        }

    def solve_both_legs(
        self,
        left_foot: np.ndarray,
        right_foot: np.ndarray,
    ) -> Optional[Dict[str, float]]:
        """
        Solve IK for both legs.

        Foot positions in base_link frame:
          Left foot at rest:  (0, +0.064, -0.757)
          Right foot at rest: (0, -0.064, -0.757)
        """
        # Convert base_link frame → hip-local frame
        lf_hip = left_foot.copy()
        lf_hip[1] -= HIP_OFFSET_Y     # remove left hip offset

        rf_hip = right_foot.copy()
        rf_hip[1] += HIP_OFFSET_Y     # remove right hip offset

        left_ang  = self.solve(lf_hip, side='left')
        right_ang = self.solve(rf_hip, side='right')

        if left_ang is None or right_ang is None:
            return None

        result = {}
        result.update(left_ang)
        result.update(right_ang)
        return result


def compute_standing_angles(hip_height: float = 0.74) -> Dict[str, float]:
    """
    Standing pose: feet directly below hips, near full extension.
    hip_height=0.757 → fully straight, 0.74 → slight bend (~24°).
    """
    ik = AngadLegIK()
    lf = np.array([0.0,  HIP_OFFSET_Y, -hip_height])
    rf = np.array([0.0, -HIP_OFFSET_Y, -hip_height])
    return ik.solve_both_legs(lf, rf)


if __name__ == "__main__":
    print("=" * 60)
    print("Angad Leg IK — FIXED with hip axis factor")
    print(f"Hip axis factor: {HIP_AXIS_FACTOR}")
    print(f"Upper={L_UPPER:.3f}m  Lower={L_LOWER:.3f}m  Max={L_FULL:.3f}m")
    print("=" * 60)

    # Standing
    a = compute_standing_angles(0.74)
    if a:
        print("\nStanding (hip=0.74m):")
        for n in sorted(a):
            print(f"  {n:20s}: {np.degrees(a[n]):+7.2f}°  ({a[n]:+.4f} rad)")

        # Verify conventions
        print("\n  CHECK: hip_pitch_l < 0?", "✓" if a['hip_pitch_l'] <= 0 else "✗ WRONG")
        print("  CHECK: knee_pitch_l < 0?", "✓" if a['knee_pitch_l'] <= 0 else "✗ WRONG")
        print("  CHECK: ankle_pitch_l > 0?", "✓" if a['ankle_pitch_l'] >= 0 else "✗ WRONG")

    # Walking pose
    ik = AngadLegIK()
    lf = np.array([0.025,  0.064, -0.73])
    rf = np.array([-0.025, -0.064, -0.73])
    w = ik.solve_both_legs(lf, rf)
    if w:
        print("\nWalking (left fwd 2.5cm):")
        for n in sorted(w):
            print(f"  {n:20s}: {np.degrees(w[n]):+7.2f}°  ({w[n]:+.4f} rad)")
