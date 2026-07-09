#!/usr/bin/env python3
"""
step_planner.py — Footstep Planner for Angad Humanoid
=====================================================
Generates a sequence of alternating left/right footsteps for bipedal walking.
Each footstep contains a target position (x, y), the foot side, and timing.

Designed for the Angad robot with hip lateral offset ~0.064 m.
"""

import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from typing import List


class FootSide(Enum):
    """Which foot is being placed."""
    LEFT = 0
    RIGHT = 1


@dataclass
class FootStep:
    """
    Represents a single footstep target.

    Attributes:
        position: (x, y, z) position of the foot placement in world frame [m]
        side: LEFT or RIGHT foot
        start_time: when this step begins [s]
        end_time: when this step ends (foot lands) [s]
    """
    position: np.ndarray
    side: FootSide
    start_time: float
    end_time: float


class StepPlanner:
    """
    Generates a simple alternating footstep plan for straight-line walking.

    Parameters:
        step_length:  forward distance per step [m] (default: 0.06)
        step_width:   lateral distance between feet [m] (default: 0.128, = 2 × hip offset)
        step_height:  maximum swing foot height [m] (default: 0.04)
        step_period:  duration of one complete step [s] (default: 0.8)
        double_support_ratio: fraction of step_period spent in double support (default: 0.2)
        num_steps:    total number of walking steps (default: 10)
        first_foot:   which foot takes the first step (default: LEFT)
    """

    def __init__(
        self,
        step_length: float = 0.06,
        step_width: float = 0.128,
        step_height: float = 0.04,
        step_period: float = 0.8,
        double_support_ratio: float = 0.2,
        num_steps: int = 10,
        first_foot: FootSide = FootSide.LEFT,
    ):
        self.step_length = step_length
        self.step_width = step_width
        self.step_height = step_height
        self.step_period = step_period
        self.double_support_ratio = double_support_ratio
        self.num_steps = num_steps
        self.first_foot = first_foot

        # Derived timing
        self.double_support_time = step_period * double_support_ratio
        self.single_support_time = step_period - self.double_support_time

    def generate_footsteps(self, start_x: float = 0.0, start_y: float = 0.0) -> List[FootStep]:
        """
        Generate a sequence of footstep targets for straight-line walking.

        The robot starts in a double-support stance. The first step is a
        half-length step to initiate walking, and the last step brings
        the feet together.

        Args:
            start_x: initial X position of the robot center [m]
            start_y: initial Y position of the robot center [m]

        Returns:
            List of FootStep objects defining the walking plan.
        """
        footsteps: List[FootStep] = []

        # ── Initial stance (both feet on the ground) ──
        # Left foot is at +step_width/2, right foot at -step_width/2 in Y
        left_y = start_y + self.step_width / 2.0
        right_y = start_y - self.step_width / 2.0

        # Current foot positions
        left_x = start_x
        right_x = start_x

        current_time = 0.0

        # Determine alternating order
        if self.first_foot == FootSide.LEFT:
            order = [FootSide.LEFT, FootSide.RIGHT]
        else:
            order = [FootSide.RIGHT, FootSide.LEFT]

        for i in range(self.num_steps):
            side = order[i % 2]

            # Compute step length:
            #   - First step: half step (to get started)
            #   - Last step:  half step (to stop / bring feet together)
            #   - Others:     full step
            if i == 0 or i == self.num_steps - 1:
                dx = self.step_length * 0.5
            else:
                dx = self.step_length

            step_start = current_time
            step_end = current_time + self.step_period

            if side == FootSide.LEFT:
                left_x += dx
                pos = np.array([left_x, left_y, 0.0])
            else:
                right_x += dx
                pos = np.array([right_x, right_y, 0.0])

            footsteps.append(FootStep(
                position=pos.copy(),
                side=side,
                start_time=step_start,
                end_time=step_end,
            ))

            current_time = step_end

        return footsteps

    def get_support_foot_position(self, footsteps: List[FootStep], step_index: int) -> np.ndarray:
        """
        Get the position of the support (stance) foot during a given step.

        The support foot is the opposite foot from the one that is swinging.
        For the first step, we assume the support foot is at the initial position.

        Args:
            footsteps: the generated footstep plan
            step_index: current step index

        Returns:
            (x, y, z) position of the support foot
        """
        swing_side = footsteps[step_index].side

        # Find the most recent placement of the OTHER foot
        for j in range(step_index - 1, -1, -1):
            if footsteps[j].side != swing_side:
                return footsteps[j].position.copy()

        # If no previous step found for the other foot, it's at the initial position
        if swing_side == FootSide.LEFT:
            # Support is right foot
            return np.array([0.0, -self.step_width / 2.0, 0.0])
        else:
            # Support is left foot
            return np.array([0.0, self.step_width / 2.0, 0.0])

    def get_swing_foot_start(self, footsteps: List[FootStep], step_index: int) -> np.ndarray:
        """
        Get the starting position of the swing foot before it lifts off.

        Args:
            footsteps: the generated footstep plan
            step_index: current step index

        Returns:
            (x, y, z) starting position of the swing foot
        """
        swing_side = footsteps[step_index].side

        # Find the most recent placement of THIS foot (before current step)
        for j in range(step_index - 1, -1, -1):
            if footsteps[j].side == swing_side:
                return footsteps[j].position.copy()

        # If no previous step, the foot is at the initial stance position
        if swing_side == FootSide.LEFT:
            return np.array([0.0, self.step_width / 2.0, 0.0])
        else:
            return np.array([0.0, -self.step_width / 2.0, 0.0])


if __name__ == "__main__":
    # ── Quick test ──
    planner = StepPlanner(num_steps=6)
    steps = planner.generate_footsteps()

    print("Generated footsteps:")
    print(f"{'Step':>4}  {'Side':>6}  {'X':>8}  {'Y':>8}  {'T_start':>8}  {'T_end':>8}")
    print("-" * 50)
    for i, s in enumerate(steps):
        print(f"{i:4d}  {s.side.name:>6}  {s.position[0]:8.3f}  {s.position[1]:8.3f}  "
              f"{s.start_time:8.2f}  {s.end_time:8.2f}")
