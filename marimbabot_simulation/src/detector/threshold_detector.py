# Simple contact detector based on a position / velocity threshold

from typing import Dict, List
from dataclasses import dataclass


@dataclass
class PosVel:
    position: float
    velocity: float


class ThresholdDetector:
    def __init__(self, pos_threshold: float = 0.01, vel_threshold: float = 0.01, flip_z: bool = False):
        """
        Initialize detector.

        Args:
            pos_threshold: Position threshold
            vel_threshold: Velocity threshold
            flip_z: Whether joints' z-axis faces up. Use this flag to flip the positive
                    direction of joint values

        Attributes:
            _contacts: A dictionary that holds which bar(s) were hit and with
                      what maximum pos deviation/velocity. Contacts persist in
                      this dict until the joint returns to its original position
                      (below pos_threshold)
        """

        self._pos_threshold = pos_threshold
        self._vel_threshold = vel_threshold
        self._flip_z = flip_z

        self._contacts: Dict[str, PosVel] = {}

    def _is_in_contact(self, s: PosVel) -> bool:
        """Decide whether the contact counts"""
        
        mult = -1. if self._flip_z else 1.
        return mult * s.position > self._pos_threshold and mult * s.velocity > self._vel_threshold

    def update(self, joint_states: Dict[str, PosVel]) -> Dict[str, PosVel]:
        """Update with joint states. Returns dict of contacts and their maximum pos, vel"""
        
        # remove joints that are not in the joint_states
        self._contacts = {k: v for k, v in self._contacts.items() if k in joint_states}
        
        # update contacts dict
        for bar, state in joint_states.items():
            if self._is_in_contact(state):
                if abs(state.position) > abs(self._contacts[bar]):
                    self._contacts[bar] = state
            else:
                 self._contacts.pop(bar, None)
        return self._contacts