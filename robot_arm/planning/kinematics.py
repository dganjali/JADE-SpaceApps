"""Inverse kinematics utilities.

Supports a yaw + 2-pitch arm (planar 2R wrist) by solving for pitch angles and yaw separately.
If your arm has an extra pitch or wrist link, you can enable the 3R solver from legacy code.
"""
import math
from typing import Optional, Tuple

class IKSolver2R:
    """2R planar arm solver (shoulder, elbow) plus a gripper offset.

    Link lengths are specified in the constructor (cm): l1, l2, l3 (l3 is end-effector offset).
    solve(x, y, phi) expects x,y = planar coordinates in cm and phi = desired end effector angle (radians).
    Returns (theta1, theta2, theta3) in radians or None if unreachable.
    """
    def __init__(self, l1: float, l2: float, l3: float):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def solve(self, x: float, y: float, phi: float, elbow: str = 'down') -> Optional[Tuple[float,float,float]]:
        # wrist position (subtract l3 along phi)
        xw = x - self.l3 * math.cos(phi)
        yw = y - self.l3 * math.sin(phi)
        r2 = xw*xw + yw*yw

        # cos theta2
        c2 = (r2 - self.l1*self.l1 - self.l2*self.l2) / (2*self.l1*self.l2)
        if c2 > 1 or c2 < -1:
            return None

        s2_mag = abs(math.sqrt(1 - c2*c2))
        s2 = s2_mag if elbow == 'down' else -s2_mag
        theta2 = math.atan2(s2, c2)

        k1 = self.l1 + self.l2 * c2
        k2 = self.l2 * s2
        theta1 = math.atan2(yw, xw) - math.atan2(k2, k1)

        theta3 = phi - theta1 - theta2

        wrap = lambda a: (a + math.pi) % (2*math.pi) - math.pi
        return wrap(theta1), wrap(theta2), wrap(theta3)

def get_yaw_from_xy(x: float, y: float) -> float:
    """Return yaw angle (radians) from base to (x,y).
    Wraps to [-pi, pi]."""
    a = math.atan2(y, x)
    return (a + math.pi) % (2*math.pi) - math.pi

# Legacy 3R solver adapter (for reuse of older code). If you want to use the 3R solver, import from legacy file or replace.
