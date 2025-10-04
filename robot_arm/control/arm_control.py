"""High-level arm commands: move to XYZ, open/close claw, pick sequence."""
import math
from ..planning.kinematics import IKSolver2R, get_yaw_from_xy
from ..planning.trajectory import interpolate_angles

def ik_angles_to_degrees(yaw_rad, pitch_angles_rad, roll=0, claw_open=1):
    """Convert radian angles into the tuple expected by the Arduino protocol.

    Basic mapping: (yaw_deg, p1_deg, p2_deg, p3_deg, roll_deg, claw)
    p3 is set to 0 if unavailable. claw: 1=open, 0=closed
    """
    yaw_deg = math.degrees(yaw_rad)
    p1_deg = math.degrees(pitch_angles_rad[0])
    p2_deg = math.degrees(pitch_angles_rad[1])
    p3_deg = math.degrees(pitch_angles_rad[2]) if len(pitch_angles_rad) > 2 else 0
    return (yaw_deg, p1_deg, p2_deg, p3_deg, roll, 1 if claw_open else 0)

class ArmController:
    def __init__(self, ik_solver: IKSolver2R, serial_comm):
        self.ik = ik_solver
        self.serial = serial_comm
        # Ensure servos are at a known neutral position when controller starts
        try:
            if hasattr(self.serial, "zero_servos"):
                self.serial.zero_servos()
        except Exception as e:
            print(f"Warning: failed to zero servos in ArmController init: {e}")

    def move_to(self, x, y, z, phi_rad=0.0, elbow='down', steps=15, claw_open=1, roll=0):
        # For this simple scaffold we project x,y,z into planar range: use sqrt(x^2+y^2) as planar radius
        planar_r = math.sqrt(x*x + y*y)
        # call IK solver for planar coordinates
        pitch = self.ik.solve(planar_r, z, phi_rad, elbow=elbow)
        if pitch is None:
            raise ValueError("Target unreachable")

        yaw = get_yaw_from_xy(x, y)
        target_angles = ik_angles_to_degrees(yaw, pitch, roll=roll, claw_open=claw_open)

        # For simplicity assume current position is zeros (a real implementation would query or track state)
        current = (0,0,0,0,0,1)
        path = interpolate_angles(current, target_angles, steps=steps)
        # send each interpolated target
        for ang in path:
            self.serial.send_targets(ang)

    def open_claw(self):
        self.serial.send_targets((0,0,0,0,0,1))

    def close_claw(self):
        self.serial.send_targets((0,0,0,0,0,0))
