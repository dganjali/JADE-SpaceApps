from robot_arm.planning.kinematics import IKSolver2R
import math

def test_simple_reach():
    ik = IKSolver2R(12.0, 12.0, 5.0)
    # point within reach
    sol = ik.solve(10, 10, 0.0)
    assert sol is not None
    t1, t2, t3 = sol
    # forward kinematics approximate: compute end effector x using t1,t2
    xw = 12.0*math.cos(t1) + 12.0*math.cos(t1+t2)
    yw = 12.0*math.sin(t1) + 12.0*math.sin(t1+t2)
    # point should be near requested planar radius sqrt(10^2+10^2)
    r = (xw**2 + yw**2)**0.5
    assert abs(r - (10**2+10**2)**0.5) < 1.0
