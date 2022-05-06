#!/usr/bin/env python

import numpy as np
from math import pi, cos, sin, atan2, acos, asin, sqrt
from rbe500_project.srv import rrpIK, rrpIKResponse
import rospy

def ik_server():
    rospy.init_node("rrp_ik_server")
    s = rospy.Service("pos_to_joint_ang", rrpIK, inverse_kinematics)

    rospy.loginfo("rrp_ik_server is ready.")

    rospy.spin()

def inverse_kinematics(req):

    rospy.loginfo("Converting end effector position to joint angles.")

    position = req.position

    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    x = position.x
    y = position.y
    z = position.z
    # add your code here to complete the computation and calculate joint 1, joint 2 and joint 3 values
    
    # Link lengths in metrs
    l0 = 0.05
    l1 = 0.45
    l2 = 0.425
    l3 = 0.345
    l4 = 0.11

    # Virtual triangle edges and angles
    phi = acos( round((x**2 + y**2 - l2**2 - l3**2) / (2 * l2 * l3), 3) )

    alpha = atan2(y, x)
    beta = acos( (l2 + l3*cos(phi)) / sqrt(x**2 + y**2) )

    joint1 = alpha - beta
    joint2 = phi
    joint3 = - (z - (l0 + l1) + l4)

    return rrpIKResponse(joint1, joint2, joint3)

if __name__ == "__main__":
    ik_server()
