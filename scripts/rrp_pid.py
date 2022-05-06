#!/usr/bin/env python

from rbe500_project.srv import rrpIK
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rospy
import numpy as np
import time

class rrp_robot():
    def __init__(self):
        rospy.init_node('PID_joint_pub', anonymous=True)
        rospy.loginfo("PID controller up")

        self.joint1_pub = rospy.Publisher('/rrp/joint1_effort_controller/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/rrp/joint2_effort_controller/command', Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher('/rrp/joint3_effort_controller/command', Float64, queue_size=10)

        self.rate = rospy.Rate(50)
        self.logging_counter = 0
        self.trajectory = []
        self.cur_pos, self.cur_vel, self.cur_eff = 0, 0, 0
        
        # self.position_list = [(0.544, 0.544, 0.34),
        #                       (0.3, 0.645, 0.24),
        #                       (0.425, 0.344, 0.29),
        #                       (-0.426, -0.344, 0.24)]

        self.position_list = [(0, 0.77, 0.34),
                              (-0.345, 0.425, 0.24),
                              (-0.67, -0.245, 0.14),
                              (0.77, 0.0, 0.39)]


        while not rospy.is_shutdown():
            self.joint_sub = rospy.Subscriber('/rrp/joint_states', JointState, self.joint_callback)
            self.rate.sleep()

            if self.cur_pos != 0 and len(self.position_list):
                self.joint_update()

            elif not len(self.position_list):
                break

        exit(0)


    def joint_update(self):
        time_step = 0.007
        kp_joint1, kd_joint1, ki_joint1 = 1.2, 0.87, 0
        kp_joint2, kd_joint2, ki_joint2 = 1.2, 1, 0

        while len(self.position_list):

            e_joint1, eint_joint1, e_joint1_prev = 0.16, 0.0, 0.0
            e_joint2, eint_joint2, e_joint2_prev = 0.16, 0.0, 0.0

            goal = self.position_list[0]

            joint1_d, joint2_d, joint3_d = self.get_joint_values(goal)
            print("\nDesired Joint values for " + str(self.position_list[0]) + ": ", joint1_d, joint2_d, joint3_d)

            tau_joint1, tau_joint2, tau_joint3 = self.cur_eff

            while abs(e_joint1) > 0.15 or abs(self.cur_vel[0])>=0.001 or abs(e_joint2) > 0.15 or abs(self.cur_vel[1])>=0.001:
                curr_joint1, curr_joint2, _ = self.cur_pos

                # joint 1
                e_joint1 = joint1_d - curr_joint1
                eint_joint1 += e_joint1 * (time_step)
                edot_joint1 = (e_joint1 - e_joint1_prev) / time_step
                e_joint1_prev = e_joint1
                         
                tau_joint1 = kp_joint1 * e_joint1 + kd_joint1 * edot_joint1 + ki_joint1 * eint_joint1
                self.joint1_pub.publish(tau_joint1)

                # joint 2
                e_joint2 = joint2_d - curr_joint2
                eint_joint2 += e_joint2 * (time_step)
                edot_joint2 = (e_joint2 - e_joint2_prev) / time_step
                e_joint2_prev = e_joint2

                tau_joint2 = kp_joint2 * e_joint2 + kd_joint2 * edot_joint2 + ki_joint2 * eint_joint2
                self.joint2_pub.publish(tau_joint2)

                self.rate.sleep()

            self.joint1_pub.publish(0)
            self.joint2_pub.publish(0)
            
            kp_joint3, kd_joint3, ki_joint3 = 900, 0, 0
            e_joint3, eint_joint3, e_joint3_prev = 1, 0.0, 0.0

            while abs(e_joint3) > 0.002:
                # joint 3
                curr_joint3 = self.cur_pos[2]

                e_joint3 = joint3_d - curr_joint3
                eint_joint3 += e_joint3 * (time_step)
                edot_joint3 = (e_joint3 - e_joint3_prev) / time_step
                e_joint3_prev = e_joint3

                tau_joint3 = kp_joint3 * e_joint3 + kd_joint3 * edot_joint3 + ki_joint3 * eint_joint3
                self.joint3_pub.publish(tau_joint3)
                self.rate.sleep()
            
            self.joint3_pub.publish(0)

            print()
            print("JOINT1 ::  " + "Error: " + str(e_joint1))
            print("JOINT2 ::  " + "Error: " + str(e_joint2))
            print("JOINT3 ::  " + "Error: " + str(e_joint3))
            print()            
            
            print("Current Joint values: ", self.cur_pos)
            print()
            
            rospy.loginfo("SLEEP FOR 1 SEC")
            rospy.sleep(1)
            print()

            self.position_list.pop(0)
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')
        

    def get_joint_values(self, position):
        rospy.wait_for_service('pos_to_joint_ang')

        try:
            joint_val_srv = rospy.ServiceProxy('pos_to_joint_ang', rrpIK)

            px, py, pz = position
            end_effector_pos = Point(px, py, pz)
            resp = joint_val_srv(end_effector_pos)

            return [resp.joint1_ang, resp.joint2_ang, resp.joint3_ang]

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def joint_callback(self, msg):
        self.cur_pos = msg.position
        self.cur_vel = msg.velocity
        self.cur_eff = msg.effort

        self.logging_counter += 1
        if self.logging_counter == 10:
            self.logging_counter = 0
            self.trajectory.append(self.cur_pos)


if __name__ == "__main__":
    bot = rrp_robot()
