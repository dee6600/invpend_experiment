#!/usr/bin/env python

import numpy as np
import control
import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point



class invpend:
    def __init__(self):
        rospy.init_node('invpend', anonymous=True)
        self.sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.state)
        self.pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0
        self.rate=rospy.Rate(10)

    def state(self, data):
        self.pos_cart = data.position[1]
        self.vel_cart = data.velocity[1]
        self.pos_pole = data.position[0]
        self.vel_pole = data.velocity[0]

    def U(self,feedbackGain,desired_pose):
        
        current_pose=np.array([self.pos_cart,self.vel_cart,self.pos_pole,self.vel_pole])
        u=np.dot(feedbackGain,(desired_pose - current_pose))
        self.pub_vel_cmd.publish(u)


if __name__ == '__main__':
    g = 9.8
    l = 0.5
    m = 2
    M = 20

    A = np.matrix([ [0, 1, 0, 0],
                [0, 0,            (-12 * m * g)/((12 * M) + m), 0],
                [0, 0,                                       0, 1],
                [0, 0, (12 * g * (M + m))/(l * ((13 * M) + m)), 0]
                ])

    B = np.matrix([ [0],
                [13 / ((13 * M) + m)],
                [0],
                [-12 / (l * ((13 * M) + m))]
                ])
    Q=np.diag([1,1,10,100])
    R=np.diag([0.05])
    K, S, E = control.lqr(A, B, Q, R)
    feedbackGain = (control.place(A, B, E))
    desired_pose=np.array([0,0,0,0])

    try:
        MyModel = invpend()
        while not rospy.is_shutdown():
            MyModel.U(feedbackGain,desired_pose)
    except rospy.ROSInterruptException:
        pass

