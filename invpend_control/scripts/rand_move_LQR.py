#!/usr/bin/env python

from __future__ import print_function

import math
import random
import time

import ubjson

import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
from simple_pid import PID
from control import lqr
import numpy as np
import control


class Testbed(object):
    """ Testbed, for the pupose of testing cart-pole system """

    def __init__(self):
        # init topics and services
        self._sub_invpend_states = rospy.Subscriber(
            '/invpend/joint_states', JointState, self.mycallback)
        self._pub_vel_cmd = rospy.Publisher(
            '/invpend/joint1_velocity_controller/command', Float64, queue_size=1)
        self._pub_set_pole = rospy.Publisher(
            '/gazebo/set_link_state', LinkState)
        self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # init parameters
        self.reset_dur = 1  # reset duration, sec
        self.freq = 50  # topics pub and sub frequency, Hz
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0
        self.PoleState = LinkState()
        self.PoleState.link_name = 'pole'
        self.PoleState.pose.position = Point(0.0, -0.25, 2.0)
        self.PoleState.reference_frame = 'world'

    def mycallback(self, data):
        #rospy.loginfo("~~~Getting Inverted pendulum joint states~~~")
        self.data = data
        self.pos_cart = data.position[1]
        self.vel_cart = data.velocity[1]
        self.pos_pole = data.position[0]
        self.vel_pole = data.velocity[0]
        # For debug purpose, uncomment the following line
        print("cart_position: {0:.5f}, cart_velocity: {1:.5f}, pole_angle: {2:.5f}, pole_angular_velocity: {3:.5f}".format(self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole))
        if math.fabs(self.pos_cart) >= 2.4:
            reset_count = 0
            print("=== reset invpend pos ===\n")
            while reset_count < self.reset_dur*self.freq:
                print("reset counter: ", str(reset_count))
                self._pub_vel_cmd.publish(0)
                self._pub_set_pole.publish(self.PoleState)
                reset_count += 1
                rospy.sleep(1./self.freq)

    def rand_move(self, feedbackGain,desired_pose):
        '''
        For the purpose of test, 
        implement random velocity control on cart.
        '''
        rate = rospy.Rate(self.freq)

        def make_cmd(elapsed):
            period_factor = .5
            amplitude_factor = 25
            w = period_factor * elapsed.to_sec()
            return amplitude_factor * math.cos(w*2*math.pi)
            

        while not rospy.is_shutdown():

            current_pose=np.array([self.pos_cart,self.vel_cart,self.pos_pole,self.vel_pole])
            u=np.dot(feedbackGain,(desired_pose - current_pose))
            # self.pub_vel_cmd.publish(u)  

            # if math.fabs(self.pos_cart) <= 2.4:
            #     cmd_vel = u
            # else:
            #     cmd_vel = 0

            #print cart position and velocity, and cmd_vel, pole angle and angular velocity
            # rospy.loginfo("cart_position: {0:.5f}, cart_velocity: {1:.5f}, pole_angle: {2:.5f}, pole_angular_velocity: {3:.5f}, cmd_vel: {4:.5f}".format(self.pos_cart, self.vel_cart, math.degrees(self.pos_pole), self.vel_pole, cmd_vel))
            
            self._pub_vel_cmd.publish(u)
            #print("---> velocity command: {:.4f}".format(cmd_vel))
            rate.sleep()

    def clean_shutdown(self):
        print("Shuting dwon...")
        self._pub_vel_cmd.publish(0)
        return True

    def _reset(self):
        rospy.wait_for_service("/gazebo/reset_simulation")
        print("reset simulation===\n")
        self.reset_sim()
        # rospy.wait_for_service("/gazebo/unpause_physics")
        # self.unpause
        # rospy.wait_for_service("/gazebo/pause_physics")
        # self.pause


def main():

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

    """ Perform testing actions provided by Testbed class
    """
    print("Initializing node... ")
    rospy.init_node('cart_PID_test')
    cart = Testbed()
    rospy.on_shutdown(cart.clean_shutdown)
    cart.rand_move(feedbackGain,desired_pose)
    rospy.spin()


if __name__ == '__main__':
    main()
