#!/usr/bin/env python


# This script tests if the self-created inverted pendulum can be controled by joint velocity controller


from __future__ import print_function

import math
import random
import time

import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point


class Testbed(object):
    """ Testbed, for the pupose of testing cart-pole system """

    def __init__(self):
        # init topics and services
        self._sub_invpend_states = rospy.Subscriber(
            '/invpend/joint_states', JointState, self.jsCB)
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

    def jsCB(self, data):
        rospy.loginfo("~~~Getting Inverted pendulum joint states~~~")
        self.pos_cart = data.position[1]
        self.vel_cart = data.velocity[1]
        self.pos_pole = data.position[0]
        self.vel_pole = data.velocity[0]
        # For debug purpose, uncomment the following line
        # print("cart_position: {0:.5f}, cart_velocity: {1:.5f}, pole_angle: {2:.5f}, pole_angular_velocity: {3:.5f} \
        # ".format(self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole))
        if math.fabs(self.pos_cart) >= 2.4:
            reset_count = 0
            print("=== reset invpend pos ===\n")
            while reset_count < self.reset_dur*self.freq:
                print("reset counter: ", str(reset_count))
                self._pub_vel_cmd.publish(0)
                self._pub_set_pole.publish(self.PoleState)
                reset_count += 1
                rospy.sleep(1./self.freq)

    def rand_move(self):
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
            if math.fabs(self.pos_cart) <= 2.4:
                # elapsed = rospy.Time.now() - start
                cmd_vel = random.uniform(-50, 50)  # make_cmd(elapsed)
            else:
                cmd_vel = 0
            self._pub_vel_cmd.publish(cmd_vel)
            print("---> velocity command: {:.4f}".format(cmd_vel))
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
    """ Perform testing actions provided by Testbed class
    """
    print("Initializing node... ")
    rospy.init_node('cart_wobble')
    cart = Testbed()
    rospy.on_shutdown(cart.clean_shutdown)
    cart.rand_move()
    rospy.spin()


if __name__ == '__main__':
    main()
