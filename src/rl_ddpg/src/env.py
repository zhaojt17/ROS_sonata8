#!/usr/bin/env python
# coding=utf-8
import rospy
import numpy as np
from std_msgs.msg import Float64


def ddpg_subCallback():
    pass


class Env(object):
    def __init__(self):
        rospy.init_node('ddpg_track')
        self.pub = rospy.Publisher('ddpg_pub', Float64, queue_size=100)
        rospy.Subscriber("ddpg_sub", Float64, ddpg_subCallback)
        self.rate = rospy.Rate(50)
        self.step_freq = 10

    def step(self):
        step_counter = 0
        done = True
        reward = 0
        observation = None
        while (not rospy.is_shutdown()) and (step_counter < self.step_freq):
            step_counter = step_counter + 1
            self.pub_data()
            self.rate.sleep()
        return observation, reward, done

    def pub_data(self):
        pass

    def reset(self):
        done = False
        while (not rospy.is_shutdown()) and (done is not True):
            done = True
            self.pub_data()
        return





