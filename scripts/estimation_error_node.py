#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import csv
import os
from datetime import datetime


class estimation_error_class():

    def __init__(self, topic="/cmd_vel"):
        print("INITIALISING NODE -> /sampling_node")
        rospy.init_node('sampling_node')

        self.counter = 0
        self.ang_temp_data = []
        self.lin_temp_data = []

        self.ang_error_history = []
        self.lin_error_history = []

        self.topic = topic
        self.cmd_msg_ang = 0.0
        self.cmd_msg_lin = 0.0
        self.sampled_ang_msg_list = [0]*3
        self.sampled_lin_msg_list = [0]*3

        self.pub_ang = rospy.Publisher("estimation_error_ang", Float32, queue_size=1)
        self.pub_lin = rospy.Publisher("estimation_error_lin", Float32, queue_size=1)
        self.sub = rospy.Subscriber(self.topic, Twist, self.sub_callback)

        self.entropy_started = False
        self.start_sub = rospy.Subscriber("/start_entropy", Float32, self.trigger_callback)

    def trigger_callback(self, msg):
        self.entropy_started = (msg.data == 1.0)
        if self.entropy_started:
            rospy.loginfo("Estimation error logging started.")
        else:
            rospy.loginfo("Estimation error logging stopped.")


    def sub_callback(self, data):
        self.cmd_msg_ang = data.angular.z
        self.cmd_msg_lin = data.linear.x

    def calc_average_estimation_error(self, event=None):
        if not self.entropy_started:
            return

        ang_msg = 0.0 if self.cmd_msg_ang == -0.0 else self.cmd_msg_ang
        lin_msg = 0.0 if self.cmd_msg_lin == -0.0 else self.cmd_msg_lin

        if self.counter < 2:
            self.ang_temp_data.append(ang_msg)
            self.lin_temp_data.append(lin_msg)
            self.counter += 1
        else:
            self.ang_temp_data.append(ang_msg)
            self.lin_temp_data.append(lin_msg)

            average_ang_msg = sum(self.ang_temp_data) / len(self.ang_temp_data)
            average_lin_msg = sum(self.lin_temp_data) / len(self.lin_temp_data)

            self.sampled_ang_msg_list[2] = self.sampled_ang_msg_list[1]
            self.sampled_ang_msg_list[1] = self.sampled_ang_msg_list[0]
            self.sampled_ang_msg_list[0] = average_ang_msg

            self.sampled_lin_msg_list[2] = self.sampled_lin_msg_list[1]
            self.sampled_lin_msg_list[1] = self.sampled_lin_msg_list[0]
            self.sampled_lin_msg_list[0] = average_lin_msg

            ang_error = ang_msg - self.estimation_model(self.sampled_ang_msg_list)
            lin_error = lin_msg - self.estimation_model(self.sampled_lin_msg_list)

            ang_error = 0.0 if ang_error == -0.0 else ang_error
            lin_error = 0.0 if lin_error == -0.0 else lin_error

            self.pub_ang.publish(ang_error)
            self.ang_error_history.append(ang_error)

            self.pub_lin.publish(lin_error)
            self.lin_error_history.append(lin_error)

            self.counter = 0
            self.ang_temp_data = []
            self.lin_temp_data = []

    def estimation_model(self, sampled_list):
        return sampled_list[0] + (sampled_list[0] - sampled_list[1]) +  0.5 * ((sampled_list[0] - sampled_list[1]) - (sampled_list[1] - sampled_list[2]))

    def spin(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    sampling_period = 0.05
    sampling_freq = 1 / sampling_period
    try:
        node = estimation_error_class()
        rospy.Timer(rospy.Duration(1.0 / sampling_freq), node.calc_average_estimation_error)
        node.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        if node.ang_error_history:
            print("alpha ang: ", np.percentile(node.ang_error_history, 90))
        else:
            print("No angular error data collected.")
        if node.lin_error_history:
            print("alpha lin: ", np.percentile(node.lin_error_history, 90))
        else:
            print("No linear error data collected.")