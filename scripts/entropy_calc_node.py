#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import math
import statistics as st
import numpy as np


class entropy_calc_topic():

    def __init__(self):
        print("INITIALISING NODE -> /entropy_node")
        rospy.init_node('entropy_node')

        self.error_list_ang = []
        self.error_list_lin = []

        self.a_ang = 0.14
        self.a_lin = 0.08

        self.bins_ang = [-5*self.a_ang, -2.5*self.a_ang, -self.a_ang, -0.5*self.a_ang,
                         0.5*self.a_ang, self.a_ang, 2.5*self.a_ang, 5*self.a_ang]
        self.bins_lin = [-5*self.a_lin, -2.5*self.a_lin, -self.a_lin, -0.5*self.a_lin,
                         0.5*self.a_lin, self.a_lin, 2.5*self.a_lin, 5*self.a_lin]

        self.entropy_started = False
        self.start_sub = rospy.Subscriber("/start_entropy", Float32, self.trigger_callback)

        self.pup_entropy = rospy.Publisher("entropy", Float32, queue_size=1)
        self.pup_entropy_ang = rospy.Publisher("entropy_ang", Float32, queue_size=1)
        self.pup_entropy_lin = rospy.Publisher("entropy_lin", Float32, queue_size=1)

        self.sub_ang_error = rospy.Subscriber("estimation_error_ang", Float32, self.estimation_error_ang_callback)
        self.sub_lin_error = rospy.Subscriber("estimation_error_lin", Float32, self.estimation_error_lin_callback)

        self.entropy = 0.0
        self.entropy_average = 0.0
        self.entropy_sum = 0.0
        self.entropy_average_counter = 0.0

    def trigger_callback(self, msg):
        self.entropy_started = (msg.data == 1.0)
        if self.entropy_started:
            rospy.loginfo("Entropy logging started.")
        else:
            rospy.loginfo("Entropy logging stopped.")

    def estimation_error_ang_callback(self, data):
        self.error_list_ang.append(data.data)

    def estimation_error_lin_callback(self, data):
        self.error_list_lin.append(data.data)

    def calculate_error_frequencies(self, error_list, bins):
        p = [0]*9
        for val in error_list:
            if val <= bins[0]:
                p[0] += 1
            elif val <= bins[1]:
                p[1] += 1
            elif val <= bins[2]:
                p[2] += 1
            elif val <= bins[3]:
                p[3] += 1
            elif val <= bins[4]:
                p[4] += 1
            elif val <= bins[5]:
                p[5] += 1
            elif val <= bins[6]:
                p[6] += 1
            elif val <= bins[7]:
                p[7] += 1
            else:
                p[8] += 1

        souma = float(sum(p))
        frequencies = [x / souma for x in p if souma != 0]
        return frequencies

    def calculate_entropy(self, event=None):
        if not self.entropy_started:
            return

        frequencies_ang = self.calculate_error_frequencies(self.error_list_ang, self.bins_ang)
        frequencies_lin = self.calculate_error_frequencies(self.error_list_lin, self.bins_lin)

        entropy_ang = sum([-val*math.log(val, 9) for val in frequencies_ang if val != 0])
        entropy_lin = sum([-val*math.log(val, 9) for val in frequencies_lin if val != 0])

        self.entropy = 0.7 * entropy_ang + 0.3 * entropy_lin

        self.pup_entropy.publish(self.entropy)
        self.pup_entropy_ang.publish(entropy_ang)
        self.pup_entropy_lin.publish(entropy_lin)

        self.entropy_sum += self.entropy
        self.entropy_average_counter += 1
        self.entropy_average = self.entropy_sum / self.entropy_average_counter

        self.error_list_ang = []
        self.error_list_lin = []

        print(f"Entropy angular: {entropy_ang:.5f}, Entropy linear: {entropy_lin:.5f}, Total Entropy: {self.entropy:.5f}")

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = entropy_calc_topic()
        rospy.Timer(rospy.Duration(2.5), node.calculate_entropy)
        node.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        print(f"Total average entropy: {node.entropy_average:.5f}")