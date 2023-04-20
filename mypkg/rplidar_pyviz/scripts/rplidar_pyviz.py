#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan 
import numpy as np
import matplotlib.pyplot as plt

def callback(msg):
    global num_points
    global x
    global y

    r = np.array(msg.ranges)

    for i in range(num_points):
        theta = msg.angle_min + msg.angle_increment * i
        x[i] = -r[i] * np.cos(theta)
        y[i] = -r[i] * np.sin(theta)

    x[-1] = -r[-1] * np.cos(msg.angle_max)
    y[-1] = -r[-1] * np.sin(msg.angle_max)

if __name__ == '__main__':
    rospy.init_node('rplidar_pyviz', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)

    global num_points
    global x
    global y

    num_points = 760
    x = np.zeros(num_points)
    y = np.zeros(num_points)

    fig, ax = plt.subplots()

    while not rospy.is_shutdown():
        ax.cla()
        ax.set_xlim(-15, 15)
        ax.set_ylim(-15, 15)
        ax.grid(True)
        ax.scatter(x, y)
        plt.draw()
        plt.pause(0.1)	

    rospy.spin()
