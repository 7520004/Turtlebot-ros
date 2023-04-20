#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node('count')    #nodeName "count"
pub = rospy.Publisher('count_up', Int32, queue_size=1) #publisher "count_up" を作成
rate = rospy.Rate(10)
n = 0 
while not rospy.is_shutdown():
    n += 1
    pub.publish(n)
    rate.sleep()