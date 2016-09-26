#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import time





rospy.init_node('test_of_BB9_republisher')

master_topic_pub = rospy.Publisher('/master_topic', PointStamped, queue_size=10)
additional_topic_pub = rospy.Publisher('/additional_topic', PointStamped, queue_size=10)

msg = PointStamped()

n = 1

while not rospy.is_shutdown():
    time.sleep(10)
    
    print 'turning on master: ', n 
    msg.header.stamp = rospy.Time.now()
    msg.point.x = 5
    master_topic_pub.publish(msg)
    msg.point.x = n
    additional_topic_pub.publish(msg)
    time.sleep(2)
    
    print 'turning off master: ', 0 
    msg.header.stamp = rospy.Time.now()
    msg.point.x = 0
    master_topic_pub.publish(msg)
    msg.point.x = 0
    additional_topic_pub.publish(msg)
    
    n *= -1
    
    
'''

rosrun alicat_flow_control alicat_ros_BB9_republishers.py --master_topic="['/master_topic', PointStamped]" --additional_topic="{'/additional_topic': PointStamped}" --coupling_function_filename='alicat_ros_BB9_republisher_coupling_functions.py' --coupling_function_fname='coupling_function_AB_primaryflow_C_addedflow'


'''
