#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PointStamped
import time

def coupling_function_AB_primaryflow_C_addedflow(master_topic, additional_topic):
    '''
    master_topic and additional_topic are both floats
    '''
    slave_topics = ['A', 'B', 'C']
    if additional_topic.point.x == 0:
        slave_topics_and_data = {   'A': 20, 
                                    'B': 20,
                                    'C': 0}
    if additional_topic.point.x == 1:
        slave_topics_and_data = {   'A': 20-master_topic.point.x, 
                                    'B': 20,
                                    'C': master_topic.point.x}
    if additional_topic.point.x == -1:
        slave_topics_and_data = {   'A': 20, 
                                    'B': 20-master_topic.point.x,
                                    'C': master_topic.point.x}
                                    
    return slave_topics_and_data
