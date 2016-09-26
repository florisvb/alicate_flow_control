#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
from geometry_msgs.msg import PointStamped
from alicat_flow_control.msg import bb9
import time
import imp
import message_filters


class BB9Synchronizer:
    def __init__(self, coupling_function, additional_topic_to_pass_to_coupler=None, master_topic=None, slave_topic='/alicat_bb9', nodename='alicat_ros_BB9_republisher_N'):
        '''
        '''
        rospy.init_node(nodename)

        self.slave_publisher = rospy.Publisher(slave_topic, bb9, queue_size=10)
        
        
        master_subscribers = [message_filters.Subscriber(master_topic[0], master_topic[1])]
        if additional_topic_to_pass_to_coupler is not None and additional_topic_to_pass_to_coupler != 'none':
            for topic, message_type in additional_topic_to_pass_to_coupler.items():
                master_subscribers.append( message_filters.Subscriber(topic, message_type ) ) # in practice, only 1 topic supported, but possible to add more

        ts = message_filters.TimeSynchronizer(master_subscribers, 10)
        ts.registerCallback(self.synchronized_callback)
        rospy.spin()    

    def synchronized_callback(self, master_topic, additional_topic):
        slave_topic_and_data = coupling_function(master_topic, additional_topic)
        
        address = slave_topic_and_data.keys()
        flowrate = [slave_topic_and_data[a] for a in address]
        
        msg = bb9()
        msg.header = master_topic.header
        msg.address = address
        msg.flowrate = flowrate
        
        self.slave_publisher.publish(msg)

    
    
    

    
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--master_topic", type="str", dest="master_topic", default="['/alicat_flow_control_A', PointStamped]",
                        help="master topic controlling the slave topics / controllers")
    parser.add_option("--additional_topic", type="str", dest="additional_topic", default="None",
                        help="For additional control with the coupling function. Use format: '{'/extra_topic': PointStamped}' ")
    parser.add_option("--slave_topic", type="str", dest="slave_topic", default="/alicat_bb9",
                        help="slave topics (whatever the bb9 node is listening to")
    parser.add_option("--coupling_function_filename", type="str", dest="coupling_function_filename",
                        help=".py filename where the coupling function can be found. Will be loaded using imp")
    parser.add_option("--coupling_function_fname", type="str", dest="coupling_function_fname",
                        help="function name of the coupling function, found in the coupling_function_filename")
    parser.add_option("--nodename", type="str", dest="nodename", default='alicat_ros_BB9_republisher_N',
                        help="name of the republisher node - if using multiple republishers ensure that their names are different.")
    
    (options, args) = parser.parse_args()
    
    coupling_function_module = imp.load_source('coupling_function_module', options.coupling_function_filename)
    coupling_function = coupling_function_module.__getattribute__(options.coupling_function_fname)
    
            
    BB9Synchronizer(    coupling_function, 
                        additional_topic_to_pass_to_coupler=eval(options.additional_topic), 
                        master_topic=eval(options.master_topic), 
                        slave_topic=options.slave_topic, 
                        nodename=options.nodename)
            
            
            
