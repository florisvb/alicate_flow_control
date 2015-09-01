#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String

class AlicatFlowController:
    def __init__(self, pulse_interval=2400, pulse_length=600, publish_name='/alicat_flow_control'):
        self.pulse_interval = pulse_interval
        self.pulse_length = pulse_length
        self.publisher = rospy.Publisher(publish_name, Float32, queue_size=10)
        
    def main(self, flow_rate=5):
        rospy.sleep(self.pulse_interval)
        rate = rospy.Rate(1 / float(self.pulse_interval) ) 
        while not rospy.is_shutdown():
        
            if type(flow_rate) is list:
                index = np.random.randint(len(flow_rate))
                f = flow_rate[index]
            else:
                f = flow_rate

            self.publisher.publish(f)
            print 'flow rate: ', flow_rate
            rospy.sleep(self.pulse_length)
            self.publisher.publish(0)
            print 'flow rate: ', 0
            rate.sleep()
            
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--pulse_interval", type="int", dest="pulse_interval", default=2400,
                        help="pulse interval")
    parser.add_option("--pulse_length", type="int", dest="pulse_length", default=600,
                        help="pulse length")
    parser.add_option("--publish_name", type="str", dest="publish_name", default='/alicat_flow_control',
                        help="topic to publish to")
    parser.add_option("--flow_rate", type="float", dest="flow_rate", default=5,
                        help="flow rate")
    
    (options, args) = parser.parse_args()
    
    rospy.init_node('alicat_ros_flow_controller')
    alicat_flow_controller = AlicatFlowController(options.pulse_interval, options.pulse_length, options.publish_name)
    alicat_flow_controller.main(flow_rate=options.flow_rate)
            
            
