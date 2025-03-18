#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
import time

import alicat

class AlicatFlowController:
    def __init__(self, port="/dev/ttyUSB0", address="A", publish_rate=1, publish_name='/alicat_flow_rate', subscribe_name='/alicat_flow_control', driver_version=1):
        self.port = port
        self.address = address
        self.flow_controller = alicat.FlowController(port, address, driver_version)
        self.publish_rate = publish_rate
        self.publish_name = publish_name
        self.publisher = rospy.Publisher(publish_name, Float32, queue_size=10)
        self.subscriber = rospy.Subscriber(subscribe_name, Float32, self.flow_control_callback, queue_size=10)

        self.data = None
        self.desired_flow_rate = None

    def flow_control_callback(self, data):
        self.data = data
        if '_2' in self.publish_name:
            if data.data != 0:
                self.desired_flow_rate = 5
            else:
                self.desired_flow_rate = 0
            self.flow_controller.set_flow_rate(self.desired_flow_rate, retries=2)
            
        else:
            self.flow_controller.set_flow_rate(data.data, retries=2)
            self.desired_flow_rate = data.data
            
    def publish_flow_rate(self):
        try:
            flow_rate = self.flow_controller.get()['mass_flow']
            self.publisher.publish(flow_rate)
            if self.desired_flow_rate is not None:
                if np.abs(flow_rate-self.desired_flow_rate) > 1:
                    self.flow_controller.set_flow_rate(0, retries=2)
                    time.sleep(0.25)
                    self.flow_control_callback(self.data)
        except:
            print('Could not get / publish flow rate')
        
    def main(self):
        rate = rospy.Rate(self.publish_rate) # 10hz
        while not rospy.is_shutdown():
            self.publish_flow_rate()
            rate.sleep()
        self.flow_controller.close()
            
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--port", type="str", dest="port", default='/dev/ttyUSB0',
                        help="port")
    parser.add_option("--address", type="str", dest="address", default='A',
                        help="address")
    parser.add_option("--publish_name", type="str", dest="publish_name", default='/alicat_flow_rate',
                        help="topic name to publish under")
    parser.add_option("--subscribe_name", type="str", dest="subscribe_name", default='/alicat_flow_control',
                        help="topic name to subscribe to")
    parser.add_option("--publish_rate", type="float", dest="publish_rate", default=0.1,
                        help="rate at which to publish flow rate")
    parser.add_option("--driver_version", type="int", dest="driver_version", default=1,
                        help="which driver to use, 1 or 2")
    
    (options, args) = parser.parse_args()
    
    rospy.init_node('alicat_ros')
    alicat_flow_controller = AlicatFlowController(options.port, options.address, options.publish_rate, options.publish_name, options.subscribe_name, options.driver_version)
    alicat_flow_controller.main()
            
            
