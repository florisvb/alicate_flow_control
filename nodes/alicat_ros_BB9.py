#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
from alicat_flow_control.msg import bb9
import time

import alicat

class BB9AlicatFlowController:
    def __init__(self, port="/dev/alicat_bb9", addresses=['A', 'B'], publish_rate=1, publish_name_base='/alicat_flow_rate', subscribe_name='/alicat_bb9'):
        self.port = port
        self.publish_rate = publish_rate
        self.addresses = addresses
        self.flow_controllers = {address: alicat.FlowController(port, address, 2) for address in addresses}
        self.subscriber = rospy.Subscriber(subscribe_name, bb9, self.flow_control_callback, queue_size=10)
        self.publishers = {address: rospy.Publisher(publish_name_base+'_'+address, Float32, queue_size=10) for address in addresses}
        
    def flow_control_callback(self, data):
        for i, address in enumerate(data.address):
            self.flow_controllers[address].set_flow_rate(data.flowrate[i])
            
    def publish_flow_rates(self):
        flowrate = []
        for address in self.addresses:
            f = self.flow_controllers[address].get()['mass_flow'] 
            flowrate.append( f )
            self.publishers[address].publish(f)
        print self.addresses
        print flowrate
            
    def main(self):
        rate = rospy.Rate(self.publish_rate) # 10hz
        while not rospy.is_shutdown():
            self.publish_flow_rates()
            rate.sleep()
        for address in self.addresses:
            self.flow_controllers[address].close()
            
            
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--port", type="str", dest="port", default='/dev/alicat_bb9',
                        help="port")
    parser.add_option("--addresses", type="str", dest="addresses", default="['A', 'B', 'C']",
                        help="addresses for all of the attached flow controllers")
    parser.add_option("--publish_rate", type="float", dest="publish_rate", default=0.1,
                        help="rate at which to publish flow rate")        
    parser.add_option("--publish_name_base", type="str", dest="publish_name_base", default='/alicat_flow_rate',
                        help="topic base name to publish under")
    parser.add_option("--subscribe_name", type="str", dest="subscribe_name", default='/alicat_bb9',
                        help="topic to subscribe to")

    (options, args) = parser.parse_args()
    
    nodename = 'alicat_flow_controller_bb9'
    rospy.init_node(nodename)
    
    alicat_flow_controller_BB9 = BB9AlicatFlowController(options.port, eval(options.addresses), options.publish_rate, options.publish_name_base, options.subscribe_name)
    
    alicat_flow_controller_BB9.main()
            
            
'''

rostopic pub /alicat_bb9 alicat_flow_control/bb9 '{address:  ['A', 'B'], flowrate: [0, 0]}'

'''
            
