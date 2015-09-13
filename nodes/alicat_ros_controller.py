#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
import time

def get_float_local_time_hours():
    t = time.localtime(time.time())
    lt = t.tm_hour + t.tm_min/60. + t.tm_sec/3600.
    return lt

class AlicatFlowController:
    def __init__(self,  pulse_interval=2400, 
                        pulse_length=600, 
                        first_pulse_time=-1,
                        first_pulse_delay=0,
                        publish_name='/alicat_flow_control'):
        print 'start'
        self.pulse_interval = pulse_interval
        self.pulse_length = pulse_length
        self.publisher = rospy.Publisher(publish_name, Float32, queue_size=10)
        self.first_pulse_time = first_pulse_time
        self.first_pulse_delay = first_pulse_delay
        time.sleep(2)
        print 'go'
        
    def main(self, flow_rate=5):
    
        # first pause until local time reached
        if self.first_pulse_time > 0:
            print 'waiting until: ', self.first_pulse_time
            rate = rospy.Rate(0.25)
            while not rospy.is_shutdown():
                lt = get_float_local_time_hours()
                if np.abs(lt-self.first_pulse_time) < 1:
                    break
                    
        print 'running'
        rospy.sleep(self.first_pulse_delay)
        rate = rospy.Rate(1 / float(self.pulse_interval) ) 
        print 'running'
        while not rospy.is_shutdown():
            
            print 'hi'
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
    parser.add_option("--first_pulse_time", type="int", dest="first_pulse_time", default=-1,
                        help="time (localtime in hours) to send first pulse, defaults to -1 which means now")
    parser.add_option("--first_pulse_delay", type="int", dest="first_pulse_delay", default=0,
                        help="number of seconds to wait before sending first pulse")
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
    alicat_flow_controller = AlicatFlowController(  pulse_interval=options.pulse_interval, 
                                                    pulse_length=options.pulse_length, 
                                                    first_pulse_time=options.first_pulse_time,
                                                    first_pulse_delay=options.first_pulse_delay,
                                                    publish_name=options.publish_name)
    alicat_flow_controller.main(flow_rate=options.flow_rate)
            
            
