#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
import time
import yaml
import os, sys
import shutil

def get_float_local_time_hours():
    t = time.localtime(time.time())
    lt = t.tm_hour + t.tm_min/60. + t.tm_sec/3600.
    return lt

class AlicatFlowController:
    def __init__(self,  config_file=None, # preferred approach
                        pulse_interval=2400, 
                        pulse_length=600, 
                        first_pulse_time=-1,
                        first_pulse_delay=0,
                        flow_rate=5,
                        publish_name='/alicat_flow_control'):
        
        loaded_from_config = False
        if config_file is not None:
            if len(config_file) > 0:
                # Load config
                with open(config_file) as stream:
                    self.config = yaml.safe_load(stream)
                print()
                print('Loaded config: ' + config_file)
                # Save config to data directory for reference
                save_dir = os.path.join(self.config['data_directory'], 'exp_code')
                if os.path.exists(save_dir):
                    print('writing exp code to: ' + save_dir)
                else:
                    os.mkdir(save_dir)
                    print('created dir: ' + save_dir)
                    print('writing exp code to: ' + save_dir)
                shutil.copy(config_file, save_dir)
                # also copy this python file
                shutil.copy(__file__, save_dir)

        
                print('start')
                self.pulse_interval = self.config['pulse_interval']
                self.pulse_length = self.config['pulse_length']
                self.publish_name = self.config['publish_name']
                self.first_pulse_time = self.config['first_pulse_time']
                self.first_pulse_delay = self.config['first_pulse_delay']
                self.flow_rate = self.config['flow_rate']

                loaded_from_config = True
        
        if not loaded_from_config:
            self.pulse_interval = pulse_interval
            self.pulse_length = pulse_length
            self.publish_name = publish_name
            self.first_pulse_time = first_pulse_time
            self.first_pulse_delay = first_pulse_delay
            self.flow_rate = flow_rate

        print('flow control parameters')
        print('pulse interval: ', self.pulse_interval)
        print('pulse_length: ', self.pulse_length)
        print('publish_name: ', self.publish_name)
        print('first_pulse_time: ', self.first_pulse_time)
        print('first_pulse_delay: ', self.first_pulse_delay)
        print('flow_rate: ', self.flow_rate)

        print('start')
        self.publisher = rospy.Publisher(self.publish_name, Float32, queue_size=10)
        time.sleep(2)
        self.publisher.publish(0)
        print('go')
        print('first pulse time: ', self.first_pulse_time)
        
    def main(self, flow_rate=None):
    
        if flow_rate is None:
            flow_rate = self.flow_rate

        # first pause until local time reached
        if self.first_pulse_time > 0:
            print('waiting until: ', self.first_pulse_time)
            rate = rospy.Rate(0.25)
            while not rospy.is_shutdown():
                lt = get_float_local_time_hours()
                if lt > self.first_pulse_time:
                    print('time reached: ', self.first_pulse_time, 'current lt: ', lt)
                    break
                    
        print('running')
        rospy.sleep(self.first_pulse_delay)
        rate = rospy.Rate(1 / float(self.pulse_interval) ) 
        print('running')
        while not rospy.is_shutdown():
            
            if type(flow_rate) is list:
                index = np.random.randint(len(flow_rate))
                f = flow_rate[index]
            else:
                f = flow_rate

            self.publisher.publish(f)
            print('flow rate: ', flow_rate)
            rospy.sleep(self.pulse_length)
            self.publisher.publish(0)
            print('flow rate: ', 0)
            rate.sleep()

        self.publisher.publish(0)
            
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--config_file", type="str", dest="config_file", default='',
                        help="path to config yaml file")
    parser.add_option("--first_pulse_time", type="float", dest="first_pulse_time", default=-1,
                        help="time (localtime in hours with decimals to indicate seconds and minutes) to send first pulse, defaults to -1 which means now")
    parser.add_option("--first_pulse_delay", type="int", dest="first_pulse_delay", default=0,
                        help="number of seconds to wait before sending first pulse")
    parser.add_option("--pulse_interval", type="int", dest="pulse_interval", default=10800,
                        help="pulse interval in seconds, default is 3 hrs")
    parser.add_option("--pulse_length", type="int", dest="pulse_length", default=600,
                        help="pulse length in seconds")
    parser.add_option("--publish_name", type="str", dest="publish_name", default='/alicat_flow_control',
                        help="topic to publish to")
    parser.add_option("--flow_rate", type="float", dest="flow_rate", default=5,
                        help="flow rate")
    
    (options, args) = parser.parse_args()
    
    rospy.init_node('alicat_ros_flow_controller')
    alicat_flow_controller = AlicatFlowController(  config_file=options.config_file,
                                                    pulse_interval=options.pulse_interval, 
                                                    pulse_length=options.pulse_length, 
                                                    first_pulse_time=options.first_pulse_time,
                                                    first_pulse_delay=options.first_pulse_delay,
                                                    publish_name=options.publish_name,
                                                    flow_rate=options.flow_rate)
    alicat_flow_controller.main()
            
            
