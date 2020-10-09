# ROS node for Alicat Flow Meter control

### Basic flow control

1. Install alicat module: from this home directory run `python ./setup.py install`
2. Possibly update rules, note port number (check `/dev` after unplugging and plugging in device), note device address (on device: `menu>about>adv setup>comm setup` and note unit id)
3. Run the ROS node (assuming your port is /dev/tty/USB0 and device address is A and you're on driver version 2 (e.g. controllers made after 2016ish): 
   `rosrun alicat_flow_control alicat_ros.py --port=/dev/ttyUSB0 --address=A --driver_version=2`
4. Change the flowrate by publishing on `/alicat_flow_control`, e.g.: `rostopic pub /alicat_flow_control std_msgs/Float32 "data: 30.0"`
5. Record the flowrate from topic `/alicat_flow_rate`. You can change the rate with an option when running alicat_flow_control alicat_ros.py
6. To turn on flow at a given local time run: `rosrun alicat_flow_ntrol alicat_ros_controller.py --first_pulse_time=13.5 --pulse_length=3600 --pulse_interval=3600`. That command will give you 1 hr long pulses every hour starting at 1:30 pm localtime.   
