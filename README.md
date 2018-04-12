# Single Flow Controller, (not using ROS)

Download the repository:
```
git clone https://github.com/florisvb/alicate_flow_control.git
```

From inside the alicat package run:
```
python ./setup.py install
```

If you are using a serial to USB adapter, you need to add your user to the group dialout:
```
sudo adduser $USER dialout
```

Then you must log out / in. 

In the mean time, set up your Alicat flow controller to respond to the serial commands. On the front panel navigate these menus:
```
menu > control > setpt source > serial/front panel
```

Here are some basic commands to get started:
```
import alicat
a = alicat.FlowController(driver_version=2) # driver_version 2 needed for most newer alicats
print( a.get()['flow_setpoint'] )
print( a.get()['mass_flow'] )
a.set_flow_rate(10)
```

# Multiple Flow Controllers

Recommended to use the BB9 controller. But you can address individual alicat's through their ports:
```
a = alicat.FlowController(port='/dev/ttyUSB0', driver_version=2)
```