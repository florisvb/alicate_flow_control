# ROS node for trisonica mini

Run using `rosrun ./trisonica_ros trisonica.py`

To set parameters on the trisonica, use minicom:
`minicom -s`

* Change hardware flow control to off (`F`)
* Set serial device (`A`)
* Hit `ctrl-c`

This opens a command line interface with a prompt:
`>`

Then run desired command, e.g.: 
`> outputrate 40`

To save parameters run: 
`> nvwrite`

How to do this over pyserial????
