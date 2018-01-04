#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This file contains an example script that uses myP functions via ROS Services.
The script combines myP script functions (which only run if the status is 'running')
and special ROS Services to control the robot if no myP script function is running.
"""
from myp_ros.scripts.myp_ros_functions import *
from myp_ros.scripts.myp_ros_listener import ROSRobotListener


# starting listener to see messages and errors and being able to react on dialogs
ROSRobotListener().start()

# standard initialization procedure
connect()
calibrate()
initialize_application()

# write your own myP application here (or copy an existing application from the myP GUI and paste it here)
# here's an example code though:
print('start of application')
while True:
    for joint in range(6):
        move_joint(joint,  10, velocity=50, acceleration=100)
        move_joint(joint, -10, velocity=50, acceleration=100)
        move_joint(joint,   0, velocity=50, acceleration=100)
    if not dialog_yes_no("repeat this motion?", "Question", 0):
        break
print('end of application')

# standard shutdown procedure
finalize_application()
disconnect()
