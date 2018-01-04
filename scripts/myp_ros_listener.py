#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This file contains a class that is needed if you want to see messages, warnings and errors of myP and if you want to
be able to react to dialogs via ROS services.

Usage example:

> from myp_ros.scripts.myp_ros_listener import ROSRobotListener
> ROSRobotListener().start()
"""
from threading import Thread

from myp_ros.scripts.myp_ros_functions import wait_for_info_from_myP, send_info_to_myP


class ROSRobotListener(Thread):
    """
    This class handles the dialogs and message functions of myP, so the user sees the messages and can react on dialogs.
    """
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        while True:
            # wait for information from myP (via dialogs or info messages)
            string, message = wait_for_info_from_myP()

            # in case of a dialog, wait for input and send reply
            if string == "dialog":
                answer = input(message)
                send_info_to_myP(answer)

            # in case of message, print message
            elif string == "message":
                print(message)
                send_info_to_myP(None)
