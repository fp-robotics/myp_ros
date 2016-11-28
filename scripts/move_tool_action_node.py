#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from myp_actions import QueuingClient
from myp_ros.srv import ConnectionCommand
from myp_ros.msg import Posture

PYTHON3 = sys.version_info[0] == 3

if PYTHON3:
    raw_input = input

def print_out(feedback):
    print(feedback)

if __name__ == '__main__':
    rospy.init_node('move_tool_action_node')

    try:
        connect = rospy.ServiceProxy('connect', ConnectionCommand)
        resp1 = connect('real', 'PRob2R', 'normal')
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        sys.exit()

    client = QueuingClient('move_tool_action')
    queuing = True
    clear = False

    while not rospy.is_shutdown():
        choice = raw_input('\nEnter X Y Z ROLL PITCH YAW;X2 Y2 Z2 ROLL2 PITCH2 YAW2 etc...\n(q=queue, d=direct, o=overried). e to exit\n')

        if choice == 'q':
            queuing = True
            print('Queuing ON\n')
            continue

        elif choice == 'd':
            queuing = False
            clear = False
            print('Queuing OFF\n')
            continue

        elif choice == 'o':
            queuing = False
            clear = True
            print('Override ON\n')
            continue

        elif choice == 'e':
            break

        else:
            list_of_arrays= choice.split(';')
            for posture in list_of_arrays:
                list_of_vals = posture.split(' ')
                vals = [float(c) for c in list_of_vals]
                goal = client.new_goal(posture=Posture(xyz=vals[:3], orientation=vals[3:6], orientation_type='Eular'), velocity=80)
                print("sending: " + str(goal))
                client.send_goal(goal, queuing, clear, feedback_cb=print_out)