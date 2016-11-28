#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from myp_actions import QueuingClient
from myp_ros.srv import ConnectionCommand

PYTHON3 = sys.version_info[0] == 3

if PYTHON3:
    raw_input = input

if __name__ == '__main__':
    rospy.init_node('move_joint_action_node')

    try:
        connect = rospy.ServiceProxy('connect', ConnectionCommand)
        resp1 = connect('real', 'PRob2R', 'normal')
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        sys.exit()

		#set up the client
    client = QueuingClient('move_joint_action')
    queuing = True
    clear = False

		#loop to receive input and send as goals
    while not rospy.is_shutdown():
        choice = raw_input('\nEnter one or more (space seperated) values to set joints 1, 2 and 3 to.\n(q=queue, d=direct, o=overried). e to exit\n')

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
						#split up the goals
            list_of_vals = choice.split(' ')

            for str_num in list_of_vals:
                val = float(str_num)

								#create the new goal
                goal = client.new_goal(position=(val, val, val, 0, 0, 0, 0))

								#send with the appropriate flags
                client.send_goal(goal, queuing, clear)



