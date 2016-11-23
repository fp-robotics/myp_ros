#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from user_code import QueuingClient, ActionLibrary
from myp_ros.srv import ConnectionCommand

if __name__ == '__main__':
    rospy.init_node('test_node')
    action_name = 'move_tool'
    action_class = ActionLibrary.action_classes[action_name]
    queuing = False

    try:
        connect = rospy.ServiceProxy('connect', ConnectionCommand)
        resp1 = connect('real', 'PRob2R', 'normal')
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        sys.exit()

    client = QueuingClient(action_name, action_class)

    while not rospy.is_shutdown():
        goal = ActionLibrary.goal_classes[action_name]()
        choice = input('a, b or r (switch queue or direct with q or d). e to exit')
        if choice == 'a':
            args = [(20, 20, 20, 0, 0, 0, 0), ('1', '2', '3', '4', '5', '6', '7')]

        elif choice == 'b':
            args = [(-20, -20, -20, 0, 0, 0, 0), ('1', '2', '3', '4', '5', '6', '7')]

        elif choice == 'r':
            args = [(0, 0, 0, 0, 0, 0, 0), ('1', '2', '3', '4', '5', '6', '7')]

        elif choice == 'q':
            queuing = True
            continue

        elif choice == 'd':
            queuing = False
            continue

        elif choice == 'e':
            break

        else:
            continue

        for arg, inputted_arg in zip(ActionLibrary.action_goal_arguments[action_name], args):
                    setattr(goal, arg[0], inputted_arg)

        client.send_goal(goal, queuing)



