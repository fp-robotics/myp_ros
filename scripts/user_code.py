#! /usr/bin/env python

import sys

PYTHON3 = sys.version_info[0] == 3

if PYTHON3:
    from queue import Queue

else:
    from Queue import Queue

import actionlib

from myp_ros.msg import MoveToJointReferenceAction, MoveToJointReferenceGoal, MoveToToolReferenceAction, MoveToToolReferenceGoal

from actionlib_msgs.msg import GoalStatus

class ActionLibrary:
    action_classes = {}
    goal_classes = {}
    action_goal_arguments = {}

    action_classes['move_joint'] = MoveToJointReferenceAction
    goal_classes['move_joint'] = MoveToJointReferenceGoal
    action_goal_arguments['move_joint'] = [
        ('joint_reference', ' (numbers separated by a comma and optionally a space):', 'list of floats'),
        ('joint_names', ' (integers separated by a comma and space):', 'list of strings')]

    action_classes['move_tool'] = MoveToToolReferenceAction
    goal_classes['move_tool'] = MoveToToolReferenceGoal
    action_goal_arguments['move_tool'] = [
        ('tool_reference', ' (Enter values as: x y z roll pitch yaw):', 'list of floats')]
        
    @staticmethod
    def parse_input(user_input, output_type):
        if output_type == 'list of strings':
            return user_input.split(', ')

        if output_type == 'list of floats':
            return [float(item) for item in user_input.split(',')]



class QueuingClient:
    def __init__(self, action_name, action_class):
        self.client = actionlib.SimpleActionClient(action_name, action_class)
        self.goal_queue = Queue()
        print('waiting for server...')
        self.client.wait_for_server()
        print('connected!')

    def send_goal(self, goal, queue=False):
        if queue:
            if self.client.get_state() != GoalStatus.ACTIVE:
                self.client.send_goal(goal, done_cb=self.done_cb)
            else:
                self.goal_queue.put(goal)

        else:
            self.client.send_goal(goal, done_cb=self.done_cb)

    def done_cb(self, terminal_state, result):
        if not self.goal_queue.empty():
            self.client.send_goal(self.goal_queue.get(), done_cb=self.done_cb)

