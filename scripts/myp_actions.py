#! /usr/bin/env python

import sys
import rospy, actionlib

from myp_ros.msg import MoveToJointReferenceAction, MoveToJointReferenceGoal, \
                        MoveToToolReferenceAction, MoveToToolReferenceGoal

from actionlib_msgs.msg import GoalStatus
from difflib import get_close_matches

PYTHON3 = sys.version_info[0] == 3

if PYTHON3:
    from queue import Queue

else:
    from Queue import Queue


class ActionLibrary:
    actions = {}
    goals = {}

    actions['move_joint_action'] = MoveToJointReferenceAction
    goals['move_joint_action'] = MoveToJointReferenceGoal

    actions['move_tool_action'] = MoveToToolReferenceAction
    goals['move_tool_action'] = MoveToToolReferenceGoal

    @staticmethod
    def get_correct_name(action_name):

        if action_name in ActionLibrary.actions:
            return action_name

        name = get_close_matches(action_name, list(ActionLibrary.actions.keys()), n=1, cutoff=0.2)

        if len(name) == 0:
            rospy.logerr('Your action name is not close enough to a real action name')
            return

        rospy.loginfo('Action name '+ action_name + ' was corrected to ' + name[0] + '.')
        return name[0]

    @staticmethod
    def get_class(action_name):
        if action_name not in ActionLibrary.actions:
            action_name = ActionLibrary.get_correct_name(action_name)
        return ActionLibrary.actions[action_name]

    @staticmethod
    def get_goal_class(action_name):
        if action_name not in ActionLibrary.actions:
            action_name = ActionLibrary.get_correct_name(action_name)
        return ActionLibrary.goals[action_name]

'''
A Class which allows for the queuing of goals on the client end of an action.
'''
class QueuingClient:
    def __init__(self, action_name, action_class=None):
        self.action_name = ActionLibrary.get_correct_name(action_name)

        if action_class is None:
            action_class = ActionLibrary.get_class(action_name)

        self.client = actionlib.SimpleActionClient(action_name, action_class)
        self.goal_queue = Queue()

        rospy.loginfo('waiting for ' + self.action_name + 'action server...')
        self.client.wait_for_server()
        rospy.loginfo('connected!')

    def send_goal(self, goal, queue=False, clear=False, feedback_cb=None):
        if queue:
            rospy.sleep(0.1)
            if self.client.get_state() != GoalStatus.ACTIVE:
                print("free")
                self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=feedback_cb)
            else:
                self.goal_queue.put((goal, feedback_cb))

        elif clear:
            self.client.send_goal(goal, done_cb=self.clear, feedback_cb=feedback_cb)

        else:
            self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=feedback_cb)

    def done_cb(self, terminal_state, result):
        if not self.goal_queue.empty():
            goal, feedback_cb = self.goal_queue.get()
            self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=feedback_cb)

    def new_goal(self, *args, **kwargs):
        return ActionLibrary.get_goal_class(self.action_name)(*args, **kwargs)

    def clear(self, terminal_state, result):
        with self.goal_queue.mutex:
            self.goal_queue.queue.clear()