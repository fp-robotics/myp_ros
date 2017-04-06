#! /usr/bin/env python

import sys
import rospy, actionlib

from myp_ros.msg import MoveToJointReferenceAction, MoveToJointReferenceGoal, \
                        MoveToToolReferenceAction, MoveToToolReferenceGoal

from actionlib_msgs.msg import GoalStatus

# Define a done callback which is called when a goal is complete and passed
# an integer (terminal state) and a result object as its arguments
def done_cb(self, terminal_state, result):
    
    # if the terminal state indicates a success, log this and the result
    if terminal_state == GoalStatus.SUCCEEDED:
        rospy.loginfo("move_tool goal succeeded!")
        rospy.loginfo(result)
        
    # otherwise log the error
    else:
        rospy.logerr("move_tool goal did not successfully complete.")
        
# A simple function that prints its only argument, will be used to print feedback
def print_message(msg):
    rospy.loginfo(msg)


# Create the action clients for the joints and tool
move_joint_client = actionlib.SimpleActionClient('move_joint_action', MoveToJointReferenceAction)
move_tool_client = actionlib.SimpleActionClient('move_tool_action', MoveToToolReferenceAction)

# Wait until the move_joint_action is up
rospy.loginfo('waiting for move_joint_action action server...')
move_joint_client.wait_for_server()
rospy.loginfo('connected!')

# Wait until the move_tool_action is up
rospy.loginfo('waiting for move_tool_action action server...')
move_tool_client.wait_for_server()
rospy.loginfo('connected!')

# Construct the goals for each of the action clients
joint_goal = MoveToJointReferenceGoal(position=(10, 20, -10.5, 0, 0, 0))
tool_goal = client.new_goal(posture=Posture(xyz=(100, 100, 1000), orientation=[0, 0, 0], orientation_type='RPY'), velocity=80)

# Send the joint goal and wait until it completes
terminal_state = move_joint_client.send_goal_and_wait(joint_goal)

# If it completes successfully then log this
if terminal_state == GoalStatus.SUCCEEDED:
        rospy.loginfo("move_joint goal succeeded!")
        rospy.loginfo(result)
        
# Otherwise log the error
else:
    rospy.logerr("move_joint goal did not successfully complete.")

# Send the tool goal, this time not waiting, but rather attaching a call back to the
# result topic, and another to feedback topic
move_tool_client.send_goal(tool_goal, done_cb=self.done_cb, feedback_cb=print_message)

# Spin to prevent exiting while callbacks are carried out
rospy.spin()

