#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, actionlib

from myp_ros.msg import MoveToJointReferenceAction, MoveToJointReferenceGoal, MoveToToolReferenceAction, MoveToToolReferenceGoal, Posture

rospy.init_node('test_node')
jg = MoveToJointReferenceGoal(position=[10,10,10,10,10,10])
tg = MoveToToolReferenceGoal(posture=Posture(xyz=(0.0, 200.0, 920.0), orientation=[30.0, -20.0, -90.0]), velocity=100)

mj = actionlib.SimpleActionClient('move_joint_action', MoveToJointReferenceAction)
mt = actionlib.SimpleActionClient('move_tool_action', MoveToToolReferenceAction)
print(tg)
mj.wait_for_server()
print("done")
mj.send_goal(jg)
raw_input('sent mj goal')
mt.send_goal(tg)
raw_input('sent mt goal')


