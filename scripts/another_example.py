#!/usr/bin/python

'''
This is an example node which subscribes to every topic and uses every service.
'''

import sys, rospy

from myp_ros.msg import *
from myp_ros.srv import *

'''
A callback message for the subscibers.
'''
def do_something(msg):
	return

'''
Initialize the node.
'''
rospy.init_node('example')


'''
Wait for the connection service to be provided by the PRob.
Connect to the PRob using this service.
Print out the response.
'''
rospy.wait_for_service('connect')
connect = rospy.ServiceProxy('connect', ConnectionCommand)
resp = connect('real', 'PRob2R', 'normal')
rospy.loginfo(resp.message)


'''
Set up the subscribers.
'''
rospy.loginfo('Subscribing to all messages')

sub1 = rospy.Subscriber('joint_angles', JointAngles, do_something)
sub2 = rospy.Subscriber('current', Current, do_something)
sub3 = rospy.Subscriber('digital_inputs', DigitalInputs, do_something)
sub4 = rospy.Subscriber('pose', Pose, do_something)
sub5 = rospy.Subscriber('posture', Posture, do_something)
sub6 = rospy.Subscriber('sensor_packet', SensorPacket, do_something)


'''
Test each service
'''

vel = 80
acc = 80

velocity = [80]*6
acceleration = [80]*6

path_vel = [80]*2
path_acc = [80]*2

rospy.loginfo('Using each service after ENTER is pressed')

raw_input('Using service open_gripper')

open_gripper = rospy.ServiceProxy('open_gripper', OpenGripper)
resp = open_gripper(velocity=vel, acceleration=acc)
rospy.loginfo(resp.message)

raw_input('Using service close_gripper')

close_gripper = rospy.ServiceProxy('close_gripper', CloseGripper)
resp = close_gripper(velocity=vel, acceleration=acc)
rospy.loginfo(resp.message)

raw_input('Using service move_joint')

move_joint = rospy.ServiceProxy('move_joint', MoveJoint)
actuator_ids = ['1', '2', '3', '4', '5', '6']
position = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
resp = move_joint(actuator_ids=actuator_ids, position=position, velocity=velocity, acceleration=acceleration)
rospy.loginfo(resp.message)

raw_input('Using service move_to_pose')

move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)
resp = move_to_pose(pose_name='pose_name', velocity=vel, acceleration=acc)
rospy.loginfo(resp.message)

raw_input('Using service move_tool')

move_tool = rospy.ServiceProxy('move_tool', MoveTool)

x, y, z = 0.0, 200.0, 920.0
orientation = [30.0, -20.0, -90.0]
resp = move_tool(x=x, y=y, z=z, orientation=orientation, velocity=velocity, acceleration=acceleration)
rospy.loginfo(resp.message)

raw_input('Using service run_simple_path')

run_simple_path = rospy.ServiceProxy('run_simple_path', RunSimplePath)
actuator_ids = ['1', '2', '3', '4', '5', '6']
positions = [[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
	     [10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
             [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]]

path_data = [JointAngles(data=pos, names=actuator_ids, length=6) for pos in positions] 

resp = run_simple_path(path_data=path_data, velocity=path_vel, acceleration=path_acc)
rospy.loginfo(resp.message)
