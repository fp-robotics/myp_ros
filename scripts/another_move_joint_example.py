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

position = [0, 0, 0, 0, 0, 0]
given = raw_input("Current Position: " +str(position) + '\nEnter 6 space seperated joints, or jX Y where X is the joint number and Y is the value to plus or minus. q to quit.\n')
while given != "q":

	if given[0] == "j":
		data = given[1:].split(" ")			
		joint = int(data[0])-1
		val = float(data[1])
		position[joint] += val

	else:
		position = [float(val) for val in given.split(" ")]

	move_joint = rospy.ServiceProxy('move_joint', MoveJoint)
	actuator_ids = ['1', '2', '3', '4', '5', '6']
	resp = move_joint(actuator_ids=actuator_ids, position=position, velocity=velocity, acceleration=acceleration)
	rospy.loginfo(resp.message)
	given = raw_input("Current Position: " +str(position) + '\nEnter 6 space seperated joints, or jXpY or jXmY where X is the joint number and Y is the value to plus or minus. q to quit.\n')
