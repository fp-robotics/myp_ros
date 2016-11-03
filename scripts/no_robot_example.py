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
#rospy.wait_for_service('connect')
rospy.loginfo('Connecting')
connect = rospy.ServiceProxy('connect', ConnectionCommand)
resp = connect('no_robot', 'PRob2R', 'default')
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
rospy.loginfo('Using each service after ENTER is pressed')

raw_input('Using service wait')

wait = rospy.ServiceProxy('wait', Wait)
resp = wait(0.5)
rospy.loginfo(resp.message)

raw_input('Using service open_gripper')

open_gripper = rospy.ServiceProxy('open_gripper', OpenGripper)
resp = open_gripper()
rospy.loginfo(resp.message)

raw_input('Using service close_gripper')

close_gripper = rospy.ServiceProxy('close_gripper', CloseGripper)
resp = close_gripper()
rospy.loginfo(resp.message)

raw_input('Using service move_joint')

move_joint = rospy.ServiceProxy('move_joint', MoveJoint)
actuator_ids = ['1', '2', '3', '4', '5', '6']
position = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
resp = move_joint(actuator_ids=actuator_ids, position=position)
rospy.loginfo(resp.message)

raw_input('Using service move_linear_axis')

move_linear_axis = rospy.ServiceProxy('move_linear_axis', MoveLinearAxis)
actuator_ids = ['1', '2', '3', '4', '5', '6']
position = 1
resp = move_linear_axis(actuator_ids=actuator_ids, position=position)
rospy.loginfo(resp.message)

raw_input('Using service move_to_pose')

move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)
resp = move_to_pose(pose_name='pose_name')
rospy.loginfo(resp.message)

raw_input('Using service move_tool')

move_tool = rospy.ServiceProxy('move_tool', MoveTool)
x, y, z = 1.0, 1.0, 1.0
orientation = [1.0, 1.0, 1.0]
resp = move_tool(x=x, y=y, z=z, orientation=orientation)
rospy.loginfo(resp.message)

raw_input('Using service play_path')

play_path = rospy.ServiceProxy('play_path', PlayPath)
resp = play_path(path_name='path_name')
rospy.loginfo(resp.message)

raw_input('Using service read_gripper_angle')

read_gripper_angle = rospy.ServiceProxy('read_gripper_angle', ReadGripperAngle)
resp = read_gripper_angle()
rospy.loginfo(resp.message)

raw_input('Using service read_tcp_pose')

read_tcp_pose = rospy.ServiceProxy('read_tcp_pose', ReadTcpPose)
resp = read_tcp_pose()
rospy.loginfo(resp.message)

raw_input('Using service recognize_object')

recognize_object = rospy.ServiceProxy('recognize_object', RecognizeObject)
resp = recognize_object(lesson_name='lesson_name')
rospy.loginfo(resp.message)

raw_input('Using service run_simple_path')

run_simple_path = rospy.ServiceProxy('run_simple_path', RunSimplePath)
actuator_ids = ['1', '2', '3', '4', '5', '6']
positions = [[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
	     [10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
             [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]]

path_data = [JointAngles(data=pos, names=actuator_ids, length=6) for pos in positions] 

resp = run_simple_path(path_data=path_data)
rospy.loginfo(resp.message)

raw_input('Using service say')

say = rospy.ServiceProxy('say', Say)
resp = say(phrase='This is what I am saying.')
rospy.loginfo(resp.message)

raw_input('Using service wait_for_motor')

wait_for_motor = rospy.ServiceProxy('wait_for_motor', WaitForMotor)
actuator_ids = ['1', '2', '3', '4', '5', '6']
resp = wait_for_motor(actuator_ids)
rospy.loginfo(resp.message)

raw_input('Using service write_digital_outputs')

write_digital_outputs = rospy.ServiceProxy('write_digital_outputs', WriteDigitalOutputs)
resp = write_digital_outputs(mask=200)
rospy.loginfo(resp.message)

raw_input('Using service get_sensors')

get_sensors = rospy.ServiceProxy('get_sensors', GetSensors)
resp = get_sensors()
rospy.loginfo(resp.message)

raw_input('Using service send_sensor_instruction')

send_sensor_instruction = rospy.ServiceProxy('send_sensor_instruction', SendSensorInstruction)
resp = send_sensor_instruction('mySensor', '<command><reset>1</reset></command>')
rospy.loginfo(resp.message)


raw_input('Using service remove_sensor')

remove_sensor = rospy.ServiceProxy('remove_sensor', RemoveSensor)
resp = remove_sensor('mySensor')
rospy.loginfo(resp.message)

rospy.loginfo('All done, disconnecting!')
resp = connect(robot_kind='disconnect')
rospy.loginfo(resp.message)
