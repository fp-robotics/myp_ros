#!/usr/bin/env python

'''
A node that uses the joystick to control the end effector:

L-LR: 		msg.axes[0]
L-UD: 		msg.axes[1]
R-LR: 		msg.axes[2]
R-UD: 		msg.axes[3]

triangle: 	buttons[12]
x:		buttons[14]
'''

import rospy
from rospy.exceptions import ROSInterruptException
from rospy import ServiceException
from sensor_msgs.msg import Joy
from myp_ros.srv import MoveTool, MoveToolRequest, MoveJoint, OpenGripper, CloseGripper, PlayPath
from copy import deepcopy


class JoystickController:

	reset_pos = MoveToolRequest(189.0, 0.0, 1189.5,  [0.0, 0.0, 0.0], [80]*6, [80]*6, False, False, '')
	move_tool = rospy.ServiceProxy('move_tool', MoveTool)
	open_gripper = rospy.ServiceProxy('open_gripper', OpenGripper)
	close_gripper = rospy.ServiceProxy('close_gripper', CloseGripper)
	move_joint = rospy.ServiceProxy('move_joint', MoveJoint)
	play_path = rospy.ServiceProxy('play_path', PlayPath)


	# init ros
	def __init__(self):

		resp = self.move_tool(self.reset_pos)
		rospy.loginfo(resp.message)

		self.startWasPressed = 0
		self.selectWasPressed = 0
		self.triangleWasPressed = 0
		self.xWasPressed = 0
		self.reset = False
		self.squareWasPressed = 0
		self.circleWasPressed = 0

		rospy.init_node('joystick_controller', anonymous=True)

		self.scale = 1.0

		self.move_joint(actuator_ids=['1', '2', '3', '4', '5', '6'], 
				   position=[0]*6, velocity = 80., acceleration = 80.)

		self.update_pos = 	reset_pos = MoveToolRequest(189.0, 0.0, 1189.5,  [0.0, 0.0, 0.0], [80]*6, [80]*6, False, False, '')
		self.last_good = deepcopy(self.update_pos)

		rospy.Subscriber('joy', Joy, self.callback)

		rospy.loginfo("Started joystick_controller!")
		rospy.loginfo("\nUse the analog joysticks to control the robot's end effector. \nUse the left joystick to control x-y and the right to control z. \nPress Triangle (PS3) to increase the joystick scale.\nPress Circle   (PS3) to decrease the joystick scale.")

	# callback for incoming raw joystick data
	def callback(self, msg):
		self.update_pos.x += self.scale*msg.axes[0]
		self.update_pos.y += -1*self.scale*msg.axes[1]
		self.update_pos.z += self.scale*msg.axes[3]

		if (self.selectWasPressed != msg.buttons[0] and self.selectWasPressed == 0):  # select button
			rospy.loginfo("Select pressed!")
			resp = self.play_path(path_name='path_name')
			rospy.loginfo(resp.message)			
			#self.update_pos = MoveToolRequest(406.0, -292.0, 1059.0, [2.0, 1.0, 30.0], [], [], False, False, '')

		if (self.startWasPressed != msg.buttons[3] and self.startWasPressed == 0):  # start button
			rospy.loginfo("Start pressed!")
			self.update_pos = deepcopy(self.reset_pos)

		if (self.triangleWasPressed != msg.buttons[12] and self.triangleWasPressed == 0):
			self.scale_up()

		if (self.xWasPressed != msg.buttons[14] and self.xWasPressed == 0):
			self.scale_down()

		if (self.squareWasPressed != msg.buttons[15] and self.squareWasPressed == 0):
			rospy.loginfo("Open Gripper!")
			resp = self.open_gripper()
			rospy.loginfo(resp.message)
		
		if (self.circleWasPressed != msg.buttons[13] and self.circleWasPressed == 0):
			rospy.loginfo("Open Gripper!")
			resp = self.close_gripper()
			rospy.loginfo(resp.message)

		self.selectWasPressed = msg.buttons[0]
		self.startWasPressed = msg.buttons[3]
		self.triangleWasPressed = msg.buttons[12]
		self.xWasPressed = msg.buttons[14]
		self.circleWasPressed = msg.buttons[13]
		self.squareWasPressed = msg.buttons[15]


	def scale_up(self):
		self.scale *= 1.1
		rospy.loginfo('New Scale: ' + str(self.scale))

	def scale_down(self):
		self.scale /= 1.1
		rospy.loginfo('New Scale: ' + str(self.scale))

	def reset_tool(self):
		resp = self.move_tool(self.reset_pos)

	def move_tool_service(self):
		pos = deepcopy(self.update_pos)
		resp = self.move_tool(pos)
		if resp.message == 'Success!':
			self.last_good = deepcopy(pos)
		else:
			self.update_pos = deepcopy(self.last_good)
			rospy.logwarn(resp.message)

	'''
	Define a function to return to neutral position to be called on shutdown.
	In shutdwon ROS services cannot provide the return value so we may use
	the ServiceException raised by this as the trigger for service completion.
	'''
	def move_back(self):
		rospy.loginfo("Moving back...")
		try:
			self.move_joint(actuator_ids=['1', '2', '3', '4', '5', '6'], 
						position=[0]*6, velocity = [80]*6, acceleration = [80]*6)
		except ServiceException as exception:
			rospy.loginfo("Done. Goodbye!")

# publish scaled and filtered data at a rate of 150Hz
if __name__ == '__main__':
	jc = JoystickController()

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():

		try:
		    jc.move_tool_service()
		    rate.sleep()

		except ROSInterruptException as error:
			rospy.loginfo("Movement interrupted by shutdown.")
			jc.move_back()
