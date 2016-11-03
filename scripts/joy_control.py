#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from myp_ros.srv import MoveTool, MoveToolRequest
from copy import deepcopy

'''
L-LR: msg.axes[0]
L-UD: msg.axes[1]
R-LR: msg.axes[2]
R-UD: msg.axes[3]

triangle: buttons[12]
x:		  buttons[14]
'''

class JoystickController:

	reset_pos = MoveToolRequest(0.0, 0.0, 1137.0, [0, 0, -0], [], [], False, False, '')
	move_tool = rospy.ServiceProxy('move_tool', MoveTool)

	# init ros
	def __init__(self):

		resp = self.move_tool(self.reset_pos)
		rospy.loginfo(resp.message)

		self.startWasPressed = 0
		self.selectWasPressed = 0
		self.triangleWasPressed = 0
		self.xWasPressed = 0
		self.reset = False

		rospy.init_node('joystick_controller', anonymous=True)

		self.scale = 1.0
		
		self.update_pos = MoveToolRequest(0.0, 0.0, 1137.0, [0, 0, -0], [], [], False, False, '')
		self.last_good = self.update_pos

		rospy.Subscriber('joy', Joy, self.callback)

		rospy.loginfo("Started joystick_controller!")
		rospy.logingo("Use the analog joysticks to control the robot's end effector. \n \
						Use the left joystick to control x-y and the right to control z. \n \
						Press Triangle (PS3) to increase the joystick scale.\n \
						Press Circle   (PS3) to decrease the joystick scale.")

	# callback for incoming raw joystick data
	def callback(self, msg):
		self.update_pos.x += self.scale*msg.axes[0]
		self.update_pos.y += self.scale*msg.axes[1]
		self.update_pos.z += self.scale*msg.axes[3]

		if (self.selectWasPressed != msg.buttons[0] and self.selectWasPressed == 0):  # select button
			rospy.loginfo("Select pressed!")

		if (self.startWasPressed != msg.buttons[3] and self.startWasPressed == 0):  # start button
			rospy.loginfo("Start pressed!")
			self.update_pos = deepcopy(self.reset_pos)

		if (self.triangleWasPressed != msg.buttons[12] and self.triangleWasPressed == 0):
			self.scale_up()

		if (self.xWasPressed != msg.buttons[14] and self.xWasPressed == 0):
			self.scale_down()

		self.selectWasPressed = msg.buttons[0]
		self.startWasPressed = msg.buttons[3]
		self.triangleWasPressed = msg.buttons[12]
		self.xWasPressed = msg.buttons[14]


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

# publish scaled and filtered data at a rate of 150Hz
if __name__ == '__main__':
	jc = JoystickController()

	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		jc.move_tool_service()
		rate.sleep()
