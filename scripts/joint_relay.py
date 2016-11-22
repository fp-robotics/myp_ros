#!/usr/bin/env python
import rospy
from myp_ros.msg import JointAngles

class Relay:
	def __init__(self):
		self.pub = rospy.Publisher('gazebo_joint_angles', JointAngles, queue_size=10)
		rospy.init_node('talker', anonymous=True)
		self.rate = rospy.Rate(100) # 10hz
		self.message = JointAngles()
		self.sub = rospy.Subscriber('joint_angles', JointAngles, self.callback)
		
	def callback(self, msg):
		self.message = msg
		
	def relay(self):
		while not rospy.is_shutdown():
				rospy.loginfo(self.message)
				self.pub.publish(self.message)
				self.rate.sleep()

if __name__ == '__main__':
	rel = Relay()
	try:
		rel.relay()
	except rospy.ROSInterruptException:
		pass
