#!/usr/bin/env python
import rospy
from myp_ros.msg import JointAngles

def talker():
    pub = rospy.Publisher('joint_angles', JointAngles, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    msg = JointAngles()
    while not rospy.is_shutdown():
        msg.data = [0,0,0,0,0,0]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
