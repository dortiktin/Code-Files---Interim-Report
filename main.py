#!/usr/bin/env python

import rospy
from math import sin,cos
from std_msgs.msg import Float64

def talker():
       pub1 = rospy.Publisher('/2arms/controller1/command', Float64, queue_size=10)
       pub2 = rospy.Publisher('/2arms/controller2/command', Float64, queue_size=10)
       rospy.init_node('2arms_talker', anonymous=True)
       rate = rospy.Rate(10)  # 10hz
       i = 0
       while not rospy.is_shutdown():
            theta1 = sin(i)   # radians
            theta2 = cos(i)   # radians
            rospy.loginfo("theta1:")
            rospy.loginfo(theta1)
            rospy.loginfo("theta2:")
            rospy.loginfo(theta2)
            pub1.publish(theta1)
            pub2.publish(theta2)
            rate.sleep()
            i += 0.01
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass