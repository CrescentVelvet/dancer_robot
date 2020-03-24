#!/usr/bin/python2
# -*- coding: utf-8 -*
from std_msgs.msg import Float64MultiArray
import rospy

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " The motion_msg is %s", data.data)

if __name__=="__main__":
    rospy.loginfo("init")
    rospy.init_node('control_keyboard', anonymous = True)
    rospy.Subscriber('/ServoInfo', Float64MultiArray, callback)
    rospy.spin()
