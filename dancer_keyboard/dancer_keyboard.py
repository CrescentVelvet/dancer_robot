#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import Float64

import roslib; roslib.load_manifest('dancer_keyboard')
import rospy
import sys, select, termios, tty

msg = """

controlling column:
#1
        'q'         # body_head
        'w'         # body_head2
        'a'         # arm_left
        's'         # hand_left
        'z'         # arm_right
        'x'         # hand_right
#2
        'e'         # body_hip_left
        'r'         # body_hip2_left
        'd'         # leg_left
        'f'         # leg2_left
        'c'         # leg3_left
        'v'         # leg4_left
#3
        't'         # body_hip_right
        'y'         # body_hip2_right
        'g'         # leg_right
        'h'         # leg2_right
        'b'         # leg3_right
        'n'         # leg4_right

CTRL-C to quit
"""

controlBindings={
#1
        'q':( 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), # body_head
#        'w':(-1, 0, 0, 0, 0, 0), # 
        'w':( 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), # body_head2
#        's':( 0,-1, 0, 0, 0, 0), # 
        'a':( 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), # arm_left
#        'x':( 0, 0,-1, 0, 0, 0), # 
        's':( 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), # hand_left
#        'r':( 0, 0, 0,-1, 0, 0), # 
        'z':( 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), # arm_right
#        'f':( 0, 0, 0, 0,-1, 0), # 
        'x':( 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), # hand_right
#        'v':( 0, 0, 0, 0, 0,-1), # 

#2
        'e':( 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), # body_hip_left
#        'w':(-1, 0, 0, 0, 0, 0), # 
        'r':( 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), # body_hip2_left
#        's':( 0,-1, 0, 0, 0, 0), # 
        'd':( 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0), # leg_left
#        'x':( 0, 0,-1, 0, 0, 0), # 
        'f':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0), # leg2_left
#        'r':( 0, 0, 0,-1, 0, 0), # 
        'c':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0), # leg3_left
#        'f':( 0, 0, 0, 0,-1, 0), # 
        'v':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0), # leg4_left
#        'v':( 0, 0, 0, 0, 0,-1), # 

#3
        't':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0), # body_hip_right
#        'w':(-1, 0, 0, 0, 0, 0), # 
        'y':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0), # body_hip2_right
#        's':( 0,-1, 0, 0, 0, 0), # 
        'g':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0), # leg_right
#        'x':( 0, 0,-1, 0, 0, 0), # 
        'h':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0), # leg2_right
#        'r':( 0, 0, 0,-1, 0, 0), # 
        'b':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0), # leg3_right
#        'f':( 0, 0, 0, 0,-1, 0), # 
        'n':( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1), # leg4_right
#        'v':( 0, 0, 0, 0, 0,-1), # 
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def robot_1(body_head, body_head2, arm_left, hand_left, arm_right, hand_right):
    return "body_head: %s\t body_head2: %s\t arm_left: %s\t hand_left: %s\t arm_right %s\t hand_right %s" % (body_head, body_head2, arm_left, hand_left, arm_right, hand_right)

def robot_2(body_hip_left, body_hip2_left, leg_left, leg2_left, leg3_left, leg4_left):
    return "body_hip_left: %s\t body_hip2_left: %s\t leg_left: %s\t leg2_left: %s\t leg3_left %s\t leg4_left %s" % (body_hip_left, body_hip2_left, leg_left, leg2_left, leg3_left, leg4_left)

def robot_3(body_hip_right, body_hip2_right, leg_right, leg2_right, leg3_right, leg4_right):
    return "body_hip_right: %s\t body_hip2_right: %s\t leg_right: %s\t leg2_right: %s\t leg3_right %s\t leg4_right %s" % (body_hip_right, body_hip2_right, leg_right, leg2_right, leg3_right, leg4_right)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
#1
    pub_body_head   = rospy.Publisher('/dancer_urdf_model/joint_body_head_controller/command',   Float64, queue_size = 1)
    pub_body_head2  = rospy.Publisher('/dancer_urdf_model/joint_body_head2_controller/command',  Float64, queue_size = 1)
    pub_arm_left    = rospy.Publisher('/dancer_urdf_model/joint_arm_left_controller/command',    Float64, queue_size = 1)
    pub_hand_left   = rospy.Publisher('/dancer_urdf_model/joint_hand_left_controller/command',   Float64, queue_size = 1)
    pub_arm_right   = rospy.Publisher('/dancer_urdf_model/joint_arm_right_controller/command',   Float64, queue_size = 1)
    pub_hand_right  = rospy.Publisher('/dancer_urdf_model/joint_hand_right_controller/command',  Float64, queue_size = 1)
#2
    pub_body_hip_left   = rospy.Publisher('/dancer_urdf_model/joint_body_hip_left_controller/command',   Float64, queue_size = 1)
    pub_body_hip2_left  = rospy.Publisher('/dancer_urdf_model/joint_body_hip2_left_controller/command',  Float64, queue_size = 1)
    pub_leg_left        = rospy.Publisher('/dancer_urdf_model/joint_leg_left_controller/command',        Float64, queue_size = 1)
    pub_leg2_left       = rospy.Publisher('/dancer_urdf_model/joint_leg2_left_controller/command',       Float64, queue_size = 1)
    pub_leg3_left       = rospy.Publisher('/dancer_urdf_model/joint_leg3_left_controller/command',       Float64, queue_size = 1)
    pub_leg4_left       = rospy.Publisher('/dancer_urdf_model/joint_leg4_left_controller/command',       Float64, queue_size = 1)
#3
    pub_body_hip_right   = rospy.Publisher('/dancer_urdf_model/joint_body_hip_right_controller/command',   Float64, queue_size = 1)
    pub_body_hip2_right  = rospy.Publisher('/dancer_urdf_model/joint_body_hip2_right_controller/command',  Float64, queue_size = 1)
    pub_leg_right        = rospy.Publisher('/dancer_urdf_model/joint_leg_right_controller/command',        Float64, queue_size = 1)
    pub_leg2_right       = rospy.Publisher('/dancer_urdf_model/joint_leg2_right_controller/command',       Float64, queue_size = 1)
    pub_leg3_right       = rospy.Publisher('/dancer_urdf_model/joint_leg3_right_controller/command',       Float64, queue_size = 1)
    pub_leg4_right       = rospy.Publisher('/dancer_urdf_model/joint_leg4_right_controller/command',       Float64, queue_size = 1)

    rospy.init_node('dancer_keyboard')
#1
    body_head  = 0
    body_head2 = 1.5
    arm_left   = 1.68
    hand_left  = 3.14
    arm_right  = 0.10
    hand_right = -2.62
#2
    body_hip_left  = 0
    body_hip2_left = -0.11
    leg_left       = -1.74
    leg2_left      = 1.00
    leg3_left      = -0.17
    leg4_left      = 0.41
#3
    body_hip_right  = 0.11
    body_hip2_right = -0.66
    leg_right       = 0.47
    leg2_right      = 0.88
    leg3_right      = -0.94
    leg4_right      = -1.10

    try:
        print(msg)
        print(robot_1(body_head, body_head2, arm_left, hand_left, arm_right, hand_right))
        print(robot_2(body_hip_left, body_hip2_left, leg_left, leg2_left, leg3_left, leg4_left))
        print(robot_3(body_hip_right, body_hip2_right, leg_right, leg2_right, leg3_right, leg4_right))
        print("----------")
        while(1):
            key = getKey()
            if key in controlBindings.keys():
#1
                body_head  = body_head  + 0.1 * controlBindings[key][0]
                body_head2 = body_head2 + 0.1 * controlBindings[key][1]
                arm_left   = arm_left   + 0.1 * controlBindings[key][2]
                hand_left  = hand_left  + 0.1 * controlBindings[key][3]
                arm_right  = arm_right  + 0.1 * controlBindings[key][4]
                hand_right = hand_right + 0.1 * controlBindings[key][5]
#2
                body_hip_left  = body_hip_left  + 0.1 * controlBindings[key][6]
                body_hip2_left = body_hip2_left + 0.1 * controlBindings[key][7]
                leg_left       = leg_left       + 0.1 * controlBindings[key][8]
                leg2_left      = leg2_left      + 0.1 * controlBindings[key][9]
                leg3_left      = leg3_left      + 0.1 * controlBindings[key][10]
                leg4_left      = leg4_left      + 0.1 * controlBindings[key][11]
#3
                body_hip_right  = body_hip_right  + 0.1 * controlBindings[key][12]
                body_hip2_right = body_hip2_right + 0.1 * controlBindings[key][13]
                leg_right       = leg_right       + 0.1 * controlBindings[key][14]
                leg2_right      = leg2_right      + 0.1 * controlBindings[key][15]
                leg3_right      = leg3_right      + 0.1 * controlBindings[key][16]
                leg4_right      = leg4_right      + 0.1 * controlBindings[key][17]
#1
                if body_head<0.0001 and body_head>-0.0001:
                    body_head = 0.0
                if body_head2<0.0001 and body_head2>-0.0001:
                    body_head2 = 0.0
                if arm_left<0.0001 and arm_left>-0.0001:
                    arm_left = 0.0
                if hand_left<0.0001 and hand_left>-0.0001:
                    hand_left = 0.0
                if arm_right<0.0001 and arm_right>-0.0001:
                    arm_right = 0.0
                if hand_right<0.0001 and hand_right>-0.0001:
                    hand_right = 0.0
#2
                if body_hip_left<0.0001 and body_hip_left>-0.0001:
                    body_hip_left = 0.0
                if body_hip2_left<0.0001 and body_hip2_left>-0.0001:
                    body_hip2_left = 0.0
                if leg_left<0.0001 and leg_left>-0.0001:
                    leg_left = 0.0
                if leg2_left<0.0001 and leg2_left>-0.0001:
                    leg2_left = 0.0
                if leg3_left<0.0001 and leg3_left>-0.0001:
                    leg3_left = 0.0
                if leg4_left<0.0001 and leg4_left>-0.0001:
                    leg4_left = 0.0
#3
                if body_hip_right<0.0001 and body_hip_right>-0.0001:
                    body_hip_right = 0.0
                if body_hip2_right<0.0001 and body_hip2_right>-0.0001:
                    body_hip2_right = 0.0
                if leg_right<0.0001 and leg_right>-0.0001:
                    leg_right = 0.0
                if leg2_right<0.0001 and leg2_right>-0.0001:
                    leg2_right = 0.0
                if leg3_right<0.0001 and leg3_right>-0.0001:
                    leg3_right = 0.0
                if leg4_right<0.0001 and leg4_right>-0.0001:
                    leg4_right = 0.0

                print(robot_1(body_head, body_head2, arm_left, hand_left, arm_right, hand_right))
                print(robot_2(body_hip_left, body_hip2_left, leg_left, leg2_left, leg3_left, leg4_left))
                print(robot_3(body_hip_right, body_hip2_right, leg_right, leg2_right, leg3_right, leg4_right))
                print("----------")
            else:
                if (key == '\x03'):
                    break

            float64 = Float64()
#1
            float64.data = body_head
            pub_body_head.publish(float64)
            float64.data = body_head2
            pub_body_head2.publish(float64)
            float64.data = arm_left
            pub_arm_left.publish(float64)
            float64.data = hand_left
            pub_hand_left.publish(float64)
            float64.data = arm_right
            pub_arm_right.publish(float64)
            float64.data = hand_right
            pub_hand_right.publish(float64)
#2
            float64.data = body_hip_left
            pub_body_hip_left.publish(float64)
            float64.data = body_hip2_left
            pub_body_hip2_left.publish(float64)
            float64.data = leg_left
            pub_leg_left.publish(float64)
            float64.data = leg2_left
            pub_leg2_left.publish(float64)
            float64.data = leg3_left
            pub_leg3_left.publish(float64)
            float64.data = leg4_left
            pub_leg4_left.publish(float64)
#3
            float64.data = body_hip_right
            pub_body_hip_right.publish(float64)
            float64.data = body_hip2_right
            pub_body_hip2_right.publish(float64)
            float64.data = leg_right
            pub_leg_right.publish(float64)
            float64.data = leg2_right
            pub_leg2_right.publish(float64)
            float64.data = leg3_right
            pub_leg3_right.publish(float64)
            float64.data = leg4_right
            pub_leg4_right.publish(float64)

    except Exception as e:
        print(e)

    finally:
        float64 = Float64()
#1
        float64.data = 0
        pub_body_head.publish(float64)
        float64.data = 0
        pub_body_head2.publish(float64)
        float64.data = 0
        pub_arm_left.publish(float64)
        float64.data = 0
        pub_hand_left.publish(float64)
        float64.data = 0
        pub_arm_right.publish(float64)
        float64.data = 0
        pub_hand_right.publish(float64)
#2
        float64.data = 0
        pub_body_hip_left.publish(float64)
        float64.data = 0
        pub_body_hip2_left.publish(float64)
        float64.data = 0
        pub_leg_left.publish(float64)
        float64.data = 0
        pub_leg2_left.publish(float64)
        float64.data = 0
        pub_leg3_left.publish(float64)
        float64.data = 0
        pub_leg4_left.publish(float64)
#3
        float64.data = 0
        pub_body_hip_right.publish(float64)
        float64.data = 0
        pub_body_hip2_right.publish(float64)
        float64.data = 0
        pub_leg_right.publish(float64)
        float64.data = 0
        pub_leg2_right.publish(float64)
        float64.data = 0
        pub_leg3_right.publish(float64)
        float64.data = 0
        pub_leg4_right.publish(float64)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
