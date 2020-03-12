#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from std_msgs.msg import Float64

import roslib; roslib.load_manifest('dancer_keyboard')
import rospy
import sys, select, termios, tty
import time
import math

# 16个关节的原版初始值
raw_init_joint_rad = (0.05, -0.83, 0.55, 0.93, -0.90, -1.00, -0.06, -0.14, -1.68, 1.05, -0.25, 0.41, 0.10, 0.39, 1.72, 0.14)
# 16个关节的小仿初始值
real_init_joint_rad = (0.0, 0.0, -0.17410689032902116, 0.6189618205980171, 1.6735735304570867, 0.0, 0.0, 0.0, -0.17410689032902116, 0.6189618205980171, 1.6735735304570867, 0.0, -0.8620966573763391, 2.5072876435374942, -0.8620966573763391, 2.5072876435374942)
real_init_joint_rad = list(real_init_joint_rad)
# 用原版初始值减去小仿初始值
for index in range(len(raw_init_joint_rad)):
    real_init_joint_rad[index] = raw_init_joint_rad[index] - real_init_joint_rad[index]

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

def robot_msg(body_hip_right,body_hip2_right,leg_right,leg2_right,leg3_right,leg4_right,body_hip_left,body_hip2_left,leg_left,leg2_left,leg3_left,leg4_left,arm_right,hand_right,arm_left,hand_left):
    return " body_hip_right: %s\t body_hip2_right: %s\n leg_right: %s\t leg2_right: %s\n leg3_right: %s\t leg4_right: %s\n body_hip_left: %s\t body_hip2_left: %s\n leg_left: %s\t leg2_left: %s\n leg3_left: %s\t leg4_left: %s\n arm_right: %s\t hand_right: %s\n arm_left: %s\t hand_left %s" % (body_hip_right,body_hip2_right,leg_right,leg2_right,leg3_right,leg4_right,body_hip_left,body_hip2_left,leg_left,leg2_left,leg3_left,leg4_left,arm_right,hand_right,arm_left,hand_left)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
#3
    pub_body_hip_right   = rospy.Publisher('/dancer_urdf_model/joint_body_hip_right_controller/command',   Float64, queue_size = 1)
    pub_body_hip2_right  = rospy.Publisher('/dancer_urdf_model/joint_body_hip2_right_controller/command',  Float64, queue_size = 1)
    pub_leg_right        = rospy.Publisher('/dancer_urdf_model/joint_leg_right_controller/command',        Float64, queue_size = 1)
    pub_leg2_right       = rospy.Publisher('/dancer_urdf_model/joint_leg2_right_controller/command',       Float64, queue_size = 1)
    pub_leg3_right       = rospy.Publisher('/dancer_urdf_model/joint_leg3_right_controller/command',       Float64, queue_size = 1)
    pub_leg4_right       = rospy.Publisher('/dancer_urdf_model/joint_leg4_right_controller/command',       Float64, queue_size = 1)
#2
    pub_body_hip_left   = rospy.Publisher('/dancer_urdf_model/joint_body_hip_left_controller/command',   Float64, queue_size = 1)
    pub_body_hip2_left  = rospy.Publisher('/dancer_urdf_model/joint_body_hip2_left_controller/command',  Float64, queue_size = 1)
    pub_leg_left        = rospy.Publisher('/dancer_urdf_model/joint_leg_left_controller/command',        Float64, queue_size = 1)
    pub_leg2_left       = rospy.Publisher('/dancer_urdf_model/joint_leg2_left_controller/command',       Float64, queue_size = 1)
    pub_leg3_left       = rospy.Publisher('/dancer_urdf_model/joint_leg3_left_controller/command',       Float64, queue_size = 1)
    pub_leg4_left       = rospy.Publisher('/dancer_urdf_model/joint_leg4_left_controller/command',       Float64, queue_size = 1)
#1
    pub_arm_left    = rospy.Publisher('/dancer_urdf_model/joint_arm_left_controller/command',    Float64, queue_size = 1)
    pub_hand_left   = rospy.Publisher('/dancer_urdf_model/joint_hand_left_controller/command',   Float64, queue_size = 1)
    pub_arm_right   = rospy.Publisher('/dancer_urdf_model/joint_arm_right_controller/command',   Float64, queue_size = 1)
    pub_hand_right  = rospy.Publisher('/dancer_urdf_model/joint_hand_right_controller/command',  Float64, queue_size = 1)
#4
    pub_body_head   = rospy.Publisher('/dancer_urdf_model/joint_body_head_controller/command',   Float64, queue_size = 1)
    pub_body_head2  = rospy.Publisher('/dancer_urdf_model/joint_body_head2_controller/command',  Float64, queue_size = 1)
    pub_body_hip    = rospy.Publisher('/dancer_urdf_model/joint_body_hip_controller/command',    Float64, queue_size = 1)

    rospy.init_node('dancer_keyboard')

#3
#    body_hip_right  =  0.05
#    body_hip2_right = -0.83
#    leg_right       =  0.55
#    leg2_right      =  0.93
#    leg3_right      = -0.90
#    leg4_right      = -1.00
#2
#    body_hip_left  = -0.06
#    body_hip2_left = -0.14
#    leg_left       = -1.68
#    leg2_left      =  1.05
#    leg3_left      = -0.25
#    leg4_left      =  0.41
#1
#    arm_left   =  1.72
#    hand_left  =  0.14
#    arm_right  =  0.10
#    hand_right =  0.39
#4
    body_head  = -0.05
    body_head2 =  0.00
    body_hip   =  0.00
    # 初始化为竖直状态（排除了头部与腰部三个关节，剩余16个关节）
    (body_hip_right,body_hip2_right,leg_right,leg2_right,leg3_right,leg4_right,body_hip_left,body_hip2_left,leg_left,leg2_left,leg3_left,leg4_left,arm_left,hand_left,arm_right,hand_right) = raw_init_joint_rad
    try:
        print(msg)
        print("----------")
        print(robot_msg(body_hip_right,body_hip2_right,leg_right,leg2_right,leg3_right,leg4_right,body_hip_left,body_hip2_left,leg_left,leg2_left,leg3_left,leg4_left,arm_right,hand_right,arm_left,hand_left))
        print("----------")
        # 执行机器人从躺平爬起来的动作(注意关节顺序有改动)
        time.sleep(10)
        print("----------")
        print("Init start  : %s" % time.ctime())

        # 读取数据
        f = open("/home/ruby/catkin_ws/src/dancer_robot/dancer_keyboard/forward_climb.txt","r")
        init_flag = 1 # 让竖直躺平sleep久一点
        line_string = f.readline()
        while line_string:

            # 读取
            line_str = line_string.split(' ')
            while '' in line_str:
                line_str.remove('')
            line_str.pop(-1) # 删除时间戳
            line_data = list(map(float,line_str))
            # 角度制转换弧度制
            for index in range(len(line_data)):
                line_data[index] = math.radians(line_data[index])

            #print(raw_init_joint_rad)
            #print(real_init_joint_rad)
            #print(f)#拿到原始数据
            #print(line_string)#提取一行数据
            #print(line_str)#转换为str数组
            #print(line_data)#转换为float数组

            # 赋值
            body_hip_right = real_init_joint_rad[0] + line_data[0]
            body_hip2_right = real_init_joint_rad[1] + line_data[1]
            leg_right = real_init_joint_rad[2] + line_data[2]
            leg2_right = real_init_joint_rad[3] + line_data[3]
            leg3_right = real_init_joint_rad[4] + line_data[4]
            leg4_right = real_init_joint_rad[5] + line_data[5]
            body_hip_left = real_init_joint_rad[6] + line_data[6]
            body_hip2_left = real_init_joint_rad[7] + line_data[7]
            leg_left = real_init_joint_rad[8] + line_data[8]
            leg2_left = real_init_joint_rad[9] + line_data[9]
            leg3_left = real_init_joint_rad[10] + line_data[10]
            leg4_left = real_init_joint_rad[11] + line_data[11]
            arm_right = real_init_joint_rad[12] + line_data[12]
            hand_right = real_init_joint_rad[13] + line_data[13]
            arm_left = real_init_joint_rad[14] + line_data[14]
            hand_left = real_init_joint_rad[15] + line_data[15]

            # 发送
            control_msg = Float64()
            #3
            control_msg.data = body_hip_right
            pub_body_hip_right.publish(control_msg)
            control_msg.data = body_hip2_right
            pub_body_hip2_right.publish(control_msg)
            control_msg.data = leg_right
            pub_leg_right.publish(control_msg)
            control_msg.data = leg2_right
            pub_leg2_right.publish(control_msg)
            control_msg.data = leg3_right
            pub_leg3_right.publish(control_msg)
            control_msg.data = leg4_right
            pub_leg4_right.publish(control_msg)
            #2
            control_msg.data = body_hip_left
            pub_body_hip_left.publish(control_msg)
            control_msg.data = body_hip2_left
            pub_body_hip2_left.publish(control_msg)
            control_msg.data = leg_left
            pub_leg_left.publish(control_msg)
            control_msg.data = leg2_left
            pub_leg2_left.publish(control_msg)
            control_msg.data = leg3_left
            pub_leg3_left.publish(control_msg)
            control_msg.data = leg4_left
            pub_leg4_left.publish(control_msg)
            #1
            control_msg.data = arm_left
            pub_arm_left.publish(control_msg)
            control_msg.data = hand_left
            pub_hand_left.publish(control_msg)
            control_msg.data = arm_right
            pub_arm_right.publish(control_msg)
            control_msg.data = hand_right
            pub_hand_right.publish(control_msg)
            #4
            control_msg.data = body_head
            pub_body_head.publish(control_msg)
            control_msg.data = body_head2
            pub_body_head2.publish(control_msg)
            control_msg.data = body_hip
            pub_body_hip.publish(control_msg)
  
            time.sleep(0.01) # 间隔10ms
            if init_flag:
                time.sleep(3)
                init_flag = 0
            line_string = f.readline()

        f.close()

        time.sleep(3)
        print(robot_msg(body_hip_right,body_hip2_right,leg_right,leg2_right,leg3_right,leg4_right,body_hip_left,body_hip2_left,leg_left,leg2_left,leg3_left,leg4_left,arm_right,hand_right,arm_left,hand_left))
        print("Init finish : %s" % time.ctime())
        print("----------")




        # 反复循环键盘控制机器人关节运动
        while(1):
            key = getKey()
            if key in controlBindings.keys():

                # 读取键盘输入
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
#4
                body_hip = body_hip

                #回到0时保留两位小数
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

                print(robot_msg(body_hip_right,body_hip2_right,leg_right,leg2_right,leg3_right,leg4_right,body_hip_left,body_hip2_left,leg_left,leg2_left,leg3_left,leg4_left,arm_right,hand_right,arm_left,hand_left))
                print("----------")
            else:
                if (key == '\x03'):
                    break

            control_msg = Float64()
#1
            control_msg.data = body_head
            pub_body_head.publish(control_msg)
            control_msg.data = body_head2
            pub_body_head2.publish(control_msg)
            control_msg.data = arm_left
            pub_arm_left.publish(control_msg)
            control_msg.data = hand_left
            pub_hand_left.publish(control_msg)
            control_msg.data = arm_right
            pub_arm_right.publish(control_msg)
            control_msg.data = hand_right
            pub_hand_right.publish(control_msg)
#2
            control_msg.data = body_hip_left
            pub_body_hip_left.publish(control_msg)
            control_msg.data = body_hip2_left
            pub_body_hip2_left.publish(control_msg)
            control_msg.data = leg_left
            pub_leg_left.publish(control_msg)
            control_msg.data = leg2_left
            pub_leg2_left.publish(control_msg)
            control_msg.data = leg3_left
            pub_leg3_left.publish(control_msg)
            control_msg.data = leg4_left
            pub_leg4_left.publish(control_msg)
#3
            control_msg.data = body_hip_right
            pub_body_hip_right.publish(control_msg)
            control_msg.data = body_hip2_right
            pub_body_hip2_right.publish(control_msg)
            control_msg.data = leg_right
            pub_leg_right.publish(control_msg)
            control_msg.data = leg2_right
            pub_leg2_right.publish(control_msg)
            control_msg.data = leg3_right
            pub_leg3_right.publish(control_msg)
            control_msg.data = leg4_right
            pub_leg4_right.publish(control_msg)
#4
            control_msg.data = body_hip
            pub_body_hip.publish(control_msg)

    # 异常处理 <名称> <数据>
    except Exception as e:
        print(e)

    finally:
        control_msg = Float64()
#1
        control_msg.data = 0
        pub_body_head.publish(control_msg)
        control_msg.data = 0
        pub_body_head2.publish(control_msg)
        control_msg.data = 0
        pub_arm_left.publish(control_msg)
        control_msg.data = 0
        pub_hand_left.publish(control_msg)
        control_msg.data = 0
        pub_arm_right.publish(control_msg)
        control_msg.data = 0
        pub_hand_right.publish(control_msg)
#2
        control_msg.data = 0
        pub_body_hip_left.publish(control_msg)
        control_msg.data = 0
        pub_body_hip2_left.publish(control_msg)
        control_msg.data = 0
        pub_leg_left.publish(control_msg)
        control_msg.data = 0
        pub_leg2_left.publish(control_msg)
        control_msg.data = 0
        pub_leg3_left.publish(control_msg)
        control_msg.data = 0
        pub_leg4_left.publish(control_msg)
#3
        control_msg.data = 0
        pub_body_hip_right.publish(control_msg)
        control_msg.data = 0
        pub_body_hip2_right.publish(control_msg)
        control_msg.data = 0
        pub_leg_right.publish(control_msg)
        control_msg.data = 0
        pub_leg2_right.publish(control_msg)
        control_msg.data = 0
        pub_leg3_right.publish(control_msg)
        control_msg.data = 0
        pub_leg4_right.publish(control_msg)
#4
        control_msg.data = 0
        pub_body_hip.publish(control_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#"right_hip_yaw",     body_hip_right
#"right_hip_roll",    body_hip2_right
#"right_hip_pitch",   leg_right
#"right_knee",        leg2_right
#"right_ankle_pitch", leg3_right
#"right_ankle_roll",  leg4_right

#"left_hip_yaw",      body_hip_left
#"left_hip_roll",     body_hip2_left
#"left_hip_pitch",    leg_left
#"left_knee",         leg2_left
#"left_ankle_pitch",  leg3_left
#"left_ankle_roll",   leg4_left

#"right_arm_upper",   arm_right
#"right_arm_lower",   hand_right
#"left_arm_upper",    arm_left
#"left_arm_lower"     hand_left