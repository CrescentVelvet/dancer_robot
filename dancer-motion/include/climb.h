

#ifndef CLIMB_H
#define CLIMB_H
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include "Utility/dmotion_math.hpp"
#include "OneFootLanding.h"
#include "ThreeInterpolation.h"
#include "ForwardKinematics.h"
#include "PendulumWalk.h"
#include "Parameters.h"
#include <ros/ros.h>

namespace dmotion
{
    class Climb
    {
        public:
            //用于发给IO舵机值
            ros::Publisher * pbr;
            //用于控制发值周期
            ros::Rate * lopr;
            //构造函数
            Climb(ros::Publisher * publisher, ros::Rate * loop_rate, std::string label, std::vector<double> position_start);
            
            void Prepare();
        private:
            string label_;
            // dmotion::OneFootLanding support;
            // dmotion::ForKin left_foot;
            // dmotion::ForKin right_foot;
            // dmotion::ForKinPlus body;
            //跌倒时的舵机值d
            std::vector<double> position_now={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            //示教爬起开始时的舵机值
            std::vector<double> std_back_pose;
            std::vector<double> std_forward_pose;
            //插值用的时间向量和舵机位置向量
            // ThreeInterpolation offset;
            std::vector<double> tit = {0, parameters.climb_param.WHOLE_TIME};
            std::vector<double> value={0,0};
            //给舵机发的位置值
            std::vector<double> servo_points= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            //插值过程中的变量
            std::vector<double> servo_split;
            std::vector<double> servo_times;
            std::vector<std::vector<double> > servo_position;
            //示教爬起过程中的变量
            std::vector<std::vector<double> > AllPosition_time;
            std::vector<std::vector<double> > AllPosition;
            //示教爬起结束时的舵机值
            std::vector<double> position_aftclimb;
            //调整质心时用到的一些变量
            std::vector<double> angle_leftleg;
            std::vector<double> angle_rightleg;
            std::vector<double> lfoot2center;
            std::vector<double> rfoot2center;
            std::vector<double> center2left;
            std::vector<double> right2left;
            // static const double body_center_x = 0.5;
            // static const double body_center_y = 0;
            // static const double body_center_z = -2;
            // static const double OneFootLanding::ankle_offset_x = 0;
            // static const double OneFootLanding::ankle_offset_y = 0;
            // static const double OneFootLanding::ankle_offset_z = 9.5;
            // double upbody_roll;
            // double upbody_pitch;
            // double upbody_yaw;
            // std::vector<double> v(3, 0);
            // std::vector<double> upbody_com(3, 0);
            // std::vector<double> lfoot_com;
            // std::vector<double> rfoot_com;
            // std::vector<double> whole_body_com;
            // std::vector<double> hang_foot1 = {0, -15, 0, 0, 0, 0};
            // std::vector<double> comx, xomy, comz, poser, posep, posey;

            std::vector<double> whole_end_pos;//调整质心后的身体舵机值
            std::vector<std::vector<double>> FinalAdjustServoMatrix; //从爬起动作结束到调整至可以开始步态的过程的舵机值矩阵
    };
}

#endif