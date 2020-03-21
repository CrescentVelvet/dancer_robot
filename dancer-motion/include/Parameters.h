//
// Created by ZJUDancer on 2019-3-21
// E-mail: zjufanwu@zju.edu.cn
// This class can load parameters from the ros param server,
// create some global parameters to construct a state machine,
// manage all these parameters
//

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <vector>
#include <deque>
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <string.h>

using namespace std;
namespace dmotion
{

struct OneFootLandingParam //用来取代原OneFootLanding.h中宏定义的参数
{
    double UPBODY_CONSTANT_ROLL = 0; //用于GetOneStep中的参数设置
    double UPBODY_CONSTANT_PITCH = 0.28;
    double ANKLE_OFFSET_X = 0.5;
    double ANKLE_OFFSET_Y = 0;
    double ANKLE_OFFSET_Z = 10.5;
    double BODY_CENTER_X = 2.5;
    double BODY_CENTER_Y = 0;
    double BODY_CENTER_Z = -2.4;
    double UPBODY_MASS = 3158;
    double FOOT_MASS = 877;

    //下面是逆运动学的参数
    double UPPER_LEG_LENGTH = 12.3;
    double LOWER_LEG_LENGTH = 13.0;
    double ANKLE_FROM_GROUND = 6.0;
    double HALF_HIP_WIDTH = 4.5;
    double HIP_X_FROM_ORIGIN = 0;
    double HIP_Z_FROM_ORIGIN = 8.0;
};

struct PendulumWalkParam
{
    double ANKLE_DIS = 15.0;  //通过观察,肉包的踝间距差不多是11到12左右,单位为cm
    double TAO = 0.30;        //通过观察,肉包的步态单元的时长是0.35s
    double TICK_NUM = 30.0;     //每个步态周期发35个值
    double COM_H = 37.0;      /*通过把机器人固定成crouch姿势然后倒挂测量周期，测得其在全身不动的情况下特征摆长度为37cm*/
    double ACC_COEF_X = 0.15; //把本次质心前进dx和上一回dx进行做差对比，乘以系数的数字作为质心的位置移动，插值后在本次中叠加在倒立摆x轨迹上
    double ACC_COEF_Y = 0.3;
    double COM_HEIGHT = 27;        //默认规划重心高度
    double Y_HALF_AMPLITUDE = 3.0; //y方向倒立摆起点坐标长度
    double COM_X_OFFSET = 1;    //在理想重心规划基础上视走路情况而定的x方向偏移
    double TURNING_ERROR = 4.0; //旋转时每步真实角度和理论角度的差异

    double slow_down_minus_x = 3; //slow down时候一步减小的量           
    double slow_down_minus_y = 1.5;
    double slow_down_minus_yaw  = 8; 

	//表示抬脚高度曲线的三个向量
	vector<double> foot_z_t;
	vector<double> foot_z_p;
	vector<double> foot_z_s;
	
    double max_step_x=8;                    //单步最大x方向距离
    double max_step_y_out = 3.5;            //对于某一只脚来说的单步最大y方向外侧距离 ，区分内外的原因是防止踩脚
    double max_step_y_in = 3;               //对于某一只脚来说的单步最大y方向内侧距离
    double max_step_yaw = 25;               //turning 的单步最大角度22度，加上误差是26，这里选取25非常合理
    double max_step_differ_x = 2;           //单步最大x方向变化量
    double max_step_differ_y = 1;           //单步最大y方向变化量
    double max_step_differ_yaw = 10;        //单步最大yaw角度变化量


};

struct ThreeInterpolationParam
{
    double DEFAULT_POINT_INTERVAL = 10.0; //默认的发值时间间隔
    double DEFAULT_BOUNDARY_SLOPE = 1.0;
};

struct ClimbParam
{
    double WHOLE_TIME = 0.5;
    double NOT_LEG_ONLY_NUMBER = 17;
};

struct KickParam
{
    double RIGHT_KICK_X = -20.0;
    double RIGHT_KICK_Y = 7.0;

    double THIGH_START_TICK = 199.0;
    double THIGH_START_SLOPE = 0.0;
    double THIGH_END_TICK = 210.0;
    double THIGH_END_SLOPE = 0.0;
    double THIGH_END_ANGLE = 50.0;
    
    double KNEE_START_TICK = 207.0;
    double KNEE_START_SLOPE = 0.0;
    double KNEE_END_TICK = 215.0;
    double KNEE_END_SLOPE = 0.0;
    
    
    double RECOVER_START_TICK = 230.0;
    double RECOVER_END_TICK = 260.0;
    double ankle_recover_slope = 100.0;
    string RIGHT_KICK_FILES = "/home/ubuntu/dancer-workspace/workspaces/core/src/dmotion/config/";

    std::vector<double> COM_X;
    std::vector<double> COM_Y;
    std::vector<double> COM_Z;
    std::vector<double> ANKLE_X;
    std::vector<double> ANKLE_Y;
    std::vector<double> ANKLE_Z;
    std::vector<double> ANKLE_PITCH;
    // std::vector<double> ANKLE_PITCH_SP;
    std::vector<double> ANKLE_ROLL;
    std::vector<double> ANKLE_YAW;
};
// status_code标记此时是步态处于什么阶段
// 从左向右依次解释
// 第一位表示动作类型，有四种类型
//      1：爬起
//          第二位:
//          1：前爬
//              第三位：
//              1：到爬起示教起点+示教爬起
//              2：示教爬起结束点恢复至非正常脚间距的crouch
//              3：走两步恢复到常规crouch
//          2：后爬
//              1：到爬起示教起点+示教爬起
//              2：示教爬起结束点恢复至非正常脚间距的crouch
//              3：走两步恢复到常规crouch
//      2：踢球
//          第二位：
//          1：左踢
//              第三位：
//              0：无意义
//          2：右踢
//              0：无意义
//      3：走路及旋转
//          第二位：
//          1：为了踢球而走向球
//              第三位：
//              1：第一阶段，转向路径规划起点处的路径方向(带yaw矫正)        //TODO
//              2：第二阶段，走路到球后方的踢球点
//              3：第三阶段，转向进攻方向
//              4：调整至右踢友好位置                                   //TODO
//              5：决定左踢右踢后再进行微调（概率较低）
//          2：走向非踢球目标点
//              1：第一阶段，转向路径规划起点处的路径方向(带yaw矫正)
//              2：第二阶段，走到目标点
//              3：第三阶段，转向目标方向的角度
//          3: 原地旋转
//              0：无意义
//      4：原地crouch
//          第二位：
//          0：无意义
//              第三位：
//              0：无意义
//      5：手柄控制
enum StatusCode
{
    FORWARD_FALL_GETUP = 111,
    FORWARD_FALL_RECOVER = 112,
    FORWARD_FALL_STEP = 113,
    BACKWARD_FALL_GETUP = 121,
    BACKWARD_FALL_RECOVER = 122,
    BACKWARD_FALL_STEP = 123,
    LEFT_KICK = 210,
    RIGHT_KICK = 220,
    WALK_TO_BALL_TURN_TO_PATH = 311,
    WALK_TO_BALL_GO_TO_BALL = 312,
    WALK_TO_BALL_TURN_T0_AIM = 313,
    WALK_TO_BALL_ADJUST_TO_RIGHT_KICK = 314,
    WALK_TO_BALL_DETERMINE_FOOT_TO_KICK = 315,
    WALK_TO_AIM_TURN_TO_PATH = 321,
    WALK_TO_AIM_GO_TO_AIM = 322,
    WALK_TO_AIM_TURN_TO_AIM = 323,
    WALK_TO_AIM_ADJUST_TO_AIM = 324,
    TURN_AROUND = 330,
    BACKWARD = 340,
    CROUCH = 400,
    JOY = 500
};

struct gait_element //单个步态的结构体，有利于vector容器的正常使用
{
    double x = 0;           //该步的x方向位移
    double y = 0;           //该步的y方向位移
    double t = 0;           //该步的旋转量
    bool isRight = true;    //该步由哪只脚迈出，是否为右脚
    string label = "walk";  //用于输出是什么阶段的步态     
};

struct StatusParam
{
    vector<gait_element> gait_queue;
    gait_element last_gait;
    gait_element tmp_gait;  //方便随处使用暂存的临时单个步态对象，注意每次对其改变都要保证其每个成员的最新

    double adjust_max_x = 4;
    double adjust_max_y = 2.5;
    double adjust_max_yaw = 10;
    double adjust_max_step_num = 5;

    bool see_ball= false;
    vector<double> ball_field = {100,0,0};
    vector<double> ball_global = {0,0,0};
    vector<double> robot_global = {0,0,0};
    vector<double> action_global = {0,0,0};


    int action_confirmation_num = 15;  //连续这么多的ActionCommand表示相同的gait_type后才确信
    int action_confirmation_count = 0; //记录当前重复相同actionCommand次数
    int now_gait_type = 0;             //记录当前收到的步态类型

    int step_num = 0;
    double turn_tolerance = 20;             //起步角度大于这个角度时要先原地旋转


    double ball_field_distance;
    double ball_field_angle;

	//LADT means Little Adjust Distance Tolerance
	//LAAT means Little Adjust Angle Tolerance
    double LADT = 20;                       //aim_distance满足小于这个条件时可以直接进入速度控制器控制的小调整GenerateNewgait

    double LADT_for_ball = 20;				//这两个条件满足时即可进入非速度控制器控制的小调整LittleAdjust
    double LAAT_for_ball = 20;


    vector<double> cur_servo_angles = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    double cur_ankle_dis =15.0;


    
    double correct_k = 0.3;           // 巡线小车系数
    double correct_y_k = 7;           // nobrain_angle除以的系数，结果为nobrain走法的y方向偏移
    double stop_walk_dis = 30;
    double shit_rob_radius = 20;
    double start_mix_distance =200;
    double one_step_y = 3;
    double dx_delta = 0.5;
    double dyaw_delta = 5;
    double panball_x = 1;

    double UPARM_ANGLE = 0;
    double LOWARM_ANGLE = 0;
//新规则backward
    double backward_step_length = -3;
    bool attack_right=true;
    
    bool support_ok = false;

};

enum SupportPhase
{
    LEFT = 0,
    RIGHT = 1,
    DOUBLE = 2,
};

class Parameters
{
  public:
    OneFootLandingParam one_foot_landing_param;
    PendulumWalkParam pendulum_walk_param;
    ThreeInterpolationParam three_interpolation_param;
    KickParam kick_param;
    StatusCode status_code = CROUCH;
    StatusParam stp;
    ClimbParam climb_param;
    SupportPhase support_phase = DOUBLE;

    ros::NodeHandle *m_nh;

    void init(ros::NodeHandle *nh);

    void update();
};
extern Parameters parameters;
} // namespace dmotion

#endif
