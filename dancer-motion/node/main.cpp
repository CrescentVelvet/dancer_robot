//
// Created by ZJUDancer on 2019-3-17
// E-mial: zjufanwu@zju.edu.cn
//
// this is a ros node named motion_hub, which is the motion generator
#include "../include/ForwardKinematics.h"
#include "../include/Parameters.h"
#include "../include/climb.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

using namespace std;
using namespace dmotion;

Climb *climb_global = NULL;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_hub");
    ros::NodeHandle n;
    ros::Publisher ServoInfo_pub = n.advertise<std_msgs::Float64MultiArray>("ServoInfo", 5);   //发布舵机角度信息
    parameters.init(&n);
    ros::Rate loop_rate(1000.0 / parameters.three_interpolation_param.DEFAULT_POINT_INTERVAL);

    std::vector<double> forward_position = {0, 0, -9.97559, 35.463900, 95.888700, 0, 0, 0, -9.97559, 35.463900, 95.888700, 0, -49.394500, 143.65700,  -49.394500, 143.65700};
    // climb_global = new Climb(&ServoInfo_pub, &loop_rate, "FORWARD", forward_position);
    // climb_global = new Climb(&ServoInfo_pub, &loop_rate, "BACK",    back_position);

    cout << "准备爬起" << endl;
    Delay(1000000); // 等待1秒钟
    cout << "成功爬起,进行下一步" <<endl;

    cout << "直线行走例程" << endl;
    PendulumWalk pen(&ServoInfo_pub, &loop_rate);
    pen.GiveAStep(1,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(3,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(5,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(8,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(10,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(8,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(5,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(3,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(1,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();

    cout << "原地旋转" << endl;
    pen.GiveAStep(0,0,11.2);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(10+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(15+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(20+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(25+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(30+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(25+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(20+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(15+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(10+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,(10+1.5));
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
        
    cout << "原地踏步" << endl;
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();

    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();

    cout << "横着走" << endl;
    pen.GiveAStep(0,-1,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-2,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-3,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-4,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-4,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-4,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-4,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-4,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-4,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-4,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-4,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-2,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,-1,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();
    pen.GiveAStep(0,0,0);
    for(int i = 0 ;i < 33 ; i++)
        pen.GiveATick();

    cout << "用于两脚平行过渡状态变换到基准crouch的一步" << endl;
    PendulumWalk pen2(&ServoInfo_pub, &loop_rate);
    pen2.GiveAStep(0,0,0);
    for(int i = 0; i< 33 ; i++)
        pen2.GiveATick();

    return 0;
    }

