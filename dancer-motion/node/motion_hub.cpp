//
// Created by ZJUDancer on 2019-3-17
// E-mial: zjufanwu@zju.edu.cn
//
// this is a ros node named motion_hub, which is the motion generator
#include "../include/ForwardKinematics.h"
#include "../include/Kick.h"
#include "../include/Parameters.h"
#include "../include/climb.h"
#include "ros/ros.h"
#include <dmsgs/BodyCommand.h>
#include <dmsgs/MotionInfo.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "dmsgs/VisionInfo.h"
#include "std_msgs/Int32.h"
#include "dmsgs/SetInitOrientation.h"
#include "dmsgs/MotionInfo.h"
#include "dmsgs/ActionCommand.h"
#include "sensor_msgs/Joy.h"
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

using namespace std;
using namespace dmotion;

PendulumWalk *pendulum_global = NULL;
KickAndTurn *kick_global = NULL;
Climb *climb_global = NULL;

long action_now_time; //用于判定一段时间没有收到ActionCommand后就直接默认进入 Crouch 400
bool walk_start = true;
template <typename T>
void DELETE(T *&x) //用于删除的模板函数
{
    if (x != NULL)
    {
        delete x;
        x = NULL;
    }
}
long gettimes() //获取当前时间，单位是纳秒ns
{
    struct timespec time1 = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time1);
    long nows = time1.tv_nsec + time1.tv_sec * 1000000000;
    return nows;
}
double ball_field_angle() //计算机器人到球的正切->弧度->角度，并限制范围
{
    return AdjustDegRange2(Rad2Deg(Atan(parameters.stp.ball_field[1], parameters.stp.ball_field[0])));
}
double ball_field_distance() //计算机器人到球的距离（field为局部坐标系即相对位置，global为全局坐标系）
{
    return sqrt(parameters.stp.ball_field[0] * parameters.stp.ball_field[0] + parameters.stp.ball_field[1] * parameters.stp.ball_field[1]);
}
void clear_status() //清空全部状态
{
    parameters.stp.gait_queue.clear();
    parameters.stp.tmp_gait.isRight = false;
    parameters.stp.tmp_gait.label = "nothing";
    parameters.stp.tmp_gait.x = 0;
    parameters.stp.tmp_gait.y = 0;
    parameters.stp.tmp_gait.t = 0;
    parameters.stp.last_gait = parameters.stp.tmp_gait;
    walk_start = true;

}
void VisionCallBack(const dmsgs::VisionInfo::ConstPtr &msg) //视觉信息回调函数，传递机器人全局位置和球的全局与相对位置信息
{ // 得到vision视觉信息

    parameters.stp.robot_global.clear(); //机器人自身的位置使用visionInfo中的机器人的位置
    parameters.stp.robot_global.push_back(msg->robot_pos.x);
    parameters.stp.robot_global.push_back(msg->robot_pos.y);
    parameters.stp.robot_global.push_back(msg->robot_pos.z);

    parameters.stp.see_ball = msg->see_ball;
    if (parameters.stp.see_ball)
    { //如果现在是要走向aim，且可以看到球的情况下，球的位置使用visionInfo中的球的位置
        parameters.stp.ball_global.clear();
        parameters.stp.ball_global.push_back(msg->ball_global.x);
        parameters.stp.ball_global.push_back(msg->ball_global.y);

        parameters.stp.ball_field.clear();
        parameters.stp.ball_field.push_back(msg->ball_field.x);
        parameters.stp.ball_field.push_back(msg->ball_field.y);
        // cout << "ball_field_distance  : " << ball_field_distance() << endl;
        parameters.stp.ball_field_distance = ball_field_distance();
        parameters.stp.ball_field_angle = ball_field_angle();
    }
}

void MotionCallBack(const dmsgs::MotionInfo::ConstPtr &msg) //运动信息回调函数，判断机器人是否前后跌倒与直立
{ // 得到跌倒信息与yaw角度信息
    if (msg->lower_board_connected)
    {
        if (!msg->stable)
        {
            if (parameters.status_code / 100 != 1)
            {
                if (msg->forward_or_backward) //机器人跌倒、分前倒和后倒
                {
                    parameters.status_code = (StatusCode)111;
                    // cout << "111111111111111111111111" << endl;
                }
                else
                {
                    parameters.status_code = (StatusCode)121;
                    // cout << "222222222222222222222222" << endl;
                }
            }
        }
        // parameters.stp.cur_yaw = msg->imuRPY.z; //记录当前机器人的yaw信息用于转身角度的矫正，经过测试感觉这部分的数值不如VisionInfo中的数值准确
        parameters.stp.support_ok = msg->support_stablizer_flag;
    }
    else
    {
        clear_status();
        parameters.status_code = (StatusCode)400; //下位机如果还没连接，直接 Crouch 400
    }
}
void TuningCallBack(const std_msgs::Int32::ConstPtr &msg)
{
    parameters.status_code = (StatusCode)msg->data;
}
void ActionCallBack(const dmsgs::ActionCommand::ConstPtr &msg) //对VisionInfo进行处理的回调函数，根据视觉信息选择0蹲着、1行走、2踢球、3转向、4爬起
{ // 得到command运动控制信息
    // cout << ("get actioncommand") << endl;
    action_now_time = gettimes();
    if (parameters.stp.now_gait_type == msg->bodyCmd.gait_type)
    {
        if (parameters.stp.action_confirmation_count < parameters.stp.action_confirmation_num)
        {
            parameters.stp.action_confirmation_count++;
        }
        else
        {
            switch (msg->bodyCmd.gait_type)
            {
            case 0:                                                                         //CROUCH
                if (parameters.status_code / 100 != 1 && parameters.status_code / 100 != 2) //如果不是爬起和踢球状态
                {
                    parameters.status_code = (StatusCode)400;
                    parameters.stp.action_confirmation_count = 0;
                    clear_status();
                }
                break;
            case 1:                                                                     //WALK_POS
                if (parameters.status_code / 10 == 31 || parameters.status_code == 400) //如果不是正在walk_to_aim而且没有在爬起、没有在踢球
                {
                    parameters.status_code = (StatusCode)321; //状态更改为walk_to_aim
                    parameters.stp.action_confirmation_count = 0;
                    clear_status();
                    SlowDown();
                }
                break;
            case 2:                                                                     //KICK_BALL
                if (parameters.status_code / 10 == 32 || parameters.status_code == 400) //如果不是正在walk_to_kickball而且没有在爬起、没有在踢球
                {
                    parameters.status_code = (StatusCode)311; //状态更改为walk_to_ball
                    ROS_FATAL("actioncommand turn to walk to ball");
                    parameters.stp.action_confirmation_count = 0;
                    clear_status();
                    SlowDown();
                }
                break;
            case 3:                                                                     //TURN
                if (parameters.status_code == 400 || parameters.status_code / 100 == 3) //如果不是正在walk而且没有在爬起、没有在踢球
                {
                    parameters.status_code = (StatusCode)330;
                    parameters.stp.action_confirmation_count = 0;
                    clear_status();
                }
                break;
            case 4:
                if (parameters.status_code == 400 || parameters.status_code / 100 == 3)
                {
                    parameters.status_code = (StatusCode)340;
                    parameters.stp.action_confirmation_count = 0;
                    clear_status();
                }
            }
        }
    }
    else
    {
        parameters.stp.now_gait_type = msg->bodyCmd.gait_type;
        parameters.stp.action_confirmation_count = 1;
    }
    parameters.stp.action_global.clear();
    parameters.stp.action_global.push_back(msg->bodyCmd.x);
    parameters.stp.action_global.push_back(msg->bodyCmd.y);
    parameters.stp.action_global.push_back(msg->bodyCmd.t);
    // cout << "msg->bodyCmd.t : " << msg->bodyCmd.t << endl;
    // cout << " parameters.stp.action_global [2] : " << parameters.stp.action_global[2] << endl;
}

void joyCallBack(const sensor_msgs::Joy::ConstPtr &msg)
{
    action_now_time = gettimes();
    if (msg->buttons[0] == 1)
    {
        ROS_FATAL("receive the joy to kick");
        parameters.status_code = (StatusCode)220;
        clear_status();
    }
    if (msg->buttons[1] == 1)
    {
        ROS_FATAL("receive the joy to crouch");
        parameters.status_code = (StatusCode)400;
        clear_status();
    }
    if (msg->axes[2] == 0 && msg->axes[3] != 0)
    {
        ROS_FATAL("receive the joy walking");
        if (parameters.status_code == 400 || parameters.status_code == 500)
            parameters.stp.ball_field.clear();
        parameters.stp.ball_field.push_back(msg->axes[3]);
        parameters.stp.ball_field.push_back(msg->axes[2]);
        parameters.stp.ball_field_angle = ball_field_angle();
        parameters.status_code = (StatusCode)500;
        clear_status();
    }
}

void teleCallBack(const std_msgs::Int32::ConstPtr &msg)
{
    action_now_time = gettimes();
    if (msg->data == 1)
    {
        ROS_FATAL("receive the joy to kick");
        parameters.status_code = (StatusCode)220;
        clear_status();
    }
    if (msg->data == 2)
    {
        ROS_FATAL("receive the joy to crouch");
        parameters.status_code = (StatusCode)400;
        clear_status();
    }
}

int main(int argc, char **argv) //主控制函数
{
    string str = getenv("ZJUDANCER_ROBOTID");
    ros::init(argc, argv, "motion_hub");
    /////////////////////////////////////////////////////////////////////////////////////////////////
    ros::NodeHandle n;
    ros::Publisher ServoInfo_pub = n.advertise<std_msgs::Float64MultiArray>("ServoInfo", 5);           //发布舵机角度信息
    ros::Subscriber MotionInfo_sub = n.subscribe("dmotion_" + str + "/MotionInfo", 3, MotionCallBack); //订阅MotionInfo得到跌倒信息和yaw角度信息
    ros::Subscriber VisionInfo_sub = n.subscribe("dvision_" + str + "/VisionInfo", 3, VisionCallBack); //订阅VisionInfo得到vision视觉信息
    ros::Subscriber ActionInfo_sub = n.subscribe("dbehavior_" + str + "/ActionCommand", 1, ActionCallBack); //订阅ActionCommand得到command运动控制信息
    ros::Subscriber TuningChannel_sub = n.subscribe("TuningCommand", 1, TuningCallBack); //订阅TuningCommand
    ros::Subscriber joy_sub = n.subscribe("joy", 1, joyCallBack); //订阅joy
    ros::Subscriber tele_sub = n.subscribe("teleop", 1, teleCallBack); //订阅teleop
    parameters.init(&n);
    ros::Rate loop_rate(1000.0 / parameters.three_interpolation_param.DEFAULT_POINT_INTERVAL);
    parameters.status_code = (StatusCode)400;

    pendulum_global = new PendulumWalk(&ServoInfo_pub, &loop_rate);

    //计算出第一组舵机角度值，防止第一次爬起或踢球的抽搐
    OneFootLanding base(false);
    vector<double> base_upbody = {0, 0, 0};
    // PrintVector(base_upbody);
    vector<double> base_com = {parameters.pendulum_walk_param.COM_X_OFFSET, -parameters.pendulum_walk_param.ANKLE_DIS / 2.0, parameters.pendulum_walk_param.COM_HEIGHT};
    // PrintVector(base_com);
    vector<double> base_ankle = {0, -parameters.pendulum_walk_param.ANKLE_DIS, 0, 0, 0, 0};
    // PrintVector(base_ankle);

    vector<double> base_angles = base.GetOneStep(base_ankle, base_com, base_upbody);
    parameters.stp.cur_servo_angles = base_angles;
    parameters.stp.cur_servo_angles.push_back(parameters.stp.UPARM_ANGLE);
    parameters.stp.cur_servo_angles.push_back(parameters.stp.LOWARM_ANGLE);
    parameters.stp.cur_servo_angles.push_back(parameters.stp.UPARM_ANGLE);
    parameters.stp.cur_servo_angles.push_back(parameters.stp.LOWARM_ANGLE);

    n.setParam("fuck_param",parameters.stp.cur_servo_angles); //设置该参数变量分享给dancer-io

    // PrintVector(base_angles);
    vector<gait_element> tmp_queue;
    double error_angle; //用于自身与目标方向的角度差值暂存

    //for walk to aim
    double aim_distance;
    double aim_angle;

    //for littleadjust
    double start_x, start_y, start_angle;
    double end_x, end_y, end_angle;

    //for dribbling while adjusting position near ball 盘球 (别称搅屎棍 shit mixer)

    double robot_panball_x = 0; //根据机器人所在位置和状态决定机器人是否盘球，该值为盘球每步向前走的距离，盘球为参数、不盘球为0
    double panball_x_max = 350;
    double panball_x_min = -350; //地图上机器人盘球的范围

    double shit_x = (1 - cos(parameters.stp.one_step_y / parameters.stp.shit_rob_radius)) * parameters.stp.shit_rob_radius; //理想绕球行走每一步应该产生的x方向增量
    double shit_y = -parameters.stp.one_step_y;                                                                             //理想绕球行走每一步应该产生的y方向增量的绝对值

    double shit_yaw = Rad2Deg(parameters.stp.one_step_y / parameters.stp.shit_rob_radius);
    double shit_y_send = shit_y;
    double shit_y_use = shit_y;
    double shit_yaw_use = shit_yaw;

    double shit_x_add = 0;
    double shit_yaw_add = 0;
    double shit_direction = 1;
    double shit_num = 0;
    double last_angle_e = 0;

    //for servo transition
    vector<vector<double>> transition;
    action_now_time = gettimes();
    long action_crouch_time = 3000000000; //三秒没收到ActionCommand会进入crouch，防止机器人乱动

    while (ros::ok())
    {
        ros::spinOnce();
        // ROS_FATAL("go to switch");
        switch (parameters.status_code)
        {
        case 111:
        case 112:
            ROS_FATAL("go into 111 or 112");
            DELETE(pendulum_global);
            //突然倒下时需要清空一些状态值
            clear_status();

            // 执行前爬起的前几步，使用正运动学算出踝间距并传递给parameters.stp.cur_ankle_dis
            climb_global = new Climb(&ServoInfo_pub, &loop_rate, "FORWARD", parameters.stp.cur_servo_angles);
            parameters.status_code = (StatusCode)113;
            delete climb_global;
            climb_global = NULL;
            // Delay(parameters.stp.stabilize_time);
            break;
        case 121:
        case 122:
            ROS_FATAL("go into 121 or 122");
            DELETE(pendulum_global);
            clear_status();

            // 执行后爬起的前几步，把status_code 转变到123，把踝间距传递给parameters.stp.cur_ankle_dis
            climb_global = new Climb(&ServoInfo_pub, &loop_rate, "BACK", parameters.stp.cur_servo_angles);
            parameters.status_code = (StatusCode)123;
            DELETE(climb_global);
            // Delay(parameters.stp.stabilize_time);
            break;
        case 113:
        case 123:
            ROS_FATAL("go into 113 or 123");

            pendulum_global = new PendulumWalk(&ServoInfo_pub, &loop_rate, parameters.stp.cur_ankle_dis);
            parameters.stp.tmp_gait.isRight = true;
            parameters.stp.tmp_gait.label = "recover_from_climb";
            parameters.stp.tmp_gait.x = 0;
            parameters.stp.tmp_gait.y = 0;
            parameters.stp.tmp_gait.t = 0;
            parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
            break;
        case 400: //crouch之前先平缓过度一些已有的状态
            // ROS_FATAL("go into 400");
            if (parameters.stp.gait_queue.size() == 0)
            {
                if (parameters.stp.last_gait.x != 0 || parameters.stp.last_gait.y != 0 || parameters.stp.last_gait.t != 0)
                {
                    parameters.stp.tmp_gait.isRight = !parameters.stp.last_gait.isRight;
                    parameters.stp.tmp_gait.label = "before 400";
                    parameters.stp.tmp_gait.x = 0;
                    parameters.stp.tmp_gait.y = 0;
                    parameters.stp.tmp_gait.t = 0;
                    parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                    parameters.stp.tmp_gait.isRight = !parameters.stp.last_gait.isRight;
                    parameters.stp.tmp_gait.label = "before 400 2";
                    parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                }
                // Delay(200000); //延时0.2秒，其他什么都不干默认每个动作都把自己的后摇处理好了
            }
            break;
        case 210:
            ROS_FATAL("go into 210");
            // TODO： 这里阻塞，左边踢球的整个动作直接执行就好了,执行完删掉就好了,
            parameters.status_code = (StatusCode)400;
            break;
        case 220:
            ROS_FATAL("go into 220");
            if (parameters.stp.gait_queue.size() == 0)
            {
                Delay(500000);
                //  这里阻塞，右边踢球的整个动作直接执行就好了,执行完删掉就好了

                kick_global = new KickAndTurn(&ServoInfo_pub, &loop_rate);
                Delay(500000);
                DELETE(kick_global);
                parameters.status_code = (StatusCode)400;
            }
            break;
        case 311: //WALK_TO_BALL_TURN_TO_PATH
        case 312:
            ROS_FATAL("go into 311");
            if (parameters.stp.gait_queue.size() == 0 && parameters.stp.see_ball)
            {
                if (parameters.stp.ball_field_distance < parameters.stp.LADT_for_ball &&
                    abs(AdjustDegRange2(parameters.stp.action_global[2] - parameters.stp.robot_global[2])) < parameters.stp.LAAT_for_ball && parameters.stp.ball_field_angle < parameters.stp.LAAT_for_ball)
                { //如果条件已经满足了可以进入小调整，就直接减速进入小调整0
                    SlowDown();
                    parameters.status_code = (StatusCode)314;
                }
                else
                {
                    if (parameters.stp.ball_field_distance > parameters.stp.stop_walk_dis) //距离球的距离大于stop_walk_dis，使用速度控制器对速度进行控制
                    {
                        parameters.stp.tmp_gait = GenerateNewGait(parameters.stp.ball_field_distance, parameters.stp.ball_field_angle, parameters.stp.last_gait);
                        parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                    }
                    else
                    {
                        if (abs(parameters.stp.ball_field_angle) < parameters.stp.turn_tolerance) //如果球在自己前方一定的小角度内，直接进入盘球模式
                        {
                            SlowDown();
                            parameters.status_code = (StatusCode)313; //去向球前调整
                        }
                        else //如果机器人不是比较好的面向球，先转向球再进入盘球
                        {
                            cout << "error_angle big, turn" << endl;
                            SlowDown();
                            error_angle = AdjustDegRange2(parameters.stp.ball_field_angle);
                            TurnAround(error_angle);
                        }
                    }
                }
            }
            break;
        case 313: //这就是可盘球球前调整模式
            ROS_FATAL("go into 313");
            if (parameters.stp.gait_queue.size() == 0)
            {
                if (parameters.stp.see_ball)
                {
                    if (walk_start)
                    {
                        double angle_error = AdjustDegRange2(parameters.stp.action_global[2] - parameters.stp.robot_global[2]);
                        last_angle_e = angle_error;
                        ROS_FATAL("parameters.stp.ball_global[2]");
                        cout << parameters.stp.ball_global[2] << endl;
                        ROS_FATAL("parameters.stp.robot_global[2]");
                        cout << parameters.stp.robot_global[2] << endl;
                        ROS_FATAL("angle_error");
                        cout << angle_error << endl;
                        if (abs(angle_error) > 10)
                        {
                            if (parameters.stp.attack_right) //按照机器人的进攻方向对开启盘球的球前调整的场地范围进行限定
                            {
                                panball_x_max = 450;
                                panball_x_min = -350;
                            }
                            else
                            {
                                panball_x_max = 350;
                                panball_x_min = -450;
                            }
                            //当机器人处在非自家球门附近且非边线附近的时候，才开启盘球，否则只进行绕球调整，不进行盘球
                            robot_panball_x = ((abs(parameters.stp.robot_global[1]) < 250 && parameters.stp.robot_global[0] > panball_x_min && parameters.stp.robot_global[0] < panball_x_max) ? (parameters.stp.panball_x) : (0));
                            // cout << "robot_panball_x : " << robot_panball_x << endl;
                            // cout << "parameters.stp.robot_global[1] : " << robot_panball_x << endl;
                            walk_start = false;
                            if (angle_error > 0)
                            {
                                shit_direction = 1;
                            }
                            else
                            {
                                shit_direction = -1;
                            }
                            shit_y_use = shit_direction * shit_y;
                            shit_yaw_use = shit_direction * shit_yaw;
                        }
                        else if (abs(angle_error) < parameters.stp.LAAT_for_ball && parameters.stp.ball_field_distance < parameters.stp.LADT_for_ball)
                        {
                            SlowDown();
                            parameters.status_code = (StatusCode)314;
                            walk_start = true;
                        }
                        shit_num = 1;
                    }
                    else
                    {
                        if (shit_num <= 4)
                        {
                            shit_y_send = shit_num / 4.0 * shit_y_use;
                            shit_num++;
                        }
                        shit_x_add = 0;
                        shit_yaw_add = 0;
                        if (parameters.stp.ball_field_distance > parameters.stp.shit_rob_radius)
                        {
                            shit_x_add += parameters.stp.dx_delta;
                        }
                        else
                        {
                            shit_x_add -= parameters.stp.dx_delta;
                        }
                        if (parameters.stp.ball_field_angle > 0)
                        {
                            shit_yaw_add += parameters.stp.dyaw_delta;
                        }
                        else
                        {
                            shit_yaw_add -= parameters.stp.dyaw_delta;
                        }

                        parameters.stp.tmp_gait.x = shit_x + robot_panball_x + shit_x_add;
                        parameters.stp.tmp_gait.y = shit_y_send;
                        parameters.stp.tmp_gait.t = shit_yaw_use + shit_yaw_add;
                        parameters.stp.tmp_gait.isRight = !parameters.stp.tmp_gait.isRight;
                        parameters.stp.tmp_gait.label = ((robot_panball_x == 0) ? ("no pan adjusting") : ("panball adjusting"));
                        parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                    }
                    if (parameters.stp.ball_field_distance > parameters.stp.stop_walk_dis)
                    {
                        parameters.status_code = (StatusCode)311;
                        walk_start = true;
                    }
                    double angle_e = AdjustDegRange2(parameters.stp.action_global[2] - parameters.stp.robot_global[2]);

                    cout << "angle_e : " << angle_e << endl;
                    //当盘着盘着发现error_angle 小于了5度或者error_angle的符号改变了的时候，即可进入小调整了
                    if ((abs(angle_e) < 5 || (sign(last_angle_e) != sign(angle_e) && abs(angle_e)<40) ) && parameters.stp.ball_field_distance < parameters.stp.stop_walk_dis)
                    {
                        SlowDown();
                        // Delay(1000000);
                        parameters.status_code = (StatusCode)314;
                        walk_start = true;
                    }
                    last_angle_e = angle_e;
                }
                else
                { // 没看到球就进入400等着
                    parameters.status_code = (StatusCode)400;
                }
            }
            break;
        case 314:
        case 315: //little_adjust before kick
            ROS_FATAL("go into 314");
            if (parameters.stp.gait_queue.size() == 0)
            {
                start_x = 0;     //换成使用ball_field//parameters.stp.robot_global[0];
                start_y = 0;     //换成使用ball_field//parameters.stp.robot_global[1];
                start_angle = 0; //dmotion::AdjustDegRange(parameters.stp.robot_global[2]);

                // end_x = parameters.stp.ball_field[0] + parameters.kick_param.RIGHT_KICK_X; //* cos(Deg2Rad(parameters.stp.ball_global[2])) - parameters.kick_param.RIGHT_KICK_Y * sin(Deg2Rad(parameters.stp.ball_global[2]));
                // end_y = parameters.stp.ball_field[1] + parameters.kick_param.RIGHT_KICK_Y; //* cos(Deg2Rad(parameters.stp.ball_global[2])) + parameters.kick_param.RIGHT_KICK_X * sin(Deg2Rad(parameters.stp.ball_global[2]));
                end_angle = AdjustDegRange2(parameters.stp.action_global[2] - parameters.stp.robot_global[2]);

                end_x = parameters.stp.ball_field[0] + parameters.kick_param.RIGHT_KICK_X * cos(Deg2Rad(end_angle)) - parameters.kick_param.RIGHT_KICK_Y * sin(Deg2Rad(end_angle));
                end_y = parameters.stp.ball_field[1] + parameters.kick_param.RIGHT_KICK_Y * cos(Deg2Rad(end_angle)) + parameters.kick_param.RIGHT_KICK_X * sin(Deg2Rad(end_angle));
                //dmotion::AdjustDegRange(parameters.stp.ball_global[2]);
                ROS_FATAL("Little Adjust");
                cout << "parameters.stp.action_global[2] : " << parameters.stp.action_global[2] << endl;
                cout << " parameters.stp.robot_global[2] : " << parameters.stp.robot_global[2] << endl;
                cout << "end_angle : " << end_angle << endl;
                cout << "end_x : " << end_x << endl;
                cout << "end_y : " << end_y << endl;
                LittleAdjust(start_x, start_y, start_angle, end_x, end_y, end_angle);
                parameters.stp.tmp_gait.isRight = !parameters.stp.last_gait.isRight;
                parameters.stp.tmp_gait.label = "before kick";
                parameters.stp.tmp_gait.x = 0;
                parameters.stp.tmp_gait.y = 0;
                parameters.stp.tmp_gait.t = 0;
                parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                parameters.stp.tmp_gait.isRight = !parameters.stp.last_gait.isRight;
                parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                parameters.status_code = (StatusCode)220;
            }
            break;
        case 321:
        case 322:
            ROS_FATAL("go into 322");
            if (parameters.stp.gait_queue.size() == 0)
            {
                aim_distance = sqrt(pow((parameters.stp.robot_global[0] - parameters.stp.action_global[0]), 2) + pow((parameters.stp.robot_global[1] - parameters.stp.action_global[1]), 2));
                cout << "aim_distance : " << aim_distance << endl;
                error_angle = AdjustDegRange2(parameters.stp.action_global[2] - parameters.stp.robot_global[2]);
                cout << "error_angle : " << error_angle << endl;
                aim_angle = AdjustDegRange2(Rad2Deg(Atan((parameters.stp.action_global[1] - parameters.stp.robot_global[1]), (parameters.stp.action_global[0] - parameters.stp.robot_global[0]))) - parameters.stp.robot_global[2]);
                cout << "aim_angle : " << aim_angle << endl;

                if (aim_distance < parameters.stp.LADT)
                {
                    parameters.stp.tmp_gait = GenerateNewGait(aim_distance, aim_angle, parameters.stp.last_gait, error_angle);
                    parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                }
                else
                {
                    parameters.stp.tmp_gait = GenerateNewGait(aim_distance, aim_angle, parameters.stp.last_gait);
                    parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                }
            }
            break;
        case 323:
        case 324:
            ROS_FATAL("go into 324");
            break;
        case 330:
            ROS_FATAL("go into 330");
            if (parameters.stp.gait_queue.size() == 0)
            {
                error_angle = AdjustDegRange2(parameters.stp.action_global[2] - parameters.stp.robot_global[2]);
                TurnAround(error_angle);
                parameters.status_code = (StatusCode)400;
            }
            break;
        case 340:
            ROS_FATAL("go into 340");
            if (parameters.stp.gait_queue.size() == 0)
            {
                SlowDown();
                for (int i = 0; i < 3; i++)
                {
                    parameters.stp.tmp_gait.x = parameters.stp.backward_step_length;
                    parameters.stp.tmp_gait.y = 0;
                    parameters.stp.tmp_gait.t = 0;
                    parameters.stp.tmp_gait.isRight = !parameters.stp.tmp_gait.isRight;
                    parameters.stp.tmp_gait.label = "step backward";
                    parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
                }
            }
            break;
        case 500:
            ROS_FATAL("The robot is in control of the joy");
            if (parameters.stp.gait_queue.size() == 0)
            {
                parameters.stp.tmp_gait = GenerateNewGait(0, parameters.stp.ball_field_angle, parameters.stp.last_gait);
                parameters.stp.tmp_gait.label = "joy";
                action_now_time = gettimes();
                parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
            }
            break;
        }
        // ROS_FATAL("out of switch");

        if (parameters.stp.gait_queue.size() != 0)
        {
            ROS_FATAL("give a step");
            pendulum_global->GiveAStepTick(parameters.stp.gait_queue[0]);
            parameters.stp.last_gait = parameters.stp.gait_queue[0];
            tmp_queue = parameters.stp.gait_queue;
            parameters.stp.gait_queue.clear();
            for (unsigned i = 1; i < tmp_queue.size(); i++)
            {
                parameters.stp.gait_queue.push_back(tmp_queue[i]);
            }

            ROS_FATAL("hello");
            if (parameters.stp.gait_queue.size() == 0 && gettimes() - action_now_time > action_crouch_time && parameters.status_code != 113 && parameters.status_code != 123 && parameters.status_code != 314 && parameters.status_code != 324 && parameters.status_code != 220 && parameters.status_code != 500)
            {
                parameters.status_code = (StatusCode)400;
            }
            if (parameters.stp.last_gait.label == "recover_from_climb")
            {
                Delay(1000000);
                parameters.status_code = (StatusCode)400;
            }
            if (parameters.stp.last_gait.label == "before 400 2")
            {
                Delay(1500000);//停下就停下，防止还没停好就直接开始走新的一步
            }
            ROS_FATAL("step gived done");
        }
        else
        {
            // ROS_FATAL("NO MORE gait");
            //     cout << gettimes() << endl;
            // cout << "action_now_time" << action_now_time << endl;
            // cout << "error " << action_now_time - gettimes() << endl;

            if (gettimes() - action_now_time > action_crouch_time && parameters.status_code != 113 && parameters.status_code != 123  && parameters.status_code != 314 && parameters.status_code != 324 && parameters.status_code != 220 && parameters.status_code != 500)
            {
                // ROS_FATAL("long time no action_command");
                parameters.status_code = (StatusCode)400;
            }
        }
    };

    return 0;
}
