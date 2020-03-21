//
// Created by zjudancer on 18-12-18.
//

#include "PendulumWalk.h"

using namespace dmotion;
namespace dmotion
{

PendulumWalk::PendulumWalk(ros::Publisher *publisher, ros::Rate *loop_rate) : pbr(publisher), lopr(loop_rate)
{

    com_ac_x = 0;
    com_ac_y = 0;
    support_is_right = false;
    hang_foot = {0, parameters.pendulum_walk_param.ANKLE_DIS, 0};
    com_pos = {0, parameters.pendulum_walk_param.ANKLE_DIS / 2.0, 0};
    com_x_changed = 0;
    com_y_changed = 0;
    Support = new OneFootLanding(false);
}

PendulumWalk::PendulumWalk(ros::Publisher *publisher, ros::Rate *loop_rate, const double &unusual_ankle_dis) : pbr(publisher), lopr(loop_rate)
{

    com_ac_x = 0;
    com_ac_y = 0;
    support_is_right = false;
    hang_foot = {0, unusual_ankle_dis, 0};
    com_pos = {0, unusual_ankle_dis / 2.0, 0};
    com_x_changed = 0;
    com_y_changed = 0;
    Support = new OneFootLanding(false);
}
PendulumWalk::~PendulumWalk()
{
    delete Support;
    Support = NULL;
}

/**
     * 生成下一步的动作数据
     * @param dx 下一步质心的x变化，相对于上半身，单位是cm
     * @param dy 下一步执行的y变化，相对于上半身，单位是cm
     * @param d_yaw 下一步执行的yaw变化，相对于上半身，角度制
     */
void PendulumWalk::GiveAStep(double dx, double dy, double d_yaw)
{
    cout << "x : " << dx << " y : " << dy << " d_yaw : " << d_yaw << " " << endl;
    cout << endl;
    delete Support;
    Support = NULL;
    Support = new OneFootLanding(support_is_right);

    x0 = com_pos[0] * cos(hang_foot[2]) - hang_foot[0] * cos(hang_foot[2]) + com_pos[1] * sin(hang_foot[2]) -
         hang_foot[1] * sin(hang_foot[2]);

    xt = (dx - ((support_is_right) ? (parameters.pendulum_walk_param.ANKLE_DIS / 2.0) : (-parameters.pendulum_walk_param.ANKLE_DIS / 2.0)) * sin(Deg2Rad(d_yaw))) / 2.0;

    //        std::cout << "x0 :" << x0 << std::endl;
    //        std::cout << "xt :" << xt << std::endl;

    tao = parameters.pendulum_walk_param.TAO;

    //算出摆的周期常数，这里的com_h暂时是由机器人crouch姿态下倒挂着摆动测量得出的
    com_h = parameters.pendulum_walk_param.COM_H;
    Tc = std::sqrt(com_h / 980);
    //        std::cout << "Tc  :" << Tc << " " << std::endl;

    //算出来这个步态单元的初速度vx

    vx = (xt - x0 * cosh(tao / Tc)) / (Tc * sinh(tao / Tc));
    //        std::cout << "vx :" << vx << std::endl;

    //y方向的研究

    y00 = com_pos[1] * cos(hang_foot[2]) - hang_foot[1] * cos(hang_foot[2]) - com_pos[0] * sin(hang_foot[2]) +
          hang_foot[0] * sin(hang_foot[2]);
    ytt = (dy + ((support_is_right) ? (parameters.pendulum_walk_param.ANKLE_DIS / 2.0) : (-parameters.pendulum_walk_param.ANKLE_DIS / 2.0)) +
           ((support_is_right) ? (parameters.pendulum_walk_param.ANKLE_DIS / 2.0) : (-parameters.pendulum_walk_param.ANKLE_DIS / 2.0)) * cos(Deg2Rad(d_yaw))) /
          2;

    //        std::cout << "y00 :" << y00 << std::endl;
    //        std::cout << "ytt :" << ytt << std::endl;

    //这里实际计算质心轨迹的是y0和yt，防止∆y太大。
    y0 = support_is_right ? (parameters.pendulum_walk_param.Y_HALF_AMPLITUDE) : (-parameters.pendulum_walk_param.Y_HALF_AMPLITUDE);
    yt = support_is_right ? (parameters.pendulum_walk_param.Y_HALF_AMPLITUDE) : (-parameters.pendulum_walk_param.Y_HALF_AMPLITUDE);

    //计算过程中换元得到的一个临时变量m
    m = exp(tao / Tc);
    //        std::cout << "m  :" << m << " " << std::endl;

    //步行单元的周期定了后y方向的最大速度是确定的
    vy = (yt - m * y0) / ((m + 1) * Tc);
    //        std::cout << "vy  :" << vy << " " << std::endl;

    //使用插值算法定义抬脚的时间位移曲线
    std::vector<double> akZ_t = parameters.pendulum_walk_param.foot_z_t;
    std::vector<double> akZ_p = parameters.pendulum_walk_param.foot_z_p;
    std::vector<double> akZ_s = parameters.pendulum_walk_param.foot_z_s;
    ThreeInterpolation ankle_z(akZ_t, akZ_p, akZ_s);
    // cout << "使用插值算法定义抬脚的时间位移曲线 "<< endl;
    // ankle_z.CalculatePoints(10);
    akZ = ankle_z.GetPoints();

    //TODO consider if we are going to plan the ankle pitch

    //线性规划上半身yaw
    std::vector<double> comYaw_t = {0, parameters.pendulum_walk_param.TAO};
    std::vector<double> comYaw_p = {Rad2Deg(com_pos[2] - hang_foot[2]), d_yaw / 2.0};
    double slope_comYaw = (d_yaw / 2.0 - Rad2Deg(com_pos[2] - hang_foot[2])) / parameters.pendulum_walk_param.TAO;
    std::vector<double> comYaw_s = {slope_comYaw, slope_comYaw};
    ThreeInterpolation com_yaw(comYaw_t, comYaw_p, comYaw_s);
    // cout << "线性规划上半身yaw "<< endl;
    // com_yaw.CalculatePoints(10);
    comYaw = com_yaw.GetPoints();
    std::cout << "comYaw :" << std::endl;
    //        PrintVector(comYaw);

    //线性规划质心的y基础位置
    std::vector<double> comY_t = {0, parameters.pendulum_walk_param.TAO};
    std::vector<double> comY_p = {y00, ytt};
    double slope_comY = (ytt - y00) / parameters.pendulum_walk_param.TAO;
    std::vector<double> comY_s = {slope_comY, slope_comY};
    ThreeInterpolation com_y(comY_t, comY_p, comY_s);
    // cout << "线性规划质心的y基础位置 "<< endl;
    // com_y.CalculatePoints(10);
    comY = com_y.GetPoints();
    //        std::cout << "comY :" << std::endl;
    //        PrintVector(comY);

    //线性的质心加速度x偏移
    double ac_x = (dx - com_x_changed) * parameters.pendulum_walk_param.ACC_COEF_X;
    std::vector<double> accX_t = {0, parameters.pendulum_walk_param.TAO};
    std::vector<double> accX_p = {com_ac_x, ac_x};
    double slope_accX = (ac_x - com_ac_x) / parameters.pendulum_walk_param.TAO;
    std::vector<double> accX_s = {slope_accX, slope_accX};
    ThreeInterpolation acc_x(accX_t, accX_p, accX_s);
    // cout << "线性的质心加速度x偏移 "<< endl;
    accX = acc_x.GetPoints();
    //        std::cout << "accX :" << std::endl;
    //        PrintVector(accX);

    //线性的质心加速度y偏移
    double ac_y = (dy - com_y_changed) * parameters.pendulum_walk_param.ACC_COEF_Y;
    std::vector<double> accY_t = {0, parameters.pendulum_walk_param.TAO};
    std::vector<double> accY_p = {com_ac_y, ac_y};
    double slope_accY = (ac_y - com_ac_y) / parameters.pendulum_walk_param.TAO;
    std::vector<double> accY_s = {slope_accY, slope_accY};
    ThreeInterpolation acc_y(accY_t, accY_p, accY_s);
    // cout << "线性的质心加速度y偏移 "<< endl;
    //        cout << "com_ac_y :" <<com_ac_y << endl;
    //        cout << "ac_y :" <<ac_y << endl;
    accY = acc_y.GetPoints();
    //        std::cout << "accY :" << std::endl;
    //        PrintVector(accY);

    //脚踝的x方向起点终点的插值
    double ak_x_0 = -hang_foot[0] * cos(hang_foot[2]) - hang_foot[1] * sin(hang_foot[2]);
    double ak_x_t = 2 * xt;
    std::vector<double> akX_t = {0, parameters.pendulum_walk_param.TAO};
    std::vector<double> akX_p = {ak_x_0, ak_x_t};
    //        cout << "x0 :" << x0 << endl;
    //        cout << "xt :" << xt << endl;
    //        cout << "ak_x_0 :" << ak_x_0 << endl;
    //        cout << "ak_x_t :" << ak_x_t << endl;
    double slope_akX = (ak_x_t - ak_x_0) / parameters.pendulum_walk_param.TAO;
    std::vector<double> akX_s = {slope_akX, slope_akX};
    ThreeInterpolation ak_x(akX_t, akX_p, akX_s);
    // cout << "脚踝的x方向起点终点的插值 "<< endl;
    akX = ak_x.GetPoints();
    //        std::cout << "akX :" << std::endl;
    //        PrintVector(akX);

    //脚踝的y方向起点终点的插值
    double ak_y_0 = hang_foot[0] * sin(hang_foot[2]) - hang_foot[1] * cos(hang_foot[2]);
    double ak_y_t = 2 * ytt;
    std::vector<double> akY_t = {0, parameters.pendulum_walk_param.TAO};
    std::vector<double> akY_p = {ak_y_0, ak_y_t};
    double slope_akY = (ak_y_t - ak_y_0) / parameters.pendulum_walk_param.TAO;
    std::vector<double> akY_s = {slope_akY, slope_akY};
    ThreeInterpolation ak_y(akY_t, akY_p, akY_s);
    // cout << "脚踝的y方向起点终点的插值 "<< endl;
    akY = ak_y.GetPoints();
    //        std::cout << "akY :" << std::endl;
    //        PrintVector(akY);

    //脚踝的yaw起点终点的插值
    std::vector<double> akYaw_t = {0, parameters.pendulum_walk_param.TAO};
    std::vector<double> akYaw_p = {Rad2Deg(-hang_foot[2]), d_yaw};
    double slope_akYaw = (akYaw_p[1] - akYaw_p[0]) / parameters.pendulum_walk_param.TAO / 2;
    std::vector<double> akYaw_s = {slope_akYaw, slope_akYaw};
    ThreeInterpolation ak_yaw(akYaw_t, akYaw_p, akYaw_s);
    // cout << "脚踝的yaw起点终点的插值 "<< endl;
    akYaw = ak_yaw.GetPoints();

    //为下一步的数据做准备
    support_is_right = !support_is_right;
    com_x_changed = dx;
    com_y_changed = dy;

    com_pos.clear();
    com_pos.push_back(xt);
    com_pos.push_back(ytt);
    com_pos.push_back(Deg2Rad(d_yaw) / 2.0);
    //        cout << "com_pos :" << endl;
    //        PrintVector(com_pos);
    hang_foot.clear();
    hang_foot.push_back(ak_x_t);
    hang_foot.push_back(ak_y_t);
    hang_foot.push_back(2.0 * com_pos[2]); //这里使得落脚点的yaw和质心一致了
                                           //        cout << "hang_foot :" << endl;
                                           //        PrintVector(hang_foot);

    com_ac_x = ac_x;
    com_ac_y = ac_y;

    tick_num = 0;
}

void PendulumWalk::GiveATick()
{

    motion_tick tmptick;
    tmptick.time_stamp = 10000000 * (tick_num); //10毫秒
    x = x0 * std::cosh(0.01 * (tick_num) / Tc) + Tc * vx * std::sinh(0.01 * (tick_num) / Tc);
    y = y0 * std::cosh(0.01 * (tick_num) / Tc) + Tc * vy * std::sinh(0.01 * (tick_num) / Tc);
    tmptick.upbody_pose.emplace_back(0);
    tmptick.upbody_pose.emplace_back(0);
    tmptick.upbody_pose.emplace_back(comYaw[tick_num]);
    tmptick.whole_com.emplace_back(accX[tick_num] + x + parameters.pendulum_walk_param.COM_X_OFFSET); //(x * 100 +1.5);
    tmptick.whole_com.emplace_back(y - y0 + comY[tick_num] + accY[tick_num]);
    tmptick.whole_com.emplace_back(parameters.pendulum_walk_param.COM_HEIGHT); //0.308637

    tmptick.hang_foot.emplace_back(akX[tick_num]); //(200 * x);
    tmptick.hang_foot.emplace_back(akY[tick_num]);
    tmptick.hang_foot.emplace_back(akZ[tick_num]);
    tmptick.hang_foot.emplace_back(0);
    tmptick.hang_foot.emplace_back(0);
    tmptick.hang_foot.emplace_back(akYaw[tick_num]);

    //        if(tick_num == 0)
    //        {
    //            cout << "comY[tick_num] :" << comY[tick_num] <<endl;
    //            dmotion::PrintVector(tmptick.whole_com);
    //            dmotion::PrintVector(tmptick.hang_foot);
    //        }
    tick_num++;

    parameters.support_phase = (SupportPhase)((int)support_is_right);

    dmotion::ServoPublish(Support->GetOneStep(tmptick.hang_foot, tmptick.whole_com, tmptick.upbody_pose), "WALK", pbr, lopr);
}

void PendulumWalk::GiveAStepTick(gait_element now_gait)
{
    cout << "gait_label: " << now_gait.label << "\t"
         << "gait_foot_is_right: " << now_gait.isRight << endl;
    cout << "real_foot_is_right: " << support_is_right << endl;

    GiveAStep(now_gait.x, now_gait.y, now_gait.t);
    for (int j = 0; j < parameters.pendulum_walk_param.TICK_NUM; j++)
    {
        if(true )
        {
            ros::spinOnce();
            while(parameters.stp.support_ok)
            {
                vector<double> tmp_cur_servo_12;
                for(int i =0;i<12 ;i++)
                {
                    tmp_cur_servo_12.push_back(parameters.stp.cur_servo_angles[i]);
                }
                dmotion::ServoPublish(tmp_cur_servo_12,"WALK" ,pbr , lopr);
                ros::spinOnce(); 
            };
        }
        GiveATick();
    }
}

/** 
 * 用于任意坐标系下两步相对于机器人GiveAStep参数的生成，这里使用了比较输入为世界坐标系下的<x,y,yaw>起点与终点
 * 输出为相对于此时机器人的位置下一步参数,可以用于GiveAStep
 * 
 * */
vector<double> OneStepTransform(double global_start_x, double global_start_y, double global_start_yaw, double global_end_x, double global_end_y, double global_end_yaw)
{
    double start_x = global_start_x;
    double start_y = global_start_y;
    double end_x = global_end_x; //限制x范围
    double end_y = global_end_y; //限制y范围
    double start_angle = AdjustDegRange2(global_start_yaw);
    double end_angle = (abs(AdjustDegRange2(global_end_yaw - start_angle)) > 25) ? ((sign(AdjustDegRange2(global_end_yaw - start_angle))) * 25 + start_angle) : AdjustDegRange2(global_end_yaw);
    cout << "start_x: " << start_x << "start_y" << start_y << "start_angle" << start_angle << endl;
    cout << "end_x: " << end_x << "edn_y" << end_y << "end_angle" << end_angle << endl;
    double result_angle = AdjustDegRange2(end_angle - start_angle);

    start_angle = Deg2Rad(start_angle);
    end_angle = Deg2Rad(end_angle);
    double result_x = end_x * cos(start_angle) - start_x * cos(start_angle) + end_y * sin(start_angle) -
                      start_y * sin(start_angle);
    double result_y = end_y * cos(start_angle) - start_y * cos(start_angle) - end_x * sin(start_angle) +
                      start_x * sin(start_angle);
    result_x = (abs(result_x) > 8) ? (sign(result_x) * 8) : result_x;
    result_y = (abs(result_y) > 4) ? (sign(result_y) * 4) : result_y;
    vector<double> result_array = {result_x, result_y, result_angle};
    return result_array;
}

void LittleAdjust(double start_x, double start_y, double start_angle, double end_x, double end_y, double end_angle)
{
    vector<double> angle_sequence;
    vector<double> x_sequence;
    vector<double> y_sequence;
    vector<double> aim_field;
    int adjust_step_num;

    double error_angle = AdjustDegRange2(end_angle - start_angle);
    cout << "little adjust error_angle : " << error_angle << endl;
    start_angle = Deg2Rad(start_angle);
    end_angle = Deg2Rad(end_angle);
    aim_field.push_back(end_x * cos(start_angle) - start_x * cos(start_angle) + end_y * sin(start_angle) -
                        start_y * sin(start_angle));
    aim_field.push_back(end_y * cos(start_angle) - start_y * cos(start_angle) - end_x * sin(start_angle) +
                        start_x * sin(start_angle));
    aim_field.push_back(error_angle);
    adjust_step_num = (int)(abs(aim_field[0]) / parameters.stp.adjust_max_x + 1);
    adjust_step_num = ((int)(abs(aim_field[1]) / parameters.stp.adjust_max_y + 1) > adjust_step_num) ? ((int)(abs(aim_field[1]) / parameters.stp.adjust_max_y + 1)) : (adjust_step_num);
    adjust_step_num = ((int)(abs(aim_field[2]) / parameters.stp.adjust_max_yaw + 1) > adjust_step_num) ? ((int)(abs(aim_field[2]) / parameters.stp.adjust_max_yaw + 1)) : (adjust_step_num);
    angle_sequence.push_back(0);
    angle_sequence.push_back(aim_field[2] / 2.0 / adjust_step_num);
    x_sequence.push_back(0);
    x_sequence.push_back(aim_field[0] / 2.0 / adjust_step_num);
    y_sequence.push_back(0);
    y_sequence.push_back(aim_field[1] / 2.0 / adjust_step_num);
    for (int i = 0; i < adjust_step_num - 1; i++)
    {
        angle_sequence.push_back(aim_field[2] / adjust_step_num + angle_sequence[i + 1]);
        x_sequence.push_back(aim_field[0] / adjust_step_num + x_sequence[i + 1]);
        y_sequence.push_back(aim_field[1] / adjust_step_num + y_sequence[i + 1]);
    }
    angle_sequence.push_back(aim_field[2] / 2.0 / adjust_step_num + angle_sequence[adjust_step_num]);
    x_sequence.push_back(aim_field[0] / 2.0 / adjust_step_num + x_sequence[adjust_step_num]);
    y_sequence.push_back(aim_field[1] / 2.0 / adjust_step_num + y_sequence[adjust_step_num]);

    for (unsigned i = 0; i < x_sequence.size() - 1; i++)
    {
        if (i < parameters.stp.adjust_max_step_num)
        {
            vector<double> adjust_step = OneStepTransform(x_sequence[i], y_sequence[i], angle_sequence[i],
                                                          x_sequence[i + 1], y_sequence[i + 1], angle_sequence[i + 1]);
            parameters.stp.tmp_gait.x = adjust_step[0];
            parameters.stp.tmp_gait.y = adjust_step[1];
            parameters.stp.tmp_gait.t = adjust_step[2];
            parameters.stp.tmp_gait.isRight = !parameters.stp.tmp_gait.isRight;
            parameters.stp.tmp_gait.label = "LittleAdjust";
            parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
        }
        else
        {
            break;
        }
    }
    parameters.stp.tmp_gait.x = 0;
    parameters.stp.tmp_gait.y = 0;
    parameters.stp.tmp_gait.t = 0;
    parameters.stp.tmp_gait.isRight = !parameters.stp.tmp_gait.isRight;
    parameters.stp.tmp_gait.label = "LittleAdjust";
    parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
}

void TurnAround(double angle)
{
    vector<double> turn_sequence = CalculateTurnSequence(angle);
    for (auto val : turn_sequence)
    {
        parameters.stp.tmp_gait.x = 0;
        parameters.stp.tmp_gait.y = 0;
        parameters.stp.tmp_gait.t = val;
        parameters.stp.tmp_gait.isRight = !parameters.stp.tmp_gait.isRight;
        parameters.stp.tmp_gait.label = "Turning";
        parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
    }
    parameters.stp.tmp_gait.x = 0;
    parameters.stp.tmp_gait.y = 0;
    parameters.stp.tmp_gait.t = 0;
    parameters.stp.tmp_gait.isRight = !parameters.stp.tmp_gait.isRight;
    parameters.stp.tmp_gait.label = "Turning";
    parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
}

vector<double> CalculateTurnSequence(double turn_angle)
{
    if (turn_angle == 0)
    {
        std::vector<double> a;
        a.push_back(0);
        return a;
    }
    double angle_sign = dmotion::sign(turn_angle);
    turn_angle = turn_angle / angle_sign;
    //选取最省事的旋转方向
    if (turn_angle > 180)
    {
        turn_angle = 360 - turn_angle;
        angle_sign = -angle_sign;
    }

    vector<double> tmp_sequence;
    if (turn_angle / 2 <= 8)
    {
        tmp_sequence.push_back(turn_angle / 2);
    }
    else if (turn_angle / 2 > 8 && turn_angle / 2 <= 24)
    {
        tmp_sequence.push_back(turn_angle / 6);
        tmp_sequence.push_back(turn_angle * 2 / 6);
    }
    else if (turn_angle / 2 > 24 && turn_angle / 2 <= 48)
    {
        tmp_sequence.push_back(turn_angle / 12);
        tmp_sequence.push_back(turn_angle * 2 / 12);
        tmp_sequence.push_back(turn_angle * 3 / 12);
    }
    else if (turn_angle / 2 > 48 && turn_angle / 2 <= 72)
    {
        tmp_sequence.push_back(turn_angle / 18);
        tmp_sequence.push_back(turn_angle * 2 / 18);
        tmp_sequence.push_back(turn_angle * 3 / 18);
        tmp_sequence.push_back(turn_angle * 3 / 18);
    }
    else if (turn_angle / 2 > 72 && turn_angle / 2 <= 90)
    {
        tmp_sequence.push_back(turn_angle / 24);
        tmp_sequence.push_back(turn_angle * 2 / 24);
        tmp_sequence.push_back(turn_angle * 3 / 24);
        tmp_sequence.push_back(turn_angle * 3 / 24);
        tmp_sequence.push_back(turn_angle * 3 / 24);
    }

    unsigned long tmp_vector_size = tmp_sequence.size();
    vector<double> result_sequence;
    for (unsigned i = 0; i < tmp_vector_size; i++)
    {
        result_sequence.push_back(angle_sign * (tmp_sequence[i] + parameters.pendulum_walk_param.TURNING_ERROR));
    }
    for (unsigned i = 0; i < tmp_vector_size; i++)
    {
        result_sequence.push_back(angle_sign * (tmp_sequence[tmp_vector_size - i - 1] + parameters.pendulum_walk_param.TURNING_ERROR));
    }
    return result_sequence;
}

void SlowDown()
{
    parameters.stp.gait_queue.clear();
    double slow_init_x = parameters.stp.last_gait.x;
    double slow_init_y = parameters.stp.last_gait.y;
    double slow_init_yaw = parameters.stp.last_gait.t;
    int a = abs(slow_init_x) / parameters.pendulum_walk_param.slow_down_minus_x;
    a = (abs(slow_init_y) / parameters.pendulum_walk_param.slow_down_minus_y > a) ? (abs(slow_init_y) / parameters.pendulum_walk_param.slow_down_minus_y) : (a);
    a = (abs(slow_init_yaw) / parameters.pendulum_walk_param.slow_down_minus_yaw > a) ? (abs(slow_init_yaw) / parameters.pendulum_walk_param.slow_down_minus_yaw) : (a);
    a = a + 1;

    for (int i = 0; i < a; i++)
    {
        parameters.stp.tmp_gait.x = slow_init_x * (a - i - 1) / a;
        parameters.stp.tmp_gait.y = slow_init_y * (a - i - 1) / a;
        parameters.stp.tmp_gait.t = slow_init_yaw * (a - i - 1) / a;
        parameters.stp.tmp_gait.isRight = !parameters.stp.tmp_gait.isRight;
        parameters.stp.tmp_gait.label = "SlowDown";
        parameters.stp.gait_queue.push_back(parameters.stp.tmp_gait);
    }
}

/**
 * aim_field_distance是距离目标点的距离
 * aim_field_angle是目标点方向相对于现在自己面对的方向的角度差,需要是-180度到180度之间的值
 * last_gait_element 上一步的数据
 * */
gait_element GenerateNewGait(double aim_field_distance, double aim_field_angle, gait_element last_gait_element, double error_aim_angle)
{
    gait_element result_gait;
    aim_field_angle = AdjustDegRange2(aim_field_angle); //调整aim_field_angle的范围

    bool this_is_right = !last_gait_element.isRight; //这一步的迈步脚应与上一步不同
    //先获得希望速度方向和最大速度四边形边界的交点
    double hope_x, hope_y;
    double x_cut, y_cut; //速度边界大菱形的目标方向的边界直线x、y轴截距
    aim_field_angle = AdjustDegRange2(aim_field_angle);
    aim_field_angle = Deg2Rad(aim_field_angle);

    if (aim_field_angle == 0)
    {
        hope_x = parameters.pendulum_walk_param.max_step_x;
        hope_y = 0;
    }
    else if (aim_field_angle == M_PI / 2)
    {
        hope_x = 0;
        hope_y = ((this_is_right) ? (parameters.pendulum_walk_param.max_step_y_in) : (parameters.pendulum_walk_param.max_step_y_out));
    }
    else if (aim_field_angle == -M_PI / 2)
    {
        hope_x = 0;
        hope_y = ((this_is_right) ? (-parameters.pendulum_walk_param.max_step_y_out) : (-parameters.pendulum_walk_param.max_step_y_in));
    }
    else if (aim_field_angle == M_PI || aim_field_angle == -M_PI)
    {
        hope_x = -parameters.pendulum_walk_param.max_step_x;
        hope_y = 0;
    }
    else
    {
        y_cut = ((this_is_right) ? ((sign(aim_field_angle) > 0) ? (parameters.pendulum_walk_param.max_step_y_in) : (-parameters.pendulum_walk_param.max_step_y_out)) : ((sign(aim_field_angle) > 0) ? (parameters.pendulum_walk_param.max_step_y_out) : (-parameters.pendulum_walk_param.max_step_y_in)));
        x_cut = sign(cos(aim_field_angle)) * parameters.pendulum_walk_param.max_step_x;
        hope_x = y_cut / (tan(aim_field_angle) + y_cut / x_cut);
        hope_y = tan(aim_field_angle) * hope_x;
    }

    //计算hope速度与上一步的速度差
    double error_x = hope_x - last_gait_element.x;
    double error_y = hope_y - last_gait_element.y;
    double error_direction = Atan(error_y, error_x);
    //计算速度改变引导下的未校验大小的速度
    double changed_x, changed_y;
    if (error_direction == 0)
    {
        changed_x = parameters.pendulum_walk_param.max_step_differ_x;
        changed_y = 0;
    }
    else if (error_direction == M_PI / 2 || error_direction == -M_PI / 2)
    {
        changed_x = 0;
        changed_y = parameters.pendulum_walk_param.max_step_differ_y * sign(error_direction);
    }
    else if (error_direction == M_PI || error_direction == -M_PI)
    {
        changed_x = -parameters.pendulum_walk_param.max_step_differ_x;
        changed_y = 0;
    }
    else
    {
        y_cut = sign(error_direction) * parameters.pendulum_walk_param.max_step_differ_y;
        x_cut = sign(cos(error_direction)) * parameters.pendulum_walk_param.max_step_differ_x;
        changed_x = y_cut / (tan(error_direction) + y_cut / x_cut);
        changed_y = tan(error_direction) * changed_x;
    }

    double pre_speed_x = last_gait_element.x + changed_x;
    double pre_speed_y = last_gait_element.y + changed_y;
    //判定这个速度是否超出了虽大速度限制四边形
    bool is_overflow;
    double pre_speed_angle = Atan(pre_speed_y, pre_speed_x);
    double pre_dir_max_x, pre_dir_max_y;
    if (pre_speed_angle == 0)
    {
        x_cut = parameters.pendulum_walk_param.max_step_x;
        y_cut = parameters.pendulum_walk_param.max_step_y_in;
        pre_dir_max_x = parameters.pendulum_walk_param.max_step_x;
        pre_dir_max_y = 0;
    }
    else if (pre_speed_angle == M_PI / 2)
    {
        pre_dir_max_x = 0;
        pre_dir_max_y = ((this_is_right) ? (parameters.pendulum_walk_param.max_step_y_in) : (parameters.pendulum_walk_param.max_step_y_out));
        x_cut = parameters.pendulum_walk_param.max_step_x;
        y_cut = pre_dir_max_y;
    }
    else if (pre_speed_angle == -M_PI / 2)
    {
        pre_dir_max_x = 0;
        pre_dir_max_y = ((this_is_right) ? (-parameters.pendulum_walk_param.max_step_y_out) : (-parameters.pendulum_walk_param.max_step_y_in));
        x_cut = parameters.pendulum_walk_param.max_step_x;
        y_cut = pre_dir_max_y;
    }
    else if (pre_speed_angle == M_PI || pre_speed_angle == -M_PI)
    {
        pre_dir_max_x = -parameters.pendulum_walk_param.max_step_x;
        pre_dir_max_y = 0;
        x_cut = -parameters.pendulum_walk_param.max_step_x;
        y_cut = parameters.pendulum_walk_param.max_step_y_in;
    }
    else
    {
        y_cut = ((this_is_right) ? ((sign(pre_speed_angle) > 0) ? (parameters.pendulum_walk_param.max_step_y_in) : (-parameters.pendulum_walk_param.max_step_y_out)) : ((sign(pre_speed_angle) > 0) ? (parameters.pendulum_walk_param.max_step_y_out) : (-parameters.pendulum_walk_param.max_step_y_in)));
        x_cut = sign(cos(pre_speed_angle)) * parameters.pendulum_walk_param.max_step_x;
        pre_dir_max_x = y_cut / (tan(pre_speed_angle) + y_cut / x_cut);
        pre_dir_max_y = tan(pre_speed_angle) * hope_x;
    }
    is_overflow = ((pow(pre_dir_max_x, 2) + pow(pre_dir_max_y, 2)) < (pow(pre_speed_x, 2) + pow(pre_speed_y, 2)));
    double final_speed_x, final_speed_y;
    if (!is_overflow)
    {
        final_speed_x = pre_speed_x;
        final_speed_y = pre_speed_y;
        result_gait.x = final_speed_x;
        result_gait.y = final_speed_y;
        result_gait.isRight = this_is_right;
    }
    else
    {
        if (error_direction == 0 || error_direction == M_PI || error_direction == -M_PI)
        {
            final_speed_y = pre_speed_y;
            final_speed_x = x_cut - x_cut / y_cut * final_speed_y;
        }
        else if (error_direction == M_PI / 2 || error_direction == -M_PI / 2)
        {
            final_speed_x = pre_speed_x;
            final_speed_y = y_cut - y_cut / x_cut * final_speed_x;
        }
        else
        {
            final_speed_x = (y_cut - pre_speed_y + tan(error_direction) * pre_speed_x) / (tan(error_direction) + y_cut / x_cut);
            final_speed_y = y_cut - y_cut / x_cut * final_speed_x;
        }
        result_gait.x = final_speed_x;
        result_gait.y = final_speed_y;
        result_gait.isRight = this_is_right;
    }
    double error_angle;
    double final_yaw;

    if (error_aim_angle == 1000) //如果是默认的角度，机器人的旋转方向目标为速度方向
    {
        error_angle = Rad2Deg(Atan(final_speed_y, final_speed_x));
        result_gait.label = "speed_controller default error_angle";
    }
    else //如果指定了error_aim_angle，机器人的旋转目标为目标位置朝向的目标
    {
        error_angle = AdjustDegRange2(error_aim_angle);
        result_gait.label = "speed_controller customized error_angle";
    }

    if (abs(error_angle) > 20)  // TODO 修复这里的hard code
    {
        if (sign(error_angle) != sign(last_gait_element.t) && last_gait_element.t != 0)
        {
            final_yaw = 0;
        }
        else
        {
            final_yaw = sign(error_angle) * 10;
        }
    }
    else
    {
        double nobrain_angle = parameters.stp.correct_k * parameters.stp.ball_field_angle;
        nobrain_angle = (abs(nobrain_angle) > 10) ? (sign(nobrain_angle) * 10) : nobrain_angle;
        double this_step_x = parameters.stp.last_gait.x + parameters.pendulum_walk_param.max_step_differ_x/3;
        if (this_step_x > parameters.pendulum_walk_param.max_step_x)
        {
            this_step_x = parameters.pendulum_walk_param.max_step_x;
        }
        result_gait.x = this_step_x;
        result_gait.y = nobrain_angle / parameters.stp.correct_y_k;
        final_yaw = nobrain_angle;
        result_gait.label = "nobrain";
    }

    if (abs(final_yaw) > parameters.pendulum_walk_param.max_step_yaw)
    {
        final_yaw = sign(final_yaw) * parameters.pendulum_walk_param.max_step_yaw;
    }
    result_gait.t = final_yaw;
    return result_gait;
} // namespace dmotion
} // namespace dmotion
