//
// Created by fw on 18/10/17.
// E-mail: zjufanwu@zju.edu.cn
//
// This .hpp contain some useful tools besides mathematics.
#ifndef DMOTION_MATH_HPP
#define DMOTION_MATH_HPP

#include <vector>
#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <iomanip>
#include <ros/ros.h>
#include <Parameters.h>
#include <ctime>
#include <sys/time.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "../ThreeInterpolation.h"

// #define UPARM_ANGLE 0
// #define LOWARM_ANGLE 30
using namespace dmotion;

namespace dmotion
{

/**
     * 把曲线斜率化为倾斜角（角度制）的函数
     */
template <class T>
double Slope_to_AngleDeg(T s)
{
    return atan(s) * 57.3;
}

/**
     * 角度制到弧度值的转换
     */
template <class T>
inline T Deg2Rad(T deg)
{
    return deg * M_PI / 180;
}

/**
     * 弧度值到角度值的转换
     */
template <class T>
inline T Rad2Deg(T rad)
{
    return rad * 180 / M_PI;
}

/**
     * vector的角度值到弧度值的转换
     */
template <typename Scalar>
inline std::vector<Scalar> &Deg2Rad(std::vector<Scalar> &deg)
{
    int size = deg.size();
    for (int i = 0; i < size; i++)
    {
        deg.push_back(Deg2Rad(deg[i]));
    }
    deg.erase(deg.begin(), deg.begin() + size);
    return deg;
}
/**
 * 延时函数，输入参数为需要延时的微秒数
 * */
void Delay(int time) //time*1000000为秒数
{
    clock_t now = clock();
    while (clock() - now < time)
        ;
}
/**
     * vector的角度值到弧度值的转换
     */
template <typename Scalar>
inline std::vector<Scalar> &Rad2Deg(std::vector<Scalar> &rad)
{
    int size = rad.size();
    for (int i = 0; i < size; i++)
    {
        rad.push_back(Rad2Deg(rad[i]));
    }
    rad.erase(rad.begin(), rad.begin() + size);
    return rad;
}

/**
     * vector模版类的全局输出
     */
template <class T>
inline void PrintVector(const std::vector<T> &vectorOb)
{
    for (unsigned i = 0; i < vectorOb.size(); i++)
        std::cout << std::fixed << std::setprecision(7) << vectorOb[i] << " ";
    std::cout << std::endl;
}

/**
     * 判断输入参数的符号的模板函数
     */
template <typename T>
inline T sign(T val)
{
    return val > 0 ? 1 : (val < 0 ? -1 : 0);
}

/**
     * 使用三角形的余弦定理求某个角度,这种运算为了保证正确性和精度,用double比较好,所以这里不使用模板
     * @param edge_1 临边1长度
     * @param edge_2 临边2长度
     * @param edge_opposite 对边长度
     * @return 要求的角的角度值
     */
inline double CosineTheorem(const double &edge_1,
                            const double &edge_2,
                            const double &edge_opposite)
{
    double cosine = (edge_1 * edge_1 + edge_2 * edge_2 - edge_opposite * edge_opposite) / (2 * edge_1 * edge_2);
    return std::acos(cosine);
}

/**
     * 获得两个三维向量之间夹角的函数
     * @param x_1
     * @param y_1
     * @param z_1
     * @param x_2
     * @param y_2
     * @param z_2
     * @return
     */
inline double GetDelta(const double &x_1, const double &y_1, const double &z_1, const double &x_2,
                       const double &y_2, const double &z_2)
{
    double tmpcos = (x_1 * x_2 + y_1 * y_2 + z_1 * z_2) /
                    sqrt((x_1 * x_1 + y_1 * y_1 + z_1 * z_1) * (x_2 * x_2 + y_2 * y_2 + z_2 * z_2));
    return std::acos(tmpcos);
}

/**
     * 获得两个二维向量之间夹角的函数
     * @param x_1
     * @param y_1
     * @param x_2
     * @param y_2
     * @return
     */
inline double GetDelta(const double &x_1, const double &y_1, const double &x_2, const double &y_2)
{
    double tmpcos = (x_1 * x_2 + y_1 * y_2) / sqrt((x_1 * x_1 + y_1 * y_1) * (x_2 * x_2 + y_2 * y_2));
    return std::acos(tmpcos);
}

/**
     * 使用Eigen的计算方法重载求两向量夹角的函数
     * @tparam order
     * @tparam T
     * @param v1
     * @param v2
     * @return
     */
template <int order, typename T>
inline double GetDelta(const Eigen::Matrix<T, order, 1> &v1, const Eigen::Matrix<T, order, 1> &v2)
{
    double tmp = v1.dot(v2) / (v1.norm() * v2.norm());
    std::cout << "tmp: " << tmp << std::endl
              << std::endl;
    if (1 < tmp)
        tmp = 1.0;
    else if (-1 > tmp)
        tmp = -1.0;
    return std::acos(1);
}

/**
     * 双变量反正切函数，用于准确求取一个角度，值域是[-pi,pi)
     * @tparam T
     * @param opposite 对边长度（直角边）
     * @param neighbor 临边长度 (直角边)
     * @return 所求的角度，弧度制
     */
inline double Atan(const double opposite, const double neighbor)
{
    if (0 < neighbor)
        return std::atan(opposite / neighbor);
    else if (0 == neighbor)
    {
        if (0 > opposite)
            return -M_PI / 2;
        else if (0 < opposite)
            return M_PI / 2;
        else
            return 0;
    }
    else
    {
        if (0 <= opposite)
            return std::atan(opposite / neighbor) + M_PI;
        else
            return std::atan(opposite / neighbor) - M_PI;
    }
}

/**
     * 用于计算三维空间中两点之间的距离
     * @tparam T
     * @param x_1
     * @param y_1
     * @param z_1
     * @param x_2
     * @param y_2
     * @param z_2
     * @return
     */
template <typename T>
inline double getDistance(const T &x_1, const T &y_1, const T &z_1, const T &x_2, const T &y_2, const T &z_2)
{
    return std::sqrt(std::pow(x_1 - x_2, 2) + std::pow(y_1 - y_2, 2) + std::pow(z_1 - z_2, 2));
}

/**
     * 把slave vector里面的元素追加到master后面
     * @tparam T
     * @param master
     * @param slave
     * @return
     */
template <typename T>
void AddElements(std::vector<T> &master, std::vector<T> slave)
{
    for (unsigned int i = 0; i < slave.size(); i++)
    {
        master.emplace_back(slave[i]);
    }
}

/**
     * 调整angle(角度制)的范围，使之可以限制在【0,360】,例如-270度转变为90度
     * @param angle
     * @return 调整好范围后的角度
     */
double AdjustDegRange(double angle)
{
    if (angle >= 360)
        return (angle - (int)(angle / 360) * 360);
    else if (angle < 0)
        return (angle - ((int)(angle / 360) - 1) * 360);
    else
        return angle;
}

/**
 * 调整angle(角度制)的范围，使之可以限制在[-180,180],例如200度转换为-160度
 * */
double AdjustDegRange2(double angle)
{
    if (angle >= 180)
        return (angle - (int)((angle - 180) / 360 + 1) * 360);
    else if (angle < 0)
        return (angle - ((int)((angle + 180) / 360 - 1)) * 360);
    else
        return angle;
}

struct motion_tick //用来描述发值瞬间的机器人动作参数的结构体
{
    unsigned time_stamp; //这是时间戳，单位是ns
    std::vector<double> upbody_pose;
    std::vector<double> whole_com;
    std::vector<double> hang_foot;
};

/** 
     * 将齐次矩阵转换为欧拉角
     * @param M 齐次矩阵
     * 
     */
std::vector<double> Matrix2Pose(Eigen::Isometry3d M) {
    std::vector<double> pose;
    pose.emplace_back(M.matrix()(0, 3));
    pose.emplace_back(M.matrix()(1, 3));
    pose.emplace_back(M.matrix()(2, 3));

    pose.emplace_back(dmotion::Rad2Deg(dmotion::Atan(M.matrix()(2, 1), M.matrix()(2, 2))));
    pose.emplace_back(dmotion::Rad2Deg(std::asin(-M.matrix()(2, 0))));
    pose.emplace_back(dmotion::Rad2Deg(dmotion::Atan(M.matrix()(1, 0), M.matrix()(0, 0))));

    return pose;
}


/** 
     * 将欧拉角转换为齐次矩阵
     * @param pose 欧拉角
     * 
     */
Eigen::Isometry3d Pose2Matrix(std::vector<double> pose){
    Eigen::Isometry3d T;
    T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd rotate_roll(pose[3], Eigen::Vector3d(1, 0, 0));
    Eigen::AngleAxisd rotate_pitch(pose[4], Eigen::Vector3d(0, 1, 0));
    Eigen::AngleAxisd rotate_yaw(pose[5], Eigen::Vector3d(0, 0, 1));
    T.rotate(rotate_roll);
    T.rotate(rotate_pitch);
    T.rotate(rotate_yaw);
    T.pretranslate(Eigen::Vector3d(pose[0], pose[1], pose[2]));
    return T;
}


std::vector<double> Planning(double start_time, double end_time, double y_0, double y_1, double s_0, double s_1)
{
    std::vector<double> x, y, s, result;
    y.push_back(y_0);
    y.push_back(y_1);
    s.push_back(s_0);
    s.push_back(s_1);
    x.push_back(start_time / 100.0);
    x.push_back(end_time / 100.0);
    //        cout << "tttttt" << endl;
    //        PrintVector(x);
    //        PrintVector(y);
    //        PrintVector(s);

    ThreeInterpolation hello(x, y, s);
    result = hello.GetPoints();
    //        PrintVector(hello.GetTimes());
    return result;
}

std::vector<std::vector<double>> ServoTransition(std::vector<double> start_vector, std::vector<double> end_vector, int tick_num)
{
    std::vector<std::vector<double>> result_matrix;
    std::vector<double> ttmp = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (int k = 0; k < tick_num; k++)
    {
        result_matrix.push_back(ttmp);
    }
    for (int i = 0; i < 12; i++)
    {
        vector<double> tmp = Planning(0, (double)tick_num, start_vector[i], end_vector[i], 0, 0);

        for (int j = 0; j < tick_num; j++)
        {
            result_matrix[j][i] = tmp[j+1];
        }
    }
    return result_matrix;
}

/** 
     * motion_hub给IO发舵机信息的内容都是用这个函数实现
     * @param data_vector 舵机值的vector
     * @param label 这个舵机值属于什么动作的一部分，分为爬起GETUP、踢球KICK和走路WALK
     * 
     */
void ServoPublish(std::vector<double> data_vector, std::string label, ros::Publisher *ServoInfo_pub, ros::Rate *loop_rate)
{
    std_msgs::Float64MultiArray result_array;
    std_msgs::MultiArrayDimension result_dim;
    if (label == "GETUP")
    {
        result_dim.label = "GETUP";
        result_array.layout.dim.push_back(result_dim);
        result_array.data = data_vector;
    }
    else if (label == "KICK")
    { //踢球可以适当的改进，如果需要IO那边需要闭环的话
        result_dim.label = "KICK";
        result_array.layout.dim.push_back(result_dim);
        result_array.layout.data_offset = (int)(parameters.support_phase); //支撑相
        result_array.data = data_vector;
        result_array.data.push_back(parameters.stp.UPARM_ANGLE);
        result_array.data.push_back(parameters.stp.LOWARM_ANGLE);
        result_array.data.push_back(parameters.stp.UPARM_ANGLE);
        result_array.data.push_back(parameters.stp.LOWARM_ANGLE);
    }
    else if (label == "WALK")
    {
        result_dim.label = "WALK";
        result_array.layout.dim.push_back(result_dim);
        result_array.layout.data_offset = (int)(!parameters.support_phase); //支撑相
        result_array.data = data_vector;
        result_array.data.push_back(parameters.stp.UPARM_ANGLE);
        result_array.data.push_back(parameters.stp.LOWARM_ANGLE);
        result_array.data.push_back(parameters.stp.UPARM_ANGLE);
        result_array.data.push_back(parameters.stp.LOWARM_ANGLE);
    }
    else if (label == "STANDBY")
    {
        result_dim.label = "STANDBY";
        result_array.layout.dim.push_back(result_dim);
        result_array.layout.data_offset = 2;
        result_array.data.push_back(parameters.stp.UPARM_ANGLE);
        result_array.data.push_back(parameters.stp.LOWARM_ANGLE);
        result_array.data.push_back(parameters.stp.UPARM_ANGLE);
        result_array.data.push_back(parameters.stp.LOWARM_ANGLE);
    }
    else if (label == "GUARD")
    {
        result_dim.label = "KICK";
        result_array.layout.dim.push_back(result_dim);
        result_array.data = data_vector;
    }
    parameters.stp.cur_servo_angles.clear();
    parameters.stp.cur_servo_angles = result_array.data;
    ServoInfo_pub->publish(result_array);
    ros::spinOnce();
    loop_rate->sleep();
}

} // namespace dmotion

#endif //DMOTION_MATH_HPP
