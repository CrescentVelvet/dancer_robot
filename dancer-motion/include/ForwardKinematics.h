//
// Created by zjudancer on 19-1-19
// E-mail: zjufanwu@zju.edu.cn
//

#ifndef PROJECT_FORWARDKINEMATICS_H
#define PROJECT_FORWARDKINEMATICS_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "Utility/dmotion_math.hpp"

namespace dmotion{
class ForKin{
public:
    ForKin(const std::vector<double> angles, bool isRight);
    std::vector<double>  angles_;

    bool isRight_;
    static const double upper_leg_length;   //大腿的长度
    static const double lower_leg_length;   //小腿的长度
    static const double ankle_from_ground;  //脚踝距离地面的高度
    static const double half_hip_width;     //两髋关节点距离的一半,相当于髋关节点相对于身体中心原点的y方向坐标
    static const double hip_x_from_origin;  //髋关节点相对于身体中心原点的x方向坐标
    static const double hip_z_from_origin;  //髋关节点相对于身体中心原点的z

    double x_result;
    double y_result;
    double z_result;
    double roll_result;
    double pitch_result;
    double yaw_result;

    std::vector<double> result_vector;
    Eigen::Isometry3d T;
private:

    std::vector<double> alpha;
    std::vector<double> a;
    std::vector<double> d;
    std::vector<double> theta;

};


class ForKinPlus
{
public:
    //输入为支撑腿相当于规划起点的x,y,z,r,p,y、摆动腿相当于规划起点的x,y,z,r,p,y
    ForKinPlus(std::vector<double> supporting, std::vector<double> hanging);

    Eigen::Isometry3d S;
    Eigen::Isometry3d H;
    //身体中心点相对于支撑脚的x,y,z,r,p,y
    std::vector<double> center2support;
    //摆动脚相对于支撑脚的x,y,z,r,p,y
    std::vector<double> hang2support;

};



}

#endif //PROJECT_FORWARDKINEMATICS_H