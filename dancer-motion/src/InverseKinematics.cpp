//
// Created by zjudancer on 18-10-29.
// E-mail: zjufanwu@zju.edu.cn
//

#include "InverseKinematics.h"
#include <iostream>
#include <cmath>
#include <vector>
#include "Utility/dmotion_math.hpp"
#include "Parameters.h"

namespace dmotion {

    const double InvKin::upper_leg_length =  parameters.one_foot_landing_param.UPPER_LEG_LENGTH;  //大腿的长度
    const double InvKin::lower_leg_length =  parameters.one_foot_landing_param.LOWER_LEG_LENGTH;  //小腿的长度
    const double InvKin::ankle_from_ground =  parameters.one_foot_landing_param.ANKLE_FROM_GROUND;  //脚踝距离地面的高度
    const double InvKin::half_hip_width =  parameters.one_foot_landing_param.HALF_HIP_WIDTH;     //两髋关节点距离的一半,相当于髋关节点相对于身体中心原点的y方向坐标
    const double InvKin::hip_x_from_origin =  parameters.one_foot_landing_param.HIP_X_FROM_ORIGIN;    //髋关节点相对于身体中心原点的x方向坐标
    const double InvKin::hip_z_from_origin =  parameters.one_foot_landing_param.HIP_Z_FROM_ORIGIN;  //髋关节点相对于身体中心原点的相反数z

    InvKin::InvKin(bool isRight) {
        // 初始化几个基本的变量
        isRight_ = isRight;
        hip_yaw_ = 0;
        hip_roll_ = 0;
        hip_pitch_ = 0;
        knee_pitch_ = 0;
        ankle_pitch_ = 0;
        ankle_roll_ = 0;
        // 初始化私有变量都为零
        foot_vertical_x = 0;
        foot_vertical_y = 0;
        foot_vertical_z = 0;
        ankle_x_to_hip = 0;
        ankle_y_to_hip = 0;
        ankle_z_to_hip = 0;
        ankle_norm = 0;
        ankle_axis_x = 0;
        ankle_axis_y = 0;
        ankle_axis_z = 0;
        vertical_x = 0;
        vertical_z = 0;
        vertical_unitz = 0;
        hip_yaw_delta = 0;
        ankle_to_hip_yaw_roll_x = 0;
        ankle_to_hip_yaw_roll_z = 0;
        hip_pitch_absolute = 0;
        foot_hip_rpy_x = 0;
        foot_hip_rpy_y = 0;
        foot_hip_rpy_z = 0;
        ankle_pitch_absolute = 0;
        finals.clear();

    }

    /**
     * 求解脚相对身体中心点的逆运行学
     * @param foot_pose 脚底的x y z r p y
     * @return
     */
    std::vector<double> InvKin::LegInvKin(std::vector<double> foot_pose) {
        /** 下面三个坐标是脚踝点相对于髋关节点的坐标 **/
        foot_pose[3] = dmotion::Deg2Rad(foot_pose[3]);
        foot_pose[4] = dmotion::Deg2Rad(foot_pose[4]);
        foot_pose[5] = dmotion::Deg2Rad(foot_pose[5]);

        foot_vertical_x = -(ankle_from_ground *
                            (std::sin(foot_pose[3]) *
                             std::sin(foot_pose[5]) +
                             std::cos(foot_pose[3]) *
                             std::cos(foot_pose[5]) *
                             std::sin(foot_pose[4])));
        foot_vertical_y = -(-ankle_from_ground *
                            (std::cos(foot_pose[5]) *
                             std::sin(foot_pose[3]) -
                             std::cos(foot_pose[3]) *
                             std::sin(foot_pose[4]) *
                             std::sin(foot_pose[5])));
        foot_vertical_z = -(ankle_from_ground * std::cos(foot_pose[4]) *
                            std::cos(foot_pose[3]));

        ankle_x_to_hip = foot_pose[0] - foot_vertical_x - hip_x_from_origin;
        ankle_y_to_hip = foot_pose[1] - foot_vertical_y - (isRight_ ? (-half_hip_width) : half_hip_width);
        ankle_z_to_hip = foot_pose[2] - foot_vertical_z + hip_z_from_origin;
//        std::cout << "x : " << ankle_x_to_hip << std::endl;
//        std::cout << "y : " << ankle_y_to_hip << std::endl;
//        std::cout << "z : " << ankle_z_to_hip << std::endl;
        ankle_norm = std::sqrt(
                ankle_x_to_hip * ankle_x_to_hip + ankle_y_to_hip * ankle_y_to_hip + ankle_z_to_hip * ankle_z_to_hip);
        /** 获得了knee_pitch的角度 **/
        knee_pitch_ = M_PI - dmotion::CosineTheorem(upper_leg_length, lower_leg_length, ankle_norm);
        /** 这里计算脚踝roll关节的转轴的矢量 **/
        ankle_axis_x = std::cos(foot_pose[4]) * std::cos(foot_pose[5]);
        ankle_axis_y = std::cos(foot_pose[4]) * std::sin(foot_pose[5]);
        ankle_axis_z = -std::sin(foot_pose[4]);
        /** 求ankle_axis和ankle_to_hip的公垂线向量,默认y方向是1**/
        vertical_x = (ankle_z_to_hip * ankle_axis_y - ankle_y_to_hip * ankle_axis_z) /
                     (ankle_x_to_hip * ankle_axis_z - ankle_axis_x * ankle_z_to_hip);
        vertical_z = (ankle_x_to_hip * ankle_axis_y - ankle_axis_x * ankle_y_to_hip) /
                     (ankle_z_to_hip * ankle_axis_x - ankle_x_to_hip * ankle_axis_z);
        /** 这里获得这个公垂线与水平面的夹角 **/
        vertical_unitz = dmotion::GetDelta(0, 0, 1, vertical_x, 1, vertical_z);
        /** 获得了hip_roll的角度 **/
        hip_roll_ = isRight_ ? (vertical_unitz - M_PI / 2) : (M_PI / 2 - vertical_unitz);
        /** 获得了hip_yaw的角度 **/
        hip_yaw_delta = dmotion::GetDelta(0, 1, vertical_x, 1);
        if (vertical_x > 0)
            hip_yaw_ = isRight_ ? hip_yaw_delta : (-hip_yaw_delta);
        else
            hip_yaw_ = isRight_ ? (-hip_yaw_delta) : hip_yaw_delta;


        /**分左右腿获得hip_pitch的角度**/
        if (!isRight_) {
            /** 先得到ankle_to_hip在ankle只经过yaw和roll变换之后的坐标系中的坐标 **/
            ankle_to_hip_yaw_roll_x = ankle_x_to_hip * std::cos(hip_yaw_) + ankle_y_to_hip * std::sin(hip_yaw_);
            /** 因为估计此时的y分量应该为0，所以不算ankle_to_hip_yaw_roll_y **/

            ankle_to_hip_yaw_roll_z =
                    ankle_z_to_hip * std::cos(hip_roll_) - ankle_y_to_hip * std::cos(hip_yaw_) * std::sin(hip_roll_) +
                    ankle_x_to_hip * std::sin(hip_roll_) * std::sin(hip_yaw_);
        } else if (isRight_) {
            ankle_to_hip_yaw_roll_x = ankle_x_to_hip * std::cos(-hip_yaw_) + ankle_y_to_hip * std::sin(-hip_yaw_);
            /** 因为估计此时的y分量应该为0，所以不算ankle_to_hip_yaw_roll_y **/

            ankle_to_hip_yaw_roll_z = ankle_z_to_hip * std::cos(-hip_roll_) -
                                      ankle_y_to_hip * std::cos(-hip_yaw_) * std::sin(-hip_roll_) +
                                      ankle_x_to_hip * std::sin(-hip_roll_) * std::sin(-hip_yaw_);
        }
        /** 获得只经过yaw和roll变换后的-z_unit向量和上面这个向量的夹角
         * 获得hip_pitch_**/
        hip_pitch_absolute = dmotion::Atan(ankle_to_hip_yaw_roll_x, (-ankle_to_hip_yaw_roll_z));
        hip_pitch_ = hip_pitch_absolute + dmotion::CosineTheorem(upper_leg_length, ankle_norm, lower_leg_length);
        /** 这里获得脚底向下的向量在经过hip的ypr变换之后的坐标系中的坐标**/
        if (!isRight_) {
            foot_hip_rpy_x = foot_vertical_x * std::cos(-hip_pitch_absolute) * std::cos(hip_yaw_) -
                             foot_vertical_z * std::cos(hip_roll_) * std::sin(-hip_pitch_absolute) +
                             foot_vertical_y * std::cos(-hip_pitch_absolute) * std::sin(hip_yaw_) +
                             foot_vertical_y * std::cos(hip_yaw_) * std::sin(-hip_pitch_absolute) *
                             std::sin(hip_roll_) -
                             foot_vertical_x * std::sin(-hip_pitch_absolute) * std::sin(hip_roll_) *
                             std::sin(hip_yaw_);
            foot_hip_rpy_y =
                    foot_vertical_z * std::sin(hip_roll_) + foot_vertical_y * std::cos(hip_roll_) * std::cos(hip_yaw_) -
                    foot_vertical_x * std::cos(hip_roll_) * std::sin(hip_yaw_);
            foot_hip_rpy_z = foot_vertical_z * std::cos(-hip_pitch_absolute) * std::cos(hip_roll_) +
                             foot_vertical_x * std::cos(hip_yaw_) * std::sin(-hip_pitch_absolute) +
                             foot_vertical_y * std::sin(-hip_pitch_absolute) * std::sin(hip_yaw_) -
                             foot_vertical_y * std::cos(-hip_pitch_absolute) * std::cos(hip_yaw_) *
                             std::sin(hip_roll_) +
                             foot_vertical_x * std::cos(-hip_pitch_absolute) * std::sin(hip_roll_) *
                             std::sin(hip_yaw_);

            ankle_pitch_absolute = dmotion::Atan(foot_hip_rpy_x, -foot_hip_rpy_z);

            ankle_pitch_ =
                    ankle_pitch_absolute + dmotion::CosineTheorem(lower_leg_length, ankle_norm, upper_leg_length);

            ankle_roll_ = dmotion::Atan(
                    -foot_hip_rpy_y, std::sqrt(foot_hip_rpy_x * foot_hip_rpy_x + foot_hip_rpy_z * foot_hip_rpy_z));

        } else if (isRight_) {
            foot_hip_rpy_x = foot_vertical_x * std::cos(-hip_pitch_absolute) * std::cos(-hip_yaw_) -
                             foot_vertical_z * std::cos(-hip_roll_) * std::sin(-hip_pitch_absolute) +
                             foot_vertical_y * std::cos(-hip_pitch_absolute) * std::sin(-hip_yaw_) +
                             foot_vertical_y * std::cos(-hip_yaw_) * std::sin(-hip_pitch_absolute) *
                             std::sin(-hip_roll_) -
                             foot_vertical_x * std::sin(-hip_pitch_absolute) * std::sin(-hip_roll_) *
                             std::sin(-hip_yaw_);
            foot_hip_rpy_y =
                    foot_vertical_z * std::sin(-hip_roll_) +
                    foot_vertical_y * std::cos(-hip_roll_) * std::cos(-hip_yaw_) -
                    foot_vertical_x * std::cos(-hip_roll_) * std::sin(-hip_yaw_);
            foot_hip_rpy_z = foot_vertical_z * std::cos(-hip_pitch_absolute) * std::cos(-hip_roll_) +
                             foot_vertical_x * std::cos(-hip_yaw_) * std::sin(-hip_pitch_absolute) +
                             foot_vertical_y * std::sin(-hip_pitch_absolute) * std::sin(-hip_yaw_) -
                             foot_vertical_y * std::cos(-hip_pitch_absolute) * std::cos(-hip_yaw_) *
                             std::sin(-hip_roll_) +
                             foot_vertical_x * std::cos(-hip_pitch_absolute) * std::sin(-hip_roll_) *
                             std::sin(-hip_yaw_);
            ankle_pitch_absolute = dmotion::Atan(foot_hip_rpy_x, -foot_hip_rpy_z);

            ankle_pitch_ =
                    ankle_pitch_absolute + dmotion::CosineTheorem(lower_leg_length, ankle_norm, upper_leg_length);

            ankle_roll_ = dmotion::Atan(
                    foot_hip_rpy_y, std::sqrt(foot_hip_rpy_x * foot_hip_rpy_x + foot_hip_rpy_z * foot_hip_rpy_z));
        }
        /** 更新当前参数**/
        finals.clear();
        finals.emplace_back(hip_yaw_);
        finals.emplace_back(hip_roll_);
        finals.emplace_back(hip_pitch_);
        finals.emplace_back(knee_pitch_);
        finals.emplace_back(ankle_pitch_);
        finals.emplace_back(ankle_roll_);
        dmotion::Rad2Deg(finals);
        return finals;

    }

    /**
     * 两条腿逆运动学对象的构造函数
     * @param ankle_distance_y
     */
    WholeBodyIK::WholeBodyIK(const double ankle_distance_y) : left_leg_(false), right_leg_(true) {
        ankle_distance_x_ = 0;
        ankle_distance_y_ = ankle_distance_y;
        ankle_distance_z_ = 0;
        offset_x = 0;
        offset_y = ankle_distance_y_ / 2.0;
        offset_z = 0;
        relative_y = 0;
        relative_p = 0;
        relative_r = 0;
        relative_left_x = 0;
        relative_left_y = 0;
        relative_left_z = 0;
        relative_right_x = 0;
        relative_right_y = 0;
        relative_right_z = 0;

    }

    /**
     * 两条腿逆运动学对象的构造函数的重载，为了应对两个脚不是平时状态的情况
     * @param ankle_distance 踝关节间距
     */
    WholeBodyIK::WholeBodyIK(const double ankle_distance_x, const double ankle_distance_y,
                             const double ankle_distance_z) : left_leg_(false), right_leg_(true) {
        ankle_distance_x_ = ankle_distance_x;
        ankle_distance_y_ = ankle_distance_y;
        ankle_distance_z_ = ankle_distance_z;
        offset_x = ankle_distance_x_ / 2.0;
        offset_y = ankle_distance_y_ / 2.0;
        offset_z = ankle_distance_z_ / 2.0;
        relative_y = 0;
        relative_p = 0;
        relative_r = 0;
        relative_left_x = 0;
        relative_left_y = 0;
        relative_left_z = 0;
        relative_right_x = 0;
        relative_right_y = 0;
        relative_right_z = 0;
    }

    /**
     * 这里是双腿逆运动学的解算实现
     * @param body_pose 输入机器人身体中心原点相对于两脚之间中心的位置和姿态，姿态的旋转顺序
     * 先绕Z_a转yaw角,再绕Y_b转pitch角,最后绕X_b转roll角
     * @return 返回左腿和右腿的关节角度值，先左腿，后右腿，舵机依照安装顺序从上到下排序
     */
    std::vector<double> &WholeBodyIK::GetIKResult(std::vector<double> &body_pose) {
        body_pose[3] = dmotion::Deg2Rad(body_pose[3]);
        body_pose[4] = dmotion::Deg2Rad(body_pose[4]);
        body_pose[5] = dmotion::Deg2Rad(body_pose[5]);

        relative_p = std::asin(-(std::sin(body_pose[3]) * std::sin(body_pose[5]) +
                                 std::cos(body_pose[3]) * std::cos(body_pose[5]) * std::sin(body_pose[4])));
        relative_r = std::asin((std::cos(body_pose[3]) * std::sin(body_pose[4]) * std::sin(body_pose[5]) -
                                std::cos(body_pose[5]) * std::sin(body_pose[3])) / std::cos(relative_p));
        relative_y = std::asin((std::cos(body_pose[5]) * std::sin(body_pose[4]) * std::sin(body_pose[3]) -
                                std::cos(body_pose[3]) * std::sin(body_pose[5])) / std::cos(relative_p));

        vector_for_left.clear();
        vector_for_right.clear();
        legs_finals.clear();


        absolute_left_x = offset_x - body_pose[0];
        absolute_right_x = -offset_x - body_pose[0];
        absolute_left_y = offset_y - body_pose[1];
        absolute_right_y = -offset_y - body_pose[1];
        absolute_left_z = offset_z - body_pose[2];
        absolute_right_z = -offset_z - body_pose[2];


        relative_left_x = absolute_left_x * std::cos(body_pose[4]) * std::cos(body_pose[5]) -
                          absolute_left_z * std::sin(body_pose[4]) +
                          absolute_left_y * std::cos(body_pose[4]) * std::sin(body_pose[5]);
        relative_left_y = absolute_left_y * std::cos(body_pose[3]) * std::cos(body_pose[5]) +
                          absolute_left_z * std::cos(body_pose[4]) * std::sin(body_pose[3]) -
                          absolute_left_x * std::cos(body_pose[3]) * std::sin(body_pose[5]) +
                          absolute_left_x * std::cos(body_pose[5]) * std::sin(body_pose[4]) * std::sin(body_pose[3]) +
                          absolute_left_y * std::sin(body_pose[4]) * std::sin(body_pose[3]) * std::sin(body_pose[5]);
        relative_left_z = absolute_left_z * std::cos(body_pose[4]) * std::cos(body_pose[3]) -
                          absolute_left_y * std::cos(body_pose[5]) * std::sin(body_pose[3]) +
                          absolute_left_x * std::sin(body_pose[3]) * std::sin(body_pose[5]) +
                          absolute_left_x * std::cos(body_pose[3]) * std::cos(body_pose[5]) * std::sin(body_pose[4]) +
                          absolute_left_y * std::cos(body_pose[3]) * std::sin(body_pose[4]) * std::sin(body_pose[5]);

        relative_right_x = absolute_right_x * std::cos(body_pose[4]) * std::cos(body_pose[5]) -
                           absolute_right_z * std::sin(body_pose[4]) +
                           absolute_right_y * std::cos(body_pose[4]) * std::sin(body_pose[5]);
        relative_right_y = absolute_right_y * std::cos(body_pose[3]) * std::cos(body_pose[5]) +
                           absolute_right_z * std::cos(body_pose[4]) * std::sin(body_pose[3]) -
                           absolute_right_x * std::cos(body_pose[3]) * std::sin(body_pose[5]) +
                           absolute_right_x * std::cos(body_pose[5]) * std::sin(body_pose[4]) * std::sin(body_pose[3]) +
                           absolute_right_y * std::sin(body_pose[4]) * std::sin(body_pose[3]) * std::sin(body_pose[5]);
        relative_right_z = absolute_right_z * std::cos(body_pose[4]) * std::cos(body_pose[3]) -
                           absolute_right_y * std::cos(body_pose[5]) * std::sin(body_pose[3]) +
                           absolute_right_x * std::sin(body_pose[3]) * std::sin(body_pose[5]) +
                           absolute_right_x * std::cos(body_pose[3]) * std::cos(body_pose[5]) * std::sin(body_pose[4]) +
                           absolute_right_y * std::cos(body_pose[3]) * std::sin(body_pose[4]) * std::sin(body_pose[5]);


        /** 两脚的x加入 **/
        vector_for_left.emplace_back(relative_left_x);
        vector_for_right.emplace_back(relative_right_x);
        /** 两脚的y加入 **/
        vector_for_left.emplace_back(relative_left_y);
        vector_for_right.emplace_back(relative_right_y);
        /** 两脚的z加入 **/
        vector_for_left.emplace_back(relative_left_z);
        vector_for_right.emplace_back(relative_right_z);


        /** 两脚的r加入 **/
        vector_for_left.emplace_back(dmotion::Rad2Deg(relative_r));
        vector_for_right.emplace_back(dmotion::Rad2Deg(relative_r));
        /** 两脚的p加入 **/
        vector_for_left.emplace_back(dmotion::Rad2Deg(relative_p));
        vector_for_right.emplace_back(dmotion::Rad2Deg(relative_p));
        /** 两脚的y加入 **/
        vector_for_left.emplace_back(dmotion::Rad2Deg(relative_y));
        vector_for_right.emplace_back(dmotion::Rad2Deg(relative_y));

        dmotion::PrintVector(vector_for_left);
        dmotion::PrintVector(vector_for_right);
        angles_for_left = left_leg_.LegInvKin(vector_for_left);
        angles_for_right = right_leg_.LegInvKin(vector_for_right);

        /** 按照先左腿后右腿的顺序加入最终的vector **/
        legs_finals.insert(legs_finals.end(), angles_for_left.begin(), angles_for_left.end());
        legs_finals.insert(legs_finals.end(), angles_for_right.begin(), angles_for_right.end());

        return legs_finals;

    }

    void WholeBodyIK::ChangeFootPos(const double new_foot_x, const double new_foot_y, const double new_foot_z) {
        ankle_distance_x_ = new_foot_x;
        ankle_distance_y_ = new_foot_y;
        ankle_distance_z_ = new_foot_z;
        offset_x = ankle_distance_x_ / 2.0;
        offset_y = ankle_distance_y_ / 2.0;
        offset_z = ankle_distance_z_ / 2.0;

    }


}

