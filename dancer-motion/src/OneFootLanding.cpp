//
// Created by zjudancer on 2018-12-25.
//

#include "OneFootLanding.h"
#include "Parameters.h"


namespace dmotion {
    const double OneFootLanding::ankle_offset_x = parameters.one_foot_landing_param.ANKLE_OFFSET_X;//脚中心相对于腿重心的位置偏差0.5
    const double OneFootLanding::ankle_offset_y = parameters.one_foot_landing_param.ANKLE_OFFSET_Y;//0
    const double OneFootLanding::ankle_offset_z = parameters.one_foot_landing_param.ANKLE_OFFSET_Z;//10.5
    const double OneFootLanding::body_center_x = parameters.one_foot_landing_param.BODY_CENTER_X;//身体中心相对于上半身质心的位置偏差2.5
    const double OneFootLanding::body_center_y = parameters.one_foot_landing_param.BODY_CENTER_Y;//0
    const double OneFootLanding::body_center_z = parameters.one_foot_landing_param.BODY_CENTER_Z;//-2.4
    const double OneFootLanding::upbody_mass = parameters.one_foot_landing_param.UPBODY_MASS;//3158
    const double OneFootLanding::foot_mass = parameters.one_foot_landing_param.FOOT_MASS;//877


    OneFootLanding::OneFootLanding(bool isRight) : isRight_(isRight) {
        left_leg_ = new InvKin(false);
        right_leg_ = new InvKin(true);
        upbody_com.clear();
    }
    OneFootLanding::~OneFootLanding(){
            delete left_leg_;
            delete right_leg_;
            left_leg_ = NULL;
            right_leg_ = NULL;
    }

    std::vector<double> OneFootLanding::GetHanglegCom(std::vector<double> hang_foot_)
    {
        //求悬荡腿质心位置
        T_BA = Pose2Matrix(hang_foot_);
        T_CB = Eigen::Isometry3d::Identity();
        T_CB.translate(Eigen::Vector3d(ankle_offset_x, ankle_offset_y, ankle_offset_z));
        T_CA = T_CB * T_BA;

        hangfoot_com.emplace_back(T_CA.matrix()(0, 3));
        hangfoot_com.emplace_back(T_CA.matrix()(1, 3));
        hangfoot_com.emplace_back(T_CA.matrix()(2, 3));
        //std::cout << "hangfoot_com " << " :" ;
        //dmotion::PrintVector(hangfoot_com);
        return hangfoot_com;
    }

    std::vector<double> OneFootLanding::GetSupportlegCom()
    {
        //求支撑腿质心位置
        landingfoot_com.emplace_back(ankle_offset_x);
        landingfoot_com.emplace_back(isRight_ ? (-ankle_offset_y) : ankle_offset_y);
        landingfoot_com.emplace_back(ankle_offset_z);
        //std::cout << "landingfoot_com " << " :" ;
        //dmotion::PrintVector(landingfoot_com);
        return landingfoot_com;
    }

    /**
     * 进行求解，这些参数都是相对于立足脚中心点的，最稳妥重心投影点根据脚板的安装位置做调整
     * @param hang_foot 悬荡脚的xyzrpy
     * @param whole_body_com 整机的重心
     * @param upbody_yaw 上半身的yaw
     * @return 12个舵机的值
     */
    std::vector<double> 
    OneFootLanding::GetOneStep(std::vector<double> hang_foot, const std::vector<double> &whole_body_com,
                               std::vector<double> upbody_pose,upbody_mode mode, bool which) {

        hang_foot[3] = dmotion::Deg2Rad(hang_foot[3]);
        hang_foot[4] = dmotion::Deg2Rad(hang_foot[4]);
        hang_foot[5] = dmotion::Deg2Rad(hang_foot[5]);
        upbody_yaw = dmotion::Deg2Rad(upbody_pose[2]);

        hangfoot_com.clear();
        landingfoot_com.clear();
        upbody_com.clear();
        //求悬荡腿质心位置
        // T_BA = Eigen::Isometry3d::Identity();
        // Eigen::AngleAxisd rotate_roll(hang_foot[3], Eigen::Vector3d(1, 0, 0));
        // Eigen::AngleAxisd rotate_pitch(hang_foot[4], Eigen::Vector3d(0, 1, 0));
        // Eigen::AngleAxisd rotate_yaw(hang_foot[5], Eigen::Vector3d(0, 0, 1));
        // T_BA.rotate(rotate_roll);
        // T_BA.rotate(rotate_pitch);
        // T_BA.rotate(rotate_yaw);
        // T_BA.translate(Eigen::Vector3d(hang_foot[0], hang_foot[1], hang_foot[2]));

        // T_CB = Eigen::Isometry3d::Identity();
        // T_CB.translate(Eigen::Vector3d(ankle_offset_x, ankle_offset_y, ankle_offset_z));
        // T_CA = T_CB * T_BA;

        // hangfoot_com.emplace_back(T_CA.matrix()(0, 3));
        // hangfoot_com.emplace_back(T_CA.matrix()(1, 3));
        // hangfoot_com.emplace_back(T_CA.matrix()(2, 3));
        // std::cout << "hangfoot_com " << " :" ;
        // dmotion::PrintVector(hangfoot_com);
        GetHanglegCom(hang_foot);



        //求支撑腿质心位置
        // landingfoot_com.emplace_back(ankle_offset_x);
        // landingfoot_com.emplace_back(isRight_ ? (-ankle_offset_y) : ankle_offset_y);
        // landingfoot_com.emplace_back(ankle_offset_z);
        // std::cout << "landingfoot_com " << " :" ;
        // dmotion::PrintVector(landingfoot_com);
        GetSupportlegCom();

        //求上半身质心位置
        if (which ) {
            upbody_com.emplace_back(
                    ((upbody_mass + foot_mass) * whole_body_com[0] - foot_mass * hangfoot_com[0]) / upbody_mass);
            upbody_com.emplace_back(
                    ((upbody_mass + foot_mass) * whole_body_com[1] - foot_mass * hangfoot_com[1]) / upbody_mass);
            upbody_com.emplace_back(
                    ((upbody_mass + foot_mass) * whole_body_com[2] - foot_mass * hangfoot_com[2]) / upbody_mass);
        } else {
            upbody_com.emplace_back(((upbody_mass + 2 * foot_mass) * whole_body_com[0] - foot_mass * hangfoot_com[0] -
                                     foot_mass * landingfoot_com[0]) / upbody_mass);
            upbody_com.emplace_back(((upbody_mass + 2 * foot_mass) * whole_body_com[1] - foot_mass * hangfoot_com[1] -
                                     foot_mass * landingfoot_com[1]) / upbody_mass);
            upbody_com.emplace_back(((upbody_mass + 2 * foot_mass) * whole_body_com[2] - foot_mass * hangfoot_com[2] -
                                     foot_mass * landingfoot_com[2]) / upbody_mass);
        }
        //std::cout << "upbody_com " << " :" ;
        //dmotion::PrintVector(upbody_com);

        TA1.clear();
        TA2.clear();
        v.clear();
        for (int i = 0; i < 3; i++)
        {
            TA1.emplace_back(upbody_com[i] - landingfoot_com[i]);
            TA2.emplace_back(upbody_com[i] - hangfoot_com[i]);
        }
        unit_arrow(TA1);
        unit_arrow(TA2);
//        std::cout << "TA1 " << " :" ;
//        dmotion::PrintVector(TA1);
//        std::cout << "TA2 " << " :" ;
//        dmotion::PrintVector(TA2);
        for (int i = 0; i < 3; i++) {
            v.emplace_back(TA1[i] + TA2[i]);
        }
        unit_arrow(v);
        //确定使用什么类型的上半身姿态
        if (mode == upbody_still )
        {
                upbody_roll = 0;
                upbody_pitch = 0;
        }else if(mode == constant_value){
                upbody_roll = parameters.one_foot_landing_param.UPBODY_CONSTANT_ROLL;
                upbody_pitch = parameters.one_foot_landing_param.UPBODY_CONSTANT_PITCH;
        }else if(mode == self_adaption){
                upbody_roll = dmotion::Atan(-v[1], v[2])*4.0/5.0;
                upbody_pitch = dmotion::Atan(v[0], v[2] / std::cos(upbody_roll))*4.0/5.0;
        }else if (mode == customized_value){
                upbody_roll = dmotion::Deg2Rad(upbody_pose[0]);
                upbody_pitch = dmotion::Deg2Rad(upbody_pose[1]);
        }

        //求身体中心位置
        // T_DA = Eigen::Isometry3d::Identity();
        // Eigen::AngleAxisd rotate_roll_1(upbody_roll, Eigen::Vector3d(1, 0, 0));
        // Eigen::AngleAxisd rotate_pitch_1(upbody_pitch, Eigen::Vector3d(0, 1, 0));
        // Eigen::AngleAxisd rotate_yaw_1(upbody_yaw, Eigen::Vector3d(0, 0, 1));
        // T_DA.rotate(rotate_roll_1);
        // T_DA.rotate(rotate_pitch_1);
        // T_DA.rotate(rotate_yaw_1);
        // T_DA.translate(Eigen::Vector3d(upbody_com[0], upbody_com[1], upbody_com[2]));
        std::vector<double> upbody_ = {upbody_com[0], upbody_com[1], upbody_com[2], upbody_roll, upbody_pitch, upbody_yaw};
        T_DA = Pose2Matrix(upbody_);

        T_ED = Eigen::Isometry3d::Identity();
        T_ED.translate(Eigen::Vector3d(body_center_x, body_center_y, body_center_z));
        T_EA = T_ED * T_DA;

        body_centre.emplace_back(T_EA.matrix()(0, 3));
        body_centre.emplace_back(T_EA.matrix()(1, 3));
        body_centre.emplace_back(T_EA.matrix()(2, 3));

        //std::cout << "body_centre:  ";
        //dmotion::PrintVector(body_centre);

        //支撑腿逆运动学计算
        landing_invkin = dmotion::Matrix2Pose(T_EA.inverse());
        
        //std::cout << "landing_invkin:  ";
        //dmotion::PrintVector(landing_invkin);

        //给悬荡脚的逆运动学向量加入值
        hanging_invkin.clear();

        hanging_invkin = dmotion::Matrix2Pose(T_BA * T_EA.inverse());
        //std::cout << "hanging_invkin:  ";
        //dmotion::PrintVector(hanging_invkin);

        one_foot_result.clear();
        if (isRight_) {
//            std::cout << "right" << std::endl;
            dmotion::AddElements(one_foot_result, right_leg_->LegInvKin(landing_invkin));
            dmotion::AddElements(one_foot_result, left_leg_->LegInvKin(hanging_invkin));
        } else if (!isRight_) {
//            std::cout << "left" << std::endl;
            dmotion::AddElements(one_foot_result, right_leg_->LegInvKin(hanging_invkin));
            dmotion::AddElements(one_foot_result, left_leg_->LegInvKin(landing_invkin));
        }

        return one_foot_result;

    }

    void OneFootLanding::unit_arrow(std::vector<double> &arrow) {
        double len = std::sqrt(arrow[0] * arrow[0] + arrow[1] * arrow[1] + arrow[2] * arrow[2]);
        for (unsigned int i = 0; i < arrow.size(); i++)
            arrow[i] = arrow[i] / len;
    }


}