//
// Created by ZJUDancer on 2019-3-21
// E-mail: zjufanwu@zju.edu.cn
// This class can load parameters from the ros param server,
// create some global parameters to construct a state machine,
// manage all these parameters
//
#include "Parameters.h"

#define MOTION_WARN(...) ROS_WARN(__VA_ARGS__); //ros的warning输出替换
//用来load参数服务器中的参数
#define GPARAM(x, y)                                        \
    do                                                      \
    {                                                       \
        if (!m_nh->getParam(x, y))                          \
        {                                                   \
            MOTION_WARN("Motion get pararm " #x " error!"); \
        }                                                   \
    } while (0)

using namespace std;
namespace dmotion
{

void Parameters::init(ros::NodeHandle *nh)
{
    m_nh = nh;
    update();
}

void Parameters::update()
{
    GPARAM("/d_motion_/dmotion/OneFootLanding/upbody_constant_roll", one_foot_landing_param.UPBODY_CONSTANT_ROLL);
    GPARAM("/d_motion_/dmotion/OneFootLanding/upbody_constant_pitch", one_foot_landing_param.UPBODY_CONSTANT_PITCH);
    GPARAM("/d_motion_/dmotion/OneFootLanding/ankle_offset_x", one_foot_landing_param.ANKLE_OFFSET_X);
    GPARAM("/d_motion_/dmotion/OneFootLanding/ankle_offset_y", one_foot_landing_param.ANKLE_OFFSET_Y);
    GPARAM("/d_motion_/dmotion/OneFootLanding/ankle_offset_z", one_foot_landing_param.ANKLE_OFFSET_Z);
    GPARAM("/d_motion_/dmotion/OneFootLanding/body_center_x", one_foot_landing_param.BODY_CENTER_X);
    GPARAM("/d_motion_/dmotion/OneFootLanding/body_center_y", one_foot_landing_param.BODY_CENTER_Y);
    GPARAM("/d_motion_/dmotion/OneFootLanding/body_center_z", one_foot_landing_param.BODY_CENTER_X);
    GPARAM("/d_motion_/dmotion/OneFootLanding/upbody_mass", one_foot_landing_param.UPBODY_MASS);
    GPARAM("/d_motion_/dmotion/OneFootLanding/foot_mass", one_foot_landing_param.FOOT_MASS);
    GPARAM("/d_motion_/dmotion/OneFootLanding/upper_leg_length", one_foot_landing_param.UPPER_LEG_LENGTH);
    GPARAM("/d_motion_/dmotion/OneFootLanding/lower_leg_length", one_foot_landing_param.LOWER_LEG_LENGTH);
    GPARAM("/d_motion_/dmotion/OneFootLanding/ankle_from_ground", one_foot_landing_param.ANKLE_FROM_GROUND);
    GPARAM("/d_motion_/dmotion/OneFootLanding/half_hip_width", one_foot_landing_param.HALF_HIP_WIDTH);
    GPARAM("/d_motion_/dmotion/OneFootLanding/hip_x_from_origin", one_foot_landing_param.HIP_X_FROM_ORIGIN);
    GPARAM("/d_motion_/dmotion/OneFootLanding/hip_z_from_origin", one_foot_landing_param.HIP_Z_FROM_ORIGIN);


    GPARAM("/d_motion_/dmotion/PendulumWalk/ankle_dis", pendulum_walk_param.ANKLE_DIS);
    GPARAM("/d_motion_/dmotion/PendulumWalk/tao", pendulum_walk_param.TAO);
    GPARAM("/d_motion_/dmotion/PendulumWalk/tick_num", pendulum_walk_param.TICK_NUM);
    GPARAM("/d_motion_/dmotion/PendulumWalk/com_h", pendulum_walk_param.COM_H);
    GPARAM("/d_motion_/dmotion/PendulumWalk/acc_coef_x", pendulum_walk_param.ACC_COEF_X);
    GPARAM("/d_motion_/dmotion/PendulumWalk/acc_coef_y", pendulum_walk_param.ACC_COEF_Y);
    GPARAM("/d_motion_/dmotion/PendulumWalk/com_height", pendulum_walk_param.COM_HEIGHT);
    GPARAM("/d_motion_/dmotion/PendulumWalk/y_half_amplitude", pendulum_walk_param.Y_HALF_AMPLITUDE);
    GPARAM("/d_motion_/dmotion/PendulumWalk/com_x_offset", pendulum_walk_param.COM_X_OFFSET);
    GPARAM("/d_motion_/dmotion/PendulumWalk/turning_error", pendulum_walk_param.TURNING_ERROR);
    GPARAM("/d_motion_/dmotion/PendulumWalk/slow_down_minus_x", pendulum_walk_param.slow_down_minus_x);
    GPARAM("/d_motion_/dmotion/PendulumWalk/slow_down_minus_y", pendulum_walk_param.slow_down_minus_y);
    GPARAM("/d_motion_/dmotion/PendulumWalk/slow_down_minus_yaw", pendulum_walk_param.slow_down_minus_yaw);
	GPARAM("/d_motion_/dmotion/foot_z/t", pendulum_walk_param.foot_z_t);
	GPARAM("/d_motion_/dmotion/foot_z/p", pendulum_walk_param.foot_z_p);
	GPARAM("/d_motion_/dmotion/foot_z/s", pendulum_walk_param.foot_z_s);
	GPARAM("/d_motion_/dmotion/PendulumWalk/max_step_x", pendulum_walk_param.max_step_x);
	GPARAM("/d_motion_/dmotion/PendulumWalk/max_step_y_out", pendulum_walk_param.max_step_y_out);
	GPARAM("/d_motion_/dmotion/PendulumWalk/max_step_y_in", pendulum_walk_param.max_step_y_in);
	GPARAM("/d_motion_/dmotion/PendulumWalk/max_step_yaw", pendulum_walk_param.max_step_yaw);
	GPARAM("/d_motion_/dmotion/PendulumWalk/max_step_differ_y", pendulum_walk_param.max_step_differ_y);
	GPARAM("/d_motion_/dmotion/PendulumWalk/max_step_differ_x", pendulum_walk_param.max_step_differ_x);
	GPARAM("/d_motion_/dmotion/PendulumWalk/max_step_differ_yaw", pendulum_walk_param.max_step_differ_yaw);


    GPARAM("/d_motion_/dmotion/ThreeInterpolation/default_point_interval", three_interpolation_param.DEFAULT_POINT_INTERVAL);
    GPARAM("/d_motion_/dmotion/ThreeInterpolation/default_boundary_slope", three_interpolation_param.DEFAULT_BOUNDARY_SLOPE);

    GPARAM("/d_motion_/dmotion/Climb/whole_time", climb_param.WHOLE_TIME);
    GPARAM("/d_motion_/dmotion/Climb/not_leg_only_number", climb_param.NOT_LEG_ONLY_NUMBER);

    GPARAM("/d_motion_/dmotion/Kick/right_kick_x", kick_param.RIGHT_KICK_X);
    GPARAM("/d_motion_/dmotion/Kick/right_kick_y", kick_param.RIGHT_KICK_Y);
    GPARAM("/d_motion_/dmotion/Kick/right_kick_files", kick_param.RIGHT_KICK_FILES);
    GPARAM("/d_motion_/dmotion/com_x/data", kick_param.COM_X);
    GPARAM("/d_motion_/dmotion/com_y/data", kick_param.COM_Y);
    GPARAM("/d_motion_/dmotion/com_z/data", kick_param.COM_Z);
    GPARAM("/d_motion_/dmotion/ankle_x/data", kick_param.ANKLE_X);
    GPARAM("/d_motion_/dmotion/ankle_y/data", kick_param.ANKLE_Y);
    GPARAM("/d_motion_/dmotion/ankle_z/data", kick_param.ANKLE_Z);
    GPARAM("/d_motion_/dmotion/ankle_pitch/data", kick_param.ANKLE_PITCH);
    GPARAM("/d_motion_/dmotion/ankle_roll/data", kick_param.ANKLE_ROLL);
    GPARAM("/d_motion_/dmotion/ankle_yaw/data", kick_param.ANKLE_YAW);

    GPARAM("/d_motion_/dmotion/Status/adjust_max_x", stp.adjust_max_x);
    GPARAM("/d_motion_/dmotion/Status/adjust_max_y", stp.adjust_max_y);
    GPARAM("/d_motion_/dmotion/Status/adjust_max_yaw", stp.adjust_max_yaw);
    GPARAM("/d_motion_/dmotion/Status/adjust_max_step_num", stp.adjust_max_step_num);
    GPARAM("/d_motion_/dmotion/Status/action_confirmation_num",stp.action_confirmation_num);
    GPARAM("/d_motion_/dmotion/Status/turn_tolerance",stp.turn_tolerance);
    GPARAM("/d_motion_/dmotion/Status/LADT",stp.LADT);
    GPARAM("/d_motion_/dmotion/Status/LADT_for_ball",stp.LADT_for_ball);
    GPARAM("/d_motion_/dmotion/Status/LAAT_for_ball",stp.LAAT_for_ball);
    GPARAM("/d_motion_/dmotion/Status/stop_walk_dis",stp.stop_walk_dis);
    GPARAM("/d_motion_/dmotion/Status/correct_k",stp.correct_k);
    GPARAM("/d_motion_/dmotion/Status/correct_y_k",stp.correct_y_k);
    GPARAM("/d_motion_/dmotion/Status/shit_rob_radius",stp.shit_rob_radius);
    GPARAM("/d_motion_/dmotion/Status/one_step_y",stp.one_step_y);
    GPARAM("/d_motion_/dmotion/Status/stop_walk_dis",stp.stop_walk_dis);
    GPARAM("/d_motion_/dmotion/Status/dx_delta",stp.dx_delta);
    GPARAM("/d_motion_/dmotion/Status/dyaw_delta",stp.dyaw_delta);
    GPARAM("/d_motion_/dmotion/Status/panball_x",stp.panball_x);
    GPARAM("/d_motion_/dmotion/Status/uparm_angle",stp.UPARM_ANGLE);
    GPARAM("/d_motion_/dmotion/Status/lowarm_angle",stp.LOWARM_ANGLE);
    GPARAM("/d_motion_/dmotion/Status/backward_step_length",stp.backward_step_length);

}

Parameters parameters;
} // namespace dmotion
