#include "climb.h"

#define LEG_ONLY_NUMBER 13
#define NOT_LEG_ONLY_NUMBER 17
using namespace std;
using namespace dmotion;
namespace dmotion
{

void Climb::Prepare()
{
    ROS_FATAL("prepare for get up");
    cout << label_ << endl;
    //示教爬起过程
    std::ifstream infile;
    if (label_ == "BACK")
    {
        infile.open(parameters.kick_param.RIGHT_KICK_FILES + "climb_param/back_climb.txt", std::ios::in | std::ios::out);
    }
    else if (label_ == "FORWARD")
    {
        // cout << "get forward "<< endl;
        infile.open(parameters.kick_param.RIGHT_KICK_FILES + "climb_param/forward_climb.txt", std::ios::in | std::ios::out);
    }
    while (!infile.eof())
    {
        double buffer;
        std::vector<double> SinglePosition;
        for (int i = 0; i < NOT_LEG_ONLY_NUMBER; i++)
        {
            infile >> buffer;
            SinglePosition.push_back(buffer);
            
        }
        AllPosition_time.push_back(SinglePosition);
        SinglePosition.clear();
    }
    AllPosition = AllPosition_time;
    for (unsigned i = 0; i < AllPosition.size(); i++)
    {
        AllPosition[i].pop_back();
    }
    if(label_ == "FORWARD") 
    {
        std_forward_pose = AllPosition[0];
    }
    else if(label_ == "BACK") {
        std_back_pose = AllPosition[0];
    }


}
/**
     * @param publisher 用于发给舵机值
     * @param lopr 用于控制发值周期
     * @param label 选择前爬或后爬
     */
Climb::Climb(ros::Publisher *publisher, ros::Rate *loop_rate, std::string label, std::vector<double> position_start) : pbr(publisher), lopr(loop_rate)
{
    label_ = label;
    //从IO订阅消息得到跌倒时的舵机值
    position_now = position_start;
    //从跌倒状态规划到示教爬起开始姿态
    Prepare();
    for (int i = 0; i < 16; i++)
    {
        value[0] = position_now[i];
        if (label_ == "BACK")
        {
            value[1] = std_back_pose[i];
        }
        else if (label_ == "FORWARD")
        {
            value[1] = std_forward_pose[i];
        }
        ROS_FATAL("std_forward_pose");
        PrintVector(std_forward_pose);
        ROS_FATAL("std_back_pose");
        PrintVector(std_back_pose);

        ThreeInterpolation offset(tit, value);
        servo_split = offset.GetPoints();
        servo_position.push_back(servo_split);
        std::cout << i << std::endl;
    }
    ROS_FATAL("servo_points");

    for (unsigned i = 0; i < unsigned(parameters.climb_param.WHOLE_TIME*100); i++)
    {
        for (unsigned j = 0; j < 16; j++)
        {
            servo_points[j] = servo_position[j][i];
        }
        dmotion::ServoPublish(servo_points, "GETUP", pbr, lopr);
        PrintVector(servo_points);
        //io.setAllJointValue(servo_points);
        //io.spinOnce();
    }
    ROS_FATAL("GETUP TEACHING DATA");

    for (unsigned i = 0; i < AllPosition.size(); i += 2)
    {
        dmotion::ServoPublish(AllPosition[i], "GETUP", pbr, lopr);
        PrintVector(AllPosition[i]);
        //io.setAllJointValue(AllPosition[i]);
        //io.spinOnce();
        if (i == AllPosition.size() - 2 || i == AllPosition.size() - 1)
        {
            position_aftclimb = AllPosition[i];
            ROS_FATAL("ALLPOSITION");
            PrintVector(position_aftclimb);
        }
    }

    // Delay(1000000);
    //从示教爬起结束状态进行质心规划到达OneFootLanding起点
    //正运动学计算身体中心和悬荡脚相对于支撑脚的xyzrpy
    std::vector<double> aa = position_aftclimb;
    angle_leftleg = {aa[6],aa[7],aa[8],aa[9],aa[10],aa[11]};
    angle_rightleg = {aa[0], aa[1], aa[2], aa[3], aa[4], aa[5]};
    dmotion::ForKin left_leg(angle_leftleg, false);
    dmotion::ForKin right_leg(angle_rightleg, true);
    lfoot2center = left_leg.result_vector;
    rfoot2center = right_leg.result_vector;
    ROS_FATAL("lfoot2center");
    PrintVector(lfoot2center);
    ROS_FATAL("rfoot2center");
    PrintVector(rfoot2center);
    dmotion::ForKinPlus body(lfoot2center, rfoot2center);
    center2left = body.center2support;
    right2left = body.hang2support;
    ROS_FATAL("center2left");
    PrintVector(center2left);
    ROS_FATAL("right2left");
    PrintVector(right2left);
    parameters.stp.cur_ankle_dis = -right2left[1];
    ROS_FATAL("cur_ankle_dis: ");
    cout << parameters.stp.cur_ankle_dis << endl;

    OneFootLanding Support(false);
    motion_tick tmptick;
    tmptick.time_stamp = 10000000 * 1; //10毫秒
    tmptick.upbody_pose.emplace_back(0);
    tmptick.upbody_pose.emplace_back(0);
    tmptick.upbody_pose.emplace_back(0);
    tmptick.whole_com.emplace_back(parameters.pendulum_walk_param.COM_X_OFFSET); //(x * 100 +1.5);
    tmptick.whole_com.emplace_back(-parameters.stp.cur_ankle_dis / 2.0);
    tmptick.whole_com.emplace_back(parameters.pendulum_walk_param.COM_HEIGHT); //0.308637

    tmptick.hang_foot.emplace_back(0); //(200 * x);
    tmptick.hang_foot.emplace_back(-parameters.stp.cur_ankle_dis);
    tmptick.hang_foot.emplace_back(0);
    tmptick.hang_foot.emplace_back(0);
    tmptick.hang_foot.emplace_back(0);
    tmptick.hang_foot.emplace_back(0);

    whole_end_pos = Support.GetOneStep(tmptick.hang_foot, tmptick.whole_com, tmptick.upbody_pose);

    FinalAdjustServoMatrix = dmotion::ServoTransition(position_aftclimb, whole_end_pos, 40);
    for (unsigned i = 0; i < FinalAdjustServoMatrix.size(); i++)
    {
        ServoPublish(FinalAdjustServoMatrix[i], "WALK", pbr, lopr);
    }
}

} // namespace dmotion
