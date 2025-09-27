#include "Admittance.h"
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Admittance");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    Admittance admittance(nh,0.05,0.01);
    admittance.changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
    arm.setNamedTarget("home");
    arm.move();

    admittance.get_tool_pose();
    Eigen::Vector3d targetPosition(admittance.pose_position_cur[0], admittance.pose_position_cur[1], admittance.pose_position_cur[2]);
    std::cout<<targetPosition.transpose()<<"\n";
    Eigen::Quaterniond targetQuat = admittance.pose_orientation_cur;
    // targetPosition[2] -=0.1; 
    Vector6d targetFtdata;
    targetFtdata.setZero();
    admittance.admittance_mode(targetPosition,targetQuat,targetFtdata);
    return 0;
}
