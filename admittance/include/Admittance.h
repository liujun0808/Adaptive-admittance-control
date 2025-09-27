#ifndef ADMITTANCE_H
#define ADMITTANCE_H
// controller_manager_msgs::SwitchController 
#include "ros/ros.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "netft_utils/SetBias.h"
#include "controller_manager_msgs/SwitchController.h"

#include <memory>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <cmath>




typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class Admittance
{
protected:
  // ROS VARIABLES:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  //ros user
  ros::Subscriber ft_sub;
  ros::Publisher toolVel_pub;
  ros::ServiceClient ft_bias_client; // ft传感器偏置客户端
  netft_utils::SetBias set_bias;
  // msg
  geometry_msgs::TwistStamped toolVec_msg;

  // ADMITTANCE PARAMETERS:
  Matrix6d M_, D_, K_;
  // Variables:
  Eigen::Vector3d      arm_position_;
  Eigen::Quaterniond   arm_orientation_;
  Vector6d     ft_data;
  Vector6d      tool_vel_ref;
  Vector6d      tool_acc_ref;


  Vector6d      error;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  double tool_max_acc_,tool_max_vel_;
  netft_utils::SetBias setBias_client;



public:
  Admittance(ros::NodeHandle &n, 
                      double arm_max_vel,
                      double arm_max_acc
                       );
  ~Admittance(){}
  void admittance_mode(const Eigen::Vector3d &pose_des,const Eigen::Quaterniond &rot_des,const Vector6d &ft_des);//导纳模式
  void admittance_teach_mode();//示教模式
  void adaptiveAdmittance_mode();//自适应变导纳
  bool get_tool_pose();
  void callBack_ftData(const geometry_msgs::WrenchStampedConstPtr &msg);
  void changeController(std::string start, std::string stop);

  Eigen::Vector3d      pose_position_cur;
  Eigen::Quaterniond   pose_orientation_cur;
  // 测试接口

private:

};



#endif