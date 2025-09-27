#ifndef SEARCH_HOLE_H
#define SEARCH_HOLE_H
#include <ros/ros.h>
#include "yolov8_ros/workpieces.h" // 工件消息类型
#include "yolov8_ros/objectArray.h" // 工件组消息
#include "yolov8_ros/grasp_flag.h"  // 抓取标志位
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <netft_utils/SetBias.h>
#include "yolov8_ros/center.h"
#include "yolov8_ros/ibvs.h"
#include <python3.8/Python.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/WrenchStamped.h"
#include <controller_manager_msgs/SwitchController.h>
#include "std_msgs/Float64MultiArray.h"
//MoveIt
#include <moveit_msgs/ExecuteTrajectoryAction.h>
// For chaning speed in computeCartesianPath
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
// 显示时间
#include <ctime>
#include <iomanip> 
#include "cartesian_algorithms/admittance/ibvs.h"
#include <visp3/core/vpCameraParameters.h>


using namespace Eigen;

typedef Matrix<double,3,1> vector3d;
typedef Matrix<double,4,4> Matrix4d;
typedef Matrix<double,3,3> Matrix3d;

class reinforcement
{
private:
    tf::TransformListener listener;
    yolov8_ros::objectArray node,ver,hor; // 各类工件容器
    ros::NodeHandle nh_;
    vector3d workp_in_pixel; // 工件在像素坐标系
    vector3d workp_in_camera; // 工件在相机坐标系
    vector3d workp_in_tool; // 工件在工具坐标系
    vector3d workp_in_base_link; // 工件在基坐标系
    Matrix4d eye_to_hand; // 手眼变换矩阵

    Matrix3d cam_k; //内参矩阵
    Matrix3d tool_to_base_link; // TCP到baselink的旋转矩阵
    vector3d tool_in_base_link; // TCP到baselink的位置
    yolov8_ros::grasp_flag grasp_flag_msg; //抓取标志发布信息
    ros::Publisher grasp_flag_pub; // 抓取标志发布者
    ros::Publisher admit_vel_pub; 
    ros::Subscriber yolo_sub_state; // yolo消息订阅者
    ros::Subscriber sub_wrench_state_; // ft消息订阅者  
    ros::ServiceClient ft_bias_client; // ft传感器偏置客户端
    ros::ServiceClient client_sec_img; // 二次拍照请求客户端
    ros::ServiceClient ibvs_srv; // ibvs请求客户端
    ros::Subscriber ibvs_data_sub; // ft消息订阅者
    moveit::planning_interface::MoveGroupInterface arm;
    tf::Quaternion tool_to_base_qua;
    vector3d wrench_ft_frame;
    Matrix3d M_,D_,K_; // 导纳参数
    ros::Rate loop_rate; // 循环频率 用于积分计算
    vector3d first_node; // 第一层首个node放置的坐标
    vector3d vertical[4]; // 4个ver的放置后的坐标
    vector3d seconde_node; // 第二层首个node放置的坐标
    ros::Subscriber hole_find_movestatus; // 订阅moveit状态信息
    moveit_msgs::ExecuteTrajectoryActionFeedback mvgr_status; // moveit状态信息
    int flag; // 是否需要搜孔的标志位 1需要；0不需要
    std::vector<double> ibvs_data; //数据格式为[x1,y1,x2,y1,x1,y2,x2,y2,z1,z2,z3,z4] 分别表示四个像素点的坐标

    ros::Subscriber sub_joint_state; // 订阅关节状态
    std::vector<double> curr_joint_state;
    // 测试
    std::vector<double> test_desired_pose_; // 测试的期望位姿
    ros::Publisher tcp_pose; // 机械臂末端位姿
    ros::Publisher ser_vel; // 机械臂末端位姿
    std::time_t now;// 获取当前时间（秒）
    ibvs ibvs_node;
    bool ibvs_start;
    std::vector<vpImagePoint> corners_pd_node1,corners_pd_node2;

public:
    reinforcement(ros::NodeHandle &n); // 构造
    ~reinforcement(); // 析构
    netft_utils::SetBias setBias_client;


    // 夹爪控制函数
    void gripper_connect();
    void gripper_close();
    void gripper_open();

    // 回调函数
    void CB_get_workpieces(const yolov8_ros::objectArrayConstPtr msg); // 回调函数获取工件信息
    void state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg); // 六维力回调
    void callback_status(const moveit_msgs::ExecuteTrajectoryActionFeedbackConstPtr& msg) {mvgr_status = *msg;} // 获取moveit状态信息
    void ibvs_callback(const std_msgs::Float64MultiArrayConstPtr& msg);
//     void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) { curr_joint_state = msg->position;
// } // 关节状态回调函数 
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) { curr_joint_state = msg->position;std::swap(curr_joint_state[0],curr_joint_state[2]);
}

    // 功能函数
    static bool compare_by_center_x(const yolov8_ros::workpieces& obj1,const yolov8_ros::workpieces& obj2); // 获得工件的X像素坐标，用以排序
    void camera_to_base(vector3d p_); // 从像素坐标系转到base link下
    bool get_rotation_matrix(Matrix3d & rotation_matrix,vector3d & position_,tf::TransformListener & listener,std::string from_frame,  std::string to_frame);
    void moveCartesian(const std::vector<geometry_msgs::Pose>& waypoints,const double& max_v); // 笛卡尔路径 可设置最大速度
    void moveCartesian(const std::vector<geometry_msgs::Pose>& waypoints); // 笛卡尔路径
    void admittance(vector3d &ft_dir,const double &threshold_z);// 导纳控制
    void changeController(std::string start, std::string stop);
    void rpy(double r,int i);
    double touch_hole(Eigen::Vector3d &delta_xyz,double velocity);
    void palce_hori(int index);
    void fixtrajectorytime(moveit_msgs::RobotTrajectory &trajectory); //修改笛卡尔轨迹
    // void get_hole_pos(double &y);
    bool space_arc_ver(geometry_msgs::Pose &target_pose,double &r); // 空间圆弧
    geometry_msgs::Quaternion rotation_grasp(double rz,int cls); // 旋转抓取 
    void movejoint6(double rad);
    void touch_ver();// 接触竖杆
    bool space_arc_node(geometry_msgs::Pose &target_pose);
    void nodeTouch_concret();// first node touch concrete
    void CartesianVelCtrl(const Eigen::Vector3d& delta_xyz,const double & velocity);
    void space_arc_ver(int axis_);

    // 任务函数
    void run(); // 执行函数
    void first_layer_node(); // 第一层节点
    void first_layer_hori(); // 第一层水平工件
    void first_layer_ver(); // 第一层竖直工件
    void seconde_layer_node(); // 第二层节点
    void seconde_layer_hori(); // 第二层水平筋



};

#endif