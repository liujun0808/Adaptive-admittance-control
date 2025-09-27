#include "Admittance.h"

Admittance::Admittance(ros::NodeHandle &n, 
                      double arm_max_vel,
                      double arm_max_acc
                       ):loop_rate_(125),nh_(n),tool_max_acc_(arm_max_acc),tool_max_vel_(arm_max_vel)
                       {
    toolVel_pub = nh_.advertise<geometry_msgs::TwistStamped>("servo_server/delta_twist_cmds",1);
    ft_sub = nh_.subscribe("transformed_world_filter",100,&Admittance::callBack_ftData,this);
    ft_bias_client = nh_.serviceClient<netft_utils::SetBias>("/bias");  // ft置0 300.0, 300.0, 300.0, 300.0, 300.0, 300.0;
    Vector6d m_diag, d_diag, k_diag;
    m_diag << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
    d_diag << 30.0, 30.0, 30.0, 30.0, 30.0, 30.0;
    k_diag << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    M_ = m_diag.asDiagonal();
    D_ = d_diag.asDiagonal();
    K_ = k_diag.asDiagonal();
    setBias_client.request.toBias = true;
    setBias_client.request.forceMax = 200.0;
    setBias_client.request.torqueMax = 100.0;
    error.setZero();
    tool_vel_ref.setZero();
    tool_acc_ref.setZero();
}

void Admittance::admittance_mode(const Eigen::Vector3d &pose_des,const Eigen::Quaterniond &rot_des,const Vector6d &ft_des)
{   
     
    ft_bias_client.call(setBias_client);
    ros::Duration(1).sleep();
    ros::Duration duration = loop_rate_.expectedCycleTime(); //得到循环一次的时间 （eg：1s执行125次，即125Hz，得到每次的执行时间0.008s）
    changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller");
    while (ros::ok())
    {   
        if (!get_tool_pose()){std::cout<<"获取末端位姿失败！"<<"\n";break;}

        error.segment(0,3) = pose_position_cur - pose_des.segment(0,3);
        if (rot_des.coeffs().dot(pose_orientation_cur.coeffs()) < 0) 
        {
            pose_orientation_cur.coeffs() = - pose_orientation_cur.coeffs();
        }
        // 计算姿态误差
        Eigen::Quaterniond rot_err_temp = pose_orientation_cur * rot_des.inverse();
        if (rot_err_temp.norm() > 1e-3){rot_err_temp.normalize();} // 归一化
        Eigen::AngleAxisd err_AngleAxisd(rot_err_temp); // 转为单位旋转轴和旋转角度
        error.segment(3,3)<< err_AngleAxisd.axis() * err_AngleAxisd.angle();//  分别计算旋转轴的xyz分量乘以旋转角度

        Vector6d coupling_wrench_arm = D_ * (tool_vel_ref) + K_ * error;
        tool_acc_ref =  M_.inverse() * (ft_data - ft_des - coupling_wrench_arm);

        double acc_norm = (tool_acc_ref.segment(0,3)).norm(); // 提起前三个元素构成一个三维向量 计算其范数 （位置加速度的范数）
        if (acc_norm > tool_max_acc_) //加速度限幅
        {
            ROS_WARN_STREAM_THROTTLE(
                1,"Admittance generate high arm accelaration!"
                <<"norm"<<acc_norm;
            );
            tool_acc_ref.segment(0,3) *= (tool_max_acc_/acc_norm); // 放缩至最大量程内
        }
        tool_vel_ref += tool_acc_ref * duration.toSec(); // 积分计算速度
        double vel_norm = (tool_vel_ref.segment(0,3)).norm(); 
        if (vel_norm > tool_max_vel_) // 速度限幅
        {
            ROS_WARN_STREAM_THROTTLE(
                1,"Admittance generate high arm accelaration!"
                <<"norm"<<vel_norm;
            );
            tool_vel_ref.segment(0,3) *= (tool_max_vel_/vel_norm); // 放缩至最大量程内
        }
        toolVec_msg.header.stamp = ros::Time::now();
        toolVec_msg.twist.linear.x = tool_vel_ref[0];
        toolVec_msg.twist.linear.y = tool_vel_ref[1];
        toolVec_msg.twist.linear.z = tool_vel_ref[2];
        toolVec_msg.twist.angular.x = tool_vel_ref[3];
        toolVec_msg.twist.angular.y = tool_vel_ref[4];
        toolVec_msg.twist.angular.z = tool_vel_ref[5];
        toolVel_pub.publish(toolVec_msg);
        ROS_INFO_THROTTLE(2.0, "Admittancing....,%5f;%5f;%5f;",tool_vel_ref[3],tool_vel_ref[4],tool_vel_ref[5]);
        ros::spinOnce();
        loop_rate_.sleep();
    }
    toolVec_msg.header.stamp = ros::Time::now();
    toolVec_msg.twist.linear = geometry_msgs::Vector3();
    toolVec_msg.twist.angular = geometry_msgs::Vector3();
    toolVel_pub.publish(toolVec_msg);
    changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
}

void Admittance::changeController(std::string start, std::string stop){
  ros::AsyncSpinner spinner(1);  
  controller_manager_msgs::SwitchController Controller;
  Controller.request.start_controllers.push_back(start);
  Controller.request.stop_controllers.push_back(stop);
  Controller.request.strictness = 2;
  ros::service::call("/controller_manager/switch_controller",Controller);  
}


bool Admittance::get_tool_pose(){
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("base","tool0",ros::Time(0),transform);
        tf::Quaternion tool_to_base_qua = transform.getRotation();
        tf::Vector3 position = transform.getOrigin();
        pose_orientation_cur.w()  = tool_to_base_qua.getW();pose_orientation_cur.x()  = tool_to_base_qua.getX();
        pose_orientation_cur.y()  = tool_to_base_qua.getY();pose_orientation_cur.z()  = tool_to_base_qua.getZ();
        pose_position_cur<<position.getX(),position.getY(), position.getZ();
    }
    catch(tf::TransformException ex)
    {
        ROS_WARN_STREAM_THROTTLE(1,"Wating for tf from: "<< " base" << " to" << " tool0");// 具有节流功能的警告信息输出，1为两次输出的时间间隔
        return false; 
    }
    return true;

}

void Admittance::callBack_ftData(const geometry_msgs::WrenchStampedConstPtr &msg){
    ft_data[0] =  - msg->wrench.force.x;ft_data[1] =  msg->wrench.force.y;ft_data[2] =  msg->wrench.force.z;
    ft_data[3] = - msg->wrench.torque.x;ft_data[4] =  msg->wrench.torque.y;ft_data[5] =  msg->wrench.torque.z;
}
