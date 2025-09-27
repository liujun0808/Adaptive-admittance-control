#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class new_lpfilter
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_ft;
    ros::Publisher pub_ft_filter;
    geometry_msgs::WrenchStamped ft_filter;

public:
    new_lpfilter(ros::NodeHandle &nh):nh_(nh)
    {
        sub_ft = nh.subscribe("transformed_world",1,&new_lpfilter::filterProcess,this);
        pub_ft_filter = nh.advertise<geometry_msgs::WrenchStamped>("transformed_world_filter",1);
    }
    
    double forceSensor_data[6]; // 存放六维力的数据
    double lastFilter_data[6];  // 上一周期的滤波数据
    double filter_data[6];  // 这周期滤波后的数据
    double filter_k = 0.01;

    void filterProcess(const geometry_msgs::WrenchStampedConstPtr &msg)
    {
        forceSensor_data[0] = msg->wrench.force.x;
        forceSensor_data[1] = msg->wrench.force.y;
        forceSensor_data[2] = msg->wrench.force.z;
        forceSensor_data[3] = msg->wrench.torque.x;
        forceSensor_data[4] = msg->wrench.torque.y;
        forceSensor_data[5] = msg->wrench.torque.z;

        for (size_t i = 0; i < 6; i++)
        {
            filter_data[i] = filter_k * forceSensor_data[i] + (1-filter_k) * lastFilter_data[i];

            lastFilter_data[i] = filter_data[i];
        }
        
        ft_filter.header.stamp = ros::Time(msg->header.stamp.toSec());

        ft_filter.wrench.force.x = filter_data[0];
        ft_filter.wrench.force.y = filter_data[1];
        ft_filter.wrench.force.z = filter_data[2];
        ft_filter.wrench.torque.x = filter_data[3];
        ft_filter.wrench.torque.y = filter_data[4];
        ft_filter.wrench.torque.z = filter_data[5];
        pub_ft_filter.publish(ft_filter);

    }

};

int main(int argc, char  **argv)
{
    ros::init(argc,argv,"ft_filter");
    ros::NodeHandle nh;
    new_lpfilter lpfilter(nh);
    ros::spin();

    return 0;
}





