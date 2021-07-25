#include <ros/ros.h>
#include "linuxserial.h"
#include <geometry_msgs/Vector3Stamped.h>

void VelocitydataCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    ROS_INFO("%f,%f,%f",msg->vector.x,msg->vector.y,msg->vector.z);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_velocitypub");

    ros::NodeHandle n;

    ros::Subscriber imudata_sub=n.subscribe("Velocity_data",10,VelocitydataCallback);

    ros::spin();

    return 0;
}