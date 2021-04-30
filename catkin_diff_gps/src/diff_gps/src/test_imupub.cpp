#include <ros/ros.h>
#include "linuxserial.h"
#include <sensor_msgs/Imu.h>

void IMUdataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO("%s",msg->header.frame_id.c_str());
    ROS_INFO("%f,%f,%f",msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_imupub");

    ros::NodeHandle n;

    ros::Subscriber imudata_sub=n.subscribe("IMU_data",10,IMUdataCallback);

    ros::spin();

    return 0;
}