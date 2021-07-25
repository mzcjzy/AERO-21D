#include <ros/ros.h>
#include "linuxserial.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

void GPSdataCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    ROS_INFO("%f,%f,%f",msg->latitude,msg->longitude,msg->altitude);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_gpspub");

    ros::NodeHandle n;

    ros::Subscriber imudata_sub=n.subscribe("GPS_data",10,GPSdataCallback);

    ros::spin();

    return 0;
}