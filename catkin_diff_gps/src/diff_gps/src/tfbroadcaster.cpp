//
// Created by lhospital on 18-10-28.
// Contributer Liqiangwei
/* 2018年10月28日修改,增加程序初始化流程，首先程序判读是否能够读取IMU和GPS的数据，若能够读取到数据，则进入坐标原点
   初始化流程，程序会判断来自GPS节点的读数，当读数保持不变4秒以上，将数值作为坐标原点，程序进入循环体，向ROS发送MAP与
   BASE_LINk之间的TF关系。若一直没有数据传入，则程序在9秒后自动退出，并显示"NO data from GPS/IMU"
*/
//
//程序功能：接收GPS和IMU数据,创建并发布tf信息

//***************************************************************************************************
//***************************************************************************************************

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>

using namespace message_filters;
using namespace sensor_msgs;
using namespace std;

const double PI = 3.1415926;

class tf_global
{
private:
    ros::NodeHandle n_;

    //话题订阅
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;
    //话题发送
    ros::Publisher tf_ready = n_.advertise<std_msgs::Bool>("tf_ready", 1000);
    int readyOk = 0;                  //存储tf_ready话题是否发布
    tf::TransformBroadcaster tf_pub_; //创建tf广播器

    //起始位置经纬度
    //TODO:写一个初始点经纬度读取函数
    double long_origin = 0;
    double la_origin = 0;
    //目标点经纬度
    double long_target = 0;
    double la_target = 0;
    double x_origin_gauss = 0;
    double y_origin_gauss = 0;
    //旋转矩阵
    geometry_msgs::Quaternion rotation; //接收imu四元数数据
    //判断gps、imu publishing节点是否有数据发出
    bool imuready = false;
    bool gpsready = false;
    bool imugpsready();
    //读取初始经纬度
    void setOrigin(const double longitude, const double latitude);
    //经纬度转高斯坐标
    void GPStoXY_update(double Latitude, double Longitude, double *Gauss_X, double *Gauss_Y);
    //计算相对坐标
    void toRelative(double x_origin, double y_origin, double x_target, double y_target, double &x_relative, double &y_relative);

    //发布tf树
    void publish_tfTree(const double &x_relative, const double &y_relative);

    //初始化坐标函数
    bool init_origin();

    //origin初始化成功
    bool origininitialized = false;
    //callback函数
    void callback_imu(const ImuConstPtr &imu);
    void callback_gps(const NavSatFixConstPtr &gps);

public:
    tf_global();
    ~tf_global(){};
    void run();
};

void tf_global::setOrigin(const double longitude, const double latitude)
{
    this->long_origin = longitude;
    this->la_origin = latitude;
}

void tf_global::GPStoXY_update(double Latitude, double Longitude, double *Gauss_X, double *Gauss_Y)
{
    //参数设定
    double a, b, f, e, e2;
    a = 6378137.0;
    b = 6356752.3142;
    f = 1 / 298.257223563;
    e = sqrt(0.0066943799013);
    e2 = sqrt(0.00673949674227);

    //选定投影带（3度带）
    int n, L0;
    double l;
    n = (Longitude - 1.5) / 3 + 1;
    L0 = n * 3;
    l = (Longitude - L0) / 180.0 * PI;

    //高斯投影
    double A1, A2, A3, A4, A5;
    A1 = a * (1 - pow(e, 2));
    A2 = A1 * pow(e, 2) * 3 / 2;
    A3 = A2 * pow(e, 2) * 5 / 4;
    A4 = A3 * pow(e, 2) * 7 / 6;
    A5 = A4 * pow(e, 2) * 9 / 8;

    double F1, F2, F3, F4, F5;
    F1 = 1.0 * A1 + 1.0 / 2.0 * A2 + 3.0 / 8.0 * A3 + 5.0 / 16.0 * A4 + 35.0 / 128.0 * A5;
    F2 = 1.0 / 2.0 * A2 + 1.0 / 2.0 * A3 + 15.0 / 32.0 * A4 + 7.0 / 16.0 * A5;
    F3 = 1.0 / 8.0 * A3 + 3.0 / 16.0 * A4 + 7.0 / 32.0 * A5;
    F4 = 1.0 / 32.0 * A4 + 1.0 / 16.0 * A5;
    F5 = 1.0 / 128.0 * A5;

    double T, M, N, P;
    double B;
    B = Latitude / 180.0 * PI;
    T = tan(B);
    M = l * cos(B);
    N = a / sqrt(1 - pow(e, 2) * pow(sin(B), 2));
    P = e2 * cos(B);

    double S;
    S = F1 * B - F2 * sin(2.0 * B) * 1 / 2.0 + F3 * sin(4.0 * B) * 1 / 4.0 - F4 * sin(6.0 * B) * 1 / 6.0 + F5 * sin(8.0 * B) * 1 / 8.0;

    double x, y;
    x = S + N * T * (pow(M, 2) * 1 / 2.0 + (5.0 - pow(T, 2) + 9.0 * pow(P, 2) + 4.0 * pow(P, 4)) * pow(M, 4.0) * 1 / 24.0 + (61.0 - 58.0 * pow(T, 2) + pow(T, 4)) * pow(M, 6) * 1 / 720.0);
    y = N * (M + (1.0 - pow(T, 2) + pow(P, 2)) * pow(M, 3) * 1 / 6.0 + (5.0 - 18.0 * pow(T, 2) + pow(T, 4) + 14.0 * pow(P, 2) - 58.0 * pow(T, 2) * pow(P, 2)) * pow(M, 5) * 1 / 120.0);

    //纵轴修正
    y = 1000000 * n + 500000 + y;

    //坐标轴转换
    *Gauss_X = x;
    *Gauss_Y = y;
}

/*
计算当前点与原点的相对坐标，存储于x_relative和y_relative中
*/
void tf_global::toRelative(double x_origin, double y_origin, double x_target, double y_target, double &x_relative,
                           double &y_relative)
{
    x_relative = x_target - x_origin;
    y_relative = y_target - y_origin;
}

/*
发布tf树：车辆坐标系相对于世界坐标系的坐标转换
*/
void tf_global::publish_tfTree(const double &x_relative, const double &y_relative)
{
    ros::Time time;
    time = ros::Time::now(); //记录当前时间

    //得到参数
    tf::Vector3 transition(x_relative, y_relative, 0);  //记录当前车辆与原点的相对坐标，不计高度
    tf::Quaternion rotation_tf;                         //四元数
    tf::quaternionMsgToTF(this->rotation, rotation_tf); //将IMU的旋转信息存储入rotation_tf中

    //tf_pub_发布消息：车辆相对于原点的旋转、坐标，时间
    tf_pub_.sendTransform(
        tf::StampedTransform(
            tf::Transform(rotation_tf, transition),
            time,
            "map",
            "base_link"));
}

/*
接收imu四元数数据，接收成功后，imuready置为true
*/
void tf_global::callback_imu(const ImuConstPtr &imu)
{
    this->rotation = imu->orientation;
    imuready = true;
    return;
}

/*
接收经纬度信息,将gps_ready置为true
若原点设置成功，则计算相对坐标，发布tf
*/
void tf_global::callback_gps(const NavSatFixConstPtr &gps)
{
    //long_target和la_target记录当前车辆经纬度
    this->long_target = (gps->longitude) / 10000000.0;
    this->la_target = (gps->latitude) / 10000000.0;

    //若原点设置成功
    if (origininitialized)
    {
        //计算原点的高斯坐标
        if (this->x_origin_gauss == 0 || this->y_origin_gauss == 0)
        {
            GPStoXY_update(this->la_origin, this->long_origin, &x_origin_gauss, &y_origin_gauss);
        }
        //计算原点与目标点的高斯坐标
        double x_target_gauss, y_target_gauss;

        GPStoXY_update(this->la_target, this->long_target, &x_target_gauss, &y_target_gauss);
        //计算相对坐标，存储x,y中
        double x, y;
        toRelative(x_target_gauss, y_target_gauss, x_origin_gauss, y_origin_gauss, x, y);

        //发布tf
        publish_tfTree(x, y);
    };
    gpsready = true;
    return;
}

void tf_global::run()
{
    //创建发布者tf_ready，发布tf_ready话题，队列长度1000
    tf_ready = n_.advertise<std_msgs::Bool>("tf_ready", 1000, this);
    ros::Rate looprate(20); //循环频率20Hz

    bool ready = init_origin(); //设置原点，返回原点是否设置成功

    //若原点设置成功，在tf_ready话题中发布true，将readyOK置为1
    while (ready == true)
    {
        if (readyOk == 0)
        {
            std_msgs::Bool Ready;
            Ready.data = true;
            tf_ready.publish(Ready);
            readyOk = 1;
        }
        //等待IMU_data和GPS_data话题
        ros::spinOnce();
        looprate.sleep();
    };
    return;
}

tf_global::tf_global()
{
    //订阅IMU_data和GPS_data，队列长度1
    imu_sub = n_.subscribe("IMU_data", 1, &tf_global::callback_imu, this);
    gps_sub = n_.subscribe("GPS_data", 1, &tf_global::callback_gps, this);
}

/*
将车辆5秒内不动的点设为原点
若接收不到imu数据，返回false
若设置成功，输出提示，返回true
*/
bool tf_global::init_origin()
{
    int counter = 0;
    double long_temp = 0, la_temp = 0;                             //存储临时经纬度数据
    ros::Rate looprate(1);                                         //循环频率1Hz
    std::cout << " Initializing origin position ..." << std::endl; //输出提示

    //接收订阅的消息并判断是否接收成功
    do
    {
        ros::spinOnce();
        counter++;
        if (counter > 2)
        {
            return false;
        } //循环三次不成功则返回false
    } while (imugpsready() == false);

    //判断车辆位置是否移动

    counter = 0;

    do
    {
        ros::spinOnce();
        if (abs(long_temp - long_target) > 0.00000000000001 || abs(la_temp - la_target) > 0.00000000000001)
        {
            long_temp = long_target;
            la_temp = la_target;
            counter = 0;
        }
        if (abs(long_temp - long_target) < 0.00000000000001 && abs(la_temp - la_target) < 0.00000000000001)
        {
            counter++;
        }
        looprate.sleep();
    } while (counter < 5);

    setOrigin(long_target, la_target);                                                                           //将origin设为当前值（经纬度）
    origininitialized = true;                                                                                    //初始化成功
    std::cout << " Origin position has been initialized successfully \n Program is entering loop " << std::endl; //输出提示
    return true;
}

bool tf_global::imugpsready()
{
    int counter = 0;
    ros::Rate looprate(1); //循环频率1Hz
    while ((imuready == false) && (gpsready == false))
    {
        ros::spinOnce(); //接受订阅消息
        looprate.sleep();
        counter++;
        if (counter > 2 && imuready == false && gpsready == false)
        {
            return false;
        } //若循环三次不成功，返回false
    };
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_map_To_base_link");
    tf_global tf_global;
    tf_global.run();
    std::cout << "NO data from GPS/IMU" << std::endl;
    return 1;
}
