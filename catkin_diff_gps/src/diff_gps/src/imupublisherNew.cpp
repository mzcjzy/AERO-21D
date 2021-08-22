#include <ros/ros.h>
#include "linuxserial.h"
#include <iostream>
#include <string>
#include <queue>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

//宏定义
#define FALSE -1
#define TRUE 0

struct GTIMU
{
    unsigned int GPSweek;
    unsigned int GPStime;
    double GyroX;
    double GyroY;
    double Gyroz;
    double AccX;
    double AccY;
    double AccZ;
    float Yaw;
    float Pitch;
    float Roll;
    int latitude;
    int longitude;
    int altitude;
    float Ve;
    float Va;
    float Vu;
};

void bufproc(const unsigned char *revbuff, int lenth);
void IMU_datatran(sensor_msgs::Imu &temimu, GTIMU const temimudata, ros::Time Time);
void GPS_datatran(sensor_msgs::NavSatFix &tmpGps, GTIMU const temimudata, ros::Time const Time);
void Vel_datatran(geometry_msgs::Vector3Stamped tmpVel, GTIMU const temimudata, ros::Time const Time);

unsigned char tembuff[1000] = {};
GTIMU IMUinfo;
sensor_msgs::Imu imu;
sensor_msgs::NavSatFix gps;
geometry_msgs::Vector3Stamped velocity;
roserialhandler roserial1("/dev/ttyUSB0", 230400);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, ""); //可输出中文

    //variables for port1
    int fd1;  //文件描述符
    int err1; //返回调用函数的状态
    int len1;
    unsigned char rcv_buf_1[2000] = {};

    int counter = 0;

    //init ros
    ros::init(argc, argv, "IMUstatepublisherNew");
    ros::NodeHandle handle;
    ros::Publisher IMU_pub = handle.advertise<sensor_msgs::Imu>("IMU_data", 1000);
    ros::Publisher GPS_pub = handle.advertise<sensor_msgs::NavSatFix>("GPS_data", 1000);
    ros::Publisher Velocity_pub = handle.advertise<geometry_msgs::Vector3Stamped>("Velocity_data", 1000);

    fd1 = roserial1.UART0_Open();

    do
    {
        err1 = roserial1.UART0_Init(0, 8, 1, 'N');
        printf("Set Port Exactly!\n");
        sleep(2);
    } while (FALSE == err1 || FALSE == fd1);

    printf("IMU_publisher starts successfully\n");

    //loop body
    while (ros::ok())
    {
        len1 = roserial1.UART0_Recv(rcv_buf_1, 2000);
        cout << "len1=" << len1 << endl;
        if (len1 > 0)
        {
            bufproc(rcv_buf_1, len1);
        }
        else
        {
            printf("/dev/ttyUSB0 cannot receive data\n");
        }

        ros::Time nowTime = ros::Time::now();
        IMU_datatran(imu, IMUinfo, nowTime);
        GPS_datatran(gps, IMUinfo, nowTime);
        Vel_datatran(velocity, IMUinfo, nowTime);

        // TODO:值的筛选

        IMU_pub.publish(imu);
        GPS_pub.publish(gps);
        Velocity_pub.publish(velocity);

        cout << "counter is " << counter << endl;
        counter++;

        usleep(100);
    }
}

void bufproc(const unsigned char *revbuff, int lenth)
{
    int startflag = 0;
    int startindex = 0;
    int datatype;
    unsigned char procbuff[4000] = {};

    if (sizeof(tembuff) > 0)
    {
        int tem_lenth = sizeof(tembuff);
        lenth += tem_lenth;
        for (int i = 0; i < tem_lenth; i++)
        {
            procbuff[i] = tembuff[i];
        }
        for (int j = 0; j < lenth; j++)
        {
            procbuff[j + tem_lenth] = revbuff[j];
        }
        memset(tembuff, '\0', sizeof(tembuff)); //将tembuff置空
    }
    else
    {
        for (int j = 0; j < lenth; j++)
        {
            procbuff[j] = revbuff[j];
            //cout<<" procbuff["<<j<<"]="<< int ( procbuff[j] );
        }
    }

    //遍历procbuff数组，处理数据
    for (int i = 0; i < lenth; i++)
    {
        if (i < (lenth - 2) && (procbuff[i] == 0xBD) && (procbuff[i + 1] == 0xDB) && (procbuff[i + 2] == 0x0B))
        {
            startflag = 1;
            startindex = i;
            cout<<"find start index"<<endl;
        }
        else
        {
            continue;
        }

        if ((lenth - startindex > 61) && (startflag == 1))
        {

            /*异或校验*/
            unsigned char auChecksum1;
            auChecksum1 = roserial1.wubJTT808CalculateChecksum(procbuff + startindex, 57);

            unsigned char auChecksum2;
            auChecksum2 = roserial1.wubJTT808CalculateChecksum(procbuff + startindex, 62);

            if (auChecksum1 == procbuff[startindex + 57] && auChecksum2 == procbuff[startindex + 62])
            {
                IMUinfo.GPSweek = *(unsigned int *)(&(procbuff[startindex + 58]));
                IMUinfo.GPStime = *(unsigned int *)(&(procbuff[startindex + 52]));
                short int Yaw = *(short int *)(&(procbuff[startindex + 7]));
                IMUinfo.Yaw = (float)Yaw * (float)360 / (float)32768;
                short int pitch = *(short int *)(&(procbuff[startindex + 5]));
                IMUinfo.Pitch = (float)pitch * (float)360 / (float)32768;
                short int Roll = *(short int *)(&(procbuff[startindex + 3]));
                IMUinfo.Roll = (float)Roll * (float)360 / (float)32768;
                IMUinfo.latitude = *(int *)(&(procbuff[startindex + 21]));
                IMUinfo.longitude = *(int *)(&(procbuff[startindex + 25]));
                IMUinfo.altitude = *(int *)(&(procbuff[startindex + 29]));
                short int Ve = *(short int *)(&procbuff[startindex + 35]);
                IMUinfo.Ve = (float)Ve * (float)100 / (float)32768;
                short int Va = *(short int *)(&procbuff[startindex + 33]);
                IMUinfo.Va = (float)Va * (float)100 / (float)32768;
                short int Vu = *(short int *)(&procbuff[startindex + 37]);
                IMUinfo.Vu = -(float)Vu * (float)100 / (float)32768;
                short int GyroX = *(short int *)(&(procbuff[startindex + 11]));
                IMUinfo.GyroX = (double)GyroX * (double)300 / (double)32768;
                short int GyroY = *(short int *)(&(procbuff[startindex + 9]));
                IMUinfo.GyroY = (double)GyroY * (double)300 / (double)32768;
                short int GyroZ = *(short int *)(&(procbuff[startindex + 13]));
                IMUinfo.Gyroz = -(double)GyroZ * (double)300 / (double)32768;
                short int AccX = *(short int *)(&(procbuff[startindex + 17]));
                IMUinfo.AccX = (double)AccX * (double)12 / (double)32768;
                short int AccY = *(short int *)(&(procbuff[startindex + 15]));
                IMUinfo.AccY = (double)AccY * (double)12 / (double)32768;
                short int AccZ = *(short int *)(&(procbuff[startindex + 19]));
                IMUinfo.AccZ = -(double)AccZ * (double)12 / (double)32768;

                cout << "IMUinfo.GPSweek = " << IMUinfo.GPSweek << endl;
                cout << "IMUinfo.GPStime = " << IMUinfo.GPStime << endl;
                cout << " IMUinfo.GyroX = " << IMUinfo.GyroX << endl;
                cout << " IMUinfo.GyroY = " << IMUinfo.GyroY << endl;
                cout << "IMUinfo.GyroZ = " << IMUinfo.Gyroz << endl;
                cout << "IMUinfo.AccX = " << IMUinfo.AccX << endl;
                cout << "  IMUinfo.AccY  = " << IMUinfo.AccY << endl;
                cout << " IMUinfo.AccZ = " << IMUinfo.AccZ << endl;
                cout << " IMUinfo.Yaw = " << IMUinfo.Yaw << endl;
                cout << " IMUinfo.Pitch = " << IMUinfo.Pitch << endl;
                cout << "IMUinfo.Roll = " << IMUinfo.Roll << endl;
                cout << " IMUinfo.latitude= " << IMUinfo.latitude << endl;
                cout << " IMUinfo.longitude = " << IMUinfo.longitude << endl;
                cout << "IMUinfo.altitude  = " << IMUinfo.altitude << endl;
                cout << " IMUinfo.Ve= " << IMUinfo.Ve << endl;
                cout << " IMUinfo.Va= " << IMUinfo.Va << endl;
                cout << " IMUinfo.Vu= " << IMUinfo.Vu << endl;
                //printf("Vu=%d",Vu);
                cout << " IMUinfo.GyroX = " << IMUinfo.GyroX << endl;
                cout << " IMUinfo.GyroY = " << IMUinfo.GyroY << endl;
                cout << "IMUinfo.GyroZ = " << IMUinfo.Gyroz << endl;
                cout << "IMUinfo.AccX = " << IMUinfo.AccX << endl;
                cout << "  IMUinfo.AccY  = " << IMUinfo.AccY << endl;
                cout << " IMUinfo.AccZ = " << IMUinfo.AccZ << endl;
            }

            else
            {
                cout << "数据丢失" << endl;
            }

            startflag = 0;
            startindex = 0;
        }
        else if (lenth - startindex <= 61 && startindex == 1)
        {
            for (int j = 0; j <= lenth - startindex; j++)
            {
                tembuff[j] = procbuff[startindex + j];
            }
            return;
        }
        else if (i == lenth - 1 && startindex == 0)
        {
            for (int j = 0; j <= lenth; j++)
            {
                tembuff[i] = procbuff[i];
            }
        }
        else
        {
            continue;
        }
    }
    procbuff[2000] = {'+'};
    // startflag = 0;
    // startindex = 0;
    // lenthoftembuff = 0;
    return;
}

void IMU_datatran(sensor_msgs::Imu &temimu, GTIMU const temimudata, ros::Time const Time)
{
    //cout<<"enter imu_datatran"<<endl;
    tf::Quaternion q = tf::createQuaternionFromRPY((temimudata.Roll * 3.1415926 / (float)180), (temimudata.Pitch * 3.1415926 / (float)180), (temimudata.Yaw * (float)3.1415926 / 180));

    //cout<<"q.x()="<<q.x()<<endl;
    temimu.header.stamp = Time;
    temimu.header.frame_id = "base_link";
    temimu.orientation.x = q.x();
    temimu.orientation.y = q.y();
    temimu.orientation.z = q.z();
    temimu.orientation.w = q.w();
    temimu.linear_acceleration.x = temimudata.AccX;
    temimu.linear_acceleration.y = temimudata.AccY;
    temimu.linear_acceleration.z = temimudata.AccZ;
    temimu.angular_velocity.x = temimudata.GyroX;
    temimu.angular_velocity.y = temimudata.GyroY;
    temimu.angular_velocity.z = temimudata.Gyroz;
    ROS_INFO("Send the IMU: [%f, %f, %f,%f, \n %f, %f, %f, \n %f, %f, %f]",
             temimu.orientation.x,
             temimu.orientation.y,
             temimu.orientation.z,
             temimu.orientation.w,
             temimu.linear_acceleration.x,
             temimu.linear_acceleration.y,
             temimu.linear_acceleration.z,
             temimu.angular_velocity.x,
             temimu.angular_velocity.y,
             temimu.angular_velocity.z);
    return;
}

void GPS_datatran(sensor_msgs::NavSatFix &tmpGps, GTIMU const temimudata, ros::Time const Time)
{
    //cout<<"enter GPS_datatran"<<endl;
    if (temimudata.longitude == 0 ||
        temimudata.latitude == 0 ||
        temimudata.altitude == 0 ||
        temimudata.longitude > 180 * 10000000.0 ||
        temimudata.latitude > 90 * 10000000.0 ||
        temimudata.altitude > 100 * 1000.0)
    {
        return;
    }

    tmpGps.header.stamp = Time;
    tmpGps.longitude = temimudata.longitude;
    tmpGps.latitude = temimudata.latitude;
    tmpGps.altitude = temimudata.altitude;

    ROS_INFO("Send the GPS location: [%f, %f, %f]",
             tmpGps.longitude / 10000000.0, tmpGps.latitude / 10000000.0, tmpGps.altitude / 1000.0);

    return;
}

void Vel_datatran(geometry_msgs::Vector3Stamped tmpVel, GTIMU const temimudata, ros::Time const Time)
{
    //cout<<"enter Vel_datatran"<<endl;
    tmpVel.header.stamp = Time;

    if (abs(temimudata.Ve) >= 150 && abs(temimudata.Va) >= 150 && abs(temimudata.Vu) >= 150)

    {
        return;
    }
    tmpVel.vector.x = temimudata.Ve; //东向
    tmpVel.vector.y = temimudata.Va; //北向
    tmpVel.vector.z = temimudata.Vu; //天向

    ROS_INFO("Send the velocity: [%f, %f, %f]", tmpVel.vector.x, tmpVel.vector.y, tmpVel.vector.z);

    return;
}