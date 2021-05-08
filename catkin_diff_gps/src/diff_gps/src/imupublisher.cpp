/*   programmed by Liqiangwei
     modified on 2018.8.30
This node is used to publish the date collected from the xwyd GI5651 sensor to ROS master.
usage:1.identify the port you used
      2.change the portname 
      3.change the Baudrate
      4.the looprate is set in line 71   */    

#include <ros/ros.h>
#include "linuxserial.h"
#include <iostream>
#include <queue>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

//宏定义
#define FALSE  -1
#define TRUE   0

struct GTIMU{
    unsigned short int GPSweek;
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
    unsigned int latitude;
    unsigned int longitude;
    unsigned int altitude;
    float Ve;
    float Va;
    float Vu;
};


queue<unsigned char> tembuff_1;

void bufproc(const unsigned char *revbuff, int lenth, queue<unsigned char> tembuff, int datatype);
void IMU_datatran(sensor_msgs::Imu &temimu, GTIMU const temimudata, ros::Time Time);
void GPS_datatran(sensor_msgs::NavSatFix &tmpGps, GTIMU const temimudata, ros::Time const Time);
void Vel_datatran(geometry_msgs::Vector3Stamped tmpVel, GTIMU const temimudata, ros::Time const Time);


GTIMU IMUinfo;
sensor_msgs::Imu imu;
sensor_msgs::NavSatFix gps;
geometry_msgs::Vector3Stamped velocity;

int main(int argc,char *argv[]){
    
    //variables for port1
    int fd1;                            //文件描述符
    int err1;                           //返回调用函数的状态
    int len1;
    unsigned char rcv_buf_1[2000] = {};

    int counter=0;
    
    //init ros
    ros::init(argc, argv, "IMUstatepublisher");
    ros::NodeHandle handle;
    ros::Publisher IMU_pub = handle.advertise<sensor_msgs::Imu>("IMU_data", 1000);
    ros::Publisher GPS_pub = handle.advertise<sensor_msgs::NavSatFix>("GPS_data", 1000);
    ros::Publisher Velocity_pub = handle.advertise<geometry_msgs::Vector3Stamped>("Velocity_data", 1000);


    roserialhandler roserial1("/dev/ttyUSB0",115200);
    fd1 = roserial1.UART0_Open();

    do{
        err1 = roserial1.UART0_Init(0,8,1,'N');
        printf("Set Port Exactly!\n");
        sleep(2);
    }while(FALSE == err1 || FALSE == fd1);

    printf("IMU_publisher starts successfully\n");

    //loop body
    while (ros::ok()){
        len1 = roserial1.UART0_Recv( rcv_buf_1,2000);
        cout<<"len1="<<len1<<endl;
        if(len1 > 0){
            bufproc(rcv_buf_1,len1,tembuff_1,1);
        }
        else{printf("/dev/ttyUSB0 cannot receive data\n");}

        ros::Time nowTime = ros::Time::now();
        IMU_datatran(imu, IMUinfo, nowTime);
        GPS_datatran(gps, IMUinfo, nowTime);
        Vel_datatran(velocity, IMUinfo, nowTime);

        // TODO:值的筛选


        IMU_pub.publish(imu);
        GPS_pub.publish(gps);
        Velocity_pub.publish(velocity);
        
        cout<<"counter is "<<counter<<endl;
        counter++;

        usleep(1000);
    }
}


void bufproc(const unsigned char *revbuff,int lenth,queue<unsigned char> tembuff,int datatype){
    //cout<<"enter bufproc"<<endl;
    int startflag = 0;
    int startindex =0;
    int pushflag =0;
    int lenthoftembuff = tembuff.size();
    unsigned char procbuff[2000] = {};

    //如果tembuff中有数据，将数据转移给procbuff，再把revbuff中的数给procbuff
    if(lenthoftembuff>0){
        for(int i=0;i < lenthoftembuff;i++){
            procbuff[i] = tembuff.front();
            tembuff.pop();
        }
        for(int j = 0;j <lenth;j++){
            procbuff[lenthoftembuff+1+j] = revbuff[j];
        }
    }
    //如果tembuff中没有数据，直接将revbuff中的数给procbuff
    else if(lenthoftembuff == 0){
        for(int j = 0;j <lenth;j++){
            procbuff[j] = revbuff[j];
        }
    }
    //遍历procbuff数组，处理数据
    for(int i = 0;i<(lenthoftembuff + lenth);i++)
    {
       // cout<<"enter loop:find start index"<<endl;

        if (int(procbuff[i]) == 170)/*aa*/
        {
            if(i<(lenthoftembuff + lenth-1))
            {
                if(int(procbuff[i+1]) == 85)/*55*/
                {
                    if (i<(lenthoftembuff + lenth-2))
                    {
                        if (int(procbuff[i+2]) == 5)
                            {
                                startflag = 1;
                                startindex = i;
                            }
                    }
                    else if (i==(lenthoftembuff + lenth-2))
                    {
                        tembuff.push(procbuff[i]);
                        tembuff.push(procbuff[i+1]);
                        pushflag =1;
                    }
                   
                }
                    
            }
           else  if(i == (lenthoftembuff + lenth-1) ){
                tembuff.push(procbuff[i]);
                pushflag =1;
            }
        }
        if ((lenth+lenthoftembuff - startindex <= 112) && (startflag == 1)&&(pushflag == 0))//数组长度不足且尚未存到tembuff
        {
            for(int j = startindex;j<=lenth;j++){
                tembuff.push(procbuff[j]);
            }
           
        }
        else if ((lenth+lenthoftembuff - startindex > 112) && (startflag == 1)&&(pushflag == 0 ))
        {
            cout<<"I am IMU1"<<endl;
            IMUinfo.GPSweek= *(unsigned short int*) (&(procbuff[startindex+3+0]));
            IMUinfo.GPStime= *(unsigned int*) (&(procbuff[startindex+3+2]));
            IMUinfo.GyroX = *(double *) (&(procbuff[startindex+3+6]));
            IMUinfo.GyroY = *(double *) (&(procbuff[startindex+3+14]));
            IMUinfo.Gyroz = *(double *) (&(procbuff[startindex+3+22]));
            IMUinfo.AccX = *(double *) (&(procbuff[startindex+3+30]));
            IMUinfo.AccY = *(double *) (&(procbuff[startindex+3+38]));
            IMUinfo.AccZ = *(double *) (&(procbuff[startindex+3+46]));
            IMUinfo.Yaw = *(float *)(&(procbuff[startindex+63+6]));
            IMUinfo.Pitch = *(float *)(&(procbuff[startindex+63+10]));
            IMUinfo.Roll = *(float *)(&(procbuff[startindex+63+14]));
            IMUinfo.latitude = *(unsigned int *)(&(procbuff[startindex+63+18]));
            IMUinfo.longitude = *(unsigned int *)(&(procbuff[startindex+63+22]));
            IMUinfo.altitude = *(unsigned int *)(&(procbuff[startindex+63+26]));
            IMUinfo.Ve = *(float *)(&(procbuff[startindex+63+30]));
            IMUinfo.Va = *(float *)(&(procbuff[startindex+63+34]));
            IMUinfo.Vu = *(float *)(&(procbuff[startindex+63+38]));
            cout<<"IMUinfo.GPSweek = "<<IMUinfo.GPSweek<<endl;
            cout<<"IMUinfo.GPStime = "<<IMUinfo.GPStime<<endl;
            cout<<" IMUinfo.GyroX = "<< IMUinfo.GyroX<<endl;
            cout<<" IMUinfo.GyroY = "<<IMUinfo.GyroY<<endl;
            cout<<"IMUinfo.GyroZ = "<< IMUinfo.Gyroz<<endl;
            cout<<"IMUinfo.AccX = "<< IMUinfo.AccX<<endl;
            cout<<"  IMUinfo.AccY  = "<<  IMUinfo.AccY <<endl;
            cout<<" IMUinfo.AccZ = "<<  IMUinfo.AccZ<<endl;
            cout<<" IMUinfo.Yaw = "<< IMUinfo.Yaw<<endl;
            cout<<" IMUinfo.Pitch = "<<IMUinfo.Pitch<<endl;
            cout<<"IMUinfo.Roll = "<< IMUinfo.Roll<<endl;
            cout<<" IMUinfo.latitude= "<< IMUinfo.latitude<<endl;
            cout<<" IMUinfo.longitude = "<<   IMUinfo.longitude<<endl;
            cout<<"IMUinfo.altitude  = "<<IMUinfo.altitude <<endl;
            cout<<" IMUinfo.Ve= "<<  IMUinfo.Ve<<endl;
            cout<<" IMUinfo.Va= "<< IMUinfo.Va<<endl;
            cout<<" IMUinfo.Vu= "<<  IMUinfo.Vu<<endl;
            startflag = 0;
            startindex = 0;
        }
        
    }
    procbuff[2000]={'+'};
    // startflag = 0;
    // startindex = 0;
    // lenthoftembuff = 0;
    return;
}

void IMU_datatran(sensor_msgs::Imu &temimu, GTIMU const temimudata, ros::Time const Time){
    cout<<"enter imu_datatran"<<endl;
    tf::Quaternion q = tf::createQuaternionFromRPY((temimudata.Roll*3.1415926/180),(temimudata.Pitch*3.1415926/180), (temimudata.Yaw*3.1415926/180));
    
    cout<<"q.x()="<<q.x()<<endl;
    temimu.header.stamp = Time;
    temimu.header.frame_id = "base_IMU";
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

void GPS_datatran(sensor_msgs::NavSatFix &tmpGps, GTIMU const temimudata, ros::Time const Time){
    cout<<"enter GPS_datatran"<<endl;
    if (temimudata.longitude == 0 ||
            temimudata.latitude == 0 ||
            temimudata.altitude == 0 ||
            temimudata.longitude > 180*10000000.0 ||
            temimudata.latitude > 90*10000000.0 ||
            temimudata.altitude > 100*1000.0
            )
    {return;}

    tmpGps.header.stamp = Time;
    tmpGps.longitude = temimudata.longitude;
    tmpGps.latitude = temimudata.latitude;
    tmpGps.altitude = temimudata.altitude;

    ROS_INFO("Send the GPS location: [%f, %f, %f]",
                    tmpGps.longitude/10000000.0, tmpGps.latitude/10000000.0, tmpGps.altitude/1000.0);

    return;

}

void Vel_datatran(geometry_msgs::Vector3Stamped tmpVel, GTIMU const temimudata, ros::Time const Time){
    cout<<"enter Vel_datatran"<<endl;
    tmpVel.header.stamp = Time;

    if (abs(temimudata.Ve) <= 0.000000001
        && abs(temimudata.Ve) <= 0.000000001
        && abs(temimudata.Ve) <= 0.000000001)
    {
        return;
    }
    tmpVel.vector.x = temimudata.Ve;   //东向
    tmpVel.vector.y = temimudata.Va;   //北向
    tmpVel.vector.z = temimudata.Vu;   //天向

    ROS_INFO("Send the velocity: [%f, %f, %f]", tmpVel.vector.x, tmpVel.vector.y, tmpVel.vector.z);

    return;

}