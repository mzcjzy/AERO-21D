//
// Created by liqiangwei on 18-8-28.
//

#ifndef IO_LINUXSERIAL_H
#define IO_LINUXSERIAL_H

#endif //IO_LINUXSERIAL_H

#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<iostream>

//宏定义
#define FALSE  -1
#define TRUE   0

class roserialhandler
{
private:
    char *portname;//串口名
    int baudrate;//波特率
    int fd;//文件描述符

public:

    //有参构造函数，输入串口名和波特率
    roserialhandler(char port[],int baudr)
    {
        portname = port;
        baudrate = baudr;
    }

    //析构函数的声明
    ~roserialhandler();

    //打开串行端口
    int UART0_Open();

    //关闭串行端口
    void UART0_Close(int fd);

    //设置串行端口
    //speed——        flow_ctrl——数据流控制参数    databits
    int UART0_Set(int speed, int flow_ctrl, int databits, int stopbits, int parity);

    int UART0_Init(int flow_ctrl,int databits,int stopbits,int parity);

    int UART0_Recv(unsigned char *rcv_buf,int data_len);

    int UART0_Send(char *send_buf,int data_len);

    unsigned char wubJTT808CalculateChecksum(unsigned char *aubData_p,int auwDataLength);


};