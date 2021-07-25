//
// Created by liqiangwei on 18-8-28.
//

#include "linuxserial.h"

//析构函数
roserialhandler::~roserialhandler() {
    roserialhandler::UART0_Close(fd);
}


//打开串行端口
int roserialhandler::UART0_Open() {


    //O_RDWR——以读写方式打开文件
    //O_NOCTTY——如果欲打开的文件为终端机设备时，则不会将该终端机当成进程控制终端机。
    //O_NDELAY——如果路径名指向 FIFO/块文件/字符文件，则把文件的打开和后继 I/O
    //fd接受open函数返回值
    fd = open( portname, O_RDWR|O_NOCTTY|O_NDELAY);

    //判断是否打开成功，如果打开失败，输出提示，并返回-1
    if (FALSE == fd)
    {
        perror("Can't Open Serial Port");
        return(FALSE);
    }


    //如果打开成功
    

    //恢复串口为阻塞状态
    //F_SETFL——设置文件描述词状态旗标, 参数0为新旗标, 但只允许O_APPEND、O_NONBLOCK 和O_ASYNC 位的改变, 其他位的改变将不受影响.
    //判断，如果设置失败，输出提示，返回-1
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return(FALSE);
    }
    //如果设置成功，输出提示
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }


    //测试是否为终端设备
    //判断，如果标准输入设备不是终端设备，输出提示，返回文件描述符
    //STDIN_FILENO——标准输入设备的文件描述符
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return fd;
    }
    //如果标准输入设备是终端机，输出提示，返回文件描述符
    else
    {
        printf("isatty success!\n");
    }
    printf("fd->open=%d\n",fd);
    return fd;
}


//设置串行端口
int roserialhandler::UART0_Set( int speed, int flow_ctrl, int databits, int stopbits, int parity) {
    int   i;
    int   status;
    int   speed_arr[] = { B230400, B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {230400, 115200,  19200,  9600,  4800,  2400,  1200,  300};
    
    //结构体，存储与fd指向的对象相关的参数
    struct termios options;

    /*
    tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */

    //如果调用失败，输出错误，返回-1
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

        case 0 ://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;

        case 1 ://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2 ://使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 5    :
            options.c_cflag |= CS5;
            break;
        case 6    :
            options.c_cflag |= CS6;
            break;
        case 7    :
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data size\n");
            return (FALSE);
    }
    //设置校验位
    switch (parity)
    {
        case 'n':
        case 'N': //无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O'://设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E'://设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': //设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB; break;
        case 2:
            options.c_cflag |= CSTOPB; break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}

int roserialhandler::UART0_Init(int flow_ctrl, int databits, int stopbits, int parity) {
    bool err;
    //设置串口数据帧格式
     err = UART0_Set(baudrate,0,8,1,'N');
    return err;

}

int roserialhandler::UART0_Recv(unsigned char *rcv_buf, int data_len) {
    int fs_sel;
    fd_set fs_read;
    ssize_t len;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 10;
    time.tv_usec = 0;

    //使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel)
    {
        len = read(fd,rcv_buf,data_len);
       // printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);
        return len;
    }
    else
    {
        printf("Sorry,I am wrong!");
        return FALSE;
    }
}

int roserialhandler::UART0_Send(char *send_buf, int data_len) {
    int len = 0;

    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        return len;
    }
    else
    {

        tcflush(fd,TCOFLUSH);
        return FALSE;
    }

}

void roserialhandler::UART0_Close(int fd) {
    close(fd);
}

int roserialhandler::wubJTT808CalculateChecksum(unsigned char *aubData_p, int auwDataLength)
{
    int aubChecksum = 0;
    int auwCnt = 0;
 
    while(auwCnt < auwDataLength)
    {
        aubChecksum ^= *(int*)(&aubData_p[auwCnt]);
        auwCnt++;
    }
 
    return aubChecksum;
}