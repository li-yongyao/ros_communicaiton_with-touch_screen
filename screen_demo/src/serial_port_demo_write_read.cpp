
//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>

//
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

using namespace std;
//数据类型很重要，对于通讯协议是16进制数据码的，收发数据应放在数组中，例如unsigned char s_buffer[sBUFFERSIZE];
//协议是字符串的用字符串类型，如std_msgs::String serial_data
#define sBUFFERSIZE 8//send buffer size 串口发送缓存长度8
unsigned char s_buffer[sBUFFERSIZE];//发送缓存，这是一个数组
uint8_t r_buffer[1024];
int r_n=9;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "baseRun");
    ros::NodeHandle nh;
    serial::Serial ser;
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ser.setTimeout(to);

    ROS_INFO("Touchscreen node Start!!");

    try
    {

        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized.\n");
    }
    else
    {
        return -1;
    }


// 工作模式寄存器读取
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x13;
    s_buffer[5]=0x00;

    s_buffer[6]=0x01;

// unsigned char dat[8]={0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};
//uint8_t dat[8]={0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};

/*****************************************************************************

    以下逻辑可以按照你自己的写，主要是为 data 赋值
    *****************************************************************************/
    // unsigned char dat[8]={0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};//发送协议可以直接初始化为这个数组
    ros::Rate loop_rate(10);
    while (ros::ok())
    {  
    //向串口写指令，再读返回
    ser.write(s_buffer,sBUFFERSIZE);
	//ser.read(r_buffer, r_n);
    //ser.write(dat,8);
    size_t n = ser.available(); //缓冲区的字节数

	if(n!=0)
		{
			uint8_t buffer[1024];
			ser.read(buffer, n);
			for(int i=0; i<n; i++)
			{
				//16进制的方式打印到屏幕
				std::cout << std::hex << (buffer[i] & 0xff) << " ";
				//例如，输入n=10，要求输出 0x0000000A;
				//　C++: sprintf( buffer, “0x%08X”, n);
                //  ROS_INFO("Buffer %d", buffer[i]& 0xff);
			}
		}
            loop_rate.sleep();

    }
}
