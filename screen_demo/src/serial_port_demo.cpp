
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baseRun");
    ros::NodeHandle nh;
    serial::Serial ser;
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ser.setTimeout(to);

    ROS_INFO("Start!!");

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

    //自检模式
    //隐藏水箱红色
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x50;
    s_buffer[5]=0x00;

    s_buffer[6]=0xFF;
    s_buffer[7]=0x00;
    ser.write(s_buffer,sBUFFERSIZE);

    //水箱位置设为55%
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x11;
    s_buffer[5]=0x12;

    s_buffer[6]=0x00;
    s_buffer[7]=0x38;
    ser.write(s_buffer,sBUFFERSIZE);

    //隐藏电池红色显示
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x50;
    s_buffer[5]=0x20;

    s_buffer[6]=0xFF;
    s_buffer[7]=0x00;
    ser.write(s_buffer,sBUFFERSIZE);
    //电池电量显示100
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x11;
    s_buffer[5]=0x16;

    s_buffer[6]=0x00;
    s_buffer[7]=0x44;

    ser.write(s_buffer,sBUFFERSIZE);


    ros::Rate loop_rate(500);

    while (ros::ok())
    {


            size_t n = ser.available(); //获取缓冲区的字节数

            if(n!=0)
            {

                
        
            
                uint8_t buffer[1024];
                
                ser.read(buffer, n);
            
                for(int i=0; i<n; i++)
                {
                    //16进制的方式打印到屏幕
                 
                    std::cout << std::hex << (buffer[i] & 0xff) << " ";
                    // if(i==2 || i==4)
                    // {
                    //     cout<<(buffer[i] & 0xff) << " " <<(buffer[i+1] & 0xff) << " ";  
                    // }
                    
                    
            
                }
            }

            loop_rate.sleep();


    }
    

    ser.close();
    return 0;

}
