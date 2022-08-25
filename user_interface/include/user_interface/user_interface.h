#pragma once
#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <unistd.h>
#include <sensor_msgs/Imu.h>
#include<sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include<user_interface/sensor_status.h>
#include<thread>
#include<serial/serial.h>
#include <std_msgs/String.h>
#include<string>
#include <iostream>
#include <pilot_bridge/sensor_status.h>

using namespace std;
#define BUFFER_SIZE 8

class User_interface
{
public:

    
    User_interface(ros::NodeHandle* nh);
    ~User_interface();

    bool initialize();

    //接收ui串口的数据
    void ui_data_receiver();
    //发送数据到ui串口
    void ui_data_sender();
    //解析ui 数据 并发布到pilot_bridge
    void ui_data_parser();

    //订阅pilot_bridge的数据的回调函数
    void pilot_bridge_callback(const pilot_bridge::sensor_status::ConstPtr& msg);

    std::thread ui_data_receiver_thread;

    ros::NodeHandle nh;
    //订阅pilot_bridge的数据
    ros::Subscriber pilot_bridge_sub;

private:
    
    serial::Serial ser;
    bool serial_status;
    user_interface::sensor_status sensor_status;

    //接收触摸屏串口的数据
    size_t ui_data_receiver_size;
    uint8_t ui_data_receiver_buffer[1024];

    //写入触摸屏传感器状态的data

    unsigned char imu_status_buffer[BUFFER_SIZE];
    unsigned char front_laser_status_buffer[BUFFER_SIZE];
    unsigned char rear_laser_status_buffer[BUFFER_SIZE];
    unsigned char sonar_status_buffer[BUFFER_SIZE];


    //命令发布
    string task_pub_topic_name = "/task_cmd_pub";
    ros::Publisher task_cmd_pub;

    //声明指令发布的消息
    std_msgs::Int8 clean_start_msg;
    std_msgs::Int8 clean_cancel_msg;
    std_msgs::Int8 clean_resume_msg;
    
    

};












#endif