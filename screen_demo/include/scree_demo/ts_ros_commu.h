#ifndef ts_ROS_commu_H
#define ts_ROS_commu_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "time.h"
#include <iostream>
#include <serial/serial.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/*
ros 和触摸屏通信程序
1.订阅传感器话题，通过数据判断是否在线  robot_one::ros_read     -----written by lyy  20220630
2.更新触摸屏状态写入 robot_one::ts_write
3.读触摸屏状态下发ros 控制机器工作 robot_one::ts_read

*/

namespace robot_one
{

class Ros_read
{
public:

    Ros_read(ros::NodeHandle& n);


    ~Ros_read() {return;}

    bool initialize();  //判断是否建立通信 初始化

    typedef boost::shared_ptr<Ros_read> Ros_readPtr; 

    bool serial_status;

private:
    
    bool front_lidar_status;
    bool rear_lidar_status;
    bool sonar_front1_status;
    bool sonar_front2_status;
    bool sonar_front3_status;
    bool sonar_front_ground_status;
    bool sonar_rear_ground_status;
    bool sonar_right_status;
    bool sonar_left_status;


    ros::Subscriber front_lidar_sub;
    ros::Subscriber rear_lidar_sub;


};







}







#endif