// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <pilot_bridge/wheel_speed.h>
// #include <pilot_bridge/sensor_status.h>

// #include <std_msgs/String.h>
// #include<string>
// #include <iostream>

#include "pilot_bridge/pilot_bridge.h"

using namespace std;


int main(int argc,  char **argv)
{
    ros::init(argc, argv, "pilot_bridge_node");
    ros::NodeHandle node_handle("~");
    ROS_INFO("pilot_bridge start...");
    // Listener listener;
    
    Listener *listener = new Listener(&node_handle);

    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        // listener.run();
        listener->all_info_pub_func();
    //     ROS_INFO("start_sweep");
    //    pilot_cleaner_msgs::task_ctl start_clean_msg;
    //    start_clean_msg.timestamp = ros::Time::now();
    //    start_clean_msg.roomlist =  "pose_display1";
    //    start_clean_msg.cmd = "start";       
    //     listener->clean_start_pub.publish(start_clean_msg);
        loop_rate.sleep();
    }

    delete listener;

    return 0;


}