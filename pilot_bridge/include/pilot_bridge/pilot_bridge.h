#ifndef _pilot_bridge_h_
#define _pilot_bridge_h_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include<sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <pilot_bridge/wheel_speed.h>
#include <pilot_bridge/sensor_status.h>
#include<thread>
#include<pilot_cleaner_msgs/task_ctl.h>
#include<pilot_cleaner_msgs/task_cancel.h>
#include<pilot_cleaner_msgs/user_path_recording_finish.h>
#include<pilot_cleaner_msgs/user_path_recording_start.h>
#include<actionlib_msgs/GoalID.h>
#include <yaml-cpp/yaml.h>
#include <vector>

#include <std_msgs/Int8.h>
#include<string>
#include <iostream>

using namespace std;

class Listener
{
public:
    Listener(ros::NodeHandle* nh);

    ~Listener();

    //处理传感器数据的回调函数
    void imu_status_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void front_laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void rear_laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void sonar1_callback(const sensor_msgs::Range::ConstPtr &msg);
    void sonar2_callback(const sensor_msgs::Range::ConstPtr &msg);
    void sonar3_callback(const sensor_msgs::Range::ConstPtr &msg);
    void sonar4_callback(const sensor_msgs::Range::ConstPtr &msg);
    void sonar5_callback(const sensor_msgs::Range::ConstPtr &msg);
    void sonar6_callback(const sensor_msgs::Range::ConstPtr &msg);
    void sonar7_callback(const sensor_msgs::Range::ConstPtr &msg);
    void sonar8_callback(const sensor_msgs::Range::ConstPtr &msg);
    // void camera_callback(const sensor_msgs::Image::ConstPtr &msg);
    // void brush_callback(const std_msgs::String::ConstPtr &msg);
    void car_speed_callback(const geometry_msgs::Twist::ConstPtr &msg);

    void run();

    void all_info_pub_func();

    //发布启动消息
    void ui_sub_callback(const std_msgs::Int8::ConstPtr &msg);

    //读取follow_waypoints yaml文件
    vector<string> load_follow_waypoint_yaml(string file_path);
    //读任务列表
    vector<string> load_task_yaml(string file_path);

     
private:

    //订阅话题名称用于自检信息
    string imu_topic_name = "/imu";
    string front_laser_topic_name = "/fcbot/laser/scan";    ///fcbot/laser/scan
    string rear_laser_topic_name = "/rear_laser_scan";
    string sonar1_topic_name = "/sensor/sonar_front_1";
    string sonar2_topic_name = "/sonar2";
    string sonar3_topic_name = "/sonar3";
    string sonar4_topic_name = "/sonar4";
    string sonar5_topic_name = "/sonar5";
    string sonar6_topic_name = "/sonar6";
    string sonar7_topic_name = "/sonar7";
    string sonar8_topic_name = "/sonar8";
    string camera_topic_name = "/camera";
    string brush_topic_name = "/brush";
    string battery_voltage_topic_name = "/battery_voltage";
    string car_speed_topic_name = "/cmd_vel";
    string water_lever_topic_name = "/water_lever";
    string auto_charge_topic_name = "/auto_charge";  //自动充电
    string navigation_topic_name = "/navigation";  //导航

    string ui_sub_topic_name = "/task_cmd_pub";  //订阅ui发布的消息

    string clean_start_pub_topic_name = "/taskctl";
    string clean_resume_pub_topic_name = "/taskctl";
    string clean_cancel_pub_topic_name = "/taskcancel";
    string record_path_pub_topic_name = "/start_record_path";
    string finish_record_path_pub_topic_name = "/end_record_path";

    //房间名称
    string room_list = "room_list";
    //follow waypoint 执行结果yaml文件路径
    string follow_waypoint_yaml_path = "/home/q/catkin_ws/src/pilot_bridge/src/last_task.yaml"; //"../../../follow_2.1/follow_waypoints-master/HLMlog/last_task.yaml";
    string task_yaml_path = "/home/q/catkin_ws/src/pilot_bridge/config/task.yaml";

    //发布话题名称
    string all_info_pub_topic_name = "/all_info";
    ros::CallbackQueue queue_;
    std::thread *thread_= nullptr;

    //订阅 传感器
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber front_laser_sub;
    ros::Subscriber rear_laser_sub;
    ros::Subscriber sonar1_sub;
    ros::Subscriber sonar2_sub;
    ros::Subscriber sonar3_sub;
    ros::Subscriber sonar4_sub;
    ros::Subscriber sonar5_sub;
    ros::Subscriber sonar6_sub;
    ros::Subscriber sonar7_sub;
    ros::Subscriber sonar8_sub;
    // ros::Subscriber camera_sub;
    // ros::Subscriber brush_sub;
    ros::Subscriber car_speed_sub;  //车速话题
    ros::Subscriber water_lever_sub;  //水位
    ros::Subscriber battery_voltage_sub; //电池电压
    ros::Subscriber crash_switch_sub;  //碰撞开关
    ros::Subscriber auto_charge_sub;  //自动充电开关

    //发布all   pilot_pridge ===> user_interface
    ros::Publisher all_info_pub;

    //订阅ui ==> pilot_pridge
    ros::Subscriber ui_sub;

    //发布 pilot_pridge ===> sensor
    ros::Publisher clean_start_pub; //启动扫地
    ros::Publisher clean_cancel_pub; //取消扫地
    ros::Publisher cancel_goals_pub; //取消导航
    ros::Publisher clean_resume_pub; //恢复扫地
    ros::Publisher start_record_path_pub; //开始录制路径
    ros::Publisher finish_record_path_pub; //停止录制路径
    ros::Publisher dock_position_save_pub; //保存充电桩位置
    ros::Publisher start_charge_pub; //开始充电
    ros::Publisher stop_charge_pub; //停止充电


    /*
 rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {} 

 or
 
 ros::Publisher　 cancle_pub_ =  nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);

　actionlib_msgs::GoalID　first_goal;
     cancle_pub_.publish(first_goal);

    */



    bool imu_status;
    pilot_bridge::sensor_status sensor_status;

};


#endif