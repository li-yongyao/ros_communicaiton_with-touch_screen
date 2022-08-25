#include "pilot_bridge/pilot_bridge.h"

using namespace  std;


Listener::Listener(ros::NodeHandle *node_handle):nh(*node_handle)
{
    imu_status = false;
    string a;
    nh.getParam("imu_topic_name",imu_topic_name);
    nh.getParam("front_laser_topic_name",front_laser_topic_name);
    nh.getParam("rear_laser_topic_name",rear_laser_topic_name);
    nh.getParam("sonar1_topic_name",sonar1_topic_name);
    nh.getParam("sonar2_topic_name",sonar2_topic_name);
    nh.getParam("sonar3_topic_name",sonar3_topic_name);
    nh.getParam("sonar4_topic_name",sonar4_topic_name);
    nh.getParam("sonar5_topic_name",sonar5_topic_name);
    nh.getParam("sonar6_topic_name",sonar6_topic_name);
    nh.getParam("sonar7_topic_name",sonar7_topic_name);
    nh.getParam("sonar8_topic_name",sonar8_topic_name);
    nh.getParam("room_list",room_list);



    cout<<"imu_topic_name: "<<imu_topic_name<<endl;
    cout<<"front_laser_topic_name: "<<front_laser_topic_name<<endl;
    all_info_pub = nh.advertise<pilot_bridge::sensor_status>(all_info_pub_topic_name, 10);

    clean_start_pub = nh.advertise<pilot_cleaner_msgs::task_ctl>(clean_start_pub_topic_name, 10);
    clean_cancel_pub = nh.advertise<pilot_cleaner_msgs::task_cancel>(clean_cancel_pub_topic_name, 10);
    cancel_goals_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
    clean_resume_pub = nh.advertise<pilot_cleaner_msgs::task_ctl>(clean_resume_pub_topic_name, 10);
    start_record_path_pub = nh.advertise<pilot_cleaner_msgs::user_path_recording_start>(record_path_pub_topic_name, 10);
    finish_record_path_pub = nh.advertise<pilot_cleaner_msgs::user_path_recording_finish>(finish_record_path_pub_topic_name, 10);
    
    thread_ = new std::thread(&Listener::run, this);

    // run();
}
Listener::~Listener()
{
    ros::shutdown();
    ROS_INFO("pilot_bridge shutdown...");
    // delete thread_;
    if(thread_)
    {
         thread_->join();
         delete thread_;
         thread_ = nullptr;
    }
}

void Listener::run()
{
    nh.setCallbackQueue(&queue_);
   //   ros::AsyncSpinner spinner(4);
   //  spinner.start();
    imu_sub = nh.subscribe(imu_topic_name, 100, &Listener::imu_status_callback, this);
    front_laser_sub = nh.subscribe(front_laser_topic_name, 100, &Listener::front_laser_callback, this);
    rear_laser_sub = nh.subscribe(rear_laser_topic_name, 100, &Listener::rear_laser_callback, this);
    sonar1_sub = nh.subscribe(sonar1_topic_name, 100, &Listener::sonar1_callback, this);
    sonar2_sub = nh.subscribe(sonar2_topic_name, 100, &Listener::sonar2_callback, this);
    sonar3_sub = nh.subscribe(sonar3_topic_name, 100, &Listener::sonar3_callback, this);
    sonar4_sub = nh.subscribe(sonar4_topic_name, 100, &Listener::sonar4_callback, this);
    sonar5_sub = nh.subscribe(sonar5_topic_name, 100, &Listener::sonar5_callback, this);
    sonar6_sub = nh.subscribe(sonar6_topic_name, 100, &Listener::sonar6_callback, this);
    sonar7_sub = nh.subscribe(sonar7_topic_name, 100, &Listener::sonar7_callback, this);
    sonar8_sub = nh.subscribe(sonar8_topic_name, 100, &Listener::sonar8_callback, this);
    car_speed_sub = nh.subscribe(car_speed_topic_name, 100, &Listener::car_speed_callback, this);
    // camera_sub = nh.subscribe(camera_topic_name, 100, &Listener::camera_callback, this);
    // brush_sub = nh.subscribe(brush_topic_name, 100, &Listener::brush_callback, this);

    ui_sub = nh.subscribe(ui_sub_topic_name, 10, &Listener::ui_sub_callback, this);  //接收UI发来的消息
    ros::AsyncSpinner spinner(0, &queue_);
    spinner.start();
   
    ros::waitForShutdown();

}

void Listener::imu_status_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(msg != NULL)
    {
         sensor_status.imu_status = true;
         sensor_status.imu_data.header = msg->header;
         sensor_status.imu_data.orientation = msg->orientation;
         sensor_status.imu_data.angular_velocity = msg->angular_velocity;
         sensor_status.imu_data.linear_acceleration = msg->linear_acceleration;
         sensor_status.imu_data.orientation_covariance = msg->orientation_covariance;
         sensor_status.imu_data.angular_velocity_covariance = msg->angular_velocity_covariance;
         sensor_status.imu_data.linear_acceleration_covariance = msg->linear_acceleration_covariance;
    }
    else
    {
       sensor_status.imu_status = false;
    }

        // ROS_INFO("IMU status: %d", sensor_status.imu_status);
        
    
}
   

void Listener::front_laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if(msg )
    {
         sensor_status.front_laser_status = true;
         sensor_status.front_laser_scan.header = msg->header;
         sensor_status.front_laser_scan.angle_min = msg->angle_min;
         sensor_status.front_laser_scan.angle_max = msg->angle_max;
         sensor_status.front_laser_scan.angle_increment = msg->angle_increment;
         sensor_status.front_laser_scan.time_increment = msg->time_increment;
         sensor_status.front_laser_scan.scan_time = msg->scan_time;
         sensor_status.front_laser_scan.range_min = msg->range_min;
         sensor_status.front_laser_scan.range_max = msg->range_max;
         sensor_status.front_laser_scan.ranges = msg->ranges;
         sensor_status.front_laser_scan.intensities = msg->intensities;

    }
    else
    {
       sensor_status.front_laser_status = false;
    }
    // ROS_INFO("front_laser_status: %d", sensor_status.front_laser_status);

}

void Listener::rear_laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if(msg )
    {
         sensor_status.rear_laser_status = true;
         sensor_status.rear_laser_scan.header = msg->header;
         sensor_status.rear_laser_scan.angle_min = msg->angle_min;
         sensor_status.rear_laser_scan.angle_max = msg->angle_max;
         sensor_status.rear_laser_scan.angle_increment = msg->angle_increment;
         sensor_status.rear_laser_scan.range_min = msg->range_min;
         sensor_status.rear_laser_scan.range_max = msg->range_max;
         sensor_status.rear_laser_scan.ranges = msg->ranges;
         sensor_status.rear_laser_scan.intensities = msg->intensities;
    }
    else
    {
       sensor_status.rear_laser_status = false;
    }
    // ROS_INFO("rear_laser_status: %d", sensor_status.rear_laser_status);

}

void Listener::sonar1_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if(msg )
    {
         sensor_status.sonar1_status = true;
         sensor_status.sonar1_data.header = msg->header;
         sensor_status.sonar1_data.range = msg->range;
         sensor_status.sonar1_data.min_range = msg->min_range;
         sensor_status.sonar1_data.max_range = msg->max_range;
         sensor_status.sonar1_data.field_of_view = msg->field_of_view;
         sensor_status.sonar1_data.radiation_type = msg->radiation_type;
    }
    else
    {
       sensor_status.sonar1_status = false;
    }
    // ROS_INFO("sonar1_status: %d", sensor_status.sonar1_status);

}

void Listener::sonar2_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if(msg )
    {
         sensor_status.sonar2_status = true;
         sensor_status.sonar2_data.header = msg->header;
         sensor_status.sonar2_data.range = msg->range;
         sensor_status.sonar2_data.min_range = msg->min_range;
         sensor_status.sonar2_data.max_range = msg->max_range;
         sensor_status.sonar2_data.field_of_view = msg->field_of_view;
         sensor_status.sonar2_data.radiation_type = msg->radiation_type;
    }
    else
    {
       sensor_status.sonar2_status = false;
    }

}

void Listener::sonar3_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if(msg)
    {
         sensor_status.sonar3_status = true;
         sensor_status.sonar3_data.header = msg->header;
         sensor_status.sonar3_data.range = msg->range;
         sensor_status.sonar3_data.min_range = msg->min_range;
         sensor_status.sonar3_data.max_range = msg->max_range;
         sensor_status.sonar3_data.field_of_view = msg->field_of_view;
         sensor_status.sonar3_data.radiation_type = msg->radiation_type;
    }
    else
    {
       sensor_status.sonar3_status = false;
    }

}

void Listener::sonar4_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if(msg )
    {
         sensor_status.sonar4_status = true;
         sensor_status.sonar4_data.header = msg->header;
         sensor_status.sonar4_data.range = msg->range;
         sensor_status.sonar4_data.min_range = msg->min_range;
         sensor_status.sonar4_data.max_range = msg->max_range;
         sensor_status.sonar4_data.field_of_view = msg->field_of_view;
         sensor_status.sonar4_data.radiation_type = msg->radiation_type;

    }
    else
    {
       sensor_status.sonar4_status = false;
    }

}

void Listener::sonar5_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if(msg )
    {
         sensor_status.sonar5_status = true;
         sensor_status.sonar5_status = true;
         sensor_status.sonar5_data.header = msg->header;
         sensor_status.sonar5_data.range = msg->range;
         sensor_status.sonar5_data.min_range = msg->min_range;
         sensor_status.sonar5_data.max_range = msg->max_range;
         sensor_status.sonar5_data.field_of_view = msg->field_of_view;
         sensor_status.sonar5_data.radiation_type = msg->radiation_type;

    }
    else
    {
       sensor_status.sonar5_status = false;
    }

}
void Listener::sonar6_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if(msg )
    {
       sensor_status.sonar6_status = true;
    }
    else
    {
       sensor_status.sonar6_status = false;
    }

}
void Listener::sonar7_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if(msg )
    {
       sensor_status.sonar7_status = true;
    }
    else
    {
       sensor_status.sonar7_status = false;
    }

}
void Listener::sonar8_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    if(msg )
    {
       sensor_status.sonar8_status = true;
    }
    else
    {
       sensor_status.sonar8_status = false;
    }

}

void Listener::car_speed_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
   
    sensor_status.car_speed = msg->linear.x;   //如果是直线行使，速度为linear,x   有转弯 待定
    // ROS_INFO("car_speed: %f", sensor_status.car_speed);


}


void Listener::all_info_pub_func()
{
    all_info_pub.publish(sensor_status);
}

//follow waypoint file loading function
vector<string> Listener::load_follow_waypoint_yaml(string file_path)
{
    YAML::Node config = YAML::LoadFile(file_path);
    string lastcmd{},roomlist{};
    vector<string> res;
    // cout<<"follow waypoint yaml file lastcmd content: "<< config["lastcmd"].as<string>() << endl;

    // vec_list.pop_back(config["lastcmd"].as<list<string>>().to_list());

  
    lastcmd.append(config["lastcmd"].as<string>());
    roomlist.append(config["roomlist"].as<string>());
    res.push_back(lastcmd);
    res.push_back(roomlist);


    return res;
}

vector<string> Listener::load_task_yaml(string file_path)
{
    YAML::Node config = YAML::LoadFile(file_path);
    vector<string> total_task;
    total_task.push_back(config["task1"].as<string>());
    total_task.push_back(config["task2"].as<string>());
    total_task.push_back(config["task3"].as<string>());
    total_task.push_back(config["task4"].as<string>());
    total_task.push_back(config["task5"].as<string>());

    return total_task;
}


//假设从ui获取的数据是int类型
void Listener::ui_sub_callback(const std_msgs::Int8::ConstPtr &msg)
{
    
    string lastcmd, roomlist;
    vector<string> res_vec;
    vector<string> total_task;

    res_vec = load_follow_waypoint_yaml(follow_waypoint_yaml_path);
    lastcmd = res_vec[0];
    roomlist = res_vec[1];
    // cout<<"lastcmd: "<< lastcmd << endl;

    // cout<<"roomlist: "<< roomlist << endl;

    // if(lastcmd == "start" | lastcmd == "resume")
    // {
    //     cout<<"last command is 'start ' or 'resume' "<<endl;
    // }
    // else{
    //     cout<<"string is not equal"<<endl;
    // }
   

   total_task = load_task_yaml(task_yaml_path);

   cout<<"task num:"<<total_task.size()<<endl;

   //需要增加判断条件
   //1、如果是启动扫地命令
   if(msg->data == 0)
   {
       ROS_INFO("start_clean...");
       pilot_cleaner_msgs::task_ctl start_clean_msg;
       start_clean_msg.timestamp = ros::Time::now();
       start_clean_msg.roomlist = room_list; //需要传入房间名称
       start_clean_msg.cmd = "start";
       clean_start_pub.publish(start_clean_msg);
       
   }

   //执行扫地任务设置，任务一、任务二。。
   if(msg->data == 11)
   {
        ROS_INFO("Task 1 start...");
        pilot_cleaner_msgs::task_ctl start_clean_msg;
        start_clean_msg.timestamp = ros::Time::now();
        start_clean_msg.roomlist = total_task[0]; //需要传入房间名称
        start_clean_msg.cmd = "start";
        clean_start_pub.publish(start_clean_msg);

   }
   if(msg->data == 12)
   {
        ROS_INFO("Task 2 start...");
        pilot_cleaner_msgs::task_ctl start_clean_msg;
        start_clean_msg.timestamp = ros::Time::now();
        start_clean_msg.roomlist = total_task[1]; //需要传入房间名称
        start_clean_msg.cmd = "start";
        clean_start_pub.publish(start_clean_msg);

   }
   if(msg->data == 13)
   {
        ROS_INFO("Task 3 start...");
        pilot_cleaner_msgs::task_ctl start_clean_msg;
        start_clean_msg.timestamp = ros::Time::now();
        start_clean_msg.roomlist = total_task[2]; //需要传入房间名称
        start_clean_msg.cmd = "start";
        cout<<"Task 3 is :"<<total_task[2]<<endl;
        clean_start_pub.publish(start_clean_msg);

   }
   if(msg->data == 14)
   {
        ROS_INFO("Task 4 start...");
        pilot_cleaner_msgs::task_ctl start_clean_msg;
        start_clean_msg.timestamp = ros::Time::now();
        start_clean_msg.roomlist = total_task[3]; //需要传入房间名称
        start_clean_msg.cmd = "start";
        clean_start_pub.publish(start_clean_msg);

   }
   if(msg->data == 15)
   {
        ROS_INFO("Task 3 start...");
        pilot_cleaner_msgs::task_ctl start_clean_msg;
        start_clean_msg.timestamp = ros::Time::now();
        start_clean_msg.roomlist = total_task[4]; //需要传入房间名称
        start_clean_msg.cmd = "start";
        clean_start_pub.publish(start_clean_msg);

   }

    //2、如果是停止扫地命令
    if(msg->data == 1)
    {
        ROS_INFO("cancel_clean...");
        pilot_cleaner_msgs::task_cancel cancel_clean_msg;
        cancel_clean_msg.timestamp = ros::Time::now();
        cancel_clean_msg.reason = "manual";     //需要传入用户取消的原因
        clean_cancel_pub.publish(cancel_clean_msg);

        actionlib_msgs::GoalID cancel_msg;
        // cancel_msg.id = "cancel";
        cancel_goals_pub.publish(cancel_msg);
        cout<<"cancel_goals_pub"<<endl;
    }

    //3、如果是恢复扫地
    if(msg->data == 2)
    {
        ROS_INFO("resume_clean...");
        pilot_cleaner_msgs::task_ctl resume_clean_msg;
        resume_clean_msg.timestamp = ros::Time::now();
        resume_clean_msg.cmd = "resume";
        clean_resume_pub.publish(resume_clean_msg);
    }

    //4、如果是录制路径
    if(msg->data == 3)
    {
        ROS_INFO("record_path...");
        pilot_cleaner_msgs::user_path_recording_start record_path_msg;
        record_path_msg.timestamp = ros::Time::now();
        record_path_msg.pathfilename = "path_record1";
        start_record_path_pub.publish(record_path_msg);
    }

    //5、如果是完成录制路径
    if(msg->data == 4)
    {
        ROS_INFO("finish_record_path...");
        pilot_cleaner_msgs::user_path_recording_finish finish_record_path_msg;
        finish_record_path_msg.timestamp = ros::Time::now();
        finish_record_path_msg.misc = "path_record1";
        clean_start_pub.publish(finish_record_path_msg);
    }

    //6、如果是记录充电桩的位置

}