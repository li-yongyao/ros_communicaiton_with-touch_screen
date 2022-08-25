#include "user_interface/user_interface.h"

User_interface::User_interface(ros::NodeHandle *node_handle):nh(*node_handle)
{
    serial_status = false;
    serial_status = initialize();
    task_cmd_pub = nh.advertise<std_msgs::Int8>(task_pub_topic_name,1);
    if(!serial_status)
    {
        ROS_ERROR("User_interface initialize failed!");
    }
    else
    {
        ROS_INFO("User_interface initialize success!");

        //创建一个线程，用来接收ui串口的数据
        std::thread ui_data_receiver_thread(&User_interface::ui_data_receiver, this);
        ui_data_receiver_thread.detach();
    }
}

User_interface::~User_interface()
{
    if(serial_status)
    {
        ser.close();
    }
}

bool User_interface::initialize()
{

    
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
        return false;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized.\n");
    }
    else
    {
        return false;
    }
    return true;
}

void User_interface::ui_data_receiver()
{
    int button_flag = 0; //0: no button pressed; 11;12;13;14;21;22;23;24....第一个数字代表第一个页面，后面数字代表当前页面的第几个按钮
    uint8_t header [2];
    header[0] = 0x5A;
    header[1] = 0xA5;
    
    while(true)
    {
        //接收ui串口的数据
      
        ui_data_receiver_size = ser.available();

        // cout<<"ui_data_receiver_size1:"<<ui_data_receiver_size<<endl;
        if(ui_data_receiver_size > 0)
        {
            ser.read(ui_data_receiver_buffer, ui_data_receiver_size);
            // int len = ui_data_receiver_size;
            // for (size_t j = 0; j<len; j++)
            // {
            //     cout<<hex<<(int)ui_data_receiver_buffer[j]<<" ";
            // }
            // cout<<endl;
            
           int temp0 = (int)ui_data_receiver_buffer[0];
           int temp1 = (int)ui_data_receiver_buffer[1];
           int temp2 = (int)ui_data_receiver_buffer[2];
           int temp3 = (int)ui_data_receiver_buffer[3];
           int temp4 = (int)ui_data_receiver_buffer[4];
           int temp5 = (int)ui_data_receiver_buffer[5];
           int temp6 = (int)ui_data_receiver_buffer[6];
           int temp7 = (int)ui_data_receiver_buffer[7];
           int temp8 = (int)ui_data_receiver_buffer[8];
            // cout<<"temp:"<< temp <<endl;
            cout<<"ui_data_receiver_fuffer:";
            for(int i=0; i<ui_data_receiver_size; i++)
            {
                //16进制的方式打印到屏幕
                if(ui_data_receiver_size<9)
                {
                    cout<<hex<<(int)ui_data_receiver_buffer[i]<<" ";
                }
                // std::cout << std::hex << (ui_data_receiver_buffer[i] & 0xff) << " ";
                cout<<hex<<(int)ui_data_receiver_buffer[i]<<" ";
                if((i+1) % ui_data_receiver_size==0)
                {
                    std::cout << std::endl;
                }
                       // {
                    //     cout<<(buffer[i] & 0xff) << " " <<(buffer[i+1] & 0xff) << " ";  
                    // }
                    
                
            }

            if(header[0] == temp0 && header[1]==temp1)
            {
                cout<<"ui_data_receiver_buffer:"<<" 0x5A  A5"<<endl;

                int fifth_num = (int)ui_data_receiver_buffer[4];
                cout<<"5th_num:" <<fifth_num<<endl;
                switch (fifth_num)
                {
                case 0x13:
                    cout<<"工作设置页面"<<endl;

                    if(temp5 == 0x00 && temp8 == 0x01)
                    {
                        if(button_flag==11)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"工作模式：自动驾驶"<<endl;
                            button_flag = 11;
                        }
                        
                    }
                    else if(temp5 == 0x00 && temp8==0x00)
                    {
                         if(button_flag==12)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"工作模式：手动工作"<<endl;
                            button_flag = 12;
                        }
                        
                    }
                    else if(temp5 == 0x20 && temp8==0x01)
                    {
                        if(button_flag==13)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"工作模式：全面清洁"<<endl;
                            button_flag = 13;
                        }
                    }
                     else if(temp5 == 0x20 && temp8==0x00)
                    {
                        if(button_flag==14)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"工作模式：清洁维护"<<endl;
                            button_flag = 14;
                        }
                    }     
                    else break;
                    break;

                case 0x14:
                    cout<<"回充/换水页面"<<endl;

                    if(temp5 == 0x00 && temp8 == 0x01)
                    {
                        if(button_flag==21)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"自动回充：启动"<<endl;
                            button_flag = 21;
                        }
                    }
                    else if(temp5 == 0x00 && temp8==0x00)
                    {
                        if(button_flag==22)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"自动回充：关闭"<<endl;
                            button_flag = 22;
                        }
                    }
                    else if(temp5 == 0x20 && temp8==0x01)
                    {
                        if(button_flag==23)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"换水：启动"<<endl;
                            button_flag = 3;
                        }
                    }
                     else if(temp5 == 0x20 && temp8==0x00)
                    {
                        if(button_flag==24)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"换水：停止"<<endl;
                            button_flag = 24;
                        }
                    }
                    else break;
                    break;

                case 0x16:                                //刷盘
                    cout<<"刷盘控制页面"<<endl;

                    if(temp5 == 0x00 && temp8 == 0x01)
                    {
                        if(button_flag==31)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"刷盘：启动"<<endl;
                            button_flag = 31;
                        }
                    }
                    else if(temp5 == 0x00 && temp8==0x00)
                    {
                        if(button_flag==32)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                           cout<<"刷盘：刷盘停止"<<endl;
                            button_flag = 32;
                        }
                        
                    }
                    else if(temp5 == 0x20 && temp8==0x01)
                    {
                        if(button_flag==33)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"吹风机：吹风机启动"<<endl;
                            button_flag = 33;
                        }        
                    }
                     else if(temp5 == 0x20 && temp8==0x00)
                    {
                        if(button_flag==34)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            cout<<"吹风机：吹风机停止"<<endl;
                            button_flag = 34;
                        }
                    }
                    
                    else break;
                    break;
                
                case 0x18:
                    cout<<"任务执行页面"<<endl;
                    if(temp8 == 0x01)
                    {
                       if(button_flag==1)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            clean_start_msg.data = 0;
                            task_cmd_pub.publish(clean_start_msg);

                            cout<<"clean_start_msg_data:"<<(int)clean_start_msg.data<<endl;
                            cout<<"任务执行：执行"<<endl;
                            button_flag = 1;
                        }
                            
                        // clean_start_msg.data = 0;
                        // task_cmd_pub.publish(clean_start_msg);

                        // cout<<"clean_start_msg_data:"<<(int)clean_start_msg.data<<endl;
                        // cout<<"任务执行：执行"<<endl;


                      
                    }
                    else if (temp8==0x00)
                    {
                        if(button_flag==2)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            clean_cancel_msg.data = 1;
                            task_cmd_pub.publish(clean_cancel_msg);

                            cout<<"clean_cancel_msg_data:"<<(int)clean_cancel_msg.data<<endl;
                            cout<<"任务执行页面：暂停"<<endl;
                            button_flag = 2;
                        }
                    }
                    else if(temp8==0x02)
                    {
                        if(button_flag==3)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            clean_resume_msg.data = 2;
                            task_cmd_pub.publish(clean_resume_msg);

                            cout<<"clean_resume_msg_data:"<<(int)clean_resume_msg.data<<endl;
                            cout<<"任务执行页面：继续"<<endl;
                            button_flag = 3;
                        }                
                    }
                    
                    else break;

                    break;

                case 0x19:
                    cout<<"扫地任务设置页面"<<endl;
                    if(temp8==0x01)
                    {
                        if(button_flag==111)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            clean_start_msg.data = 11;
                            task_cmd_pub.publish(clean_start_msg);

                            cout<<"clean_start_msg_data:"<<(int)clean_start_msg.data<<endl;
                            cout<<"扫地任务设置：任务一"<<endl;
                            
                            button_flag = 111;
                        }

                    }
                    else if (temp8==0x02)
                    {
                        if(button_flag==112)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            clean_start_msg.data = 12;
                            task_cmd_pub.publish(clean_start_msg);

                            cout<<"clean_start_msg_data:"<<(int)clean_start_msg.data<<endl;
                            cout<<"扫地任务设置：任务二"<<endl;
                            button_flag = 112;
                        }
                    }
                    else if (temp8==0x03)
                    {
                        if(button_flag==113)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            clean_start_msg.data = 13;
                            task_cmd_pub.publish(clean_start_msg);

                            cout<<"clean_start_msg_data:"<<(int)clean_start_msg.data<<endl;
                            cout<<"扫地任务设置：任务三"<<endl;
                            button_flag = 113;
                        }
                    }
                    else if (temp8==0x04)
                    {
                        if(button_flag==114)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            clean_start_msg.data = 14;
                            task_cmd_pub.publish(clean_start_msg);

                            cout<<"clean_start_msg_data:"<<(int)clean_start_msg.data<<endl;
                            cout<<"扫地任务设置：任务四"<<endl;
                            button_flag = 114;
                        }
                    }
                    else if (temp8==0x05)
                    {
                        if(button_flag==115)
                        {
                            cout<<"button has been pressed!"<<endl;
                            break;
                        }
                        else{
                            clean_start_msg.data = 15;
                            task_cmd_pub.publish(clean_start_msg);

                            cout<<"clean_start_msg_data:"<<(int)clean_start_msg.data<<endl;
                            cout<<"扫地任务设置：任务五"<<endl;
                            button_flag = 115;
                        }
                    }
                    
                    

                    else break;
                    break;

                default:
                    
                    break;
                }

            }
            else{
                 cout<<"ui_data_receiver_buffer:"<<"input error!"<<endl;
            }
            



         
        }
        usleep(500000);
        memset(ui_data_receiver_buffer,'\0',sizeof(ui_data_receiver_buffer));
        // cout<<"ui_data_receiver_size2:"<<ui_data_receiver_size<<endl;
    }
}
void User_interface::ui_data_sender()
{
    //下面的代码是发送数据到ui串口
    //可以加入判断语句，将各个传感器状态进行判断，如果为false 或者需要持续发送的数据，则发送数据，触摸屏显示的传感器状态默认都为正常。

}

void User_interface::ui_data_parser()
{
      //解析ui 数据 并发布到pilot_bridge

      //发布到pilot_bridge的数据

}

//订阅pilot_bridge的数据的回调函数,判断传感器状态，并更新触摸屏状态
void User_interface::pilot_bridge_callback(const pilot_bridge::sensor_status::ConstPtr& msg)
{
    //将状态反馈给触摸屏
    // if(msg->imu_status == true)
    // {
    //     //触摸屏显示imu正常
    //     imu_status_buffer[0]=0x5A;
    //     imu_status_buffer[1]=0xA5;
    //     imu_status_buffer[2]=0x04;
    //     imu_status_buffer[3]=0x82;
    //     imu_status_buffer[4]=0xDD;
    //     imu_status_buffer[5]=0xDD;
    //     imu_status_buffer[6]=0x01;
    //     imu_status_buffer[7]=0x00;
    // }
    // else
    // {
    //     //触摸屏显示imu异常
    // }

    if(msg->front_laser_status == true && msg->rear_laser_status == true)
    {
        //触摸屏显示雷达正常
        front_laser_status_buffer[0]=0x5A;
        front_laser_status_buffer[1]=0xA5;
        front_laser_status_buffer[2]=0x05;
        front_laser_status_buffer[3]=0x82;
        front_laser_status_buffer[4]=0x11;
        front_laser_status_buffer[5]=0x02;
        front_laser_status_buffer[6]=0x00;
        front_laser_status_buffer[7]=0x01;
        ser.write(front_laser_status_buffer, BUFFER_SIZE);
        
    }
    else
    {
        //触摸屏显示激光雷达异常
        front_laser_status_buffer[0]=0x5A;
        front_laser_status_buffer[1]=0xA5;
        front_laser_status_buffer[2]=0x05;
        front_laser_status_buffer[3]=0x82;
        front_laser_status_buffer[4]=0x11;
        front_laser_status_buffer[5]=0x02;
        front_laser_status_buffer[6]=0x00;
        front_laser_status_buffer[7]=0x02;
        ser.write(front_laser_status_buffer, BUFFER_SIZE);
    }

    if(msg->sonar1_status && msg->sonar2_status && msg->sonar3_status && msg->sonar4_status && msg->sonar5_status &&
        msg->sonar6_status && msg->sonar7_status && msg->sonar8_status)
    {
        //触摸屏超声波雷达正常
        sonar_status_buffer[0]=0x5A;
        sonar_status_buffer[1]=0xA5;
        sonar_status_buffer[2]=0x05;
        sonar_status_buffer[3]=0x82;
        sonar_status_buffer[4]=0x11;
        sonar_status_buffer[5]=0x06;
        sonar_status_buffer[6]=0x00;
        sonar_status_buffer[7]=0x01;
        ser.write(sonar_status_buffer, BUFFER_SIZE);
    }
    else
    {
        //触摸屏显示异常
        sonar_status_buffer[0]=0x5A;
        sonar_status_buffer[1]=0xA5;
        sonar_status_buffer[2]=0x05;
        sonar_status_buffer[3]=0x82;
        sonar_status_buffer[4]=0x11;
        sonar_status_buffer[5]=0x06;
        sonar_status_buffer[6]=0x00;
        sonar_status_buffer[7]=0x02;
        ser.write(sonar_status_buffer, BUFFER_SIZE);
    }

    //调用uisender更新传感器状态
    
}
