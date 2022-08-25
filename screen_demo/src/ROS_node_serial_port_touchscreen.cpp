
//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include "screen_demo/ts_ROS_node.hpp" // header in local directory
//
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include <nav_msgs/Odometry.h>

using namespace std;
//数据类型很重要，对于通讯协议是16进制数据码的，收发数据应放在数组中，例如unsigned char s_buffer[sBUFFERSIZE];
//协议是字符串的用字符串类型，如std_msgs::String serial_data
#define sBUFFERSIZE 8//send buffer size 串口发送缓存长度8
unsigned char s_buffer[sBUFFERSIZE];//发送缓存，这是一个数组
int r_n=8;

int device_status;   // 系统硬件状态
bool LiDARStat;  // 激光雷达硬件状态
bool cameraStat;  // 摄像头硬件状态
bool sonarStat;  // 超声波雷达硬件状态
double battPower;  // 电池电量
bool chargeStat;  // 充电状态
double waterLevel;  // 水箱水位
int timecollapse;  // 运行时间
int cleanarea;  // 清洁面积
int cleantime;  // 清洁时间
bool workmode;  // 工作模式 0 自动驾驶模式  1 手动模式
int cleanmode;  // 清洁模式
bool autocharge;  // 自动充电设置状态   true 设置自动充电
bool waterchange;  // 自动换水设置状态  true 设置自动换水
bool brushStat;  // 设置刷盘状态
bool fanStat;  // 风扇状态
bool speakerStat;  // 扬声器状态
int init_x;  // 初始位置x
int init_y;  // 初始位置y
int init_angle;  // 初始角度
int taskStat;  // 任务状态

int ts_start_hour;   //清洁开始小时
int ts_start_min;   // 清洁开始分钟
int ts_stop_hour;   // 清洁结束小时
int ts_stop_min;  // 清洁开始小时
bool currentask;  // 当前任务状态
bool taskloop;  // 循环执行时间
bool load_maplist_ts;
unsigned char cleantask_ts;
bool emergencyStop;
int totalMileage;
int uptime;
std::string IP_address;
int software_version;

// open serial port in this domain for all write and read
 
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
 
//i要转化的十进制整数，width转化后的宽度，位数不足则补0 
std::string dec2hex(int i, int width)
 {
    std::stringstream ioss; //定义字符串流 
    std::string s_temp; //存放转化后字符
    ioss << std::hex << i; //以十六制形式输出 
    ioss >> s_temp; 
    if (width > s_temp.size()) { 
        std::string s_0(width - s_temp.size(), '0'); //位数不够则补0                 
        s_temp = s_0 + s_temp; //合并
    } 
    std::string s = s_temp.substr(s_temp.length() - width,   s_temp.length()); //取右width位 
    return s;
 }    

//上述代码是将字符串转换为16进制，并将该16进制数用string保存起来

string str2hex(const string& str) //transfer string to hex-string
{
    string result="0x";
    string tmp;
    stringstream ss;
    for(int i=0;i<str.size();i++)
    {
        ss<<hex<<int(str[i])<<endl;
        ss>>tmp;
        result+=tmp;
    }
    return result;
}



/////////////////////////////////
//  read variables from ROS space
/////////////////////////////////

void ros_read::read_ros_device_status(const nav_msgs::Odometry::ConstPtr& odom)
{
	//更新导航系统状态%

}

void ros_read::read_ros_LiDAR_status(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if (scan->ranges[1]>0)
	{
		LiDARStat=true;
	}
	else
	{
		LiDARStat=false;
	}
}

void ros_read::read_ros_sonar_status(const sensor_msgs::Range::ConstPtr& msg)
{
	if (msg->range>0)
	{
		sonarStat=true;
	}
	else
	{
		sonarStat=false;
	}
}

void ros_read::read_ros_camera_status(const sensor_msgs::ImageConstPtr& msg)
{
	if (msg->data>0)
	{
		cameraStat=true;
	}
	else
	{
		cameraStat=false;
	}
}

void ros_read::read_ros_batt_status(const std_msgs::Float32::ConstPtr& msg)
{
	if (msg->data>0)
	{
		battStat=true;
	}
	else
	{
		battStat=false;
	}
}

void ros_read::read_ros_charger_status(const std_msgs::Float32::ConstPtr& msg)
{
	if (msg->data>0)
	{
		chargerStat=true;
	}
	else
	{
		chargerStat=false;
	}
}

void ros_read::read_ros_water_level(const std_msgs::Float32::ConstPtr& msg)
{
	if (msg->data>0)
	{
		waterLevel=true;
	}
	else
	{
		waterLevel=false;
	}
}

void ros_read::read_ros_time_collapse(const std_msgs::Float32::ConstPtr& msg)
{
	if (msg->data>0)
	{
		timecollapse=true;
	}
	else
	{
		timecollapse=false;
	}
}

void ts_read::read_ts_workmode() // two-way communications
{
	// 读取触摸屏工作模式设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x13;
    s_buffer[5]=0x00;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Set work mode to self-driving mode");
		workmode=true;
	}
	// read manual reg to check mode
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x13;
    s_buffer[5]=0x10;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Set work mode to self-driving mode");
		workmode=true;
	}
	std_msgs::Bool msg;
	msg.data=workmode;
	workmode_pub(msg);
}

void ts_read::read_ts_cleanmode()
{
		// 读取触摸屏清洁模式设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x13;
    s_buffer[5]=0x20;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Set clean mode to all-area mode");
		cleanmode=0;
	}
	// read manual reg to check mode
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x13;
    s_buffer[5]=0x30;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Set work mode to maintain mode");
		cleanmode=1;
	}
	std_msgs::Int8 msg;
	msg.data=cleanmode;
	workmode_pub(msg);
}

void ros_read::read_ros_cleantask_queue()  //已有任务列表
{
	
}

void ts_read::read_ts_cleantask_queue()  //返回已有任务列表
{
	// 读取触摸屏已有任务列表选择 目前只支持5个 
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x19;
    s_buffer[5]=0x00;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	cleantask_ts=r_buffer[7][8]; // read back new task name
}

void ts_read::read_ros_maplist()   // 当前任务地图列表
{
	
}
void ts_read::read_ts_load_maplist()   // 加载当前任务地图列表
{
	// 读取触摸屏已有任务列表选择 目前只支持5个 
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x19;
    s_buffer[5]=0x00;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0) // read back load maplist
	{
		load_maplist_ts=true;
	}
	else
	{
		load_maplist_ts=false;
	}
}

void ros_read::read_ros_map_queue()   // 清洁地图列表
{
	
}

void ts_read::read_ts_map_queue()   //// 清洁地图列表按键返回
{
	// 读取触摸屏清洁模式设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x21;
    s_buffer[5]=0x60;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	new_task=r_buffer[7][8][9][10][11][12]; // read back new task name
}

void ts_read::read_ts_newtask_queue()
{
	// 读取触摸屏清洁模式设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x0A;
    s_buffer[3]=0x83;

    s_buffer[4]=0x21;
    s_buffer[5]=0x80;

    s_buffer[6]=0x03; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	new_task=r_buffer[7][8][9][10][11][12]; // read back new task name 
	
}

void ts_read::read_ts_currentask()
{
	// 读取触摸屏清洁模式设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x17;
    s_buffer[5]=0x00;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Set to current task mode");
		currentask=true;
	}
	else
	{
		currentask=false;
	}
}

void ts_read::read_ts_taskschedule()
{
	// 读取触摸屏定时清洁时间设置（小时）
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x17;
    s_buffer[5]=0x16;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	ts_start_hour=r_buffer[7][8]; // hour
	
	// 读取触摸屏定时清洁时间设置（分钟）
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x17;
    s_buffer[5]=0x26;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	ts_min=r_buffer[7][8]; // min
	
	// 读取触摸屏定时清洁结束时间设置（小时）
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x17;
    s_buffer[5]=0x16;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	ts_stop_hour=r_buffer[7][8]; // hour
	
	// 读取触摸屏定时清洁结束时间设置（分钟）
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x17;
    s_buffer[5]=0x26;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	ts_stop_min=r_buffer[7][8]; // min
	
	// 读取触摸屏循环清洁模式设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x17;
    s_buffer[5]=0x50;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Set to task loop mode");
		taskloop=true;
	}
	else
	{
		taskloop=false;
	}
		
	std_msgs::Bool msg;
	msg.data=taskloop;
	taskloop_pub(msg);
}

void ts_read::read_ts_autocharge()
{
		// 读取触摸屏自动充电设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x14;
    s_buffer[5]=0x00;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Start auto-charging");
		autocharge=true;
	}
	// read manual reg to check mode
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x14;
    s_buffer[5]=0x10;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Stop auto-charging");
		autocharge=false;
	}
	
	std_msgs::Bool msg;
	msg.data=autocharge;
	autocharge_pub(msg);
}

void ts_read::read_ts_waterchange()
{
		// 读取触摸屏换水设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x14;
    s_buffer[5]=0x20;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Start changing water");
		waterchange=true;
	}
	// read reg to water change
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x14;
    s_buffer[5]=0x30;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Stop water changing");
		waterchange=false;
	}

	std_msgs::Bool msg;
	msg.data=waterchange;
	waterchange_pub(msg);
}

void ros_read::read_ros_cleanarea()
{
	
}

void ros_read::read_ros_cleantime()
{
	
}

void ts_read::read_ts_brush_status()
{
	// 读取触摸屏刷盘设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x16;
    s_buffer[5]=0x00;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Start brush!!");
		brushStat=true;
	}
	// read manual reg to brush
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x16;
    s_buffer[5]=0x10;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Stop brush");
		brushStat=false;
	}

	std_msgs::Bool msg;
	msg.data=brushStat;
	brushStat_pub(msg);
}

void ts_read::read_ts_fan_status()
{
		// 读取触摸屏风扇设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x16;
    s_buffer[5]=0x20;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Start fan!!");
		fanStat=true;
	}
	// read reg to fan
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x16;
    s_buffer[5]=0x30;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Stop fan!!");
		fanStat=false;
	}

	std_msgs::Bool msg;
	msg.data=fanStat;
	fanStat_pub(msg);
}

void ts_read::read_ts_speaker_status()
{
		// 读取触摸屏扬声器设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x17;
    s_buffer[5]=0x80;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Start speaker!!");
		speakerStat=true;
	}
	// read reg to speaker
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x16;
    s_buffer[5]=0x30;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Stop speaker");
		speakerStat=false;
	}

	std_msgs::Bool msg;
	msg.data=speakerStat;
	speakerStat_pub(msg);
}

void ts_read::read_ts_init_position()
{
	// 读取触摸屏初始位置x设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x18;
    s_buffer[5]=0x00;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	ROS_INFO_STREAM("Start speaker!!");
	int init_x=r_buffer[7][8];
	// 读取触摸屏初始位置y设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x18;
    s_buffer[5]=0x10;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	ROS_INFO_STREAM("Start speaker!!");
	int init_y=r_buffer[7][8];
	// 读取触摸屏初始位置角度设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x18;
    s_buffer[5]=0x20;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	ROS_INFO_STREAM("Start speaker!!");
	int init_angle=r_buffer[7][8];

	std_msgs::int8 msg;
	msg.data=init_x;
	init_x_pub(msg);
	
	std_msgs::int8 msg;
	msg.data=init_y;
	init_y_pub(msg);
	
	std_msgs::int8 msg;
	msg.data=init_angle;
	init_angle_pub(msg);
}

void ts_read::read_ts_task_status()
{
	// 读取触摸屏任务设置
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x18;
    s_buffer[5]=0x30;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Start task!!");
		taskStat=1;
	}
	else
	{
		taskStat=0;
	}
	// read reg to task settings
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x18;
    s_buffer[5]=0x40;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Task pause!!");
		taskStat=2;
	}

	// read reg to task settings
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x06;
    s_buffer[3]=0x83;

    s_buffer[4]=0x18;
    s_buffer[5]=0x50;

    s_buffer[6]=0x01; //  00 正在检测(白色)；01 正常（绿色）；
	ser.write(s_buffer,sBUFFERSIZE); // write reg address and read back from serial port  
	uint8_t r_buffer[1024];
    ser.read(r_buffer,r_n);
	if (r_buffer[8]==0x01)
	{
		ROS_INFO_STREAM("Task reset!!");
		taskStat=3;
	}
}
/////////////////////////////////
//  write to touch screen
/////////////////////////////////

void ts_write::write_ts_device_status(int device_status, int sw_version, std::string IP_address)
{
	//更新导航模块状态%
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x11;
    s_buffer[5]=0x00;

    s_buffer[6]=0x00;
    s_buffer[7]=0x01;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
    ser.write(s_buffer,sBUFFERSIZE);
	//版本信息%
	Sw_version_hex=dec2hex(sw_version,2);
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x12;
    s_buffer[5]=0x02;

    s_buffer[6]=0x00;
    s_buffer[7]=sw_version;  //  
    ser.write(s_buffer,sBUFFERSIZE);	
	
	//IP地址%
	IP_address_hex=str2hex(IP_address,2);
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x07;
    s_buffer[3]=0x82;

    s_buffer[4]=0x12;
    s_buffer[5]=0x04;

    s_buffer[6]=0x00;
    s_buffer[7]=0x02;  

    s_buffer[8]=IP_address_hex[0];
    s_buffer[9]=IP_address_hex[1];  //  
    ser.write(s_buffer,sBUFFERSIZE);
	
	s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x07;
    s_buffer[3]=0x82;

    s_buffer[4]=0x12;
    s_buffer[5]=0x08;

    s_buffer[6]=0x00;
    s_buffer[7]=0x03;  

    s_buffer[8]=IP_address_hex[2];
    s_buffer[9]=IP_address_hex[3];  //  
    ser.write(s_buffer,sBUFFERSIZE);
	
}

void ts_write::write_ts_LiDAR_status(bool LiDARStat)
{
	//更新激光雷达状态%
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x11;
    s_buffer[5]=0x02;

    s_buffer[6]=0x00;
    s_buffer[7]=0x01;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
    ser.write(s_buffer,sBUFFERSIZE);
}

void ts_write::write_ts_sonar_status(bool sonarStat)
{
	//更新超声波雷达状态%
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x11;
    s_buffer[5]=0x06;

    s_buffer[6]=0x00;
    s_buffer[7]=0x01;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
    ser.write(s_buffer,sBUFFERSIZE);
}

void ts_write::write_ts_camera_status(bool cameraStat)
{
	//更新摄像头模块状态%
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x11;
    s_buffer[5]=0x08;

    s_buffer[6]=0x00;
    s_buffer[7]=0x01;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
    ser.write(s_buffer,sBUFFERSIZE);
}

void ts_write::write_ts_charger_status(bool chargeStat)
{
	//更新自动充电模块状态%
    s_buffer[0]=0x5A;
    s_buffer[1]=0xA5;

    s_buffer[2]=0x05;
    s_buffer[3]=0x82;

    s_buffer[4]=0x11;
    s_buffer[5]=0x04;

    s_buffer[6]=0x00;
    s_buffer[7]=0x01;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
    ser.write(s_buffer,sBUFFERSIZE);
}

void ts_write::write_ts_batt_status(double battPower)
{
	//更新电池状态%
		int BATT_POW;
		BATT_POW=int(batt_power*100);
		Batt_pow_dex=dec2hex(BATT_POW,2);
	
	if (BATT_POW <= 10)  // less than 10%
	{

		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x11;
		s_buffer[5]=0x14;

		s_buffer[6]=0x00;
		s_buffer[7]=Batt_pow_dex;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
		/////////////////////////////////////
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x50;
		s_buffer[5]=0x20;

		s_buffer[6]=0xFF;
		s_buffer[7]=0x00;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
    }
	
	else
	{
		BATT_POW=int(batt_power*100);
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x11;
		s_buffer[5]=0x16;

		s_buffer[6]=0x00;
		s_buffer[7]=Batt_pow_dex;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
		/////////////////////////////////////
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x50;
		s_buffer[5]=0x30;

		s_buffer[6]=0xFF;
		s_buffer[7]=0x00;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
	}
}

void ts_write::write_ts_water_level(double waterLevel)
{
	//更新水箱水量%
		int WATER_LEVEL;
		WATER_LEVEL=int(water_level*100);
		Water_level_dex=dec2hex(WATER_LEVEL,2);
	
	if (WATER_LEVEL <= 10)  // less than 10%
	{

		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x11;
		s_buffer[5]=0x10;

		s_buffer[6]=0x00;
		s_buffer[7]=Water_level_dex;  
		ser.write(s_buffer,sBUFFERSIZE);
		/////////////////////////////////////
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x50;
		s_buffer[5]=0x00;

		s_buffer[6]=0xFF;
		s_buffer[7]=0x00;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
    }
	
	else
	{
		WATER_LEVEL=int(water_level*100);
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x11;
		s_buffer[5]=0x12;

		s_buffer[6]=0x00;
		s_buffer[7]=Water_level_dex;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
		/////////////////////////////////////
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x50;
		s_buffer[5]=0x10;

		s_buffer[6]=0xFF;
		s_buffer[7]=0x00;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
	}
}

void ts_write::write_ts_time_collapse(int timecollapse)
{
	
}

void ts_write::set_workmode()  // deprecated ..................
{
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x06;
		s_buffer[3]=0x83;
		s_buffer[4]=0x11;
		s_buffer[5]=0x12;

		s_buffer[6]=0x00;
		s_buffer[7]=Water_level_dex;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
}

void ts_write::set_cleanmode()     // deprecated ..................
{
	
}

void ts_write::write_ts_cleantask_queue()     // deprecated ..................
{
	
}

void ts_write::write_ts_autocharge()    // deprecated ..................
{
	
}

void ts_write::write_ts_waterchange()  // deprecated ..................
{
		//更新水箱水量%
		int WATER_LEVEL;
		WATER_LEVEL=int(water_level*100);
		Water_level_hex=dec2hex(WATER_LEVEL,2);
	
	if (WATER_LEVEL <= 10)  // less than 10%
	{

		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x15;
		s_buffer[5]=0x30;

		s_buffer[6]=0x00;
		s_buffer[7]=Water_level_hex;  
		ser.write(s_buffer,sBUFFERSIZE);
		/////////////////////////////////////
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x50;
		s_buffer[5]=0x80;

		s_buffer[6]=0xFF;
		s_buffer[7]=0x00;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
    }
	
	else
	{
		WATER_LEVEL=int(water_level*100);
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x15;
		s_buffer[5]=0x40;

		s_buffer[6]=0x00;
		s_buffer[7]=Water_level_hex;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
		/////////////////////////////////////
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x50;
		s_buffer[5]=0x90;

		s_buffer[6]=0xFF;
		s_buffer[7]=0x00;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
		/////////////////////////////////////
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x05;
		s_buffer[3]=0x82;
		s_buffer[4]=0x50;
		s_buffer[5]=0x90;

		s_buffer[6]=0x15;
		s_buffer[7]=0x40;  //  00 正在检测(白色)；01 正常（绿色）；02 异常 （红色）
		ser.write(s_buffer,sBUFFERSIZE);
	}
}

void ts_write::write_ts_cleanarea(int cleanarea)
{
		cleanarea_hex=dec2hex(cleanarea,2);
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x07;
		s_buffer[3]=0x82;
		s_buffer[4]=0x15;
		s_buffer[5]=0x00;

		s_buffer[6]=0x00;
		s_buffer[7]=0x00;
		s_buffer[8]=cleanarea_hex; 
		s_buffer[9]=cleanarea_hex;   		
		ser.write(s_buffer,sBUFFERSIZE);
}

void ts_write::write_ts_cleantime(int cleantime)
{
		String cleantime_hex=dec2hex(cleantime,2);
		s_buffer[0]=0x5A;
		s_buffer[1]=0xA5;

		s_buffer[2]=0x07;
		s_buffer[3]=0x82;
		s_buffer[4]=0x15;
		s_buffer[5]=0x00;

		s_buffer[6]=0x00;
		s_buffer[7]=0x00;
		s_buffer[8]=cleanarea_hex; 
		s_buffer[9]=cleanarea_hex;   		
		ser.write(s_buffer,sBUFFERSIZE);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROS_ts");
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
    ros::Rate loop_rate(10);  // ROS node更新频率
	ts_read ts_read;
	ros_read ros_read;
	ts_write ts_write;
	
	ros::Subscriber sub = n.subscribe("odom", 10, read_ros_device_status);
	ros::Subscriber sub = n.subscribe("scan", 10, read_ros_LiDAR_status);
	ros::Subscriber sub = n.subscribe("sonarStat", 10, read_ros_sonar_status);
	ros::Subscriber sub = n.subscribe("cameraStat", 10, read_ros_camera_status);
	ros::Subscriber sub = n.subscribe("/bw_auto_dock/Batterypower", 10, read_ros_batt_status);
	ros::Subscriber sub = n.subscribe("/bw_auto_dock/Chargestatus", 10, read_ros_charger_status);
	ros::Subscriber sub = n.subscribe("waterLevel", 10, read_ros_water_level);
	ros::Subscriber sub = n.subscribe("timecollapse", 10, read_ros_time_collapse);
	ros::Publisher workmode_pub = n.advertise<std_msgs::Bool>("workmode", 10);
	ros::Publisher cleanmode_pub = n.advertise<std_msgs::Bool>("cleanmode", 10);
	ros::Publisher autocharge_pub = n.advertise<std_msgs::Int8>("autocharge", 10);
	ros::Publisher waterchange_pub = n.advertise<std_msgs::Bool>("waterchange", 10);
	ros::Subscriber sub = n.subscribe("cleanarea", 10, read_ros_cleanarea);
	ros::Subscriber sub = n.subscribe("cleantime", 10, read_ros_cleantime);
	ros::Subscriber sub = n.subscribe("cleantask_queue", 10, read_ros_cleantask_queue);
	ros::Subscriber sub = n.subscribe("map_queue", 10, read_ros_map_queue);
	ros::Subscriber sub = n.subscribe("maplist", 10, read_ros_maplist);
	ros::Publisher brushStat_pub = n.advertise<std_msgs::Bool>("brushStat", 10);
	ros::Publisher fanStat_pub = n.advertise<std_msgs::Bool>("fanStat", 10);
	ros::Publisher taskloop_pub = n.advertise<std_msgs::Bool>("taskloop", 10);
	ros::Publisher init_x_pub = n.advertise<std_msgs::Bool>("init_x", 10);
	ros::Publisher init_y_pub = n.advertise<std_msgs::Bool>("init_y", 10);
	ros::Publisher init_angle_pub = n.advertise<std_msgs::Bool>("init_angle", 10);

    while (ros::ok())
    {
            size_t n = ser.available(); //获取缓冲区的字节数
			ros_read.read_ros_device_status(); // 定时读取触摸屏导航模块寄存器
			ts_write.write_device_status(int device_status, int sw_version, string IP_address);
		    ros_read.read_ros_LiDAR_status();	// 定时读取触摸屏激光雷达寄存器
			ts_write.write_LiDAR_status(int lidar_status);
		    ros_read.read_ros_sonar_status();	// 定时读取触摸屏超声波寄存器
			ts_write.write_sonar_status();
		    ros_read.read_ros_camera_status();	// 定时读取触摸屏摄像头寄存器
			ts_write.write_camera_status();
		    ros_read.read_ros_batt_status();	// 定时读取触摸屏电池寄存器
			ts_write.write_batt_status(double batt_power);
			ros_read.read_ros_charger_status();	// 定时读取触摸屏充电模块寄存器
			ts_write.write_charger_status();
			ros_read.read_ros_water_level();	// 定时读取触摸屏水箱寄存器
			ts_write.write_water_level(double water_level);
			ros_read.read_ros_time_collapse();	// 定时读取触摸屏运行时间寄存器
			ts_write.write_time_collapse();
			ts_read.read_ts_workmode();	// 定时读取触摸屏工作模式寄存器
			//ts_write.set_workmode();
			ts_read.read_ts_cleanmode();	// 定时读取触摸屏清扫模式寄存器
			//ts_write.set_cleanmode();
			ts_read.read_ts_cleantask_queue();	// 定时读取触摸屏清扫任务寄存器
			//ts_write.write_cleantask_queue();
			ts_read.read_ts_autocharge();	// 定时读取触摸屏自动充电寄存器
			//ts_write.write_autocharge();
			ts_read.read_ts_waterchange();	// 定时读取触摸屏自动加水寄存器
			//ts_write.write_waterchange();
			ros_read.read_ros_cleanarea();	// 定时读取触摸屏清扫面积寄存器
			ts_write.write_cleanarea();
			ros_read.read_ros_cleantime();	// 定时读取触摸屏清扫时间寄存器
			ts_write.write_cleantime();
			ts_read.read_ts_brush_status();	// 定时读取触摸屏刷盘寄存器
			
			ts_read.read_ts_fan_status();	// 定时读取触摸屏风扇寄存器
			
            if(n!=0)
            {
  
                uint8_t buffer[1024];
                
                ser.read(buffer, n);
            
                for(int i=0; i<n; i++)
                {
                    //16进制的方式打印到屏幕
                    std::cout << std::hex << (buffer[i] & 0xff) << " ";
                }
            }
            loop_rate.sleep();
    }
    ser.close();
    return 0;
}



