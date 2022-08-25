/**
 * @file /include/screen_demo/ts_ROS_node.hpp
 *
 *
 * @date November 2022
 **/
#ifndef ts_ROS_NODE_H
#define ts_ROS_NODE_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <serial/serial.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

/*****************************************************************************
** Interface [rosserial]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

namespace robot_one{
class ts_write
{
public:
	void write_ts_device_status(int device_status, int sw_version, std::string IP_address);
	void write_ts_LiDAR_status(bool LiDARStat);
	void write_ts_sonar_status(bool sonarStat);
	void write_ts_camera_status(bool cameraStat);
	void write_ts_batt_status(double battPower);
	void write_ts_charger_status(bool chargeStat);
	void write_ts_water_level(double waterLevel);
	void write_ts_time_collapse(int timecollapse);
	void set_workmode();    // deprecated 
	void set_cleanmode();    // deprecated 
	void write_ts_cleantask_queue();    // deprecated 
	void write_ts_autocharge();     // deprecated 
	void write_ts_waterchange();   // deprecated 
	void write_ts_cleanarea(int cleanarea);
	void write_ts_cleantime(int cleantime);

};

class ts_read
{
public:
	void read_ts_workmode();
	void read_ts_cleanmode();
	void read_ts_cleantask_queue();
	void read_ros_maplist();
	void read_ts_load_maplist();
	void read_ts_map_queue();
	void read_ts_newtask_queue();
	void read_ts_currentask();
	void read_ts_taskschedule();
	void read_ts_autocharge();
	void read_ts_waterchange();
	void read_ts_brush_status();
	void read_ts_fan_status();
	void read_ts_speaker_status();
	void read_ts_init_position();
	void read_ts_task_status();

};

class ros_read
{
public:
	void read_ros_device_status(const nav_msgs::Odometry::ConstPtr& odom);
	void read_ros_LiDAR_status(const sensor_msgs::LaserScan::ConstPtr& scan);
	void read_ros_sonar_status(const sensor_msgs::Range::ConstPtr& msg);
	void read_ros_camera_status(const sensor_msgs::ImageConstPtr& msg);
	void read_ros_batt_status(const std_msgs::Float32::ConstPtr& msg);
	void read_ros_charger_status(const std_msgs::Float32::ConstPtr& msg);
	void read_ros_water_level(const std_msgs::Float32::ConstPtr& msg);
	void read_ros_time_collapse(const std_msgs::Float32::ConstPtr& msg);
	void read_ros_cleantask_queue();
	void read_ros_map_queue();
	void read_ros_cleanarea();
	void read_ros_cleantime();

};

}  // namespace robot_one

#endif // ts_ROS_NODE_H
