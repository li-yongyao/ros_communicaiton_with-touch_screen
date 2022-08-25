#include  <ros/ros.h>

#include "scree_demo/ts_ros_commu.h"

using namespace std;

bool serial_status = false;

int main(int argc,  char **argv)
{

    ros::init(argc, argv, "ts_ros_commu_node");
    ros::NodeHandle nh;


    // robot_one::Ros_readPtr read(
    //                         new robot_one::Ros_read ros_read(nh));
    
    robot_one::Ros_read ros_read(nh);
    serial_status = ros_read.initialize();
 
    if(!serial_status)
    {
        ROS_INFO("Cannot initial the touch screen...");
    }
    




    return 0;
}