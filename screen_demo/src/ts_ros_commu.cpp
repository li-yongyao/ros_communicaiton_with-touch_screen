#include "scree_demo/ts_ros_commu.h"

using namespace std;

namespace robot_one{

Ros_read::Ros_read(ros::NodeHandle& n)
{

    serial_status = false;
    cout<<"get it!"<<endl;

}

bool Ros_read::initialize()
{
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

}