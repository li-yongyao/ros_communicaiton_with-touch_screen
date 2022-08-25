#include "user_interface/user_interface.h"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_interface");
    ros::NodeHandle node_handle("~");
    User_interface *ui = new User_interface(&node_handle);
    ros::Rate rate(1);
    while(ros::ok())
    {
        ui->pilot_bridge_sub = ui->nh.subscribe("/all_info", 10, &User_interface::pilot_bridge_callback, ui);
        ros::spin();
        rate.sleep();
    }
    // ui->pilot_bridge_sub = ui->nh.subscribe("/all_info", 10, &User_interface::pilot_bridge_callback, ui);

    // ui.pilot_bridge_sub = ui.nh.subscribe("pilot_bridge", 10, &user_interface::pilot_bridge_callback, &ui);

    // ros::spin();
    // ui->ui_data_receiver_thread.detach();
    delete ui;
    return 0;
}