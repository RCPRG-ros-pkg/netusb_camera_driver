#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "netusb_node");
  nodelet::Loader manager(true);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;
  manager.load(ros::this_node::getName(), "netusb_camera/driver", remappings, my_argv);
  ros::spin();
}
