#include <ros/ros.h>
#include <nodelet/loader.h>

#include <camera.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pgr_camera");

  ros::NodeHandle nh("~");
  nh.setParam("num_worker_threads", 2);

  // Shared parameters to be propagated to nodelet private namespaces
  nodelet::Loader manager(true); // Don't bring up the manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;



  manager.load(ros::this_node::getName(), "pgr_camera/camera_nodelet", remappings, my_argv);

  ros::spin();
  return 0;
}