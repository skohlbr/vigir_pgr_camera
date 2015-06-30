#include <ros/ros.h>
#include <nodelet/loader.h>

#include "stereocamera.h"

int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_camera_stereo");

  pgr_camera::StereoCamera stereo(ros::NodeHandle(), ros::NodeHandle("~"));

  ros::spin();
  return 0;
}

