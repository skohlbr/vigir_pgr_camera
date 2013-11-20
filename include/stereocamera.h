#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

#include "flycapture/FlyCapture2.h"

namespace pgr_camera {

class StereoCamera {
  public:
    StereoCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(ros::Time time);
    void feedImages();
    ~StereoCamera();

  private:
    
    void initCamera(FlyCapture2::GigECamera &cam, const int serial);
    
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport it;
    bool ok;

    FlyCapture2::GigECamera cam_left, cam_right;
    FlyCapture2::Image rawImage_l, rawImage_r;
    FlyCapture2::Image convertedImage_l, convertedImage_r;
    
    int width, height, fps, skip_frames, frames_to_skip;
    std::string frame;
    int left_serial, right_serial;
    bool rotate_left, rotate_right;

    camera_info_manager::CameraInfoManager left_info_mgr, right_info_mgr;

    image_transport::Publisher left_pub, right_pub;
    ros::Publisher left_info_pub, right_info_pub;

    boost::thread image_thread;
};

};

