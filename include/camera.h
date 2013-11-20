#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include "flycapture/FlyCapture2.h"

namespace pgr_camera {

class Camera {
  public:
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
    void feedImages();
    ~Camera();

  private:
    void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo );
    
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport it;
    bool ok;

    int width, height, fps, skip_frames, frames_to_skip;
    std::string frame;
    int serial;
    bool rotate;

    camera_info_manager::CameraInfoManager info_mgr;

    image_transport::Publisher pub;
    ros::Publisher info_pub;

    FlyCapture2::GigECamera cam;
    FlyCapture2::Image rawImage;  
    FlyCapture2::Image convertedImage;
    boost::thread image_thread;
};

};

