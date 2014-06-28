#ifndef PGR_CAMERA__H_
#define PGR_CAMERA__H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>

#include "camera_config.h"
#include "pgr_camera/PGRCameraConfig.h"


#include "flycapture/FlyCapture2.h"

namespace pgr_camera {

class Camera {
  public:
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(const sensor_msgs::ImagePtr &image, ros::Time time);
    void feedImages();
    ~Camera();

  private:
    typedef pgr_camera::PGRCameraConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

    void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo );

    void configCb(Config &config, uint32_t level);
    
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

    boost::mutex cam_mutex_;
    FlyCapture2::GigECamera cam;
    FlyCapture2::Image rawImage;
    FlyCapture2::Image convertedImage;
    boost::thread image_thread;

    // Dynamic reconfigure
    //boost::recursive_mutex config_mutex_;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

    CameraConfig* camera_config_manager_;

    //Temporary cvMat for rotating images
    cv::Mat tmp_cvmat;
};

};

#endif
