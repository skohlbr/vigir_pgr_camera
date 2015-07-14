/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt, Team ViGIR, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Based on PGR camera code here:
// https://github.com/RCPRG-ros-pkg/pgr_camera

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

    //int width, height, fps, skip_frames, frames_to_skip;
    int roi_offset_x_, roi_offset_y_, roi_width_, roi_height_;

    std::string frame;
    int serial;
    bool rotate;
    bool mono_mode_;

    camera_info_manager::CameraInfoManager info_mgr;

    image_transport::Publisher pub;
    ros::Publisher info_pub;

    boost::mutex cam_mutex_;
    FlyCapture2::GigECamera cam;
    FlyCapture2::Image rawImage;
    FlyCapture2::Image convertedImage;
    boost::thread image_thread;

    // Dynamic reconfigure
    boost::recursive_mutex config_mutex_;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;
    Config config_;
    bool config_updated_;

    CameraConfig* camera_config_manager_;

    //Temporary cvMat for rotating images
    cv::Mat tmp_cvmat;
};

};

#endif
