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

