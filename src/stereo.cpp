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

#include "stereocamera.h"

using namespace sensor_msgs;

namespace pgr_camera {

StereoCamera::StereoCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh) :
  node(comm_nh), pnode(param_nh), it(comm_nh),
  left_info_mgr(ros::NodeHandle(comm_nh, "left"), "left_camera"),
  right_info_mgr(ros::NodeHandle(comm_nh, "right"), "right_camera") {

  /* default config values */
  width = 640;
  height = 480;
  fps = 10;
  skip_frames = 0;
  frames_to_skip = 0;
  frame = "camera";

  /* set up information managers */
  std::string left_url, right_url;

  pnode.getParam("left/camera_info_url", left_url);
  pnode.getParam("right/camera_info_url", right_url);

  left_info_mgr.loadCameraInfo(left_url);
  right_info_mgr.loadCameraInfo(right_url);

  /* pull other configuration */
  pnode.getParam("left/serial", left_serial);
  pnode.getParam("right/serial", right_serial);

  pnode.getParam("fps", fps);
  pnode.getParam("skip_frames", skip_frames);

  pnode.getParam("width", width);
  pnode.getParam("height", height);

  pnode.getParam("frame_id", frame);

  /* advertise image streams and info streams */
  left_pub = it.advertise("left/image_raw", 1);
  right_pub = it.advertise("right/image_raw", 1);

  left_info_pub = node.advertise<CameraInfo>("left/camera_info", 1);
  right_info_pub = node.advertise<CameraInfo>("right/camera_info", 1);

  /* initialize the cameras */

  initCamera(cam_left, left_serial);
  initCamera(cam_right, right_serial);

  /* and turn on the streamer */
  ok = true;
  image_thread = boost::thread(boost::bind(&StereoCamera::feedImages, this));
  
  cam_left.StartCapture();
  cam_right.StartCapture();
}

void StereoCamera::initCamera(FlyCapture2::GigECamera &cam, const int serial) {
  FlyCapture2::Error error;
  FlyCapture2::BusManager busMgr;
  
  FlyCapture2::PGRGuid guid;
  error = busMgr.GetCameraFromSerialNumber(serial, &guid);

  if (error != FlyCapture2::PGRERROR_OK)
  {
    printf( "Error\n" );
      //PrintError( error );
      //return -1;
  }

  // Connect to a camera
  error = cam.Connect(&guid);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    printf( "Error\n" );
      //PrintError( error );
      //return -1;
  }
  
  // Get the camera information
  FlyCapture2::CameraInfo camInfo;
  error = cam.GetCameraInfo(&camInfo);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    printf( "Error\n" );
      //PrintError( error );
      //return -1;
  }

  //PrintCameraInfo(&camInfo);

  FlyCapture2::GigEImageSettingsInfo imageSettingsInfo;
  error = cam.GetGigEImageSettingsInfo( &imageSettingsInfo );
  if (error != FlyCapture2::PGRERROR_OK)
  {
    printf( "Error\n" );
//          PrintError( error );
//P          return -1;
  }

  FlyCapture2::GigEImageSettings imageSettings;
  imageSettings.offsetX = 0;
  imageSettings.offsetY = 0;
  imageSettings.height = imageSettingsInfo.maxHeight;
  imageSettings.width = imageSettingsInfo.maxWidth;
  imageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;

  printf( "Setting GigE image settings...\n" );

  error = cam.SetGigEImageSettings( &imageSettings );
  if (error != FlyCapture2::PGRERROR_OK)
  {
    printf( "Error\n" );
//         PrintError( error );
//         return -1;
  }
}

void StereoCamera::sendInfo(ros::Time time) {
  CameraInfoPtr info_left(new CameraInfo(left_info_mgr.getCameraInfo()));
  CameraInfoPtr info_right(new CameraInfo(right_info_mgr.getCameraInfo()));

  info_left->header.stamp = time;
  info_right->header.stamp = time;
  info_left->header.frame_id = frame;
  info_right->header.frame_id = frame;

  left_info_pub.publish(info_left);
  right_info_pub.publish(info_right);
}


void StereoCamera::feedImages() {
  unsigned int pair_id = 0;
  FlyCapture2::Error error;
  while (ok) {
  
    // Retrieve an image
    error = cam_left.RetrieveBuffer( &rawImage_l );
    if (error != FlyCapture2::PGRERROR_OK) {
//    PrintError( error );
      continue;
    }
    
    error = cam_right.RetrieveBuffer( &rawImage_r );
    if (error != FlyCapture2::PGRERROR_OK) {
//    PrintError( error );
      continue;
    }

    ros::Time capture_time = ros::Time::now();

    // Convert the raw image
    error = rawImage_l.Convert( FlyCapture2::PIXEL_FORMAT_RGB, &convertedImage_l );
    if (error != FlyCapture2::PGRERROR_OK) {
//    PrintError( error );
      continue;
    }

    error = rawImage_r.Convert( FlyCapture2::PIXEL_FORMAT_RGB, &convertedImage_r );
    if (error != FlyCapture2::PGRERROR_OK) {
//    PrintError( error );
      continue;
    }

    ImagePtr image_left(new Image);
    ImagePtr image_right(new Image);

    image_left->height = convertedImage_l.GetRows();
    image_left->width = convertedImage_l.GetCols();
    image_left->step = convertedImage_l.GetStride();
    image_left->encoding = image_encodings::RGB8;

    image_left->header.stamp = capture_time;
    image_left->header.seq = pair_id;

    image_left->header.frame_id = frame;

    int data_size = convertedImage_l.GetDataSize();

    image_left->data.resize(data_size);

    memcpy(&image_left->data[0], convertedImage_l.GetData(), data_size);

    image_right->height = convertedImage_r.GetRows();
    image_right->width = convertedImage_r.GetCols();
    image_right->step = convertedImage_r.GetStride();
    image_right->encoding = image_encodings::RGB8;

    image_right->header.stamp = capture_time;
    image_right->header.seq = pair_id;

    image_right->header.frame_id = frame;

    data_size = convertedImage_r.GetDataSize();

    image_right->data.resize(data_size);

    memcpy(&image_right->data[0], convertedImage_r.GetData(), data_size);

    left_pub.publish(image_left);
    right_pub.publish(image_right);

    sendInfo(capture_time);
    ++pair_id;
  }
}

StereoCamera::~StereoCamera() {
  ok = false;
  image_thread.join();
  
  cam_left.StopCapture();
  cam_left.Disconnect();

  cam_right.StopCapture();
  cam_right.Disconnect();
}

};
