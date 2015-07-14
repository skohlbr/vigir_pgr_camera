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

#include <cv_bridge/cv_bridge.h>

#include "camera.h"

using namespace sensor_msgs;

namespace pgr_camera {

Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) :
      node(_comm_nh), pnode(_param_nh), it(_param_nh),
      info_mgr(_param_nh, "camera"), cam(),
      config_updated_(false), camera_config_manager_(0){

      FlyCapture2::Error error;

      frame = "camera";

      /* set up information manager */
      std::string url;
      pnode.getParam("camera_info_url", url);

      info_mgr.loadCameraInfo(url);

      /* Basic camera config */
      pnode.getParam("serial", serial);
      pnode.getParam("frame_id", frame);

      //ROI config
      pnode.getParam("roi_offset_x", roi_offset_x_);
      pnode.getParam("roi_offset_y", roi_offset_y_);
      pnode.getParam("roi_width", roi_width_);
      pnode.getParam("roi_height", roi_height_);

      mono_mode_ = false;
      pnode.getParam("mono_mode", mono_mode_);

      /* advertise image streams and info streams */
      pub = it.advertise("image", 1);

      info_pub = node.advertise<CameraInfo>("camera_info", 1);

      /* initialize the cameras */
      
      FlyCapture2::BusManager busMgr;
      
      FlyCapture2::PGRGuid guid;
      error = busMgr.GetCameraFromSerialNumber(serial, &guid);
      
      // Connect to a camera
      error = cam.Connect(&guid);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        ROS_ERROR("Failed to connect to PGR Camera: %s . Will keep trying.", error.GetDescription());

        while (error != FlyCapture2::PGRERROR_OK && ros::ok()){
          error = cam.Connect(&guid);
          sleep(1);
        }
      }

      ROS_INFO("Connected to PGR Camera with serial %d set to frame_id %s ",
               serial,
               frame.c_str());

      unsigned int num_channels;
      cam.GetNumStreamChannels(&num_channels);


      for (unsigned int i = 0; i < num_channels; ++i){
        FlyCapture2::GigEStreamChannel channel;

        cam.GetGigEStreamChannelInfo(i, &channel);

        printf(
              "\n*** CHANNEL INFORMATION for channel %u ***\n"
              "networkInterfaceIndex - %u\n"
              "hostPost - %u\n"
              "doNotFragment - %u\n"
              "packetSize - %u\n"
              "interPacketDelay - %u\n"
              "sourcePort - %u\n",
              i,
              channel.networkInterfaceIndex,
              channel.hostPost,
              channel.doNotFragment,
              channel.packetSize,
              channel.interPacketDelay,
              channel.sourcePort);

        channel.packetSize=7186;
        channel.interPacketDelay = 400;
        channel.doNotFragment = false;
        cam.SetGigEStreamChannelInfo(i, &channel);
      }

      FlyCapture2::GigEConfig config;
      error = cam.GetGigEConfig(&config);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        ROS_ERROR("Error in PGR Camera: %s", error.GetDescription());
      }

      config.enablePacketResend = true;
      cam.SetGigEConfig(&config);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        ROS_ERROR("Error in PGR Camera: %s", error.GetDescription());
      }




      // Get the camera information
      FlyCapture2::CameraInfo camInfo;
      error = cam.GetCameraInfo(&camInfo);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        ROS_ERROR("Error in PGR Camera: %s", error.GetDescription());
        return ;
      }

      //PrintCameraInfo(&camInfo);

      FlyCapture2::GigEImageSettingsInfo imageSettingsInfo;
      error = cam.GetGigEImageSettingsInfo( &imageSettingsInfo );
      if (error != FlyCapture2::PGRERROR_OK)
      {
        ROS_ERROR("Error in PGR Camera: %s", error.GetDescription());
        return;
      }

      FlyCapture2::GigEImageSettings imageSettings;
      imageSettings.offsetX = 0;
      imageSettings.offsetY = 0;
      imageSettings.height = imageSettingsInfo.maxHeight;
      imageSettings.width = imageSettingsInfo.maxWidth;

      imageSettings.pixelFormat = mono_mode_ ? FlyCapture2::PIXEL_FORMAT_MONO8 : FlyCapture2::PIXEL_FORMAT_422YUV8;


      imageSettings.offsetX = roi_offset_x_;
      imageSettings.width = roi_width_;
      imageSettings.offsetY = roi_offset_y_;
      imageSettings.height = roi_height_;

      ROS_INFO("PGR Camera with serial %d set to frame_id %s, roi_offset_x: %d roi_offset_y: %d roi_width: %d roi_height: %d ",
               serial,
               frame.c_str(),
               roi_offset_x_,
               roi_offset_y_,
               roi_width_,
               roi_height_);


      //printf( "Setting GigE image settings...\n" );

      error = cam.SetGigEImageSettings( &imageSettings );
      if (error != FlyCapture2::PGRERROR_OK)
      {
        ROS_ERROR("Error setting image settings: %s", error.GetDescription());
        return ;
      }


      //FlyCapture2::VideoMode videoMode = FlyCapture2::VIDEOMODE_1280x960RGB;
      //FlyCapture2::FrameRate frameRate = FlyCapture2::FRAMERATE_30;
      //cam.SetVideoModeAndFrameRate(videoMode, frameRate);

      //FlyCapture2::Property prop;
      //prop.type = FlyCapture2::FRAME_RATE;
      //prop.onOff = true;
      //prop.autoManualMode = false;
      //prop.valueA = FlyCapture2::FRAMERATE_30;
      //cam.SetProperty(&prop);

      //camPropInfo.absValue = 30.0;
      //cam.SetProperty(&camPropInfo);
      //FlyCapture2::Property property = properties[FlyCapture2::FRAME_RATE];
      //property.absValue = 20.0;
      //cam.SetProperty(&property);

      camera_config_manager_ = new CameraConfig(&cam);

      //camera_config_manager_->printDetailedInfo();

      reconfigure_server_.reset(new ReconfigureServer(config_mutex_, _param_nh));
      ReconfigureServer::CallbackType f;

      f = boost::bind(&Camera::configCb, this, _1, _2);
      reconfigure_server_->setCallback(f);

      /* and turn on the streamer */
      error = cam.StartCapture();
      if (error != FlyCapture2::PGRERROR_OK)
      {
        ROS_ERROR("Error in PGR Camera: %s", error.GetDescription());
        return ;
      }
      ok = true;
      image_thread = boost::thread(boost::bind(&Camera::feedImages, this));
    }

void Camera::PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo )
{
    char macAddress[64];
    sprintf( 
        macAddress, 
        "%02X:%02X:%02X:%02X:%02X:%02X", 
        pCamInfo->macAddress.octets[0],
        pCamInfo->macAddress.octets[1],
        pCamInfo->macAddress.octets[2],
        pCamInfo->macAddress.octets[3],
        pCamInfo->macAddress.octets[4],
        pCamInfo->macAddress.octets[5]);

    char ipAddress[32];
    sprintf( 
        ipAddress, 
        "%u.%u.%u.%u", 
        pCamInfo->ipAddress.octets[0],
        pCamInfo->ipAddress.octets[1],
        pCamInfo->ipAddress.octets[2],
        pCamInfo->ipAddress.octets[3]);

    char subnetMask[32];
    sprintf( 
        subnetMask, 
        "%u.%u.%u.%u", 
        pCamInfo->subnetMask.octets[0],
        pCamInfo->subnetMask.octets[1],
        pCamInfo->subnetMask.octets[2],
        pCamInfo->subnetMask.octets[3]);

    char defaultGateway[32];
    sprintf( 
        defaultGateway, 
        "%u.%u.%u.%u", 
        pCamInfo->defaultGateway.octets[0],
        pCamInfo->defaultGateway.octets[1],
        pCamInfo->defaultGateway.octets[2],
        pCamInfo->defaultGateway.octets[3]);

    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n"
        "GigE version - %u.%u\n"
        "User defined name - %s\n"
        "XML URL 1 - %s\n"
        "XML URL 2 - %s\n"
        "MAC address - %s\n"
        "IP address - %s\n"
        "Subnet mask - %s\n"
        "Default gateway - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime,
        pCamInfo->gigEMajorVersion,
        pCamInfo->gigEMinorVersion,
        pCamInfo->userDefinedName,
        pCamInfo->xmlURL1,
        pCamInfo->xmlURL2,
        macAddress,
        ipAddress,
        subnetMask,
        defaultGateway );
}

    void Camera::sendInfo(const ImagePtr &image, ros::Time time) {
      CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));

      /* Throw out any CamInfo that's not calibrated to this camera mode */
      if (info->K[0] != 0.0 &&
           (image->width != info->width
              || image->height != info->height)) {
        info.reset(new CameraInfo());
      }

      /* If we don't have a calibration, set the image dimensions */
      if (info->K[0] == 0.0) {
        info->width = image->width;
        info->height = image->height;
      }

      info->header.stamp = time;
      info->header.frame_id = frame;

      info_pub.publish(info);
    }

    void Camera::feedImages() {
      
      FlyCapture2::Error error;

      int processed_images = 0;
      int retrieval_errors = 0;

      while (ok) {
        unsigned char *img_frame = NULL;
        uint32_t bytes_used;

        ROS_INFO_THROTTLE(60.0, "Camera with frame \"%s\" requested %d images, %d erroneous transmissions (%f percent)",
                          frame.c_str(),
                          processed_images,
                          retrieval_errors,
                          ((float)retrieval_errors/float(processed_images))*100.0f);

        {
          boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);

          if (config_updated_){
            camera_config_manager_->configure(config_, 0);
            config_updated_ = false;
          }
        }

        {
          // Retrieve an image
          boost::mutex::scoped_lock cam_lock (cam_mutex_);
          error = cam.RetrieveBuffer( &rawImage );
          processed_images++;
        }
        if (error != FlyCapture2::PGRERROR_OK)
        {
          //ROS_ERROR("Error in PGR Camera Retrieve: %s. Continuing", error.GetDescription());
          retrieval_errors++;
          continue;
        }

        ros::Time capture_time = ros::Time::now();

        error = rawImage.Convert( mono_mode_ ? FlyCapture2::PIXEL_FORMAT_MONO8 : FlyCapture2::PIXEL_FORMAT_RGB,
                                  &convertedImage );


        if (error != FlyCapture2::PGRERROR_OK)
        {
          //ROS_ERROR("Error in PGR Camera Convert: %s", error.GetDescription());
          continue;
        }

        cv_bridge::CvImage cv_image;

        //Zero copy use of camera data
        cv_image.image = cv::Mat (convertedImage.GetRows(),
                                  convertedImage.GetCols(),
                                  mono_mode_? CV_8UC1 : CV_8UC3,
                                  convertedImage.GetData());

        cv_image.encoding = mono_mode_ ? "mono8" : "rgb8";
        cv_image.header.stamp = capture_time;
        cv_image.header.frame_id = frame;

        if( camera_config_manager_->getRotationConfig() != 0 )
        {
          cv::transpose( cv_image.image, tmp_cvmat );

          if(camera_config_manager_->getRotationConfig() == 1)
            cv::flip( tmp_cvmat, cv_image.image, 0);
          else
            cv::flip( tmp_cvmat, cv_image.image, 1);
        }

        const sensor_msgs::ImagePtr image_msg = cv_image.toImageMsg();

        pub.publish(image_msg);

        sendInfo(image_msg, capture_time);
      }
    }

    void Camera::configCb(Config &config, uint32_t level)
    {
      config_updated_ = true;
      config_ = config;
    }

    Camera::~Camera() {
      ok = false;
      image_thread.join();
      
      cam.StopCapture();
      cam.Disconnect();
    }


};

