#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "camera.h"

using namespace sensor_msgs;

namespace pgr_camera {

Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) :
      node(_comm_nh), pnode(_param_nh), it(_comm_nh),
      info_mgr(_comm_nh, "camera"), cam(),
    camera_config_manager_(0){

      FlyCapture2::Error error;

      /* default config values */
      width = 640;
      height = 480;
      fps = 10;
      frame = "camera";
      rotate = false;

      /* set up information manager */
      std::string url;

      pnode.getParam("camera_info_url", url);

      info_mgr.loadCameraInfo(url);

      /* pull other configuration */
      pnode.getParam("serial", serial);

      pnode.getParam("fps", fps);
      pnode.getParam("skip_frames", skip_frames);

      pnode.getParam("width", width);
      pnode.getParam("height", height);

      pnode.getParam("frame_id", frame);

      /* advertise image streams and info streams */
      pub = it.advertise("image_raw", 1);

      info_pub = node.advertise<CameraInfo>("camera_info", 1);

      /* initialize the cameras */
      
      FlyCapture2::BusManager busMgr;
      
      FlyCapture2::PGRGuid guid;
      error = busMgr.GetCameraFromSerialNumber(serial, &guid);
      
      // Connect to a camera
      error = cam.Connect(&guid);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        ROS_ERROR("Error in PGR Camera: %s", error.GetDescription());
        return ;
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
//          PrintError( error );
//P          return -1;
      }

      FlyCapture2::GigEImageSettings imageSettings;
      imageSettings.offsetX = 0;
      imageSettings.offsetY = 0;
      imageSettings.height = imageSettingsInfo.maxHeight;
      imageSettings.width = imageSettingsInfo.maxWidth;
      imageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RGB8;

      int cut_x = 190;
      int cut_y = 70;

      imageSettings.offsetX = cut_x;
      imageSettings.width = imageSettingsInfo.maxWidth-cut_x*2;
      imageSettings.offsetY = cut_y;
      imageSettings.height = imageSettingsInfo.maxHeight-cut_y*2;


      printf( "Setting GigE image settings...\n" );

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

      camera_config_manager_->printDetailedInfo();

      reconfigure_server_.reset(new ReconfigureServer());
      ReconfigureServer::CallbackType f;

      f = boost::bind(&Camera::configCb, this, _1, _2);
      reconfigure_server_->setCallback(f);

      cv::namedWindow( "bla", cv::WINDOW_AUTOSIZE );



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

        ROS_INFO_THROTTLE(60.0, "Camera with frame \"%s\" requested %d images, %d erroneous transmissions: (%f) percent",
                          frame.c_str(),
                          processed_images,
                          retrieval_errors,
                          ((float)retrieval_errors/float(processed_images))*100.0f);

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

        // Convert the raw image
        error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_RGB, &convertedImage );
        if (error != FlyCapture2::PGRERROR_OK)
        {
          //ROS_ERROR("Error in PGR Camera Convert: %s", error.GetDescription());
          continue;
        }

        cv_bridge::CvImage cv_image;

        //Zero copy use of camera data
        cv_image.image = cv::Mat (convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3, convertedImage.GetData());
        cv_image.encoding = "rgb8";
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
      boost::mutex::scoped_lock cam_lock (cam_mutex_);

      if (camera_config_manager_){
          camera_config_manager_->configure(config, level);
        }

    }

    Camera::~Camera() {
      ok = false;
      image_thread.join();
      
      cam.StopCapture();
      cam.Disconnect();
    }


};

