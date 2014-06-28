#ifndef PGR_CAMERA_CONFIG_H_
#define PGR_CAMERA_CONFIG_H_

#include "flycapture/FlyCapture2.h"

#include "pgr_camera/PGRCameraConfig.h"

class CameraConfig{

public:

  CameraConfig(FlyCapture2::GigECamera* cam);

  void configure(pgr_camera::PGRCameraConfig &config, uint32_t level);

  void setExposure(bool _auto, bool onoff, float value = 0.0);
  void setGain(bool _auto, float value = 0.0);
  void setFrameRate(float value);
  void setShutter (bool _auto, float value = 0.015);
  void setWhiteBalance(bool _auto, unsigned int red, unsigned int blue);

  void printDetailedInfo();

  int getRotationConfig() const { return rotation_config_; };


private:
  FlyCapture2::GigECamera* cam;

  int rotation_config_;

};


#endif
