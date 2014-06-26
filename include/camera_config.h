#ifndef PGRCAMERA_H
#define PGRCAMERA_H

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




private:
  FlyCapture2::GigECamera* cam;

};


#endif
