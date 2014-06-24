#ifndef PGRCAMERA_H
#define PGRCAMERA_H

#include "flycapture/FlyCapture2.h"

#include "pgr_camera/PGRCameraConfig.h"

class CameraConfig{


  public:

    CameraConfig(FlyCapture2::GigECamera* cam);

    void configure(pgr_camera::PGRCameraConfig &config, uint32_t level);

    void SetExposure(bool _auto, bool onoff, unsigned int value = 50);
    void SetGain(bool _auto, float value = 0.0);
    void setFrameRate(float value);
    void SetShutter (bool _auto, float value = 0.015);

    void printDetailedInfo();




  private:
    FlyCapture2::GigECamera* cam;

    //std::vector<FlyCapture2::Property> properties;
};


#endif
