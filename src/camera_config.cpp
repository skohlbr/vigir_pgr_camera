#include <camera_config.h>

CameraConfig::CameraConfig(FlyCapture2::GigECamera* cam)
  : cam(cam)
{

}

void CameraConfig::configure(pgr_camera::PGRCameraConfig &config, uint32_t level)
{
  // Exposure
  if (config.auto_exposure)
    this->setExposure (true, true, 0);
  else
    this->setExposure (false, true, config.exposure);


  // Shutter
  if (config.auto_shutter)
    this->setShutter (true);
  else
    this->setShutter (false, (float)config.shutter);

  // Gain
  if(config.auto_gain)
    this->setGain(true);
  else
    this->setGain(false, (float)config.gain);

  this->setWhiteBalance(config.auto_whitebalance, config.whitebalance_red, config.whitebalance_blue);


  this->setFrameRate( (float) config.framerate );
}


void CameraConfig::setExposure(bool _auto, bool onoff, float value)
{
  printf( "Set Exposure: _auto: %d, onoff %d, value %f\n", _auto, onoff, value);
  FlyCapture2::Property prop;
  prop.type = FlyCapture2::AUTO_EXPOSURE;
  prop.autoManualMode = _auto;
  prop.onOff = true;
  //prop.valueA = value;
  prop.absValue = value;
  prop.absControl = true;
  cam->SetProperty(&prop);
}

void CameraConfig::setGain(bool _auto, float value)
{
  printf( "Set Gain: _auto: %d, value %f\n", _auto, value);
  FlyCapture2::Property prop;
  prop.type = FlyCapture2::GAIN;
  prop.autoManualMode = _auto;
  prop.onOff = true;
  prop.absValue = value;
  prop.absControl = true;
  cam->SetProperty(&prop);
}


void CameraConfig::setFrameRate(float value)
{
  FlyCapture2::Property prop;
  prop.type = FlyCapture2::FRAME_RATE;
  prop.autoManualMode = false;
  prop.onOff = true;
  prop.absControl = true;
  prop.absValue = value;
  cam->SetProperty(&prop);
}

void CameraConfig::setShutter (bool _auto, float value)
{
  printf( "Set Shutter: _auto: %d, value %f\n", _auto, value);
  FlyCapture2::Property prop;
  prop.type = FlyCapture2::SHUTTER;
  prop.absControl = true;
  prop.autoManualMode = _auto;
  prop.onOff = true;
  prop.absValue = value;
  cam->SetProperty(&prop);
}

void CameraConfig::setWhiteBalance(bool _auto, unsigned int red, unsigned int blue)
{
  printf( "Set WhiteBalance: _auto: %d, red %u, blue: %u \n", _auto, red, blue);
  FlyCapture2::Property prop;
  prop.type = FlyCapture2::WHITE_BALANCE;
  prop.absControl = false;
  prop.autoManualMode = _auto;
  prop.onOff = true;
  prop.valueA = red;
  prop.valueB = blue;
  cam->SetProperty(&prop);
}

void CameraConfig::printDetailedInfo()
{
  for (int i = 0; i < FlyCapture2::UNSPECIFIED_PROPERTY_TYPE; ++i){
    FlyCapture2::Property property;
    property.type = (FlyCapture2::PropertyType)i;
    FlyCapture2::Error getPropErr = cam->GetProperty( &property );

    printf("Property: %d\n"
           " \tpresent        %d\n"
           " \tabsControl     %d\n"
           " \tonePush         %d\n"
           " \tonOff          %d\n"
           " \tautoManualMode %d\n"
           " \tvalueA         %u\n"
           " \tvalueB         %u\n"
           " \tabsValue       %f\n ",
           property.type,
           property.present,
           property.absControl,
           property.onePush,
           property.onOff,
           property.autoManualMode,
           property.valueA,
           property.valueB,
           property.absValue );


    FlyCapture2::PropertyInfo property_info;
    property_info.type = (FlyCapture2::PropertyType)i;
    FlyCapture2::Error getPropInfoErr = cam->GetPropertyInfo( &property_info );

    printf("Property Info: %d\n"
           " \tpresent        %d\n"
           " \tautoSupported     %d\n"
           " \tmanualSupported         %d\n"
           " \tonOffSupported          %d\n"
           " \tonePushSupported %d\n"
           " \tabsValSupported         %d\n"
           " \tmin         %u\n"
           " \tmax       %u\n "
           " \tabsMin       %f\n "
           " \tabsMax       %f\n "
           " \tpUnits       %s\n "
           " \tpUnitAbbr    %s\n ",
           property_info.type,
           property_info.present,
           property_info.autoSupported,
           property_info.manualSupported,
           property_info.onOffSupported,
           property_info.onePushSupported,
           property_info.absValSupported,
           property_info.min,
           property_info.max,
           property_info.absMin,
           property_info.absMax,
           property_info.pUnits,
           property_info.pUnitAbbr);

    printf("-----------------------------------------------------------------\n");
  }
}


