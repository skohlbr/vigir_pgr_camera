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

  pgr_camera::PGRCameraConfig last_config;

};


#endif
