#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <camera.h>

namespace pgr_camera{

class PgrCameraNodelet : public nodelet::Nodelet{
public:
  PgrCameraNodelet()
    : nodelet::Nodelet()
  {

  }

  void onInit()
  {
    camera_.reset(new pgr_camera::Camera(this->getNodeHandle(), this->getPrivateNodeHandle()));
  }

protected:
  boost::shared_ptr<pgr_camera::Camera> camera_;

};

}

PLUGINLIB_EXPORT_CLASS( pgr_camera::PgrCameraNodelet, nodelet::Nodelet)

