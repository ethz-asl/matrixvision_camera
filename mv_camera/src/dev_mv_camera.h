
#ifndef DEV_MATRIX_VISION_CAMERA_HH
#define DEV_MATRIX_VISION_CAMERA_HH

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

// ROS includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/shared_ptr.hpp>
#include "matrix_vision_camera/MatrixVisionCameraConfig.h"


typedef std::map<std::string, mvIMPACT::acquire::Property> StringPropMap;


class Features;

namespace matrix_vision_camera
{
//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent)		\
  class name  : public parent {			\
  public:					\
    name (const char* msg) : parent (msg) {}	\
  }

//! A standard MatrixVisionCamera exception
DEF_EXCEPTION(Exception, std::runtime_error);

class MatrixVisionCamera
{
public:
  MatrixVisionCamera();
  ~MatrixVisionCamera();

  int open(matrix_vision_camera::MatrixVisionCameraConfig &newconfig);
  int close();
  void readData(sensor_msgs::Image &image);

  /** check whether CameraInfo matches current video mode
   *
   *  @param image corresponding Image message
   *  @param ci CameraInfo message to check
   *  @return true if camera dimensions match calibration
   */
  bool checkCameraInfo(const sensor_msgs::Image &image, const sensor_msgs::CameraInfo &ci)
  {
    return (ci.width == image.width && ci.height == image.height);
  }

  /** set operational parameter fields in CameraInfo message
   *
   *  @param ci CameraInfo message to update
   *
   *  @post CameraInfo fields filled in (if needed):
   *    roi (region of interest)
   *    binning_x, binning_y
   */
  void setOperationalParameters(sensor_msgs::CameraInfo &ci)
  {
    //      if (format7_.active())
    //        format7_.setOperationalParameters(ci);
  }

  std::string device_id_;
  boost::shared_ptr<Features> features_;

  // make it public
  StringPropMap propertyMap_;

  void saveCameraSettings(std::string path);
  void loadCameraSettings(std::string path);
  void readSingleImage(sensor_msgs::Image& image);
  void clearRequestQueue();

private:
  bool use_ros_time_;

  sm::timing::TimestampCorrector<double> timestampCorrector_;

  int64_t rosTimeOffset_; // time offset between camera and ros time (as long as it is -1 => not initialised)

  boost::shared_ptr<mvIMPACT::acquire::DeviceManager> dev_mgr_;
  mvIMPACT::acquire::Device *cam_;
  boost::shared_ptr<mvIMPACT::acquire::FunctionInterface> cam_fi_;
  boost::shared_ptr<mvIMPACT::acquire::SystemSettings> cam_ss_;
  boost::shared_ptr<mvIMPACT::acquire::Statistics> cam_stats_;
  boost::shared_ptr<mvIMPACT::acquire::ImageRequestControl> cam_irc_;

  // define a string -> property map for property manipulation:
  // define the string -> Property map
  void populatePropertyMap(StringPropMap& m, ComponentIterator it, const std::string& currentPath = "");
  void generatePropertyMap();

  void fillSensorMsgs(sensor_msgs::Image& image, const Request* req, ros::Time time_now);

  void SafeCleanup();
};
}
;

#endif // DEV_MATRIX_VISION_CAMERA_HH
