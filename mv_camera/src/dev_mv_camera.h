/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (C) 2012, Markus Achtelik, Luc Oth
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
 *   * Neither the name of the author nor other contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
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

#ifndef DEV_MATRIX_VISION_CAMERA_HH
#define DEV_MATRIX_VISION_CAMERA_HH

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

// ROS includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/shared_ptr.hpp>
#include "mv_camera/MVCameraConfig.h"

class Features;

namespace mv_camera
{

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent)		\
  class name  : public parent {			\
  public:					\
    name (const char* msg) : parent (msg) {}	\
  }

//! A standard MVCamera exception
DEF_EXCEPTION(Exception, std::runtime_error);

typedef std::map<std::string, mvIMPACT::acquire::Property> StringPropMap;

inline double timeUs2Double(const mvIMPACT::acquire::PropertyI64::value_type & t)
{
  return static_cast<double>(t) * 1e-6;
}

class MVCamera
{
public:
  MVCamera();
  ~MVCamera();

  int open(MVCameraConfig &newconfig);
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
  bool embed_image_info_;


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

};
}
;

#endif // DEV_MATRIX_VISION_CAMERA_HH
