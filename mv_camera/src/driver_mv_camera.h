/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010 Jack O'Quin
 *  Copyright (c) 2012 Markus Achtelik, Luc Oth
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

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <driver_base/driver.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>

#include "dev_mv_camera.h"
#include <mv_camera/MVCameraConfig.h>
#include <mv_camera/PropertyMap.h>

/** @file

 @brief ROS driver interface for Matrix Vision digital cameras.

 */

namespace mv_camera
{

class MVCameraDriver
{
public:

  // public methods
  MVCameraDriver(ros::NodeHandle priv_nh, ros::NodeHandle camera_nh);
  ~MVCameraDriver();
  void poll(void);
  void setup(void);
  void shutdown(void);
  void pollSingle(std::string& outputString);

  bool pollPropertyMapCallback(PropertyMap::Request &req, PropertyMap::Response &res);
  std::string getPropertyData(const mvIMPACT::acquire::Property& prop);

  std::pair<int,std::string> setProperty(const std::string & key, const std::string & value);

private:

  // private methods
  void closeCamera();
  bool openCamera(MVCameraConfig &newconfig);
  void publish(const sensor_msgs::ImagePtr &image);
  bool read(sensor_msgs::ImagePtr &image);
  void reconfig(MVCameraConfig &newconfig, uint32_t level);

  void processParameterList();

  /** Non-recursive mutex for serializing callbacks with device polling. */
  boost::mutex mutex_;

  volatile driver_base::Driver::state_t state_; // current driver state
  volatile bool reconfiguring_;        // true when reconfig() running

  ros::NodeHandle priv_nh_;             // private node handle
  ros::NodeHandle camera_nh_;           // camera name space handle
  std::string camera_name_;             // camera name

  /** mv_camera camera device interface */
  boost::shared_ptr<mv_camera::MVCamera> dev_;

  /** dynamic parameter configuration */
  MVCameraConfig config_;
  dynamic_reconfigure::Server<mv_camera::MVCameraConfig> srv_;
  ros::Rate cycle_;                     // polling rate when closed

  /** camera calibration information */
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  bool calibration_matches_;            // CameraInfo matches video mode

  /** image transport interfaces */
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_;

  bool continuousPoll_;

  ros::ServiceServer serviceServer_;

  bool processedParameterList_;

};
// end class MVCameraDriver

}
;
// end namespace mv_camera
