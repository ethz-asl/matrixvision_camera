/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
*  Copyright (c) 2012 Markus Achtelik
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

#include <signal.h>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "driver_mv_camera.h"

/** @file

    @brief ROS driver nodelet for Matrix Vision digital cameras.

*/

/** Matrix Vision Camera nodelet implementation. */
class MVCameraNodelet: public nodelet::Nodelet
{
public:
  MVCameraNodelet():
    running_(false)
  {}

  virtual ~MVCameraNodelet()
  {
    if (running_)
      {
        NODELET_INFO("shutting down driver thread");
        running_ = false;
        deviceThread_->join();
        NODELET_INFO("driver thread stopped");
      }
    dvr_->shutdown();
  }

private:
  virtual void onInit();
  virtual void devicePoll();

  volatile bool running_;               ///< device is running
  boost::shared_ptr<mv_camera::MVCameraDriver> dvr_;
  boost::shared_ptr<boost::thread> deviceThread_;
};

/** Nodelet initialization.
 *
 *  @note MUST return immediately.
 */
void MVCameraNodelet::onInit()
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
  ros::NodeHandle node(getNodeHandle());
  ros::NodeHandle camera_nh(node, "camera");
  dvr_.reset(new mv_camera::MVCameraDriver(priv_nh, camera_nh));
  dvr_->setup();

  // spawn device thread
  running_ = true;
  deviceThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&MVCameraNodelet::devicePoll, this)));
}

/** Nodelet device poll thread main function. */
void MVCameraNodelet::devicePoll()
{
  while (running_)
    {
      dvr_->poll();
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(mv_camera, driver, MVCameraNodelet, nodelet::Nodelet);
