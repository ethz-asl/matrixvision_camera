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

#include <stdint.h>

#include <sensor_msgs/image_encodings.h>
#include "dev_mv_camera.h"
#include "features.h"
#include "formats.h"

#define NUM_DMA_BUFFERS 4

// @todo eliminate these macros
//! Macro for throwing an exception with a message
#define CAM_EXCEPT(except, msg)					\
		{								\
	char buf[100];						\
	snprintf(buf, 100, "[MVCamera::%s]: " msg, __FUNCTION__); \
	throw except(buf);						\
		}

//! Macro for throwing an exception with a message, passing args
#define CAM_EXCEPT_ARGS(except, msg, ...)				\
		{									\
	char buf[100];							\
	snprintf(buf, 100, "[MVCamera::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
	throw except(buf);							\
		}

using namespace mv_camera;
using namespace mvIMPACT::acquire;
using namespace std;

MVCamera::MVCamera() :
    use_ros_time_(true), embed_image_info_(true), cam_(NULL)
{
  rosTimeOffset_ = -1;

  int ret = 0;

  // from http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
#define xstr(s) str(s)
#define str(s) #s

  ret += setenv("GENICAM_ROOT", xstr(GENICAM_ROOT), 1);
  ret += setenv("GENICAM_ROOT_V2_3", xstr(GENICAM_ROOT_V2_3), 1);
  ret += setenv("GENICAM_GENTL64_PATH", xstr(GENICAM_GENTL64_PATH), 1);
  ret += setenv("GENICAM_LOG_CONFIG_V2_3", xstr(GENICAM_LOG_CONFIG_V2_3), 1);

  ROS_WARN_STREAM_COND(ret != 0, "couldn't set BlueCOUGAR environment variables ");

//  ROS_INFO_STREAM("ENV_test:"<<getenv("GENICAM_ROOT"));

  dev_mgr_.reset(new mvIMPACT::acquire::DeviceManager);
}

MVCamera::~MVCamera()
{

}

/** Open the mv_camera device and start streaming
 *
 *  @param newconfig new configuration parameters
 *  @return 0 if successful
 *
 *  TODO (if successful):
 *     * update newconfig.guid
 *     * validate newconfig.video_mode
 *     * initialize Features class
 */
int MVCamera::open(mv_camera::MVCameraConfig &newconfig)
{
  dev_mgr_->updateDeviceList();

  if (dev_mgr_->deviceCount() == 0)
  {
    CAM_EXCEPT(mv_camera::Exception, "No cameras found");
    return -1;
  }

  if (newconfig.guid == "" || newconfig.guid.size() < 8)
  {
    ROS_INFO_STREAM(
        "no GUID specified or not properly specified (guid=\"" << newconfig.guid << "\"), trying to open first available camera");
    for (unsigned int i = 0; i < dev_mgr_->deviceCount(); i++)
    {
      cam_ = dev_mgr_->getDevice(i);
      if (cam_ != NULL)
      {
        try
        {
          cam_->open();
          break;
        }
        catch (const mvIMPACT::acquire::ImpactAcquireException& e)
        {
          cam_ = NULL;
          ROS_WARN_STREAM("Unable to open camera: " << e.what());
        }
      }
    }
  }
  else
  {
    std::string serial = newconfig.guid.substr(newconfig.guid.size() - 8);
    ROS_INFO_STREAM("trying to open camera " << serial);
    cam_ = dev_mgr_->getDeviceBySerial(serial);
    if (cam_ != NULL)
    {
      try
      {
        cam_->open();
      }
      catch (const mvIMPACT::acquire::ImpactAcquireException& e)
      {
        cam_ = NULL;
        ROS_WARN_STREAM("Unable to open camera: " << e.what());
      }
    }
  }

  if (cam_ == NULL)
  {
    if (newconfig.guid == "" || newconfig.guid.find("MV_CAMERA_") == std::string::npos)
    {
      CAM_EXCEPT(mv_camera::Exception, "Could not find a camera");
    }
    else
    {
      CAM_EXCEPT_ARGS(mv_camera::Exception, "Could not find camera with guid %s", newconfig.guid.c_str());
    }
    return -1;
  }

  device_id_ = "MV_CAMERA_" + cam_->serial.read();

  cam_fi_.reset(new FunctionInterface(cam_));
  cam_stats_.reset(new Statistics(cam_));
  cam_ss_.reset(new SystemSettings(cam_));
  cam_irc_.reset(new ImageRequestControl(cam_));

  ROS_INFO_STREAM(
      "camera family:"<<cam_->family.read()<<"; camera model: " << cam_->product.read() << "; firmware version: " << cam_->firmwareVersion.read() << "; serial number: " << cam_->serial.read());

  //////////////////////////////////////////////////////////////
  // initialize camera
  //////////////////////////////////////////////////////////////

  int MAX_REQUESTS = 4;
  // make sure enough requests are available:
  cam_ss_->requestCount.write(MAX_REQUESTS);
  int request_result = DMR_NO_ERROR;
  int request_count = 0;
  while ((request_result = cam_fi_->imageRequestSingle(cam_irc_.get())) == DMR_NO_ERROR)
  {
    ++request_count;
  }

  use_ros_time_ = newconfig.use_ros_time;
  embed_image_info_ = newconfig.embed_image_info;
  //////////////////////////////////////////////////////////////
  // initialize feature settings
  //////////////////////////////////////////////////////////////

  // TODO: pass newconfig here and eliminate initialize() method
  features_.reset(new Features(cam_));

  // generate the property map
  generatePropertyMap();

  return 0;
}

/** close the device */
int MVCamera::close()
{
  if (cam_)
  {
    cam_fi_->imageRequestReset(0, 0);
    cam_->close();

    // reset the timestamp corrector:
  }

  return 0;
}

template<typename T>
size_t writeToBuffer(std::vector<boost::uint8_t> & data, size_t offset, const T & type)
{
    const T * ptr = &type;
    const boost::uint8_t * cp = reinterpret_cast< const boost::uint8_t * >(ptr);
    for(size_t i = 0; i < sizeof(T) && offset < data.size(); ++i)
    {
        data[offset++] = cp[i];
    }

    return offset;
}


void MVCamera::fillSensorMsgs(sensor_msgs::Image& image, const Request* req, ros::Time time_now)
{

  // save image info in published message:
  image.data.resize(req->imageSize.read());
  image.height = req->imageHeight.read();
  image.width = req->imageWidth.read();
  image.step = req->imageLinePitch.read();
  image.header.seq = req->infoFrameNr.read();
  // get the image encoding and bayer infos
  if (req->imageBayerMosaicParity.read() != mvIMPACT::acquire::bmpUndefined)
  {
    int bpp = req->imageBytesPerPixel.read() * 8;
    image.encoding = bayerString(req->imageBayerMosaicParity.read(), bpp);
    // ROS_INFO_STREAM_THROTTLE(1, "raw image, encoding: "<<image.encoding<<" bpp"<<bpp);
  }
  else
  {
    image.encoding = pixelFormat(req->imagePixelFormat.read());
    // ROS_INFO_STREAM_THROTTLE(1, "(processed) image, encoding: "<<image.encoding);
  }
  // copy the image data
  memcpy(&image.data[0], req->imageData.read(), image.data.size());

  if (use_ros_time_)
  {
    image.header.stamp = time_now;
  }
  else
  {
    // get the image timestamp:
    double current_image_time = timeUs2Double(req->infoTimeStamp_us.read()) * 1.0e-6;
    image.header.stamp = ros::Time(current_image_time);
  }

  if(embed_image_info_)
  {
      //ROS_INFO_STREAM("Embedding info");
      CameraSettingsBlueDevice bfs(cam_);
           
      // http://www.matrix-vision.com/manuals/SDK_CPP/classmvIMPACT_1_1acquire_1_1Request.html
      const boost::uint16_t magic = 0xA8;
      
      boost::uint64_t ts = req->infoTimeStamp_us.read();
      //boost::uint32_t es = req->infoExposeStart_us.read();
      boost::uint32_t et = req->infoExposeTime_us.read();
      boost::uint64_t nr = req->infoFrameNr.read();
      //float gain = req->infoGain_dB.read();
      boost::uint64_t pclock = bfs.pixelClock_KHz.read();
      //ROS_INFO_STREAM("ts: " << ts << ", exp start: " << es << ", exp: " << et << ", num: " << nr << ", gain: " << gain << ", pclock: " << pclock);
      
      size_t offset = 0;
      offset = writeToBuffer(image.data, offset, magic);
      offset = writeToBuffer(image.data, offset, ts);
      //offset = writeToBuffer(image.data, offset, es);
      offset = writeToBuffer(image.data, offset, et);
      offset = writeToBuffer(image.data, offset, nr);
      //offset = writeToBuffer(image.data, offset, gain);
      offset = writeToBuffer(image.data, offset, pclock);

      //ROS_INFO_STREAM("Embedding complete");
  }

}


// requests and saves a single image
void MVCamera::readSingleImage(sensor_msgs::Image& image)
{
  ROS_ASSERT_MSG(cam_, "Attempt to read from camera that is not open.");

  // request one image:
  cam_fi_->imageRequestSingle(); // cam_irc_.get()
  // max wait time:
  const int iMaxWaitTime_ms = 500;
  // wait for results from the default capture queue
  int requestNr = cam_fi_->imageRequestWaitFor(iMaxWaitTime_ms);

  ros::Time time_now = ros::Time::now();

  // check if the image has been captured without any problems
  if (!cam_fi_->isRequestNrValid(requestNr))
  {
    ROS_ERROR_STREAM("Failed to Capture Image. RequestNr: " << requestNr);
    return;
  }

  const Request* req = cam_fi_->getRequest(requestNr);
  // check request validity
  if (!req->isOK())
  {
    ROS_ERROR_STREAM("Request result: " << req->requestResult.readS());
    return;
  }

  fillSensorMsgs(image, req, time_now);

  // unlock the request
  cam_fi_->imageRequestUnlock(requestNr);

}

void MVCamera::clearRequestQueue()
{
  cam_fi_->imageRequestReset(0, 0);
}

/** Return an image frame */
void MVCamera::readData(sensor_msgs::Image& image)
{
  ROS_ASSERT_MSG(cam_, "Attempt to read from camera that is not open.");
  static int err_cnt = 0;

  // always keep request queue filled
  int request_result = DMR_NO_ERROR;
  int request_count = 0;
  while ((request_result = cam_fi_->imageRequestSingle(cam_irc_.get())) == DMR_NO_ERROR)
  {
    ++request_count;
  }

  // wait for results from the default capture queue
  int timeout_ms = static_cast<int>(1.0 / features_->getFPS() * 1.0e3 * 2); // wait max 200% of frametime
  int request_nr;

  if (err_cnt < 5)
    request_nr = cam_fi_->imageRequestWaitFor(timeout_ms);
  else
    request_nr = cam_fi_->imageRequestWaitFor(timeout_ms * 5);

  const mvIMPACT::acquire::Request * req;
  ros::Time time_now = ros::Time::now();

  if (cam_fi_->isRequestNrValid(request_nr))
  {
    req = cam_fi_->getRequest(request_nr);
    if (req->isOK())
    {

      fillSensorMsgs(image, req, time_now);
      features_->getImageInfo().width = image.width;
      features_->getImageInfo().height = image.height;
      features_->getImageInfo().color_coding = image.encoding;
      features_->getImageInfo().exposure_time = usToS(req->infoExposeTime_us.read());
      features_->getImageInfo().gain = req->infoGain_dB.read();
    }
    else
    {
      cam_fi_->imageRequestUnlock(request_nr);
      CAM_EXCEPT_ARGS(mv_camera::Exception, "Error while grabbing frame: %s", req->requestResult.readS().c_str());
    }

    // this image has been copied, the buffer is no longer needed...
    cam_fi_->imageRequestUnlock(request_nr);

    err_cnt = 0;
  }
  else
  {
    err_cnt++;
    ROS_WARN_STREAM("state: "<<cam_->state.read(0)<<" fps "<<features_->getFPS()<< " timeout "<< timeout_ms);
    CAM_EXCEPT_ARGS(mv_camera::Exception, "imageRequestWaitFor failed! reason: %d", request_nr);
  }

}

/** Generates a Property Map which links Camera Properties to an Identifier String:
 *
 * @param newconfig configuration parameters
 * @return true, if successful
 *
 * if successful:
 *   state_ is Driver::OPENED
 *   camera_name_ set to GUID string
 */
void MVCamera::generatePropertyMap()
{

  DeviceComponentLocator locator(cam_, dltSetting, "Base");
  populatePropertyMap(propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild());

  try
  {
    // this category is not supported by every device, thus we can expect an exception if this feature is missing
    locator = DeviceComponentLocator(cam_, dltIOSubSystem);
    populatePropertyMap(propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild());
  }
  catch (const ImpactAcquireException&)
  {
  }

  locator = DeviceComponentLocator(cam_, dltRequest);
  populatePropertyMap(propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild());
  locator = DeviceComponentLocator(cam_, dltSystemSettings);
  populatePropertyMap(propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild(), string("SystemSettings"));
  locator = DeviceComponentLocator(cam_, dltInfo);
  populatePropertyMap(propertyMap_, ComponentIterator(locator.searchbase_id()).firstChild(), string("Info"));
  populatePropertyMap(propertyMap_, ComponentIterator(cam_->hDev()).firstChild(), string("Device"));

}

// mvIMPACT SDK Examples
// Helper funciton to generate the property maps
//-----------------------------------------------------------------------------
void MVCamera::populatePropertyMap(StringPropMap& m, ComponentIterator it, const std::string& currentPath)
//-----------------------------------------------------------------------------
{
  while (it.isValid())
  {
    std::string fullName(currentPath);
    if (fullName != "")
    {
      fullName += "/";
    }
    fullName += it.name();
    if (it.isList())
    {
      populatePropertyMap(m, it.firstChild(), fullName);
    }
    else if (it.isProp())
    {
      m.insert(make_pair(fullName, Property(it)));
    }
    ++it;
    // method object will be ignored...
  }
}
// \mvIMPACT SDK Examples

void MVCamera::saveCameraSettings(std::string path)
{

    ROS_INFO_STREAM("Saving camera parameters to " << path);
    int rval = cam_fi_->saveSetting(path, mvIMPACT::acquire::TStorageFlag(sfFile | sfIgnoreBasicData));
    ROS_INFO_STREAM("Return code: " << rval);
}

void MVCamera::loadCameraSettings(std::string path)
{
    ROS_INFO_STREAM("Loading camera parameters from " << path);
    int rval = cam_fi_->loadSetting(path, mvIMPACT::acquire::TStorageFlag(sfFile | sfIgnoreBasicData));//sfFile | sfIgnoreBasicData);
    ROS_INFO_STREAM("Return code: " << rval);
}

