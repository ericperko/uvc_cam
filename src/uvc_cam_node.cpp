/*
 * Copyright (c) 2009, Morgan Quigley, Clemens Eppner, Tully Foote
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Modified Apr 6, 2010 by Adam Leeper - changed to use "image_transport"

// Much of this code is either heavily inspired by or taken directly from the camera1394 ROS driver by Jack O'Quinn

#include <signal.h>
#include <cstdio>
#include <ros/ros.h>
#include <ros/time.h>
#include <uvc_cam/uvc_cam.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

#include "uvc_cam/UVCCamConfig.h"

typedef driver_base::Driver Driver;
typedef driver_base::SensorLevels Levels;

/** Segfault signal handler */
void sigsegv_handler(int sig)
{
	signal(SIGSEGV, SIG_DFL);
	fprintf(stderr, "Segmentation fault, stopping uvc camera driver.\n");
	ROS_ERROR("Segmentation fault, stopping uvc camera driver.");
	ros::shutdown();                      // stop the main loop
}

class UVCCamNode {
private:
	Driver::state_t state_;               // current driver state

	ros::NodeHandle privNH_;              // private node handle
	ros::NodeHandle camera_nh_;           // camera name space handle
	sensor_msgs::Image image_;
	sensor_msgs::CameraInfo cam_info_;
	std::string device;

	int width, height, fps;

	/** dynamic parameter configuration */
	typedef uvc_cam::UVCCamConfig Config;
	Config config_;

	/** camera calibration information */
	CameraInfoManager cinfo_;
	bool calibration_matches_;            // cam_info_ matches video mode

	/** image transport interfaces */
	image_transport::ImageTransport it_;
	image_transport::CameraPublisher image_pub_;

public:
	UVCCamNode():
		privNH_("~"),
		camera_nh_("camera"),
		cinfo_(camera_nh_),
		it_(camera_nh_)
	{
		state_ = Driver::CLOSED;
		calibration_matches_ = true;
		device = "/dev/video0";
	};

	/** Close camera device
	 *
	 *  postcondition: state_ is Driver::CLOSED
	 */
	void closeCamera() {
		if (state_ != Driver::CLOSED)
		{
			ROS_INFO_STREAM("[" << device << "] closing device");
			//			dev_->close();
			state_ = Driver::CLOSED;
		}
	}

	/** Open the camera device.
	 *
	 * @param newconfig configuration parameters
	 * @return true, if successful
	 *
	 * if successful:
	 *   state_ is Driver::OPENED
	 *   camera_name_ set to GUID string
	 */
	bool openCamera(Config &newconfig)
	{
		bool success = true;

		ROS_INFO("opening uvc_cam at %dx%d, %d fps", width, height, fps);
		uvc_cam::Cam cam(device.c_str(), uvc_cam::Cam::MODE_RGB, width, height, fps);
		//	    try
		//	      {
		//	        if (dev_->open(newconfig.guid.c_str(), newconfig.video_mode.c_str(),
		//	                       newconfig.frame_rate, newconfig.iso_speed,
		//	                       newconfig.bayer_pattern.c_str(),
		//	                       newconfig.bayer_method.c_str())
		//	            == 0)
		//	          {
		//	            if (camera_name_ != dev_->device_id_)
		//	              {
		//	                camera_name_ = dev_->device_id_;
		//	                if (!cinfo_->setCameraName(camera_name_))
		//	                  {
		//	                    // GUID is 16 hex digits, which should be valid.
		//	                    // If not, use it for log messages anyway.
		//	                    ROS_WARN_STREAM("[" << camera_name_ << "] name not valid"
		//	                                    << " for camera_info_manger");
		//	                  }
		//	              }
		//	            ROS_INFO_STREAM("[" << camera_name_
		//	                            << "] opened: " << newconfig.video_mode << ", "
		//	                            << newconfig.frame_rate << " fps, "
		//	                            << newconfig.iso_speed << " Mb/s");
		//	            state_ = Driver::OPENED;
		//	            calibration_matches_ = true;
		//	          }
		//
		//	      }
		//	    catch (camera1394::Exception& e)
		//	      {
		//	        ROS_FATAL_STREAM("[" << camera_name_
		//	                         << "] exception opening device: " << e.what());
		//	        success = false;
		//	      }

		return success;
	}

	/** Read camera data.
	 *
	 * @return true if successful
	 */
	bool read() {
		return false;
	}

	void publish() {
		image_pub_.publish(image_, cam_info_);
	}

	/** driver main spin loop */\
	void spin() {

		ros::NodeHandle node;

		privNH_.param<std::string>("device", device, "/dev/video0");
		privNH_.param("width", width, 640);
		privNH_.param("height", height, 480);
		privNH_.param("fps", fps, 30);

		image_pub_ = it_.advertiseCamera("image_raw", 1);

		while (node.ok())
		{
			if (state_ != Driver::CLOSED)
			{
				if (read())
				{
					publish();
				}
			}

			ros::spinOnce();
		}

		closeCamera();
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uvc_cam_node");

	ros::NodeHandle node;
	UVCCamNode cam;

	cam.spin();

	return 0;
	//	IplImage *imageIpl = cvCreateImageHeader(cvSize(width,height), 8, 3);
	//
	//	ros::Time t_prev(ros::Time::now());
	//	int count = 0, skip_count = 0;
	//	while (n.ok())
	//	{
	//		unsigned char *frame = NULL;
	//		uint32_t bytes_used;
	//		int buf_idx = cam.grab(&frame, bytes_used);
	//		if (count++ % fps == 0)
	//		{
	//			ros::Time t(ros::Time::now());
	//			ros::Duration d(t - t_prev);
	//			ROS_INFO("%.1f fps skip %d", (double)fps / d.toSec(), skip_count);
	//			t_prev = t;
	//		}
	//		if (frame)
	//		{
	//			//cv::WImageBuffer3_b image( frame );
	//			//cv::Mat data(height, width, CV_8UC1, frame, 3 * width);
	//			imageIpl->imageData = (char *)frame;
	//			sensor_msgs::Image::Ptr image = sensor_msgs::CvBridge::cvToImgMsg( imageIpl, "bgr8");
	//
	//			//sensor_msgs::Image image;
	//
	//			image->header.stamp = ros::Time::now();
	//			image->encoding = sensor_msgs::image_encodings::BGR8;
	//			image->height = height;
	//			image->width = width;
	//			image->step = 3 * width;
	//
	//			//image->set_data_size( image.step * image.height );
	//
	//			/*
	//      uint8_t* bgr = &(image.data[0]);
	//      for (uint32_t y = 0; y < height; y++)
	//        for (uint32_t x = 0; x < width; x++)
	//        {
	//          // hack... flip bgr to rgb
	//          uint8_t *p = frame + y * width * 3 + x * 3;
	//          uint8_t *q = bgr   + y * width * 3 + x * 3;
	//          q[0] = p[2]; q[1] = p[1]; q[2] = p[0];
	//        }
	//			 */
	//			//memcpy(&image.data[0], frame, width * height * 3);
	//			cam.release(buf_idx);
	//		}
	//		else
	//			skip_count++;
	//	}
}

