/*
 * Copyright (c) 2009, Tully Foote
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <ros/ros.h>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"

#include "image_msgs/Image.h"

const unsigned WIDTH = 640, HEIGHT = 480;



int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: sender DEVICE\n");
    return 1;
  }

  ros::init(argc, argv, "uvc_cam");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<image_msgs::Image>("image", 10);

  uvc_cam::Cam cam(argv[1]);

  ros::Time t_prev(ros::Time::now());
  int count = 0;
  while (n.ok())
    {
      unsigned char *frame = NULL;
      uint32_t bytes_used;
      int buf_idx = cam.grab(&frame, bytes_used);
      if (count++ % 30 == 0)
	{
	  ros::Time t(ros::Time::now());
	  ros::Duration d(t - t_prev);
	  ROS_INFO("%.1f fps\n", 30.0 / d.toSec());
	  t_prev = t;
	}
      if (frame)
	{
	  image_msgs::Image image; 
      
	  image.header.stamp = ros::Time::now();
	  image.label = "UVC Camera Image";
	  image.encoding = "rgb";
	  image.depth = "uint8";

	  std_msgs::UInt8MultiArray & multi_arr = image.uint8_data;
	  multi_arr.layout.dim.resize(3);
	  multi_arr.layout.dim[0].label  = "height";
	  multi_arr.layout.dim[0].size   = HEIGHT;
	  multi_arr.layout.dim[0].stride = 3 * HEIGHT * WIDTH;
	  multi_arr.layout.dim[1].label  = "width";
	  multi_arr.layout.dim[1].size   = WIDTH;
	  multi_arr.layout.dim[1].stride = 3 * WIDTH;
	  multi_arr.layout.dim[2].label  = "channel";
	  multi_arr.layout.dim[2].size   = 3;
	  multi_arr.layout.dim[2].stride = 3;
  
	  multi_arr.data.resize(3 * HEIGHT * WIDTH);
	  uint8_t* bgr = &multi_arr.data[0];
	  for (uint32_t y = 0; y < HEIGHT; y++)
	    for (uint32_t x = 0; x < WIDTH; x++)
	      {
		uint8_t *p = frame + y * WIDTH * 3 + x * 3;
		uint8_t *q = bgr   + y * WIDTH * 3 + x * 3;
		q[0] = p[2]; q[1] = p[1]; q[2] = p[0];
	      }
      pub.publish(image);
      cam.release(buf_idx);
    }
  }
  return 0;
}

