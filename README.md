uvc_cam
=======

Fork of the ROS uvc_cam USB webcam driver. Note that this driver actually uses the Video4Linux API and so won't work on non-Linux platforms, even if they support UVC devices.

Please note that this driver is *no longer maintained*. It is left up just in case people find it useful, but whenever rosbuild support goes away I intend to remove the repo completely.

If you got here because you needed a UVC-compatible camera driver, you should take a look at [libuvc_camera](http://wiki.ros.org/libuvc_camera) for a maintained driver.
