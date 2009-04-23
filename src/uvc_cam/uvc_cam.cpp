#include <cstring>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include "uvc_cam/uvc_cam.h"

using std::string;
using namespace uvc_cam;

static const unsigned WIDTH = 640, HEIGHT = 480, FPS = 30;

Cam::Cam(const char *_device, mode_t _mode)
: mode(_mode), device(_device), rgb_frame(NULL)
{
  if ((fd = open(_device, O_RDWR)) == -1)
    throw std::runtime_error("couldn't open " + device);
  memset(&fmt, 0, sizeof(v4l2_format));
  memset(&cap, 0, sizeof(v4l2_capability));
  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
    throw std::runtime_error("couldn't query " + device);
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error(device + " does not support capture");
  if (!(cap.capabilities & V4L2_CAP_STREAMING))
    throw std::runtime_error(device + " does not support streaming");
  // enumerate formats
  v4l2_fmtdesc f;
  memset(&f, 0, sizeof(f));
  f.index = 0;
  f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int ret;
  while ((ret = ioctl(fd, VIDIOC_ENUM_FMT, &f)) == 0)
  {
    printf("pixfmt %d = '%4s' desc = '%s'\n",
           f.index++, (char *)&f.pixelformat, f.description);
    // enumerate frame sizes
    v4l2_frmsizeenum fsize;
    fsize.index = 0;
    fsize.pixel_format = f.pixelformat;
    while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0)
    {
      fsize.index++;
      if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
      {
        printf("  discrete: %ux%u:   ",
               fsize.discrete.width, fsize.discrete.height);
        // enumerate frame rates
        v4l2_frmivalenum fival;
        fival.index = 0;
        fival.pixel_format = f.pixelformat;
        fival.width = fsize.discrete.width;
        fival.height = fsize.discrete.height;
        while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0)
        {
          fival.index++;
          if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
          {
            printf("%u/%u ",
                   fival.discrete.numerator, fival.discrete.denominator);
          }
          else
            printf("I only handle discrete frame intervals...\n");
        }
        printf("\n");
      }
      else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS)
      {
        printf("  continuous: %ux%u to %ux%u\n",
               fsize.stepwise.min_width, fsize.stepwise.min_height,
               fsize.stepwise.max_width, fsize.stepwise.max_height);
      }
      else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
      {
        printf("  stepwise: %ux%u to %ux%u step %ux%u\n",
               fsize.stepwise.min_width,  fsize.stepwise.min_height,
               fsize.stepwise.max_width,  fsize.stepwise.max_height,
               fsize.stepwise.step_width, fsize.stepwise.step_height);
      }
      else
      {
        printf("  fsize.type not supported: %d\n", fsize.type);
      }
    }
  }
  if (errno != EINVAL)
    throw std::runtime_error("error enumerating frame formats");
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  if (mode == MODE_RGB) // grab in yuyv, we'll convert later
    fmt.fmt.pix.pixelformat = 'Y' | ('U' << 8) | ('Y' << 16) | ('V' << 24);
  else
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; 
  fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if ((ret = ioctl(fd, VIDIOC_S_FMT, &fmt)) < 0)
    throw std::runtime_error("couldn't set format");
  if (fmt.fmt.pix.width != WIDTH || fmt.fmt.pix.height != HEIGHT)
    throw std::runtime_error("pixel format unavailable");
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator = FPS;
  if ((ret = ioctl(fd, VIDIOC_S_PARM, &streamparm)) < 0)
    throw std::runtime_error("unable to set framerate");
  v4l2_queryctrl queryctrl;
  memset(&queryctrl, 0, sizeof(queryctrl));
  uint32_t i = V4L2_CID_BASE;
  while (i != V4L2_CID_LAST_EXTCTR)
  {
    queryctrl.id = i;
    if ((ret = ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) == 0 &&
        !(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED))
    {
      const char *ctrl_type = NULL;
      if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER)
        ctrl_type = "int";
      else if (queryctrl.type == V4L2_CTRL_TYPE_BOOLEAN)
        ctrl_type = "bool";
      else if (queryctrl.type == V4L2_CTRL_TYPE_BUTTON)
        ctrl_type = "button";
      else if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
        ctrl_type = "menu";
      printf("  %s (%s, %d, id = %x): %d to %d (%d)\n", 
             ctrl_type, 
             queryctrl.name, queryctrl.flags, queryctrl.id, 
             queryctrl.minimum, queryctrl.maximum, queryctrl.step);
      if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
      {
        v4l2_querymenu querymenu;
        memset(&querymenu, 0, sizeof(querymenu));
        querymenu.id = queryctrl.id;
        querymenu.index = 0;
        while (ioctl(fd, VIDIOC_QUERYMENU, &querymenu) == 0)
        {
          printf("    %d: %s\n", querymenu.index, querymenu.name);
          querymenu.index++;
        }
      }

    }
    else if (errno != EINVAL)
      throw std::runtime_error("couldn't query control");
    i++;
    if (i == V4L2_CID_LAST_NEW)
      i = V4L2_CID_CAMERA_CLASS_BASE_NEW;
    else if (i == V4L2_CID_CAMERA_CLASS_LAST)
      i = V4L2_CID_PRIVATE_BASE_OLD;
    else if (i == V4L2_CID_PRIVATE_LAST)
      i = V4L2_CID_BASE_EXTCTR;
  }
  //set_control(V4L2_CID_EXPOSURE_AUTO_NEW, 2);
  set_control(10094851, 0);
  set_control(10094849, 1);
  //set_control(0x9a9010, 100);

  set_control(V4L2_CID_EXPOSURE_ABSOLUTE_NEW, 300);
  set_control(V4L2_CID_BRIGHTNESS, 140);
  set_control(V4L2_CID_CONTRAST, 40);
  //set_control(V4L2_CID_WHITE_BALANCE_TEMP_AUTO_OLD, 0);
  set_control(9963788, 0); // auto white balance
  set_control(9963802, 1000); // color temperature
  set_control(9963800, 2);  // power line frequency to 60 hz
  set_control(9963795, 200); // gain
  set_control(9963803, 100); // sharpness
  set_control(9963778, 50); // saturation
/*
  v4l2_jpegcompression v4l2_jpeg;
  if (ioctl(fd, VIDIOC_G_JPEGCOMP, &v4l2_jpeg) < 0)
    throw std::runtime_error("no jpeg compression iface exposed");
  printf("jpeg quality: %d\n", v4l2_jpeg.quality);
*/

  memset(&rb, 0, sizeof(rb));
  rb.count = NUM_BUFFER;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &rb) < 0)
    throw std::runtime_error("unable to allocate buffers");
  for (unsigned i = 0; i < NUM_BUFFER; i++)
  {
    memset(&buf, 0, sizeof(buf));
    buf.index = i;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.flags = V4L2_BUF_FLAG_TIMECODE;
    buf.timecode = timecode;
    buf.timestamp.tv_sec = 0;
    buf.timestamp.tv_usec = 0;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
      throw std::runtime_error("unable to query buffer");
    if (buf.length <= 0)
      throw std::runtime_error("buffer length is bogus");
    mem[i] = mmap(0, buf.length, PROT_READ, MAP_SHARED, fd, buf.m.offset);
    //printf("buf length = %d at %x\n", buf.length, mem[i]);
    if (mem[i] == MAP_FAILED)
      throw std::runtime_error("couldn't map buffer");
  }
  buf_length = buf.length;
  for (unsigned i = 0; i < NUM_BUFFER; i++)
  {
    memset(&buf, 0, sizeof(buf));
    buf.index = i;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.flags = V4L2_BUF_FLAG_TIMECODE;
    buf.timecode = timecode;
    buf.timestamp.tv_sec = 0;
    buf.timestamp.tv_usec = 0;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
      throw std::runtime_error("unable to queue buffer");
  }
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    throw std::runtime_error("unable to start capture");
  rgb_frame = new unsigned char[WIDTH * HEIGHT * 3];
}

Cam::~Cam()
{
  // stop stream
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE, ret;
  if ((ret = ioctl(fd, VIDIOC_STREAMOFF, &type)) < 0)
    perror("VIDIOC_STREAMOFF");
  for (unsigned i = 0; i < NUM_BUFFER; i++)
    if (munmap(mem[i], buf_length) < 0)
      perror("failed to unmap buffer");
  close(fd);
  if (rgb_frame)
    delete[] rgb_frame;
  rgb_frame = NULL;
}

void Cam::enumerate()
{
  string v4l_path = "/sys/class/video4linux";
  DIR *d = opendir(v4l_path.c_str());
  if (!d)
    throw std::runtime_error("couldn't open " + v4l_path);
  struct dirent *ent, *ent2;
  int fd, ret;
  struct v4l2_capability v4l2_cap;
  while ((ent = readdir(d)) != NULL)
  {
    if (strncmp(ent->d_name, "video", 5))
      continue; // ignore anything not starting with "video"
    string dev_name = string("/dev/") + string(ent->d_name);
    printf("enumerating %s ...\n", dev_name.c_str());
    if ((fd = open(dev_name.c_str(), O_RDWR)) == -1)
      throw std::runtime_error("couldn't open " + dev_name + "  perhaps the " +
                               "permissions are not set correctly?");
    if ((ret = ioctl(fd, VIDIOC_QUERYCAP, &v4l2_cap)) < 0)
      throw std::runtime_error("couldn't query " + dev_name);
    printf("name = [%s]\n", v4l2_cap.card);
    printf("driver = [%s]\n", v4l2_cap.driver);
    printf("location = [%s]\n", v4l2_cap.bus_info);
    close(fd);
    string v4l_dev_path = v4l_path + string("/") + string(ent->d_name) + 
                          string("/device");
    // my kernel is using /sys/class/video4linux/videoN/device/inputX/id
    DIR *d2 = opendir(v4l_dev_path.c_str());
    if (!d2)
      throw std::runtime_error("couldn't open " + v4l_dev_path);
    string input_dir;
    while ((ent2 = readdir(d2)) != NULL)
    {
      if (strncmp(ent2->d_name, "input", 5))
        continue; // ignore anything not beginning with "input"
      input_dir = ent2->d_name;
      break;
    }
    closedir(d2);
    if (!input_dir.length())
      throw std::runtime_error("couldn't find input dir in " + v4l_dev_path);
    string vid_fname = v4l_dev_path + string("/") + input_dir + 
                       string("/id/vendor");
    string pid_fname = v4l_dev_path + string("/") + input_dir + 
                       string("/id/product");
    string ver_fname = v4l_dev_path + string("/") + input_dir + 
                       string("/id/version");
    char vid[5], pid[5], ver[5];
    FILE *vid_fp = fopen(vid_fname.c_str(), "r");
    if (!vid_fp)
      throw std::runtime_error("couldn't open " + vid_fname);
    if (!fgets(vid, sizeof(vid), vid_fp))
      throw std::runtime_error("couldn't read VID from " + vid_fname);
    fclose(vid_fp);
    vid[4] = 0;
    printf("vid = [%s]\n", vid);
    FILE *pid_fp = fopen(pid_fname.c_str(), "r");
    if (!pid_fp)
      throw std::runtime_error("couldn't open " + pid_fname);
    if (!fgets(pid, sizeof(pid), pid_fp))
      throw std::runtime_error("couldn't read PID from " + pid_fname);
    fclose(pid_fp);
    printf("pid = [%s]\n", pid);
    FILE *ver_fp = fopen(ver_fname.c_str(), "r");
    if (!ver_fp)
      throw std::runtime_error("couldn't open " + ver_fname);
    if (!fgets(ver, sizeof(ver), ver_fp))
      throw std::runtime_error("couldn't read version from " + ver_fname);
    fclose(ver_fp);
    printf("ver = [%s]\n", ver);
  }
  closedir(d);
}

// saturate input into [0, 255]
inline unsigned char sat(float f)
{
  return (unsigned char)( f >= 255 ? 255 : (f < 0 ? 0 : f)); 
}

int Cam::grab(unsigned char **frame, uint32_t &bytes_used)
{
  *frame = NULL;
  int ret = 0;
  fd_set rdset;
  timeval timeout;
  FD_ZERO(&rdset);
  FD_SET(fd, &rdset);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  bytes_used = 0;
  ret = select(fd + 1, &rdset, NULL, NULL, &timeout);
  if (ret == 0)
  {
    printf("select timeout in grab\n");
    return -1;
  }
  else if (ret < 0)
  {
    perror("couldn't grab image");
    return -1;
  }
  if (!FD_ISSET(fd, &rdset))
    return -1;
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
    throw std::runtime_error("couldn't dequeue buffer");
  bytes_used = buf.bytesused;
  if (mode == MODE_RGB)
  {
    unsigned char *pyuv = (unsigned char *)mem[buf.index];
    *frame = rgb_frame;
    // yuyv is 2 bytes per pixel. step through every pixel pair.
    unsigned char *prgb = rgb_frame;
    for (unsigned i = 0; i < WIDTH * HEIGHT * 2; i += 4)
    {
      /*
      *prgb++ = sat(pyuv[i]+1.402f  *(pyuv[i+3]-128));
      *prgb++ = sat(pyuv[i]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
      *prgb++ = sat(pyuv[i]+1.772f  *(pyuv[i+1]-128));
      *prgb++ = sat(pyuv[i+2]+1.402f*(pyuv[i+3]-128));
      *prgb++ = sat(pyuv[i+2]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
      *prgb++ = sat(pyuv[i+2]+1.772f*(pyuv[i+1]-128));
      */
      *prgb++ = sat(pyuv[i]+1.772f  *(pyuv[i+1]-128));
      *prgb++ = sat(pyuv[i]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
      *prgb++ = sat(pyuv[i]+1.402f  *(pyuv[i+3]-128));
      *prgb++ = sat(pyuv[i+2]+1.772f*(pyuv[i+1]-128));
      *prgb++ = sat(pyuv[i+2]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
      *prgb++ = sat(pyuv[i+2]+1.402f*(pyuv[i+3]-128));
    }
  }
  else
  {
    //if (bytes_used > 100)
      *frame = (unsigned char *)mem[buf.index];
  }
  return buf.index;
}

void Cam::release(unsigned buf_idx)
{
  if (buf_idx < NUM_BUFFER)
    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
      throw std::runtime_error("couldn't requeue buffer");
}

void Cam::set_control(uint32_t id, int val)
{
  v4l2_control c;
  c.id = id;

  if (ioctl(fd, VIDIOC_G_CTRL, &c) == 0)
  {
    printf("current value of %d is %d\n", id, c.value);
    /*
    perror("unable to get control");
    throw std::runtime_error("unable to get control");
    */
  }
  c.value = val;
  if (ioctl(fd, VIDIOC_S_CTRL, &c) < 0)
  {
    perror("unable to set control");
    throw std::runtime_error("unable to set control");
  }
}

