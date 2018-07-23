/*

   (c) 2014 Séverin Lemaignan <severin.lemaignan@epfl.ch>
   (c) 2008 Hans de Goede <hdegoede@redhat.com> for yuyv_to_rgb24

 This program is free software; you can redistribute it and/or modify it
 under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or (at
 your option) any later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software Foundation,
 Inc., 51 Franklin Street, Suite 500, Boston, MA  02110-1335  USA

 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <string.h> // strerrno
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <stdexcept>

#include <linux/videodev2.h>

#include "camera.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

using namespace std;

static int xioctl(int fh, unsigned long int request, void *arg)
{
    int r;

    do
    {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

/*****
 * Taken from libv4l2 (in v4l-utils)
 *
 * (C) 2008 Hans de Goede <hdegoede@redhat.com>
 *
 * Released under LGPL
 */
#define CLIP(color) (unsigned char)(((color) > 0xFF) ? 0xff : (((color) < 0) ? 0 : (color)))

static void v4lconvert_yuyv_to_rgb24(const unsigned char *src,
                                     unsigned char *dest,
                                     int width, int height,
                                     int stride)
{
    int j;

    while (--height >= 0)
    {
        for (j = 0; j + 1 < width; j += 2)
        {
            int u = src[1];
            int v = src[3];
            int u1 = (((u - 128) << 7) + (u - 128)) >> 6;
            int rg = (((u - 128) << 1) + (u - 128) +
                      ((v - 128) << 2) + ((v - 128) << 1)) >>
                     3;
            int v1 = (((v - 128) << 1) + (v - 128)) >> 1;

            *dest++ = CLIP(src[0] + v1);
            *dest++ = CLIP(src[0] - rg);
            *dest++ = CLIP(src[0] + u1);

            *dest++ = CLIP(src[2] + v1);
            *dest++ = CLIP(src[2] - rg);
            *dest++ = CLIP(src[2] + u1);
            src += 4;
        }
        src += stride - (width * 2);
    }
}
/*******************************************************************/

Camera::Camera(const string &device, int width, int height, bool grayscale, int gain, int exposure) : device(device),
                                                                              xres(width),
                                                                              yres(height),
                                                                              grayscale(grayscale)
{
    open_device();
    init_device();
    // xres and yres are set to the actual resolution provided by the cam

    // frame stored as RGB888 (ie, RGB24)
    frame.width = xres;
    frame.height = yres;
    frame.size = xres * yres * 3;
    frame.data = (unsigned char *)malloc(frame.size * sizeof(char));

    updateGain(gain);
    updateBrightness();
    updateExposure(exposure);

    start_capturing();
}

Camera::~Camera()
{
    stop_capturing();
    uninit_device();
    close_device();
    running = false;
    if (auto_clear_th.joinable())
        auto_clear_th.join();

    free(frame.data);
}

void Camera::autoClearAfterCapture(float time)
{
    chrono::milliseconds j((int) (time * 1000));
    running = true;

    auto_clear_th = thread([=]() {
        while (running)
        {
            clearAfterCapture.lock();
            this_thread::sleep_for(j);
            clearFrame(); 
        }
    });
}

void Camera::clearFrame(int timeout)
{
    captureFrame(true, timeout);
}

const Image &Camera::captureFrame(bool throwaway, int timeout)
{
    for (;;)
    {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = timeout;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno)
                continue;
            throw runtime_error("select");
        }

        if (0 == r)
        {
            throw runtime_error(device + ": select timeout");
        }
        if (read_frame())
        {   
            return frame;
            clearAfterCapture.unlock();
        }
        /* EAGAIN - continue select loop. */
    }
}

void Camera::updateGain(int gain)
{
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_GAIN;
    ctrl.value = gain;

    if (-1 == xioctl(fd, VIDIOC_S_CTRL, &ctrl))
        throw runtime_error("VIDIOC_S_CTRL");
}

void Camera::updateBrightness(int brightness)
{
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_BRIGHTNESS;
    ctrl.value = brightness;

    if (-1 == xioctl(fd, VIDIOC_S_CTRL, &ctrl))
        throw runtime_error("VIDIOC_S_CTRL");

}

void Camera::updateExposure(int exposure)
{
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = exposure;
    
    if (-1 == xioctl(fd, VIDIOC_S_CTRL, &ctrl))
        throw runtime_error("VIDIOC_S_CTRL");

}


bool Camera::read_frame(bool throwaway)
{

    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
        case EAGAIN:
            return false;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
            throw runtime_error("VIDIOC_DQBUF");
        }
    }

    assert(buf.index < n_buffers);

    if (!throwaway){
        if (!grayscale)
        {
            v4lconvert_yuyv_to_rgb24((unsigned char *)buffers[buf.index].data,
                                     frame.data,
                                     xres,
                                     yres,
                                     stride);
        }
        else {
            for (int i = 0; i < xres*yres; i ++){
                frame.data[i] = * ((unsigned char *) buffers[buf.index].data + i);
            }
        }
    }
    

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        throw runtime_error("VIDIOC_QBUF");

    return true;
}

void Camera::open_device(void)
{
    struct stat st;

    if (-1 == stat(device.c_str(), &st))
    {
        throw runtime_error(device + ": cannot identify! " + to_string(errno) + ": " + strerror(errno));
    }

    if (!S_ISCHR(st.st_mode))
    {
        throw runtime_error(device + " is no device");
    }

    fd = open(device.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        throw runtime_error(device + ": cannot open! " + to_string(errno) + ": " + strerror(errno));
    }
}

void Camera::init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            throw runtime_error(device + " does not support memory mapping");
        }
        else
        {
            throw runtime_error("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 1)
    {
        throw runtime_error(string("Insufficient buffer memory on ") + device);
    }

    buffers = (buffer *)calloc(req.count, sizeof(*buffers));

    if (!buffers)
    {
        throw runtime_error("Out of memory");
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            throw runtime_error("VIDIOC_QUERYBUF");

        buffers[n_buffers].size = buf.length;
        buffers[n_buffers].data =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].data)
            throw runtime_error("mmap");
    }
}

void Camera::close_device(void)
{
    if (-1 == close(fd))
        throw runtime_error("close");

    fd = -1;
}

void Camera::init_device(void)
{
    struct v4l2_capability cap;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            throw runtime_error(device + " is no V4L2 device");
        }
        else
        {
            throw runtime_error("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        throw runtime_error(device + " is no video capture device");
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        throw runtime_error(device + " does not support streaming i/o");
    }

    set_fmt();

    init_mmap();
}

void Camera::set_crop(void)
{
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }
}

void Camera::set_fmt(void)
{
    struct v4l2_format fmt;
    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (force_format)
    {
        fmt.fmt.pix.width = xres;
        fmt.fmt.pix.height = yres;
        if (grayscale) {
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        } else {
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        }

        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
            throw runtime_error("VIDIOC_S_FMT");

        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV && !grayscale){
            throw runtime_error("Camera does not support YUYV format. Support for more format need to be added!");
        } else if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY && grayscale){
            throw runtime_error("Camera does not support GREY format. Support for more format need to be added!");
        }
            

        /* Note VIDIOC_S_FMT may change width and height. */
        xres = fmt.fmt.pix.width;
        yres = fmt.fmt.pix.height;

        stride = fmt.fmt.pix.bytesperline;
    }
    else
    {
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
            throw runtime_error("VIDIOC_G_FMT");
    }
}

void Camera::uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].data, buffers[i].size))
            throw runtime_error("munmap");

    free(buffers);
}

void Camera::start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < n_buffers; ++i)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            throw runtime_error("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        throw runtime_error("VIDIOC_STREAMON");
}

void Camera::stop_capturing(void)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        throw runtime_error("VIDIOC_STREAMOFF");
}
