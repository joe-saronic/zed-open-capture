#include "videocapture.hpp"

#include <sys/stat.h>         // for stat, S_ISCHR
#include <errno.h>            // for errno, EBADRQC, EINVAL, ENOBUFS, ENOENT
#include <fcntl.h>            // for open, O_NONBLOCK, O_RDONLY, O_RDWR
#include <unistd.h>           // for usleep, close

#include <linux/usb/video.h>  // for UVC_GET_CUR, UVC_SET_CUR, UVC_GET_LEN
#include <linux/uvcvideo.h>   // for uvc_xu_control_query, UVCIOC_CTRL_QUERY
#include <linux/videodev2.h>  // for v4l2_buffer, v4l2_queryctrl, V4L2_BUF_T...
#include <sys/mman.h>         // for mmap, munmap, MAP_SHARED, PROT_READ
#include <sys/ioctl.h>        // for ioctl

#include <sstream>
#include <fstream>            // for char_traits, basic_istream::operator>>

#include <cmath>              // for round

#define cbs_xu_unit_id          0x04 //mapped to wIndex 0x0400
#define cbs_xu_control_selector 0x02 //mapped to wValue 0x0200
#define READ_MODE   1
#define WRITE_MODE  2

#define ISP_LEFT    0x80181033
#define ISP_RIGHT   0x80181833

#define MASK_ON     0x02
#define MASK_OFF    0xFD

#define ADD_EXP_H   0x3500
#define ADD_EXP_M   0x3501
#define ADD_EXP_L   0x3502
#define ADD_GAIN_H  0x3507
#define ADD_GAIN_M  0x3508
#define ADD_GAIN_L  0x3509

#define XU_TASK_SET     0x50
#define XU_TASK_GET     0x51
#define XU_ISP_CTRL     0x07
#define XU_EXP_GAIN     0x25

#define LINUX_CTRL_BRIGHTNESS   9963776
#define LINUX_CTRL_CONTRAST     9963777
#define LINUX_CTRL_HUE          9963779
#define LINUX_CTRL_SATURATION   9963778
#define LINUX_CTRL_GAIN         9963795
#define LINUX_CTRL_AWB          9963802
#define LINUX_CTRL_AWB_AUTO     9963788
#define LINUX_CTRL_SHARPNESS    9963803
#define LINUX_CTRL_GAMMA        9963792

#define DEFAULT_GAMMA_NOECT 1
#define DEFAULT_MIN_GAMMA   1
#define DEFAULT_MAX_GAMMA   9

#define DEFAULT_MIN_GAIN   0
#define DEFAULT_MAX_GAIN   100
#define DEFAULT_MIN_EXP    0
#define DEFAULT_MAX_EXP    100

// Gain working zones
#define GAIN_ZONE1_MIN 0
#define GAIN_ZONE1_MAX 255
#define GAIN_ZONE2_MIN 378
#define GAIN_ZONE2_MAX 511
#define GAIN_ZONE3_MIN 890
#define GAIN_ZONE3_MAX 1023
#define GAIN_ZONE4_MIN 1914
#define GAIN_ZONE4_MAX 2047

// Raw exposure max values
#define EXP_RAW_MAX_15FPS   1550
#define EXP_RAW_MAX_30FPS   1100
#define EXP_RAW_MAX_60FPS   880
#define EXP_RAW_MAX_100FPS  720

#define EXP_RAW_MIN         2

namespace zed
{

VideoCapture::VideoCapture(VideoParams params)
{
    mVerbose = params.verbose;

    if( mVerbose )
    {
        std::string ver =
                "ZED Driver Version: "
                + std::to_string(mDrvMajorVer) + "."
                + std::to_string(mDrvMinorVer) + "."
                + std::to_string(mDrvPatchVer);
        VERBOSE_OUT( ver );
    }

    checkResFps( params );

    mGainSegMax = (GAIN_ZONE4_MAX-GAIN_ZONE4_MIN)+(GAIN_ZONE3_MAX-GAIN_ZONE3_MIN)+(GAIN_ZONE2_MAX-GAIN_ZONE2_MIN)+(GAIN_ZONE1_MAX-GAIN_ZONE1_MIN);

    if( mFps <= 15 )
        mExpoureRawMax = EXP_RAW_MAX_15FPS;
    else if( mFps <= 30 )
        mExpoureRawMax = EXP_RAW_MAX_30FPS;
    else if( mFps <= 60 )
        mExpoureRawMax = EXP_RAW_MAX_60FPS;
    else
        mExpoureRawMax = EXP_RAW_MAX_100FPS;
}

VideoCapture::~VideoCapture()
{
    reset();
}

void VideoCapture::reset()
{
    setLEDValue( false );

    mStopCapture = true;

    if( mGrabThread.joinable() )
    {
        mGrabThread.join();
    }

    // ----> Stop capturing
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (mFileDesc != -1)
        xioctl(mFileDesc, VIDIOC_STREAMOFF, &type);
    // <---- Stop capturing

    // ----> deinit device
    if( mInitialized && mBuffers)
    {
        for (unsigned int i = 0; i < mBufCount; ++i)
            munmap(mBuffers[i].start, mBuffers[i].length);
        if (mBuffers)
            free(mBuffers);

        mBuffers = nullptr;
    }
    // <---- deinit device

    if (mFileDesc)
    {
        close(mFileDesc);
        mFileDesc=-1;
    }

    if(mLastFrame.data)
    {
        delete [] mLastFrame.data;
        mLastFrame.data = nullptr;
    }

    if( mVerbose && mInitialized)
    {
        std::string msg = "Device closed";
        VERBOSE_OUT( msg );
    }

    mInitialized=false;
}

void VideoCapture::checkResFps( VideoParams par )
{
    mWidth = cameraResolution[static_cast<int>(par.res)].width*2;
    mHeight = cameraResolution[static_cast<int>(par.res)].height;
    mFps = static_cast<int>(par.fps);

    switch (par.res)
    {
    default:
        WARNING_OUT("RESOLUTION not supported. Using the best value");

    case RESOLUTION::HD2K:
        if( mFps!=15 )
        {
            WARNING_OUT("FPS not supported for the chosen resolution. Using the best value");
            mFps = 15;
        }
        break;

    case RESOLUTION::HD1080:
        if( mFps!=15 && mFps!=30 )
        {
            WARNING_OUT("FPS not supported for the chosen resolution. Using the best value");

            if( mFps <= 22  )
                mFps = 15;
            else
                mFps = 30;
        }
        break;

    case RESOLUTION::HD720:
        if( mFps!=15 && mFps!=30 && mFps!=60 )
        {
            WARNING_OUT("FPS not supported for the chosen resolution. Using the best value");

            if( mFps <= 22  )
                mFps = 15;
            else if( mFps < 45  )
                mFps = 30;
            else
                mFps = 60;
        }
        break;

    case RESOLUTION::VGA:
        if( mFps!=15 && mFps!=30 && mFps!=60 && mFps!=100)
        {
            WARNING_OUT("FPS not supported for the chosen resolution. Using the best value");

            if( mFps <= 22  )
                mFps = 15;
            else if( mFps < 45  )
                mFps = 30;
            else if( mFps < 80  )
                mFps = 60;
            else
                mFps = 100;
        }
    }

    if(mVerbose)
    {
        std::string msg = std::string("Camera resolution: ")
                + std::to_string(mWidth)
                + std::string("x")
                + std::to_string(mHeight)
                + std::string("@")
                + std::to_string(mFps)
                +std::string("Hz");

        VERBOSE_OUT(msg);
    }
}

bool VideoCapture::init( int devId/*=-1*/ )
{
    reset();

    bool opened=false;

    if( devId==-1 )
    {
        // Try to open all the devices until the first success (max allowed by v4l: 64)
        for( uint8_t id=0; id<64; id++ )
        {
            opened = openCamera( id );
            if(opened) break;
        }
    }
    else
    {
        opened = openCamera( static_cast<uint8_t>(devId) );
    }

    if(!opened)
    {
        return false;
    }

    mInitialized = startCapture();

    if( mVerbose && mInitialized)
    {
        std::string msg = "Device '" + mDevName + "' opened";
        VERBOSE_OUT( msg );
    }

    setLEDValue( true );

    return mInitialized;
}

bool VideoCapture::openCamera( uint8_t devId )
{
    mDevId = devId;

    mDevName = std::string("/dev/video") + std::to_string(mDevId);

    if( mVerbose )
    {
        std::string msg = "Trying to open the device '" + mDevName + "'";
        VERBOSE_OUT( msg );
    }

    // Check camera model
    mCameraModel = getCameraModel(mDevName);

    if( mCameraModel==zed::SL_DEVICE::NONE )
    {
        std::string msg = "The device '" + mDevName + "' is not a Stereolabs camera";
        VERBOSE_OUT( msg );
        return false;
    }

    if( mCameraModel==zed::SL_DEVICE::ZED ||
            mCameraModel==zed::SL_DEVICE::ZED_M )
    {
        std::string msg = "The FW of the device '" + mDevName + "' is not supported. Please update it.";
        VERBOSE_OUT( msg );
        return false;
    }

    // ----> Open
    struct stat st;
    memset(&st, 0, sizeof (struct stat));
    if (-1 == stat(mDevName.c_str(), &st))
    {
        if(mVerbose)
        {
            std::string msg = std::string("Cannot identify '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(msg);

            return false;
        }
    }

    if (!S_ISCHR(st.st_mode))
    {
        if(mVerbose)
        {
            std::string msg = mDevName + std::string(" is no device");
            ERROR_OUT(msg);

            return false;
        }
    }

    mFileDesc = 0;

    mFileDesc = open(mDevName.c_str(), O_RDWR|O_NONBLOCK,0); // Reading are non blocking

    if (-1 == mFileDesc)
    {
        if(mVerbose)
        {
            std::string msg = std::string("Cannot open '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(msg);
        }

        return false;
    }
    // <---- Open

    // ----> Init
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof (v4l2_capability));
    struct v4l2_cropcap cropcap;
    memset(&cropcap, 0, sizeof (v4l2_cropcap));
    struct v4l2_crop crop;
    memset(&crop, 0, sizeof (v4l2_crop));
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof (v4l2_format));

    if( -1==xioctl(mFileDesc, VIDIOC_QUERYCAP, &cap) )
    {
        if(mVerbose)
        {
            std::string msg = std::string("Cannot query capabilities of '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(msg);
        }

        return false;
    }

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Query the Capture
    if (0 == xioctl(mFileDesc, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
    }

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (mWidth * mHeight != 0)
    {
        fmt.fmt.pix.width = mWidth;
        fmt.fmt.pix.height = mHeight;
    }

    int width_tmp = mWidth;
    int height_tmp = mHeight;

    /* Preserve original settings as set by v4l2-ctl for example */
    if( -1==xioctl(mFileDesc, VIDIOC_S_FMT/*VIDIOC_G_FMT*/, &fmt) )
    {
        if(mVerbose)
        {
            std::string msg = std::string("Cannot set pixel format of '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(msg);
        }

        return false;
    }

    mWidth = fmt.fmt.pix.width;
    mHeight = fmt.fmt.pix.height;
    mChannels = fmt.fmt.pix.bytesperline / mWidth;

    // Asked resolution not available, exiting
    if (mWidth != width_tmp || mHeight != height_tmp)
    {
        ERROR_OUT("Error setting the camera resolution");
        return false;
    }

    if( -1==input_set_framerate(mFps) )
    {
        ERROR_OUT("Error setting the camera framerate");
    }

    // ----> Output frame allocation
    mLastFrame.width = mWidth;
    mLastFrame.height = mHeight;
    mLastFrame.channels = mChannels;
    int bufSize = mLastFrame.width * mLastFrame.height * mLastFrame.channels;
    mLastFrame.data = new unsigned char[bufSize];
    // <---- Output frame allocation

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof (v4l2_requestbuffers));

    req.count = mBufCount;
    mStartTs = getCurrentTs(); // Starting system timestamp

    struct timespec now = {0};
    clock_gettime(CLOCK_MONOTONIC, &now);

    mInitTs = ((uint64_t) now.tv_sec * NSEC_PER_SEC + (uint64_t) now.tv_nsec) / 1000;

    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if( -1==xioctl(mFileDesc, VIDIOC_REQBUFS, &req) )
    {
        if(mVerbose)
        {
            std::string msg = std::string("Cannot request buffers for '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(msg);
        }

        return false;
    }

    // Create buffers
    mBuffers = (Buffer*) calloc(req.count, sizeof(*mBuffers));

    for(mBufCount = 0; mBufCount < req.count; ++mBufCount)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof (v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = mBufCount;
        buf.flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        if( -1==xioctl(mFileDesc, VIDIOC_QUERYBUF, &buf))
        {
            if(mVerbose)
            {
                std::string msg = std::string("Cannot query buffer for '") + mDevName + "': ["
                        + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
                ERROR_OUT(msg);
            }

            return false;
        }

        mBuffers[mBufCount].length = buf.length;

        mBuffers[mBufCount].start =
                mmap(nullptr /* start anywhere */,
                     buf.length,
                     PROT_READ | PROT_WRITE /* required */,
                     MAP_SHARED /* recommended */,
                     mFileDesc, buf.m.offset);
    }

    mBufCount = req.count;
    // <---- Init

    return true;
}

bool VideoCapture::startCapture()
{
    // ----> Start capturing
    enum v4l2_buf_type type;
    for (unsigned int i = 0; i < mBufCount; ++i)
    {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if( -1==xioctl(mFileDesc, VIDIOC_QBUF, &buf) )
        {
            if(mVerbose)
            {
                std::string msg = std::string("Cannot queue buffer for '") + mDevName + "': ["
                        + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
                ERROR_OUT(msg);
            }

            return false;
        }
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Set priority
    int priority = V4L2_PRIORITY_RECORD;
    if( -1==xioctl(mFileDesc, VIDIOC_G_PRIORITY, &priority) )
    {
        if(mVerbose)
        {
            std::string msg = std::string("Cannot set priority for '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(msg);
        }

        return false;
    }

    // Start streaming
    if( -1==xioctl(mFileDesc, VIDIOC_STREAMON, &type) )
    {
        if(mVerbose)
        {
            std::string msg = std::string("Cannot start streaming for '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(msg);
        }

        return false;
    }
    // <---- Start capturing

    mGrabThread = std::thread( &VideoCapture::grabThreadFunc,this );

    return true;
}

int VideoCapture::input_set_framerate(int fps)
{
    struct v4l2_streamparm streamparm = {0}; // v4l2 stream parameters struct

    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = fps;

    return xioctl(mFileDesc, VIDIOC_S_PARM, &streamparm);
}

#define IOCTL_RETRY 3

int VideoCapture::xioctl(int fd, uint64_t IOCTL_X, void *arg)
{
    int ret = 0;
    int tries = IOCTL_RETRY;
    do
    {
        ret = ioctl(fd, IOCTL_X, arg);
        // usleep(1);
    } while (ret && tries-- &&
             ((errno == EINTR) || (errno == EAGAIN) || (errno == ETIMEDOUT)));

    if( ret==-1 )
    {
        if(mVerbose)
        {
            perror( "xioctl");
        }
    }

    return (ret);
}

zed::SL_DEVICE VideoCapture::getCameraModel( std::string dev_name)
{
    zed::SL_DEVICE camera_device = zed::SL_DEVICE::NONE;
    int vid = 0, pid = 0;
    std::string modalias = "";
    std::string name = dev_name.erase(0, 5); //remove /dev/
    if (!(std::ifstream("/sys/class/video4linux/" + name + "/device/modalias") >> modalias))
    {
        if(mVerbose)
        {
            std::string msg =
                    std::string(" Not a modalias : /sys/class/video4linux/")
                    + name + std::string("/device/modalias");
            WARNING_OUT( msg);
        }
        return camera_device;
    }

    if (modalias.size() < 14 || modalias.substr(0, 5) != "usb:v" || modalias[9] != 'p')
    {
        if(mVerbose)
        {
            std::string msg = std::string(" not a modalias 2" );
            WARNING_OUT( msg);
        }
        return camera_device;
    }

    if (!(std::istringstream(modalias.substr(5, 4)) >> std::hex >> vid))
    {
        if(mVerbose)
        {
            std::string msg = std::string("unable to read Vendor ID" );
            WARNING_OUT( msg);
        }


        return camera_device;
    }

    if (!(std::istringstream(modalias.substr(10, 4)) >> std::hex >> pid))
    {
        if(mVerbose)
        {
            std::string msg = std::string("unable to read Product ID" );
            WARNING_OUT( msg);
        }

        return camera_device;
    }

    // check PID VID
    if (pid == SL_USB_PROD_ZED && vid == SL_USB_VENDOR)
        camera_device = zed::SL_DEVICE::ZED;
    else if (pid == SL_USB_PROD_ZED_M && vid == SL_USB_VENDOR)
        camera_device = zed::SL_DEVICE::ZED_M;
    else if (pid == SL_USB_PROD_ZED_CBS && vid == SL_USB_VENDOR)
        camera_device = zed::SL_DEVICE::ZED_CBS;
    else if (pid == SL_USB_PROD_ZED_M_CBS && vid == SL_USB_VENDOR)
        camera_device = zed::SL_DEVICE::ZED_M_CBS;
    else if (pid == SL_USB_PROD_ZED_2_CBS && vid == SL_USB_VENDOR)
        camera_device = zed::SL_DEVICE::ZED_2_CBS;

    return camera_device;
}

void VideoCapture::grabThreadFunc()
{
    mNewFrame = false;
    mStopCapture = false;

    fd_set fds;
    struct timeval tv = {0};

    FD_ZERO(&fds);

    if (mFileDesc < 0)
        return;

    FD_SET(mFileDesc, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    select(mFileDesc + 1, &fds, nullptr, nullptr, &tv);

    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.bytesused = -1;
    buf.length = 0;

    uint64_t current_ts = 0;
    int capture_frame_count = 0;

    while (!mStopCapture)
    {
        mGrabRunning=true;

        mComMutex.lock();
        int ret = ioctl(mFileDesc, VIDIOC_DQBUF, &buf);
        mComMutex.unlock();

        if (buf.bytesused == buf.length && ret == 0 && buf.index < mBufCount)
        {
            mCurrentIndex = buf.index;
            // get buffer timestamp in us
            current_ts = ((uint64_t) buf.timestamp.tv_sec * (1000 * 1000) + (uint64_t) buf.timestamp.tv_usec) - mInitTs;
            // cvt to ns
            current_ts *= 1000;

            //if(!grabClass->vflip)
            mBufMutex.lock();
            mLastFrame.timestamp = mStartTs + current_ts;

            if (mLastFrame.data != nullptr && mWidth != 0 && mHeight != 0 && mBuffers[mCurrentIndex].start != nullptr)
            {
                mLastFrame.frame_id++;
                mLastFrame.timestamp = current_ts;
                memcpy(mLastFrame.data, (unsigned char*) mBuffers[mCurrentIndex].start, mBuffers[mCurrentIndex].length);
                mNewFrame=true;
            }
            mBufMutex.unlock();

            mComMutex.lock();
            ioctl(mFileDesc, VIDIOC_QBUF, &buf);
            mComMutex.unlock();

            capture_frame_count++;
        }
        else
        {
            if (buf.bytesused != buf.length)
            {
                mComMutex.lock();
                ioctl(mFileDesc, VIDIOC_QBUF, &buf);
                mComMutex.unlock();
            }
            usleep(200);
            buf.bytesused = -1;
            buf.length = 0;
        }
    }

    mGrabRunning = false;
}

const Frame *VideoCapture::getLastFrame( uint64_t timeout_msec )
{
    // ----> Wait for a new frame
    uint64_t time_count = timeout_msec*10;
    while( !mNewFrame )
    {
        if(--time_count==0)
        {
            return nullptr;
        }
        usleep(100);
    }
    // <---- Wait for a new frame

    // Get the frame mutex
    const std::lock_guard<std::mutex> lock(mBufMutex);
    mNewFrame = false;
    return &mLastFrame;
}



int VideoCapture::linux_cbs_VendorControl(unsigned char *buf, int len, int readMode, bool safe) {

    if (len > 384)
        return -2;

    if (!mInitialized)
        return -3;

    unsigned char tmp[2] = {0};
    struct uvc_xu_control_query xu_query_info;
    xu_query_info.unit = cbs_xu_unit_id;
    xu_query_info.selector = cbs_xu_control_selector;
    xu_query_info.query = UVC_GET_LEN;
    xu_query_info.size = 2;
    xu_query_info.data = tmp;

    const std::lock_guard<std::mutex> lock(mComMutex);

    int io_err = ioctl(mFileDesc, UVCIOC_CTRL_QUERY, &xu_query_info);

    if (io_err != 0) {
        return -4;
    } else {
        len = (xu_query_info.data[1] << 8) + xu_query_info.data[0];
    }


    //len should be now 384 for USB3 and 64 for USB2
    // we use the UVC_SET_CUR to write the cmd
    struct uvc_xu_control_query xu_query_send;
    xu_query_send.unit = cbs_xu_unit_id;
    xu_query_send.selector = cbs_xu_control_selector;
    xu_query_send.query = UVC_SET_CUR;
    xu_query_send.size = static_cast<__u16> (len); //64 for USB2
    xu_query_send.data = buf;

    io_err = ioctl(mFileDesc, UVCIOC_CTRL_QUERY, &xu_query_send);
    if (io_err != 0) {
        int res = errno;

        const char *err=nullptr;
        switch (res) {
        case ENOENT:
            err = "Extension unit or control not found";
            break;
        case ENOBUFS:
            err = "Buffer size does not match control size";
            break;
        case EINVAL:
            err = "Invalid request code";
            break;
        case EBADRQC:
            err = "Request not supported by control";
            break;
        default:
            err = strerror(res);
            break;
        }
        if(mVerbose)
        {
            std::string msg = std::string("CBS SET failed") +
                    std::string(err) +
                    std::string(". (System code: ") +
                    std::to_string(res) +
                    std::string(") ") +
                    std::to_string(xu_query_send.size);
            ERROR_OUT(msg);
        }
        return -1;
    }

#if 1
    if (!safe)
        usleep(300);
    else
        usleep(2000);

    if (readMode == READ_MODE) {

        struct uvc_xu_control_query xu_query;
        xu_query.unit = cbs_xu_unit_id,
                xu_query.selector = cbs_xu_control_selector,
                xu_query.query = UVC_GET_CUR,
                xu_query.size = static_cast<__u16> (len),
                xu_query.data = buf;

        io_err = ioctl(mFileDesc, UVCIOC_CTRL_QUERY, &xu_query);
        if (io_err != 0) {
            int res = errno;

            const char *err;
            switch (res) {
            case ENOENT: err = "Extension unit or control not found";
                break;
            case ENOBUFS: err = "Buffer size does not match control size";
                break;
            case EINVAL: err = "Invalid request code";
                break;
            case EBADRQC: err = "Request not supported by control";
                break;
            default: err = strerror(res);
                break;
            }

            if(mVerbose)
            {
                std::string msg = std::string("CBS GET failed") +
                        std::string(err) +
                        std::string(". (System code: ") +
                        std::to_string(res) +
                        std::string(") ") +
                        std::to_string(xu_query_send.size);
                ERROR_OUT(msg);
            }
            return -1;
        }

        return 0;
    }
    else
    {
        return 0;
    }

#endif


}

/**
 * @brief cbs_set_gpio_value
 * @param gpio_number (0 to 4)
 * @param value(0x00: low, 0x01 : high)
 * @return 0 if success
 */
int VideoCapture::linux_cbs_get_gpio_value(int gpio_number, uint8_t *value)
{
    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);

    //Set xubuf
    xu_buf[0] = XU_TASK_GET;
    xu_buf[1] = 0x13;
    xu_buf[2] = gpio_number;

    int hr = linux_cbs_VendorControl(xu_buf, 384, 1);
    *value = xu_buf[17];
    return hr;
}

/**
 * @brief cbs_set_gpio_value
 * @param gpio_number (0 to 4)
 * @param value(0x00: low, 0x01 : high)
 * @return 0 if success
 */
int VideoCapture::linux_cbs_set_gpio_value(int gpio_number, uint8_t value)
{
    unsigned char xu_buf[64];
    memset(xu_buf, 0, 64);

    //Set xubuf
    xu_buf[0] = XU_TASK_SET;
    xu_buf[1] = 0x12;
    xu_buf[2] = gpio_number;
    xu_buf[3] = value;

    int hr = linux_cbs_VendorControl(xu_buf, 64, 0);
    return hr;
}

/**
 * @brief cbs_set_gpio_direction
 * @param gpio_number (0 to 4)
 * @param direction (0x00 output, 0x01 input)
 * @return  0 if success
 */
int VideoCapture::linux_cbs_set_gpio_direction(int gpio_number, int direction)
{
    unsigned char xu_buf[64];
    memset(xu_buf, 0, 64);

    //Set xubuf
    xu_buf[0] = XU_TASK_SET;
    xu_buf[1] = 0x10;
    xu_buf[2] = gpio_number;
    xu_buf[3] = direction;

    int hr = linux_cbs_VendorControl(xu_buf, 64, 0);
    return hr;
}

int VideoCapture::linux_cbs_read_system_register(uint64_t address, uint8_t *value)
{
    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);

    //Set xubuf
    xu_buf[0] = XU_TASK_GET;
    xu_buf[1] = 0xA2;
    xu_buf[2] = 0;
    xu_buf[3] = 0x04; //Address width in bytes
    xu_buf[4] = 0x01; //data width in bytes

    xu_buf[5] = ((address) >> 24) & 0xff;
    xu_buf[6] = ((address) >> 16) & 0xff;
    xu_buf[7] = ((address) >> 8) & 0xff;
    xu_buf[8] = (address) & 0xff;
    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;

    int hr = linux_cbs_VendorControl(xu_buf, 384, READ_MODE);
    *value = xu_buf[17];
    return hr;
}

int VideoCapture::linux_cbs_write_system_register(uint64_t address, uint8_t value)
{
    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);

    //Set xubuf
    xu_buf[0] = XU_TASK_SET;
    xu_buf[1] = 0xA2;
    xu_buf[2] = 0;
    xu_buf[3] = 0x04; //Address width in bytes
    xu_buf[4] = 0x01; //data width in bytes

    xu_buf[5] = ((address) >> 24) & 0xff;
    xu_buf[6] = ((address) >> 16) & 0xff;
    xu_buf[7] = ((address) >> 8) & 0xff;
    xu_buf[8] = (address) & 0xff;
    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;
    xu_buf[16] = value;

    int hr = linux_cbs_VendorControl(xu_buf, 384, 0);
    return hr;
}

#define ASIC_INT_NULL_I2C    0xa3
#define ASIC_INT_I2C         0xa5

int VideoCapture::linux_cbs_read_sensor_register(int side, int sscb_id, uint64_t address, uint8_t* value)
{
    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);


    //Set xubuf
    xu_buf[0] = XU_TASK_GET;
    if (side == 0)
        xu_buf[1] = ASIC_INT_NULL_I2C;
    else
        xu_buf[1] = ASIC_INT_I2C;
    xu_buf[2] = 0x6c;
    xu_buf[3] = sscb_id + 1; //Address width in bytes
    xu_buf[4] = 0x01; //data width in bytes

    xu_buf[5] = ((address) >> 24) & 0xff;
    xu_buf[6] = ((address) >> 16) & 0xff;
    xu_buf[7] = ((address) >> 8) & 0xff;
    xu_buf[8] = (address) & 0xff;

    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;

    int limit = 1;

    xu_buf[9] = (limit >> 8) & 0xff;
    xu_buf[10] = (limit >> 0) & 0xff;

    //set page addr
    xu_buf[9] = xu_buf[9] & 0x0f;
    xu_buf[9] = xu_buf[9] | 0x10;
    xu_buf[9] = xu_buf[9] | 0x80;

    int hr = linux_cbs_VendorControl(xu_buf, 384, READ_MODE);
    *value = xu_buf[17];
    return hr;
}

int VideoCapture::linux_cbs_write_sensor_register(int side, int sscb_id, uint64_t address, uint8_t value)
{

    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);

    //Set xubuf
    xu_buf[0] = XU_TASK_SET;
    if (side == 0)
        xu_buf[1] = ASIC_INT_NULL_I2C;
    else
        xu_buf[1] = ASIC_INT_I2C;
    xu_buf[2] = 0x6c;
    xu_buf[3] = sscb_id + 1; //Address width in bytes
    xu_buf[4] = 0x01; //data width in bytes

    xu_buf[5] = ((address) >> 24) & 0xff;
    xu_buf[6] = ((address) >> 16) & 0xff;
    xu_buf[7] = ((address) >> 8) & 0xff;
    xu_buf[8] = (address) & 0xff;
    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;
    xu_buf[16] = value;


    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;

    int limit = 1;
    xu_buf[9] = (limit >> 8) & 0xff;
    xu_buf[10] = (limit >> 0) & 0xff;

    //set page addr
    xu_buf[9] = xu_buf[9] & 0x0f;
    xu_buf[9] = xu_buf[9] | 0x10;
    xu_buf[9] = xu_buf[9] | 0x80;
    memcpy(&xu_buf[16], &value, sizeof (uint8_t));
    int hr = linux_cbs_VendorControl(xu_buf, 384, 0);

    return hr;
}

int VideoCapture::linux_cbs_ispaecagc_enable(int side, bool enable) {
    uint64_t Address = 0;
    uint8_t value = 0;
    int hr = 0;
    if (side == 0)
        Address = ISP_LEFT; //ISP L
    else if (side == 1)
        Address = ISP_RIGHT; //ISP R
    else
        return -2;

    //Read Current sysregister
    hr += linux_cbs_read_system_register(Address, &value);

    // Adjust Value
    if (enable)
        value = value | MASK_ON;
    else
        value = value & MASK_OFF;

    hr += linux_cbs_write_system_register(Address, value);

    return hr;
}

int VideoCapture::linux_cbs_is_aecagc(int side) {
    uint64_t Address = 0;
    uint8_t value = 0;
    int result = 0;
    int hr = 0;
    if (side == 0)
        Address = ISP_LEFT; //ISP L
    else if (side == 1)
        Address = ISP_RIGHT; //ISP R
    else
        return -2;

    //Read Current sysregister
    hr += linux_cbs_read_system_register(Address, &value);

    // Adjust Value
    if (hr == 0)
        result = ((value & MASK_ON) == MASK_ON); //check that secodn bit is on
    else
        return hr;

    return result;
}

int VideoCapture::linux_ISPGainGet(uint8_t *val, uint8_t sensorID) {
    int hr = 0;
    uint8_t buffL, buffM, buffH;

    hr += linux_cbs_read_sensor_register(sensorID, 1, ADD_GAIN_H, &buffH);
    hr += linux_cbs_read_sensor_register(sensorID, 1, ADD_GAIN_M, &buffM);
    hr += linux_cbs_read_sensor_register(sensorID, 1, ADD_GAIN_L, &buffL);

    *val = buffL;
    *(val + 1) = buffM;
    *(val + 2) = buffH;

    return hr;
}

int VideoCapture::linux_ISPManualGain(unsigned char ucGainH, unsigned char ucGainM, unsigned char ucGainL, int sensorID)
{
    int hr = 0;
    hr += linux_cbs_write_sensor_register(sensorID, 1, ADD_GAIN_H, ucGainH);
    hr += linux_cbs_write_sensor_register(sensorID, 1, ADD_GAIN_M, ucGainM);
    hr += linux_cbs_write_sensor_register(sensorID, 1, ADD_GAIN_L, ucGainL);
    return hr;
}

int VideoCapture::linux_ISPExposureGet(unsigned char *val, unsigned char sensorID)
{
    int hr = 0;
    uint8_t buffL = 0;
    uint8_t buffM = 0;
    uint8_t buffH = 0;

    hr += linux_cbs_read_sensor_register(sensorID, 1, ADD_EXP_H, (uint8_t*) & buffH);
    usleep(10);
    hr += linux_cbs_read_sensor_register(sensorID, 1, ADD_EXP_M, (uint8_t*) & buffM);
    usleep(10);
    hr += linux_cbs_read_sensor_register(sensorID, 1, ADD_EXP_L, (uint8_t*) & buffL);

    *val = buffL;
    *(val + 1) = buffM;
    *(val + 2) = buffH;

    return hr;
}

int VideoCapture::linux_ISPManualExposure(unsigned char ucExpH, unsigned char ucExpM, unsigned char ucExpL, int sensorID)
{
    int hr = 0;
    hr += linux_cbs_write_sensor_register(sensorID, 1, ADD_EXP_H, ucExpH);
    hr += linux_cbs_write_sensor_register(sensorID, 1, ADD_EXP_M, ucExpM);
    hr += linux_cbs_write_sensor_register(sensorID, 1, ADD_EXP_L, ucExpL);
    return hr;
}

int VideoCapture::setLEDValue(bool status)
{
    int hr = 0;
    //LED GPIO : GPIO 2
    if (status) {
        hr += linux_cbs_set_gpio_direction(2, 0);
        hr += linux_cbs_set_gpio_value(2, 1);
    } else {
        hr += linux_cbs_set_gpio_direction(2, 0);
        hr += linux_cbs_set_gpio_value(2, 0);
    }

    return hr;
}

int VideoCapture::getLEDValue(bool *value)
{
    if( value==nullptr)
    {
        return -1;
    }

    uint8_t val;
    int hr = linux_cbs_set_gpio_direction(2, 1);
    hr += linux_cbs_get_gpio_value(2, &val);
    *value = val!=0;
    return hr;
}

int VideoCapture::toggleLED(bool* value)
{
    bool curVal;

    int ret = getLEDValue( &curVal );

    if(ret==0)
    {
        bool newVal = !curVal;
        ret = setLEDValue( newVal );

        if( ret==0 && value!=nullptr )
        {
            *value = newVal;
        }
    }

    return ret;
}

int VideoCapture::getCameraControlSettings(int ctrl_id)
{
    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof (queryctrl));
    memset(&control_s, 0, sizeof (control_s));
    int res = -1;

    // save_controls(fd);
    queryctrl.id = ctrl_id;

    if (0 != ioctl(mFileDesc, VIDIOC_QUERYCTRL, &queryctrl))
        return res;

    control_s.id = ctrl_id;
    if (ioctl(mFileDesc, VIDIOC_G_CTRL, &control_s) == 0)
        res = (int) control_s.value;

    return res;
}

void VideoCapture::setCameraControlSettings(int ctrl_id, int ctrl_val) {
    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof (queryctrl));
    memset(&control_s, 0, sizeof (control_s));
    int min, max/*, step, val_def*/;

    // save_controls(fd);
    queryctrl.id = ctrl_id;
    int res = ioctl(mFileDesc, VIDIOC_QUERYCTRL, &queryctrl);
    if (0 == res) {
        min = queryctrl.minimum;
        max = queryctrl.maximum;
        //step = queryctrl.step;
        //val_def = queryctrl.default_value;

        if (ctrl_id == LINUX_CTRL_GAMMA) {
            min = DEFAULT_MIN_GAMMA;
            max = DEFAULT_MAX_GAMMA;
        }

    } else {
        min = 0; // queryctrl.minimum;
        max = 6500; // queryctrl.maximum;
        //step = queryctrl.step;
        //val_def = queryctrl.default_value;
    }

    if ((ctrl_val >= min) && (ctrl_val <= max)) {
        control_s.id = ctrl_id;
        control_s.value = ctrl_val;


        if (ioctl(mFileDesc, VIDIOC_S_CTRL, &control_s) == 0)
            return;
    } else
        return;
}

void VideoCapture::resetCameraControlSettings(int ctrl_id) {

    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof (queryctrl));
    memset(&control_s, 0, sizeof (control_s));
    int val_def;
    // save_controls(fd);
    queryctrl.id = ctrl_id;
    ioctl(mFileDesc, VIDIOC_QUERYCTRL, &queryctrl);
    val_def = queryctrl.default_value;

    control_s.id = ctrl_id;
    control_s.value = val_def;
    ioctl(mFileDesc, VIDIOC_S_CTRL, &control_s);
    return;
}

int VideoCapture::setGammaPreset(int side, int value)
{
    if (!mInitialized)
        return -1;

    if(value < DEFAULT_MIN_GAMMA)
        value = DEFAULT_MIN_GAMMA;
    if(value > DEFAULT_MAX_GAMMA)
        value = DEFAULT_MAX_GAMMA;

    uint64_t ulAddr = 0x80181500;

    if (side == 1)
        ulAddr = 0x80181D00;

    int hr = 0;

    for (int i = 0; i < 15; i++) {
        hr += linux_cbs_write_system_register(ulAddr, cbs::PRESET_GAMMA[value-1][i]);
        usleep(10);
        uint8_t valRead = 0x0;
        hr += linux_cbs_read_system_register(ulAddr, &valRead);
        if (valRead != cbs::PRESET_GAMMA[value-1][i]) {
            return -3;
        }
        ulAddr++;
    }

    ulAddr = 0x80181510;

    if (side == 1)
        ulAddr = 0x80181D10;
    hr += linux_cbs_write_system_register(ulAddr, 0x01);
    usleep(10);
    uint8_t valRead = 0x0;
    hr += linux_cbs_read_system_register(ulAddr, &valRead);
    if (valRead != 0x01)
        return -2;

    return hr;
}

void VideoCapture::setBrightnessSetting(int value)
{
    setCameraControlSettings(LINUX_CTRL_BRIGHTNESS, value);
}

void VideoCapture::resetBrightnessSetting()
{
    resetCameraControlSettings(LINUX_CTRL_BRIGHTNESS);
}

int VideoCapture::getBrightnessSetting()
{
    return getCameraControlSettings(LINUX_CTRL_BRIGHTNESS);
}

void VideoCapture::setSharpnessSetting(int value)
{
    setCameraControlSettings(LINUX_CTRL_SHARPNESS, value);
}

void VideoCapture::resetSharpnessSetting()
{
    resetCameraControlSettings(LINUX_CTRL_SHARPNESS);
}

int VideoCapture::getSharpnessSetting()
{
    return getCameraControlSettings(LINUX_CTRL_SHARPNESS);
}

void VideoCapture::setContrastSetting(int value)
{
    setCameraControlSettings(LINUX_CTRL_CONTRAST, value);
}

void VideoCapture::resetContrastSetting()
{
    resetCameraControlSettings(LINUX_CTRL_CONTRAST);
}

int VideoCapture::getContrastSetting()
{
    return getCameraControlSettings(LINUX_CTRL_CONTRAST);
}

void VideoCapture::setHueSetting(int value)
{
    setCameraControlSettings(LINUX_CTRL_HUE, value);
}

void VideoCapture::resetHueSetting()
{
    resetCameraControlSettings(LINUX_CTRL_HUE);
}

int VideoCapture::getHueSetting()
{
    return getCameraControlSettings(LINUX_CTRL_HUE);
}

void VideoCapture::setSaturationSetting(int value)
{
    setCameraControlSettings(LINUX_CTRL_SATURATION, value);
}

void VideoCapture::resetSaturationSetting()
{
    resetCameraControlSettings(LINUX_CTRL_SATURATION);
}

int VideoCapture::getSaturationSetting()
{
    return getCameraControlSettings(LINUX_CTRL_SATURATION);
}

int VideoCapture::getWhiteBalanceSetting()
{
    return getCameraControlSettings(LINUX_CTRL_AWB);
}

void VideoCapture::setWhiteBalanceSetting(int value)
{
    // Disable auto white balance if active
    if (getAutoWhiteBalanceSetting() != 0)
        setAutoWhiteBalanceSetting(false);

    setCameraControlSettings(LINUX_CTRL_AWB, value);
}

void VideoCapture::resetWhiteBalanceSetting()
{
    resetAutoWhiteBalanceSetting();
}

bool VideoCapture::getAutoWhiteBalanceSetting()
{
    return (getCameraControlSettings(LINUX_CTRL_AWB_AUTO)!=0);
}

void VideoCapture::setAutoWhiteBalanceSetting(bool active)
{
    setCameraControlSettings(LINUX_CTRL_AWB_AUTO, active?1:0);
}

void VideoCapture::resetAutoWhiteBalanceSetting()
{
    setAutoWhiteBalanceSetting(true);
}

void VideoCapture::setGammaSetting(int value)
{
    int current_gamma = getCameraControlSettings(LINUX_CTRL_GAMMA);

    if (value!=current_gamma)
    {
        setGammaPreset(0,value);
        setGammaPreset(1,value);
        setCameraControlSettings(LINUX_CTRL_GAMMA, value);
    }
}

void VideoCapture::resetGammaSetting()
{
    int def_value = DEFAULT_GAMMA_NOECT;
    setGammaPreset(0,def_value);
    setGammaPreset(1,def_value);
    setCameraControlSettings(LINUX_CTRL_GAMMA, def_value);
}

int VideoCapture::getGammaSetting() {
    return getCameraControlSettings(LINUX_CTRL_GAMMA);
}

int VideoCapture::setAECAGC(bool active)
{
    int res = 0;
    res += linux_cbs_ispaecagc_enable(0, active);
    res += linux_cbs_ispaecagc_enable(1, active);
    return res;
}

bool VideoCapture::getAECAGC()
{
    int resL = linux_cbs_is_aecagc(0);
    int resR = linux_cbs_is_aecagc(1);
    return (resL && resR);
}

void VideoCapture::resetAECAGC()
{
    setAECAGC(true);
}

void VideoCapture::setGainSetting(CAM_SENS_POS cam, int gain)
{
    if(getAECAGC())
        setAECAGC(false);

    if (gain <= DEFAULT_MIN_GAIN)
        gain = DEFAULT_MIN_GAIN;
    else if (gain >= DEFAULT_MAX_GAIN)
        gain = DEFAULT_MAX_GAIN;

    uint8_t ucGainH=0, ucGainM=0, ucGainL=0;

    int rawGain = calcRawGainValue(gain);

    int sensorId = static_cast<int>(cam);

    ucGainM = (rawGain >> 8) & 0xff;
    ucGainL = rawGain & 0xff;
    linux_ISPManualGain(ucGainH, ucGainM, ucGainL, sensorId);

}

int VideoCapture::getGainSetting(CAM_SENS_POS cam)
{
    int rawGain=0;

    uint8_t val[3];
    memset(val, 0, 3);

    int sensorId = static_cast<int>(cam);
    int r = linux_ISPGainGet(val, sensorId);
    if(r<0)
        return r;

    rawGain = (int) ((val[1] << 8) + val[0]);
    return calcGainValue(rawGain);
}

void VideoCapture::setExposureSetting(CAM_SENS_POS cam, int exposure)
{
    unsigned char ucExpH, ucExpM, ucExpL;

    if(getAECAGC())
        setAECAGC(false);

    if(exposure < DEFAULT_MIN_EXP)
        exposure = DEFAULT_MIN_EXP;
    if(exposure > DEFAULT_MAX_EXP)
        exposure = DEFAULT_MAX_EXP;

    int rawExp = (mExpoureRawMax * ((float) exposure / 100.0));
    if(rawExp<EXP_RAW_MIN)
        rawExp = EXP_RAW_MIN;

    std::cout << "Set Raw Exp: " << rawExp << std::endl;

    int sensorId = static_cast<int>(cam);

    ucExpH = (rawExp >> 12) & 0xff;
    ucExpM = (rawExp >> 4) & 0xff;
    ucExpL = (rawExp << 4) & 0xf0;
    linux_ISPManualExposure(ucExpH, ucExpM, ucExpL, sensorId);
}

int VideoCapture::getExposureSetting(CAM_SENS_POS cam)
{
    int rawExp=0;

    unsigned char val[3];
    memset(val, 0, 3);

    int sensorId = static_cast<int>(cam);

    int r = linux_ISPExposureGet(val, sensorId);
    if(r<0)
        return r;
    rawExp = (int) ((val[2] << 12) + (val[1] << 4) + (val[0] >> 4));

    std::cout << "Get Raw Exp: " << rawExp << std::endl;

    int exposure = static_cast<int>(std::round((100.0*rawExp)/mExpoureRawMax));
    return exposure;
}

int VideoCapture::calcRawGainValue(int gain) {

    // From [0,100] to segmented gain
    int segmentedGain = static_cast<int>(std::round(mGainSegMax * (static_cast<double>(gain) / 100.0)));
    int rawGain = 0;

    // ----> Calculate correct GAIN ZONE
    int gainZone1nbVal = GAIN_ZONE1_MAX - GAIN_ZONE1_MIN;
    int gainZone2nbVal = GAIN_ZONE2_MAX - GAIN_ZONE2_MIN;
    int gainZone3nbVal = GAIN_ZONE3_MAX - GAIN_ZONE3_MIN;
    int gainZone4nbVal = GAIN_ZONE4_MAX - GAIN_ZONE4_MIN;

    if (segmentedGain <= gainZone1nbVal)
        rawGain = segmentedGain + GAIN_ZONE1_MIN;
    else if (segmentedGain <= gainZone1nbVal + gainZone2nbVal)
        rawGain = segmentedGain + GAIN_ZONE2_MIN - (gainZone1nbVal);
    else if (segmentedGain <= gainZone1nbVal + gainZone2nbVal + gainZone3nbVal)
        rawGain = segmentedGain + GAIN_ZONE3_MIN - (gainZone1nbVal + gainZone2nbVal);
    else if (segmentedGain <= gainZone1nbVal + gainZone2nbVal + gainZone3nbVal + gainZone4nbVal)
        rawGain = segmentedGain + GAIN_ZONE4_MIN - (gainZone1nbVal + gainZone2nbVal + gainZone3nbVal);
    // <---- Calculate correct GAIN ZONE

    return rawGain;
}

int VideoCapture::calcGainValue(int rawGain)
{
    int segmentedGain;

    // ----> Calculate correct GAIN ZONE
    int gainZone1nbVal = GAIN_ZONE1_MAX - GAIN_ZONE1_MIN;
    int gainZone2nbVal = GAIN_ZONE2_MAX - GAIN_ZONE2_MIN;
    int gainZone3nbVal = GAIN_ZONE3_MAX - GAIN_ZONE3_MIN;
    if (rawGain >= GAIN_ZONE1_MIN && rawGain <= GAIN_ZONE1_MAX)
        segmentedGain = rawGain - GAIN_ZONE1_MIN;
    else if (rawGain >= GAIN_ZONE2_MIN && rawGain <= GAIN_ZONE2_MAX)
        segmentedGain = rawGain - GAIN_ZONE2_MIN + gainZone1nbVal;
    else if (rawGain >= GAIN_ZONE3_MIN && rawGain <= GAIN_ZONE3_MAX)
        segmentedGain = rawGain - GAIN_ZONE3_MIN + gainZone1nbVal + gainZone2nbVal;
    else if (rawGain >= GAIN_ZONE4_MIN && rawGain <= GAIN_ZONE4_MAX)
        segmentedGain = rawGain - GAIN_ZONE4_MIN + gainZone1nbVal + gainZone2nbVal + gainZone3nbVal;
    else
        segmentedGain = -1;
    // <---- Calculate correct GAIN ZONE

    // From segmented gain to [0,100]
    int gain = static_cast<int>(std::round((100.0*segmentedGain)/mGainSegMax));

    return gain;
}


} // namespace zed

