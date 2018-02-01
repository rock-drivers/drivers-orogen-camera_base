/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VideoCapture.hpp"
#include <camera_interface/CamInterface.h>
#include <opencv2/videoio.hpp>

using namespace camera;
using namespace camera_base;

// links the opencv interface to the CamInterface
class CamVideoCapture :public camera::CamInterface
{
    public:
        virtual ~CamVideoCapture(){};
        virtual int listCameras(std::vector<CamInfo> &cam_infos)const
        {
            return 0;
        }
        virtual void openCamera(std::string camera_name)
        {
            frame_available = false;
            if(!capture.open(camera_name))
                throw std::runtime_error("cannot open camera");
            capture.set(cv::CAP_PROP_CONVERT_RGB,1);
        }
        virtual bool isOpen()const
        {
            return capture.isOpened();
        }
        virtual bool close()
        {
            capture = cv::VideoCapture();
            return true;
        }
        virtual bool grab(const GrabMode mode = SingleFrame, const int buffer_len=1)
        {
            // changing grab mode is not supported
            return true;
        }
        virtual bool retrieveFrame(base::samples::frame::Frame &frame,const int timeout=1000)
        {
            base::Time time = base::Time::now();
            do
            {
                if(isFrameAvailable())
                {
                    // create copy because we are not allowed to modify internal
                    // buffer
                    frame_available = false;
                    if(capture.retrieve(image))
                    {
                        frame_helper::FrameHelper::copyMatToFrame(image,frame);
                        frame.time = time;
                        frame.frame_status = base::samples::frame::STATUS_VALID;
                        return true;
                    }
                }
                usleep(100);
            }
            while((base::Time::now()-time).toMilliseconds() <= timeout);
            return false;
        }
        virtual bool isFrameAvailable()
        {
            if(!frame_available)
                frame_available = capture.grab();
            return frame_available;
        }

        virtual bool setFrameSettings(const base::samples::frame::frame_size_t size, 
                const base::samples::frame::frame_mode_t mode,
                const uint8_t color_depth,
                const bool resize_frames)
        {
            base::samples::frame::Frame frame(0,0,color_depth,mode);
            // this does not work for video streams and some cameras!
         //   capture.set(cv::CAP_PROP_FRAME_WIDTH,size.width);
         //   capture.set(cv::CAP_PROP_FRAME_HEIGHT,size.height);
         //   capture.set(cv::CAP_PROP_FORMAT,frame_helper::FrameHelper::getOpenCvType(frame));
            return true;
        }
        bool isAttribAvail(const int_attrib::CamAttrib attrib)
        {
            switch(attrib)
            {
            case int_attrib::GainValue:
                return true;
            default:
                return false;
            }
            return true;
        }
        bool isAttribAvail(const double_attrib::CamAttrib attrib)
        {
            switch(attrib)
            {
            case double_attrib::FrameRate:
                return true;
            default:
                return false;
            }
            return true;
        }
        bool isAttribAvail(const enum_attrib::CamAttrib attrib)
        {
            return false;
        }
        bool setAttrib(const int_attrib::CamAttrib attrib,const int value)
        {
            switch(attrib)
            {
            case int_attrib::GainValue:
                capture.set(cv::CAP_PROP_GAIN,value);
                break;
            default:
                return false;
            }
            return true;
        }
        bool setAttrib(const double_attrib::CamAttrib attrib,const double value)
        {
            switch(attrib)
            {
            case double_attrib::FrameRate:
                capture.set(cv::CAP_PROP_FPS,value);
                break;
            default:
                return false;
            }
            return true;
        }
        int getAttrib(const int_attrib::CamAttrib attrib)
        {
            switch(attrib)
            {
            case int_attrib::GainValue:
                return capture.get(cv::CAP_PROP_GAIN);
            default:
                return false;
            }
            return true;
        }
        double getAttrib(const double_attrib::CamAttrib attrib)
        {
            switch(attrib)
            {
            case double_attrib::FrameRate:
                return capture.get(cv::CAP_PROP_FPS);
            default:
                return false;
            }
            return true;
        }

    private:
        cv::VideoCapture capture;
        cv::Mat image;
        bool frame_available;
};

//CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
//CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
//CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
//CAP_PROP_CONTRAST Contrast of the image (only for cameras).
//CAP_PROP_SATURATION Saturation of the image (only for cameras).
//CAP_PROP_HUE Hue of the image (only for cameras).
//CAP_PROP_EXPOSURE Exposure (only for cameras).
//

VideoCapture::VideoCapture(std::string const& name)
    : VideoCaptureBase(name)
{
}

VideoCapture::VideoCapture(std::string const& name, RTT::ExecutionEngine* engine)
    : VideoCaptureBase(name, engine)
{
}

VideoCapture::~VideoCapture()
{
}


void VideoCapture::processImage()
{
    // resize output otherwise processing goes crazy
    // this is needed because the size is not always known at the beginning
    base::samples::frame::Frame *frame_ptr = output_frame.write_access();
    frame_ptr->init(camera_frame->size.width,camera_frame->size.height,camera_frame->data_depth,_output_format.get());
    output_frame.reset(frame_ptr);
    Task::processImage();
}

bool VideoCapture::configureHook()
{
    if (! VideoCaptureBase::configureHook())
        return false;

    CamVideoCapture* camera = new CamVideoCapture();
    camera->openCamera(_camera_id.value());
    cam_interface = camera;
    return true;
}
bool VideoCapture::startHook()
{
    if (! VideoCaptureBase::startHook())
        return false;
    return true;
}
void VideoCapture::updateHook()
{
    VideoCaptureBase::updateHook();
}
void VideoCapture::errorHook()
{
    VideoCaptureBase::errorHook();
}
void VideoCapture::stopHook()
{
    VideoCaptureBase::stopHook();
}
void VideoCapture::cleanupHook()
{
    VideoCaptureBase::cleanupHook();
}
