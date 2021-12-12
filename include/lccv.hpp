#ifndef LCCV_HPP
#define LCCV_HPP

#include <opencv2/opencv.hpp>

#include "libcamera_app.hpp"
#include "libcamera_app_options.hpp"

namespace lccv {

class PiCamera {
public:
    PiCamera();
    ~PiCamera();

    bool captureFrame(cv::Mat &frame);
    Options *options;

protected:
    void run();
private:
    LibcameraApp *app;
    void getImage(cv::Mat &frame, CompletedRequestPtr &payload);
    void getVideoFrame(cv::Mat &frame, CompletedRequestPtr &payload);
    unsigned int still_flags;
    bool GoOn;
};

}
#endif
