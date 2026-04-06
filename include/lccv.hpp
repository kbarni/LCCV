#ifndef LCCV_HPP
#define LCCV_HPP

#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>
#include <vector>
#include <opencv2/opencv.hpp>

#include "libcamera_app.hpp"
#include "libcamera_app_options.hpp"

namespace lccv {

class PiCamera {
public:
    PiCamera();
    ~PiCamera();

    Options *options;

    //Photo mode
    bool startPhoto();
    bool capturePhoto(cv::Mat &frame);
    bool stopPhoto();

    //Video mode
    bool startVideo();
    bool getVideoFrame(cv::Mat &frame, unsigned int timeout);
    void stopVideo();

    //Applies new zoom options. Before invoking this func modify options->roi.
    void ApplyZoomOptions();

protected:
    std::unique_ptr<LibcameraApp> app;
    void getImage(cv::Mat &frame, CompletedRequestPtr &payload);
    void videoThread();

    unsigned int still_flags;
    unsigned int vw, vh, vstr;

    // video frame delivery
    std::vector<uint8_t> front_buffer_;
    std::vector<uint8_t> back_buffer_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    bool frame_ready_ = false;

    std::thread video_thread_;
    std::atomic<bool> running{false};

    std::mutex mtx;
    bool camerastarted;
};

}
#endif
