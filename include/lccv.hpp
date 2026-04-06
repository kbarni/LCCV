#ifndef LCCV_HPP
#define LCCV_HPP

#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>
#include <vector>
#include <functional>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <libcamera/transform.h>

#include "libcamera_app.hpp"

namespace lccv {

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

enum class Exposure {
    NORMAL  = libcamera::controls::ExposureNormal,
    SHORT   = libcamera::controls::ExposureShort,
    CUSTOM  = libcamera::controls::ExposureCustom
};

enum class Metering {
    CENTRE  = libcamera::controls::MeteringCentreWeighted,
    SPOT    = libcamera::controls::MeteringSpot,
    MATRIX  = libcamera::controls::MeteringMatrix,
    CUSTOM  = libcamera::controls::MeteringCustom
};

enum class WhiteBalance {
    AUTO        = libcamera::controls::AwbAuto,
    INCANDESCENT= libcamera::controls::AwbIncandescent,
    TUNGSTEN    = libcamera::controls::AwbTungsten,
    FLUORESCENT = libcamera::controls::AwbFluorescent,
    INDOOR      = libcamera::controls::AwbIndoor,
    DAYLIGHT    = libcamera::controls::AwbDaylight,
    CLOUDY      = libcamera::controls::AwbCloudy,
    CUSTOM      = libcamera::controls::AwbCustom
};

enum class PixelFormat {
    BGR,        // 8-bit BGR — OpenCV native, default
    RGB,        // 8-bit RGB
    GRAYSCALE,  // 8-bit single channel (converted from colour stream)
    BAYER       // Raw Bayer (from still raw stream, single channel)
};

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------

class Options {
public:
    Options()
        : photo_width(4056), photo_height(3040),
          video_width(1280), video_height(720),
          viewfinder_width(640), viewfinder_height(480),
          format(PixelFormat::BGR),
          framerate(30),
          verbose(false),
          timeout(1000),
          shutter(0), gain(0), ev(0),
          brightness(0), contrast(1), saturation(1), sharpness(1),
          awb_gain_r(0), awb_gain_b(0),
          denoise("auto"),
          camera(0),
          transform(libcamera::Transform::Identity),
          zoom(1.0f), pan_x(0.5f), pan_y(0.5f),
          metering_(Metering::MATRIX),
          exposure_(Exposure::NORMAL),
          wb_(WhiteBalance::AUTO)
    {}

    void print() const;

    void setMetering(Metering m)         { metering_ = m; }
    void setExposureMode(Exposure e)     { exposure_ = e; }
    void setWhiteBalance(WhiteBalance w) { wb_ = w; }

    int getMeteringMode() const { return static_cast<int>(metering_); }
    int getExposureMode() const { return static_cast<int>(exposure_); }
    int getWhiteBalance() const { return static_cast<int>(wb_); }

    // Resolution
    unsigned int photo_width, photo_height;
    unsigned int video_width, video_height;
    unsigned int viewfinder_width, viewfinder_height;

    // Pixel format
    PixelFormat format;

    // Capture
    float framerate;
    bool verbose;
    uint64_t timeout;   // ms

    // Exposure / image
    float shutter;      // µs, 0 = auto
    float gain;         // 0 = auto
    float ev;
    float brightness;   // -1..1
    float contrast;
    float saturation;
    float sharpness;
    float awb_gain_r, awb_gain_b;
    std::string denoise;

    // Camera selection
    unsigned int camera;

    // Image transform
    libcamera::Transform transform;

    // Zoom and pan — changes take effect on the next captured frame
    // zoom: 1.0 = full sensor, 2.0 = 2× zoom (>= 1.0)
    // pan_x/pan_y: centre of crop as fraction of sensor (0.0–1.0, default 0.5)
    float zoom;
    float pan_x;
    float pan_y;

private:
    Metering     metering_;
    Exposure     exposure_;
    WhiteBalance wb_;
};

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

class Camera {
public:
    Camera();
    ~Camera();

    Options *options;

    // Photo mode
    bool startPhoto();
    bool capturePhoto(cv::Mat &frame);
    bool stopPhoto();

    // Video mode
    bool startVideo();
    bool getVideoFrame(cv::Mat &frame, unsigned int timeout);
    void stopVideo();

    // ROI / zoom
    void ApplyZoomOptions();

private:
    std::unique_ptr<LibcameraApp> app_;
    void getImage(cv::Mat &frame, CompletedRequestPtr &payload);
    void videoThread();

    unsigned int still_flags_;
    unsigned int vw_, vh_, vstr_;

    std::vector<uint8_t> front_buffer_;
    std::vector<uint8_t> back_buffer_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    bool frame_ready_ = false;

    std::thread video_thread_;
    std::atomic<bool> running_{false};

    bool camera_started_ = false;
};

} // namespace lccv

#endif // LCCV_HPP
