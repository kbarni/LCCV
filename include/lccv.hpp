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
    AUTO         = libcamera::controls::AwbAuto,
    INCANDESCENT = libcamera::controls::AwbIncandescent,
    TUNGSTEN     = libcamera::controls::AwbTungsten,
    FLUORESCENT  = libcamera::controls::AwbFluorescent,
    INDOOR       = libcamera::controls::AwbIndoor,
    DAYLIGHT     = libcamera::controls::AwbDaylight,
    CLOUDY       = libcamera::controls::AwbCloudy,
    CUSTOM       = libcamera::controls::AwbCustom
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

    // Zoom and pan — changes are picked up automatically on the next frame
    // zoom: >= 1.0 (1.0 = full sensor)
    // pan_x/pan_y: 0.0–1.0, centre of crop window (default 0.5 = centred)
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

    // Viewfinder mode — combinable with photo or video
    bool startViewfinder(std::function<void(cv::Mat &)> callback);
    void stopViewfinder();

    // Explicit zoom apply (the dispatcher also applies it automatically)
    void ApplyZoomOptions();

private:
    std::unique_ptr<LibcameraApp> app_;

    // --- Mode state ---
    bool camera_started_   = false; // photo config loaded
    bool video_running_    = false; // video mode active
    bool viewfinder_active_= false; // viewfinder callback active
    unsigned int still_flags_;

    // --- Video stream dimensions ---
    unsigned int vw_ = 0, vh_ = 0, vstr_ = 0;

    // --- Video frame delivery (for getVideoFrame polling) ---
    std::vector<uint8_t> front_buffer_;
    std::vector<uint8_t> back_buffer_;
    std::mutex           frame_mutex_;
    std::condition_variable frame_cv_;
    bool frame_ready_ = false;

    // --- Still frame delivery (for capturePhoto when dispatcher is running) ---
    CompletedRequestPtr  still_pending_;
    bool                 still_ready_ = false;
    std::mutex           still_mutex_;
    std::condition_variable still_cv_;

    // --- Viewfinder callback ---
    std::function<void(cv::Mat &)> viewfinder_cb_;

    // --- Dispatcher thread ---
    std::thread dispatcher_;
    std::atomic<bool> dispatcher_running_{false};

    // --- Zoom change detection ---
    float last_zoom_  = 1.0f;
    float last_pan_x_ = 0.5f;
    float last_pan_y_ = 0.5f;

    // --- Internal helpers ---
    // Convert a raw buffer from a libcamera stream to cv::Mat.
    // is_video: true → use video dimensions (vw_/vh_/vstr_),
    //           false → stream provides its own dimensions via w/h/stride args
    void toMat(cv::Mat &dst,
               const uint8_t *src, unsigned int w, unsigned int h,
               unsigned int stride);

    void getImage(cv::Mat &frame, CompletedRequestPtr &payload);

    // Dispatcher thread body
    void dispatcherThread();

    // Start/stop the dispatcher (camera must already be started/stopped by caller)
    void startDispatcher();
    void stopDispatcher();

    // Reconfigure camera for the current combination of active modes.
    // Caller must have called StopCamera()+Teardown() first.
    void reconfigure();
};

} // namespace lccv

#endif // LCCV_HPP
