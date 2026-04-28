#ifndef LCCV_HPP
#define LCCV_HPP

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/transform.h>

#include <opencv2/opencv.hpp>

#include "libcamera_app.hpp"

namespace lccv {

enum class PixelFormat {
    BGR,
    RGB,
    GRAYSCALE,
    BAYER
};

enum class Metering {
    CENTRE = libcamera::controls::MeteringCentreWeighted,
    SPOT   = libcamera::controls::MeteringSpot,
    MATRIX = libcamera::controls::MeteringMatrix,
    CUSTOM = libcamera::controls::MeteringCustom
};

enum class Exposure {
    NORMAL = libcamera::controls::ExposureNormal,
    SHORT  = libcamera::controls::ExposureShort,
    CUSTOM = libcamera::controls::ExposureCustom
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

class Options {
public:
    Options();
    void print() const;

    void setMetering(Metering m)     { metering_index = static_cast<int>(m); }
    void setExposureMode(Exposure e) { exposure_index = static_cast<int>(e); }
    void setWhiteBalance(WhiteBalance wb) { awb_index = static_cast<int>(wb); }

    Metering     getMetering()     const { return static_cast<Metering>(metering_index); }
    Exposure     getExposureMode() const { return static_cast<Exposure>(exposure_index); }
    WhiteBalance getWhiteBalance() const { return static_cast<WhiteBalance>(awb_index); }

    int getMeteringIndex()    const { return metering_index; }
    int getExposureIndex()    const { return exposure_index; }
    int getWhiteBalanceIndex() const { return awb_index; }

    bool verbose;
    unsigned int photo_width, photo_height;
    unsigned int video_width, video_height;
    unsigned int viewfinder_width, viewfinder_height;
    PixelFormat format;
    float zoom;
    float pan_x, pan_y;
    libcamera::Transform transform;
    float shutter;
    float gain;
    float ev;
    float awb_gain_r;
    float awb_gain_b;
    float brightness;
    float contrast;
    float saturation;
    float sharpness;
    float framerate;
    std::string denoise;
    unsigned int camera;

private:
    int metering_index;
    int exposure_index;
    int awb_index;
};

using ViewfinderCallback = std::function<void(cv::Mat &)>;

class Camera {
public:
    Camera();
    ~Camera();

    Options *options;

    bool startPhoto();
    bool capturePhoto(cv::Mat &frame);
    bool stopPhoto();

    bool startVideo();
    bool getVideoFrame(cv::Mat &frame, unsigned int timeout);
    void stopVideo();

    // Viewfinder mode: register a callback that will be invoked on the
    // viewfinder worker thread for every preview frame. The cv::Mat passed
    // to the callback is only valid for the duration of the call. Keep the
    // callback short and non-blocking; do not call GUI toolkits from it
    // unless you know they are thread-safe on your platform.
    //
    // capturePhoto() can be called while the viewfinder is running; the
    // library pauses the viewfinder, captures at full resolution, then
    // restores the viewfinder automatically. startPhoto() is incompatible
    // with viewfinder mode and will stop it.
    void addViewfinder(ViewfinderCallback cb);
    bool startViewfinder();
    void stopViewfinder();

    void ApplyZoomOptions();

    // Enumerate supported photo/video resolutions for the currently selected
    // camera. Opens and closes the camera internally if it is not already
    // open; do not call these while photo or video mode is active.
    std::vector<cv::Size> supportedPhotoSizes();
    std::vector<cv::Size> supportedVideoSizes();

protected:
    std::unique_ptr<LibcameraApp> app;
    void getImage(cv::Mat &frame, CompletedRequestPtr &payload);
    void videoThreadFunc();
    void viewfinderThreadFunc();
    std::thread videothread;
    std::thread vfthread;
    unsigned int still_flags;
    unsigned int vw, vh, vstr;
    unsigned int vfw, vfh, vfstr;
    std::atomic<bool> running, frameready;
    std::atomic<bool> vf_running;
    std::vector<uint8_t> framebuffer;
    size_t vbytesused;
    libcamera::Stream *vstream;
    libcamera::Stream *vfstream;
    std::mutex mtx;
    std::condition_variable frameready_cv;
    ViewfinderCallback vf_callback;
    bool camerastarted;
};

} // namespace lccv

#endif
