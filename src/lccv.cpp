#include "lccv.hpp"

#include <cstring>
#include <iostream>
#include <time.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace lccv;

Options::Options()
    : verbose(false),
      photo_width(0), photo_height(0),
      video_width(0), video_height(0),
      viewfinder_width(640), viewfinder_height(480),
      format(PixelFormat::BGR),
      zoom(1.0f), pan_x(0.5f), pan_y(0.5f),
      transform(libcamera::Transform::Identity),
      shutter(0.0f), gain(0.0f), ev(0.0f),
      awb_gain_r(0.0f), awb_gain_b(0.0f),
      brightness(0.0f), contrast(1.0f), saturation(1.0f), sharpness(1.0f),
      framerate(0.0f),
      denoise("auto"),
      camera(0),
      metering_index(static_cast<int>(Metering::CENTRE)),
      exposure_index(static_cast<int>(Exposure::NORMAL)),
      awb_index(static_cast<int>(WhiteBalance::AUTO))
{
}

void Options::print() const
{
    std::cerr << "Options:" << std::endl;
    std::cerr << "    verbose: " << verbose << std::endl;
    std::cerr << "    photo resolution: " << photo_width << " x " << photo_height << std::endl;
    std::cerr << "    video resolution: " << video_width << " x " << video_height << std::endl;
    std::cerr << "    viewfinder resolution: " << viewfinder_width << " x " << viewfinder_height << std::endl;
    std::cerr << "    transform: " << libcamera::transformToString(transform) << std::endl;
    std::cerr << "    zoom: " << zoom << "  pan: " << pan_x << "," << pan_y << std::endl;
    if (shutter)
        std::cerr << "    shutter: " << shutter << std::endl;
    if (gain)
        std::cerr << "    gain: " << gain << std::endl;
    std::cerr << "    metering: " << metering_index << std::endl;
    std::cerr << "    exposure: " << exposure_index << std::endl;
    std::cerr << "    ev: " << ev << std::endl;
    std::cerr << "    awb: " << awb_index << std::endl;
    if (awb_gain_r && awb_gain_b)
        std::cerr << "    awb gains: red " << awb_gain_r << " blue " << awb_gain_b << std::endl;
    std::cerr << "    brightness: " << brightness << std::endl;
    std::cerr << "    contrast: " << contrast << std::endl;
    std::cerr << "    saturation: " << saturation << std::endl;
    std::cerr << "    sharpness: " << sharpness << std::endl;
    std::cerr << "    framerate: " << framerate << std::endl;
    std::cerr << "    denoise: " << denoise << std::endl;
}

namespace {

// Format-aware copy from a libcamera buffer plane into a BGR cv::Mat. Handles:
//   * BGR888 / RGB888 — direct memcpy row-by-row, with optional R<->B swap
//   * MJPEG          — JPEG-decode via OpenCV
// All other formats produce an empty Mat and a stderr warning. Shared by the
// still-photo path (synchronous) and the video path (consumer side).
void bufferToMat(const libcamera::StreamConfiguration &cfg,
                 const uint8_t *data, size_t bytesused, cv::Mat &out)
{
    const libcamera::PixelFormat &fmt = cfg.pixelFormat;

    if (fmt == libcamera::formats::MJPEG)
    {
        cv::Mat encoded(1, static_cast<int>(bytesused), CV_8UC1,
                        const_cast<uint8_t *>(data));
        out = cv::imdecode(encoded, cv::IMREAD_COLOR);
        return;
    }

    if (fmt == libcamera::formats::BGR888 || fmt == libcamera::formats::RGB888)
    {
        out.create(cfg.size.height, cfg.size.width, CV_8UC3);
        const size_t row_bytes = static_cast<size_t>(cfg.size.width) * 3;
        const uint8_t *src = data;
        for (unsigned int y = 0; y < cfg.size.height; ++y, src += cfg.stride)
            std::memcpy(out.ptr(y), src, row_bytes);
        // libcamera uses DRM fourcc naming: formats::BGR888 stores bytes as
        // R,G,B in memory (= V4L2_PIX_FMT_RGB24) and formats::RGB888 stores
        // bytes as B,G,R (= V4L2_PIX_FMT_BGR24). OpenCV wants B,G,R, so the
        // swap is needed for BGR888 (despite the name).
        if (fmt == libcamera::formats::BGR888)
            cv::cvtColor(out, out, cv::COLOR_RGB2BGR);
        return;
    }

    std::cerr << "lccv: unsupported pixel format " << fmt.toString() << std::endl;
    out = cv::Mat();
}

// Convert a BGR cv::Mat in-place to the user-requested output format.
// BAYER is not supported in this path (requires the raw stream).
void applyOutputFormat(cv::Mat &frame, PixelFormat fmt)
{
    if (frame.empty())
        return;
    switch (fmt)
    {
        case PixelFormat::BGR:
            break;
        case PixelFormat::RGB:
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
            break;
        case PixelFormat::GRAYSCALE:
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            break;
        case PixelFormat::BAYER:
            std::cerr << "lccv: PixelFormat::BAYER not supported via processed stream" << std::endl;
            frame = cv::Mat();
            break;
    }
}

} // namespace

Camera::Camera()
{
    app = std::make_unique<LibcameraApp>(std::make_unique<Options>());
    options = app->GetOptions();
    still_flags = LibcameraApp::FLAG_STILL_NONE;
    options->photo_width = 4056;
    options->photo_height = 3040;
    options->video_width = 640;
    options->video_height = 480;
    options->framerate = 30;
    options->denoise = "auto";
    options->setMetering(Metering::MATRIX);
    options->setExposureMode(Exposure::NORMAL);
    options->setWhiteBalance(WhiteBalance::AUTO);
    options->contrast = 1.0f;
    options->saturation = 1.0f;
    // BGR matches OpenCV's native channel order, so when the pipeline
    // handler can satisfy this directly we avoid the cvtColor step.
    still_flags |= LibcameraApp::FLAG_STILL_BGR;
    running.store(false, std::memory_order_release);
    frameready.store(false, std::memory_order_release);
    vbytesused = 0;
    vstream = nullptr;
    camerastarted = false;
}

Camera::~Camera() = default;

void Camera::getImage(cv::Mat &frame, CompletedRequestPtr &payload)
{
    libcamera::Stream *stream = app->StillStream();
    auto *buffer = payload->buffers[stream];
    const auto mem = app->Mmap(buffer);
    const size_t bytesused = buffer->metadata().planes()[0].bytesused;
    bufferToMat(stream->configuration(), mem[0].data(), bytesused, frame);
    applyOutputFormat(frame, options->format);
}

bool Camera::startPhoto()
{
    try {
        app->OpenCamera();
        app->ConfigureStill(still_flags);
        app->StartCamera();
        camerastarted = true;
        return true;
    } catch (const std::exception &e) {
        std::cerr << "lccv: startPhoto failed: " << e.what() << std::endl;
        // Best-effort rollback of whichever step succeeded before the throw.
        // Each of Stop/Teardown/CloseCamera is a no-op when the matching
        // bring-up step did not run, so blanket cleanup is safe.
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
        camerastarted = false;
        return false;
    }
}
bool Camera::stopPhoto()
{
    if(camerastarted){
        camerastarted = false;
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
    }
    return true;
}

bool Camera::capturePhoto(cv::Mat &frame)
{
    // In "started" mode (startPhoto was called) the camera is already
    // streaming and must stay that way across calls; we only Wait for the
    // next frame. In standalone mode we do the full open/configure/start —
    // wait — stop/teardown/close cycle for this single shot.
    const bool standalone = !camerastarted;
    if(standalone){
        app->OpenCamera();
        app->ConfigureStill(still_flags);
        app->StartCamera();
    }
    LibcameraApp::Msg msg = app->Wait();
    bool ok = false;
    if (msg.type == LibcameraApp::MsgType::RequestComplete && app->StillStream())
    {
        getImage(frame, std::get<CompletedRequestPtr>(msg.payload));
        ok = true;
    }
    else if (msg.type != LibcameraApp::MsgType::Quit)
    {
        std::cerr<<"Incorrect stream received"<<std::endl;
    }
    if(standalone){
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
    }
    return ok;
}

bool Camera::startVideo()
{
    if(camerastarted)stopPhoto();
    if(running.load(std::memory_order_relaxed)){
        std::cerr<<"Video thread already running"<<std::endl;
        return false;
    }
    frameready.store(false, std::memory_order_release);
    try {
        app->OpenCamera();
        app->ConfigureViewfinder();
        app->StartCamera();
    } catch (const std::exception &e) {
        std::cerr << "lccv: startVideo failed: " << e.what() << std::endl;
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
        return false;
    }
    vstream = app->ViewfinderStream(&vw, &vh, &vstr);
    // Set running before spawning so the worker loop sees it on entry.
    running.store(true, std::memory_order_release);
    videothread = std::thread(&Camera::videoThreadFunc, this);
    return true;
}

void Camera::stopVideo()
{
    if(!running.load(std::memory_order_acquire))return;

    running.store(false, std::memory_order_release);
    frameready_cv.notify_all();

    if (videothread.joinable())
        videothread.join();

    app->StopCamera();
    app->Teardown();
    app->CloseCamera();
    frameready.store(false, std::memory_order_release);
}

bool Camera::getVideoFrame(cv::Mat &frame, unsigned int timeout)
{
    if(!running.load(std::memory_order_acquire))return false;

    std::unique_lock<std::mutex> lock(mtx);
    const bool got = frameready_cv.wait_for(
        lock, std::chrono::milliseconds(timeout),
        [this] { return frameready.load(std::memory_order_acquire)
                     || !running.load(std::memory_order_acquire); });
    if (!got || !frameready.load(std::memory_order_acquire))
        return false;

    bufferToMat(vstream->configuration(), framebuffer.data(), vbytesused, frame);
    applyOutputFormat(frame, options->format);
    frameready.store(false, std::memory_order_release);
    return !frame.empty();
}

void Camera::videoThreadFunc()
{
    while(running.load(std::memory_order_acquire)){
        LibcameraApp::Msg msg = app->Wait();
        if (msg.type == LibcameraApp::MsgType::Quit){
            std::cerr<<"Quit message received"<<std::endl;
            running.store(false, std::memory_order_release);
            break;
        }
        if (msg.type != LibcameraApp::MsgType::RequestComplete){
            std::cerr<<"lccv: unrecognised message in video thread"<<std::endl;
            running.store(false, std::memory_order_release);
            break;
        }

        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
        auto *buffer = payload->buffers[vstream];
        auto mem = app->Mmap(buffer);
        // bytesused is the actual data size in this frame; for fixed-size
        // formats (RGB/BGR) it equals plane.length, for MJPEG it varies.
        const size_t bytesused = buffer->metadata().planes()[0].bytesused;

        {
            std::lock_guard<std::mutex> lock(mtx);
            framebuffer.assign(mem[0].data(), mem[0].data() + bytesused);
            vbytesused = bytesused;
            frameready.store(true, std::memory_order_release);
        }
        frameready_cv.notify_one();
    }
    // Final notify so any consumer waiting past the loop exit wakes up.
    frameready_cv.notify_all();
}

void Camera::ApplyZoomOptions()
{
    app->ApplyZoom(options->zoom, options->pan_x, options->pan_y);
}

namespace {
std::vector<cv::Size> enumerateSizes(LibcameraApp &app, libcamera::StreamRole role)
{
    std::vector<cv::Size> out;
    try {
        app.OpenCamera();
    } catch (const std::exception &e) {
        std::cerr << "lccv: enumerateSizes open failed: " << e.what() << std::endl;
        return out;
    }
    for (const libcamera::Size &s : app.SupportedSizes(role))
        out.emplace_back(static_cast<int>(s.width), static_cast<int>(s.height));
    app.CloseCamera();
    return out;
}
} // namespace

std::vector<cv::Size> Camera::supportedPhotoSizes()
{
    return enumerateSizes(*app, libcamera::StreamRole::StillCapture);
}

std::vector<cv::Size> Camera::supportedVideoSizes()
{
    return enumerateSizes(*app, libcamera::StreamRole::VideoRecording);
}
