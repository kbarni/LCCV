#include "lccv.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
      lens_position(1.0f),
      af_timeout_ms(2000),
      metering_index(static_cast<int>(Metering::CENTRE)),
      exposure_index(static_cast<int>(Exposure::NORMAL)),
      awb_index(static_cast<int>(WhiteBalance::AUTO)),
      af_mode_index(static_cast<int>(AutoFocus::CONTINUOUS)),
      af_range_index(static_cast<int>(FocusRange::NORMAL)),
      af_speed_index(static_cast<int>(FocusSpeed::NORMAL))
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
    std::cerr << "    autofocus: " << af_mode_index << "  range: " << af_range_index
              << "  speed: " << af_speed_index << std::endl;
    if (getAutoFocus() == AutoFocus::MANUAL)
        std::cerr << "    lens position: " << lens_position << std::endl;
    std::cerr << "    af timeout: " << af_timeout_ms << " ms" << std::endl;
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
    vf_running.store(false, std::memory_order_release);
    vbytesused = 0;
    vstream = nullptr;
    vfstream = nullptr;
    vfw = vfh = vfstr = 0;
    camerastarted = false;
}

Camera::~Camera()
{
    // Destructor order runs members before base; make sure any threads we
    // own have exited before their dependencies (app, callback) die.
    if (vf_running.load(std::memory_order_acquire))
        stopViewfinder();
    if (running.load(std::memory_order_acquire))
        stopVideo();
}

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
    if (vf_running.load(std::memory_order_acquire)) stopViewfinder();
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
    // Three cases:
    //   A) viewfinder running — pause pipeline, capture, restore.
    //   B) standalone (no startPhoto) — full open/configure/start/stop cycle.
    //   C) startPhoto mode — camera already streaming; just Wait.
    const bool restore_vf    = vf_running.load(std::memory_order_acquire);
    const bool restore_video = restore_vf && running.load(std::memory_order_acquire);
    const bool standalone    = !camerastarted;

    if (restore_vf) {
        // Pause the viewfinder pipeline so we can reconfigure for StillCapture.
        vf_running.store(false, std::memory_order_release);
        running.store(false, std::memory_order_release);
        frameready_cv.notify_all();
        if (vfthread.joinable())
            vfthread.join();
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
        frameready.store(false, std::memory_order_release);

        try {
            app->OpenCamera();
            app->ConfigureStill(still_flags);
            app->StartCamera();
        } catch (const std::exception &e) {
            std::cerr << "lccv: capturePhoto still configure failed: " << e.what() << std::endl;
            return false;
        }
    } else if (standalone) {
        app->OpenCamera();
        app->ConfigureStill(still_flags);
        app->StartCamera();
    }

    bool ok = false;
    CompletedRequestPtr req = waitForFocusedFrame();
    if (req && app->StillStream())
    {
        getImage(frame, req);
        ok = true;
    }
    req.reset(); // hand the buffer back before tearing down / continuing

    if (restore_vf) {
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
        // Restore the viewfinder pipeline.
        try {
            app->OpenCamera();
            app->ConfigureVideoWithViewfinder();
            app->StartCamera();
        } catch (const std::exception &e) {
            std::cerr << "lccv: capturePhoto viewfinder restore failed: " << e.what() << std::endl;
            return ok;
        }
        vstream  = app->VideoStream(&vw, &vh, &vstr);
        vfstream = app->ViewfinderStream(&vfw, &vfh, &vfstr);
        if (restore_video) {
            frameready.store(false, std::memory_order_release);
            running.store(true, std::memory_order_release);
        }
        vf_running.store(true, std::memory_order_release);
        vfthread = std::thread(&Camera::viewfinderThreadFunc, this);
    } else if (standalone) {
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
    }

    return ok;
}

// Pull still frames until the lens has settled, then return that frame.
// Frames that are not used are dropped right away so their buffer is
// re-queued: the still stream may have only one buffer.
//   MANUAL:     wait until the reported lens position reaches the request.
//   AUTO:       trigger a scan, wait for it to start and then finish.
//   CONTINUOUS: wait for AfState == Focused.
// Gives up after options->af_timeout_ms and returns the current frame.
// Returns nullptr if the camera sent something other than a frame.
CompletedRequestPtr Camera::waitForFocusedFrame()
{
    namespace ctl = libcamera::controls;
    const bool has_af = app->HasControl(ctl::AfMode) && options->af_timeout_ms > 0;
    const AutoFocus mode = options->getAutoFocus();

    float target_pos = options->lens_position;
    if (has_af && mode == AutoFocus::MANUAL) {
        if (const auto *info = app->GetControlInfo(ctl::LensPosition))
            target_pos = std::clamp(target_pos, info->min().get<float>(), info->max().get<float>());
    }
    if (has_af && mode == AutoFocus::AUTO)
        app->SetControl(ctl::AfTrigger, ctl::AfTriggerStart);

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(options->af_timeout_ms);
    bool scan_started = false;

    for (;;) {
        LibcameraApp::Msg msg = app->Wait();
        if (msg.type != LibcameraApp::MsgType::RequestComplete) {
            if (msg.type != LibcameraApp::MsgType::Quit)
                std::cerr << "lccv: unexpected message while waiting for a still frame" << std::endl;
            return nullptr;
        }
        CompletedRequestPtr req = std::get<CompletedRequestPtr>(msg.payload);
        if (!has_af)
            return req;

        const auto state = req->metadata.get(ctl::AfState);
        bool settled = false;
        switch (mode) {
        case AutoFocus::MANUAL: {
            const auto pos = req->metadata.get(ctl::LensPosition);
            settled = !pos || std::fabs(*pos - target_pos) < 0.05f;
            break;
        }
        case AutoFocus::AUTO:
            if (!state)
                settled = true;
            else if (*state == ctl::AfStateScanning)
                scan_started = true;
            else if (scan_started &&
                     (*state == ctl::AfStateFocused || *state == ctl::AfStateFailed))
                settled = true;
            break;
        case AutoFocus::CONTINUOUS:
            settled = !state || *state == ctl::AfStateFocused;
            break;
        }

        if (settled) {
            if (options->verbose && state && *state == ctl::AfStateFailed)
                std::cerr << "lccv: autofocus failed, capturing anyway" << std::endl;
            return req;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            if (options->verbose)
                std::cerr << "lccv: autofocus timeout, capturing anyway" << std::endl;
            return req;
        }
        // req is released here, re-queueing its buffer for the next frame.
    }
}

bool Camera::triggerAutofocus()
{
    const bool active = camerastarted
                     || running.load(std::memory_order_acquire)
                     || vf_running.load(std::memory_order_acquire);
    if (!active || !app->HasControl(libcamera::controls::AfTrigger))
        return false;
    app->SetControl(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerStart);
    return true;
}

bool Camera::startVideo()
{
    // When the viewfinder pipeline is already running, startVideo() is just a
    // software gate: the video stream is already flowing; we start delivering
    // its frames to getVideoFrame() callers.
    if (vf_running.load(std::memory_order_acquire)) {
        if (!vstream) {
            std::cerr << "lccv: no video stream in current viewfinder configuration" << std::endl;
            return false;
        }
        if (running.load(std::memory_order_relaxed)) {
            std::cerr << "lccv: video already running" << std::endl;
            return false;
        }
        frameready.store(false, std::memory_order_release);
        running.store(true, std::memory_order_release);
        return true;
    }

    // Standalone video mode: open camera and start dedicated video thread.
    if (camerastarted) stopPhoto();
    if (running.load(std::memory_order_relaxed)) {
        std::cerr << "lccv: video thread already running" << std::endl;
        return false;
    }
    frameready.store(false, std::memory_order_release);
    try {
        app->OpenCamera();
        app->ConfigureVideo();
        app->StartCamera();
    } catch (const std::exception &e) {
        std::cerr << "lccv: startVideo failed: " << e.what() << std::endl;
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
        return false;
    }
    vstream = app->VideoStream(&vw, &vh, &vstr);
    running.store(true, std::memory_order_release);
    videothread = std::thread(&Camera::videoThreadFunc, this);
    return true;
}

void Camera::stopVideo()
{
    if (!running.load(std::memory_order_acquire)) return;

    // When the viewfinder pipeline owns the camera, just close the gate.
    if (vf_running.load(std::memory_order_acquire)) {
        running.store(false, std::memory_order_release);
        frameready_cv.notify_all();
        frameready.store(false, std::memory_order_release);
        return;
    }

    // Standalone video teardown.
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

void Camera::addViewfinder(ViewfinderCallback cb)
{
    vf_callback = std::move(cb);
}

bool Camera::startViewfinder()
{
    if (!vf_callback) {
        std::cerr << "lccv: startViewfinder called with no callback registered" << std::endl;
        return false;
    }
    if (camerastarted) stopPhoto();
    if (running.load(std::memory_order_acquire)) stopVideo();
    if (vf_running.load(std::memory_order_acquire)) {
        std::cerr << "lccv: viewfinder already running" << std::endl;
        return false;
    }
    try {
        app->OpenCamera();
        app->ConfigureVideoWithViewfinder();
        app->StartCamera();
    } catch (const std::exception &e) {
        std::cerr << "lccv: startViewfinder failed: " << e.what() << std::endl;
        app->StopCamera();
        app->Teardown();
        app->CloseCamera();
        return false;
    }
    vstream  = app->VideoStream(&vw, &vh, &vstr);
    vfstream = app->ViewfinderStream(&vfw, &vfh, &vfstr); // null on single-stream devices
    vf_running.store(true, std::memory_order_release);
    vfthread = std::thread(&Camera::viewfinderThreadFunc, this);
    return true;
}

void Camera::stopViewfinder()
{
    if (!vf_running.load(std::memory_order_acquire)) return;

    vf_running.store(false, std::memory_order_release);
    // Also close the video gate so any blocked getVideoFrame() unblocks.
    running.store(false, std::memory_order_release);
    frameready_cv.notify_all();

    if (vfthread.joinable())
        vfthread.join();

    app->StopCamera();
    app->Teardown();
    app->CloseCamera();
    frameready.store(false, std::memory_order_release);
}

void Camera::viewfinderThreadFunc()
{
    cv::Mat frame;
    while (vf_running.load(std::memory_order_acquire)) {
        LibcameraApp::Msg msg = app->Wait();
        if (msg.type == LibcameraApp::MsgType::Quit) {
            std::cerr << "lccv: quit received in viewfinder thread" << std::endl;
            vf_running.store(false, std::memory_order_release);
            break;
        }
        if (msg.type != LibcameraApp::MsgType::RequestComplete) {
            std::cerr << "lccv: unrecognised message in viewfinder thread" << std::endl;
            vf_running.store(false, std::memory_order_release);
            break;
        }

        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
        const auto &bufs = payload->buffers;

        // Video gate: copy the video buffer to the shared framebuffer so that
        // getVideoFrame() callers can consume it, but only when the gate is open.
        auto vit = vstream ? bufs.find(vstream) : bufs.end();
        if (vit != bufs.end() && running.load(std::memory_order_acquire)) {
            auto mem = app->Mmap(vit->second);
            const size_t bytesused = vit->second->metadata().planes()[0].bytesused;
            {
                std::lock_guard<std::mutex> lock(mtx);
                framebuffer.assign(mem[0].data(), mem[0].data() + bytesused);
                vbytesused = bytesused;
                frameready.store(true, std::memory_order_release);
            }
            frameready_cv.notify_one();
        }

        // Viewfinder callback: prefer the dedicated viewfinder stream when
        // present; fall back to the video stream on single-stream devices.
        if (vf_callback) {
            libcamera::Stream *cbstream = nullptr;
            if (vfstream) {
                auto fit = bufs.find(vfstream);
                if (fit != bufs.end()) cbstream = vfstream;
            }
            if (!cbstream && vstream) {
                auto fit = bufs.find(vstream);
                if (fit != bufs.end()) cbstream = vstream;
            }
            if (cbstream) {
                auto *buffer = bufs.at(cbstream);
                auto mem = app->Mmap(buffer);
                const size_t bytesused = buffer->metadata().planes()[0].bytesused;
                bufferToMat(cbstream->configuration(), mem[0].data(), bytesused, frame);
                applyOutputFormat(frame, options->format);
                if (!frame.empty())
                    vf_callback(frame);
            }
        }
    }
    frameready_cv.notify_all();
}
