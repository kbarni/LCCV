#include "lccv.hpp"

using namespace lccv;

// ---------------------------------------------------------------------------
// Options::print()
// ---------------------------------------------------------------------------

void Options::print() const
{
    std::cerr << "Options:" << std::endl;
    std::cerr << "    verbose: " << verbose << std::endl;
    std::cerr << "    photo resolution: " << photo_width << " x " << photo_height << std::endl;
    std::cerr << "    video resolution: " << video_width << " x " << video_height << std::endl;
    std::cerr << "    viewfinder resolution: " << viewfinder_width << " x " << viewfinder_height << std::endl;
    std::cerr << "    framerate: " << framerate << std::endl;
    std::cerr << "    zoom: " << zoom << "  pan: " << pan_x << "," << pan_y << std::endl;
    std::cerr << "    transform: " << transformToString(transform) << std::endl;
    if (shutter > 0.0f)
        std::cerr << "    shutter: " << shutter << std::endl;
    if (gain > 0.0f)
        std::cerr << "    gain: " << gain << std::endl;
    std::cerr << "    metering: " << getMeteringMode() << std::endl;
    std::cerr << "    exposure: " << getExposureMode() << std::endl;
    std::cerr << "    ev: " << ev << std::endl;
    std::cerr << "    awb: " << getWhiteBalance() << std::endl;
    if (awb_gain_r > 0.0f && awb_gain_b > 0.0f)
        std::cerr << "    awb gains: red " << awb_gain_r
                  << " blue " << awb_gain_b << std::endl;
    std::cerr << "    brightness: " << brightness << std::endl;
    std::cerr << "    contrast: " << contrast << std::endl;
    std::cerr << "    saturation: " << saturation << std::endl;
    std::cerr << "    sharpness: " << sharpness << std::endl;
    std::cerr << "    denoise: " << denoise << std::endl;
}

// ---------------------------------------------------------------------------
// Camera — construction
// ---------------------------------------------------------------------------

Camera::Camera()
{
    app_ = std::make_unique<LibcameraApp>(std::make_unique<Options>());
    options = static_cast<Options *>(app_->GetOptions());
    still_flags_ = LibcameraApp::FLAG_STILL_RGB;
    options->setMetering(Metering::MATRIX);
    options->setExposureMode(Exposure::NORMAL);
    options->setWhiteBalance(WhiteBalance::AUTO);
}

Camera::~Camera()
{
    // Stop any active modes so the camera is cleanly released.
    if (dispatcher_running_.load())
        stopDispatcher();
    if (video_running_ || camera_started_) {
        if (app_) {
            app_->StopCamera();
            app_->Teardown();
            app_->CloseCamera();
        }
    }
}

// ---------------------------------------------------------------------------
// toMat — pixel format conversion
// ---------------------------------------------------------------------------

void Camera::toMat(cv::Mat &dst,
                   const uint8_t *src, unsigned int w, unsigned int h,
                   unsigned int stride)
{
    switch (options->format) {
    case PixelFormat::BGR: {
        // Camera configured for BGR888 — memcpy row by row
        dst.create(h, w, CV_8UC3);
        const uint8_t *ptr = src;
        for (unsigned int i = 0; i < h; i++, ptr += stride)
            memcpy(dst.ptr(i), ptr, w * 3);
        break;
    }
    case PixelFormat::RGB: {
        // Camera configured for RGB888 — memcpy row by row
        dst.create(h, w, CV_8UC3);
        const uint8_t *ptr = src;
        for (unsigned int i = 0; i < h; i++, ptr += stride)
            memcpy(dst.ptr(i), ptr, w * 3);
        break;
    }
    case PixelFormat::GRAYSCALE: {
        // Camera in BGR/RGB — copy into temp, then convert
        cv::Mat tmp(h, w, CV_8UC3);
        const uint8_t *ptr = src;
        for (unsigned int i = 0; i < h; i++, ptr += stride)
            memcpy(tmp.ptr(i), ptr, w * 3);
        // Stream is BGR888 when format would be BGR, RGB888 otherwise.
        // Since GRAYSCALE is not BGR, reconfigure() picks FLAG_STILL_RGB → RGB888.
        cv::cvtColor(tmp, dst, cv::COLOR_RGB2GRAY);
        break;
    }
    case PixelFormat::BAYER: {
        // Raw stream: single plane, 16-bit packed on most sensors
        // Deliver as CV_16UC1; user can demosaic as needed
        dst.create(h, w, CV_16UC1);
        const size_t row_bytes = std::min((size_t)stride, (size_t)w * 2);
        const uint8_t *ptr = src;
        for (unsigned int i = 0; i < h; i++, ptr += stride)
            memcpy(dst.ptr(i), ptr, row_bytes);
        break;
    }
    }
}

// ---------------------------------------------------------------------------
// getImage — extract still frame into cv::Mat
// ---------------------------------------------------------------------------

void Camera::getImage(cv::Mat &frame, CompletedRequestPtr &payload)
{
    if (options->format == PixelFormat::BAYER) {
        // Use raw stream
        libcamera::Stream *raw = app_->RawStream();
        if (!raw) {
            std::cerr << "Raw stream not available for BAYER format" << std::endl;
            return;
        }
        unsigned int w, h, stride;
        app_->StreamDimensions(raw, &w, &h, &stride);
        auto mem = app_->Mmap(payload->buffers[raw]);
        toMat(frame, mem[0].data(), w, h, stride);
    } else {
        libcamera::Stream *stream = app_->StillStream();
        unsigned int w, h, stride;
        app_->StreamDimensions(stream, &w, &h, &stride);
        auto mem = app_->Mmap(payload->buffers[stream]);
        toMat(frame, mem[0].data(), w, h, stride);
    }
}

// ---------------------------------------------------------------------------
// reconfigure — set up camera streams for the current active modes
// Caller must have called StopCamera()+Teardown() first (if running).
// ---------------------------------------------------------------------------

void Camera::reconfigure()
{
    // Choose pixel format for the colour streams
    unsigned int flags = (options->format == PixelFormat::BGR)
                         ? LibcameraApp::FLAG_STILL_BGR
                         : LibcameraApp::FLAG_STILL_RGB;

    if (camera_started_ && viewfinder_active_)
        app_->ConfigureStillWithViewfinder(flags);
    else if (camera_started_)
        app_->ConfigureStill(flags);
    else if (video_running_)
        app_->ConfigureViewfinder(options->video_width, options->video_height);
    else // viewfinder-only
        app_->ConfigureViewfinder(options->viewfinder_width, options->viewfinder_height);

    // Capture stream dimensions for the video/viewfinder buffer
    if (!camera_started_ || viewfinder_active_) {
        libcamera::Stream *vf = app_->ViewfinderStream(&vw_, &vh_, &vstr_);
        if (vf) {
            size_t bufsize = (size_t)vh_ * vstr_;
            front_buffer_.resize(bufsize);
            back_buffer_.resize(bufsize);
        }
    }
}

// ---------------------------------------------------------------------------
// Dispatcher
// ---------------------------------------------------------------------------

void Camera::startDispatcher()
{
    dispatcher_running_.store(true, std::memory_order_release);
    dispatcher_ = std::thread(&Camera::dispatcherThread, this);
}

void Camera::stopDispatcher()
{
    dispatcher_running_.store(false, std::memory_order_release);
    // Post a Quit message to unblock the dispatcher if it's waiting on the queue
    app_->PostQuit();
    // Wake up any callers blocked on frame_cv_ or still_cv_
    frame_cv_.notify_all();
    still_cv_.notify_all();
    if (dispatcher_.joinable())
        dispatcher_.join();
}

void Camera::dispatcherThread()
{
    libcamera::Stream *vf_stream    = app_->ViewfinderStream();
    libcamera::Stream *still_stream = app_->StillStream();

    while (dispatcher_running_.load(std::memory_order_acquire)) {
        LibcameraApp::Msg msg = app_->Wait();

        if (msg.type == LibcameraApp::MsgType::Quit) {
            dispatcher_running_.store(false, std::memory_order_release);
            break;
        }
        if (msg.type != LibcameraApp::MsgType::RequestComplete)
            continue;

        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);

        // --- Auto zoom/pan ---
        if (options->zoom != last_zoom_ ||
            options->pan_x != last_pan_x_ ||
            options->pan_y != last_pan_y_)
        {
            app_->ApplyZoom(options->zoom, options->pan_x, options->pan_y);
            last_zoom_  = options->zoom;
            last_pan_x_ = options->pan_x;
            last_pan_y_ = options->pan_y;
        }

        // --- Still frame (photo+viewfinder mode) ---
        if (still_stream && payload->buffers.count(still_stream)) {
            std::lock_guard<std::mutex> lock(still_mutex_);
            still_pending_ = payload;
            still_ready_   = true;
            still_cv_.notify_one();
        }

        // --- Viewfinder / video frame ---
        if (vf_stream && payload->buffers.count(vf_stream)) {
            auto mem = app_->Mmap(payload->buffers[vf_stream]);

            // Update video polling buffer
            if (video_running_) {
                memcpy(back_buffer_.data(), mem[0].data(), (size_t)vh_ * vstr_);
                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    std::swap(front_buffer_, back_buffer_);
                    frame_ready_ = true;
                }
                frame_cv_.notify_one();
            }

            // Call viewfinder callback
            if (viewfinder_active_ && viewfinder_cb_) {
                cv::Mat frame;
                toMat(frame, mem[0].data(), vw_, vh_, vstr_);
                try {
                    viewfinder_cb_(frame);
                } catch (const std::exception &e) {
                    std::cerr << "Viewfinder callback threw: " << e.what() << std::endl;
                }
            }
        }
    }

    frame_cv_.notify_all();
    still_cv_.notify_all();
}

// ---------------------------------------------------------------------------
// Photo mode
// ---------------------------------------------------------------------------

bool Camera::startPhoto()
{
    if (camera_started_) return false;
    if (video_running_) return false;

    if (viewfinder_active_) {
        // Viewfinder is running — upgrade to still+viewfinder config
        stopDispatcher();
        app_->StopCamera();
        app_->Teardown();
        camera_started_ = true;
        reconfigure();
        app_->StartCamera();
        startDispatcher();
    } else {
        app_->OpenCamera();
        camera_started_ = true;
        reconfigure();
        // Don't StartCamera yet; capturePhoto does it (or dispatcher will)
    }
    return true;
}

bool Camera::stopPhoto()
{
    if (!camera_started_) return false;
    camera_started_ = false;

    if (viewfinder_active_) {
        // Downgrade to viewfinder-only config
        stopDispatcher();
        app_->StopCamera();
        app_->Teardown();
        reconfigure();  // viewfinder-only now
        app_->StartCamera();
        startDispatcher();
    } else {
        // Photo-only: camera may or may not be started (capturePhoto starts/stops it)
        app_->Teardown();
        app_->CloseCamera();
    }
    return true;
}

bool Camera::capturePhoto(cv::Mat &frame)
{
    if (dispatcher_running_.load()) {
        // Dispatcher mode: wait for a still frame from the queue
        std::unique_lock<std::mutex> lock(still_mutex_);
        still_ready_ = false;
        bool got = still_cv_.wait_for(lock, std::chrono::milliseconds(5000),
            [this] { return still_ready_ || !dispatcher_running_.load(); });
        if (!got || !still_ready_) return false;
        auto payload = still_pending_;
        still_pending_.reset();
        still_ready_ = false;
        lock.unlock();
        getImage(frame, payload);
        return true;
    }

    // Non-dispatcher mode (photo-only, no viewfinder)
    bool opened_here = false;
    if (!camera_started_) {
        app_->OpenCamera();
        reconfigure();
        opened_here = true;
    }
    app_->StartCamera();
    LibcameraApp::Msg msg = app_->Wait();
    if (msg.type != LibcameraApp::MsgType::RequestComplete) {
        app_->StopCamera();
        if (opened_here) { app_->Teardown(); app_->CloseCamera(); }
        return false;
    }
    if (!app_->StillStream()) {
        std::cerr << "Still stream not available" << std::endl;
        app_->StopCamera();
        if (opened_here) { app_->Teardown(); app_->CloseCamera(); }
        return false;
    }
    app_->StopCamera();
    auto payload = std::get<CompletedRequestPtr>(msg.payload);
    getImage(frame, payload);
    if (opened_here) { app_->Teardown(); app_->CloseCamera(); }
    return true;
}

// ---------------------------------------------------------------------------
// Video mode
// ---------------------------------------------------------------------------

bool Camera::startVideo()
{
    if (video_running_) {
        std::cerr << "Video already running" << std::endl;
        return false;
    }
    if (camera_started_) return false;  // photo and video are mutually exclusive

    video_running_ = true;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_ready_ = false;
    }

    if (viewfinder_active_) {
        // Viewfinder is running on the same stream — just set the flag.
        // The dispatcher already running will start filling the video buffer.
        return true;
    }

    app_->OpenCamera();
    reconfigure();
    app_->StartCamera();
    startDispatcher();
    return true;
}

void Camera::stopVideo()
{
    if (!video_running_) return;
    video_running_ = false;
    frame_cv_.notify_all();

    if (!viewfinder_active_) {
        stopDispatcher();
        app_->StopCamera();
        app_->Teardown();
        app_->CloseCamera();
    }
    // If viewfinder is still active, the dispatcher keeps running
}

bool Camera::getVideoFrame(cv::Mat &frame, unsigned int timeout)
{
    if (!video_running_) return false;

    std::unique_lock<std::mutex> lock(frame_mutex_);
    bool got = frame_cv_.wait_for(lock, std::chrono::milliseconds(timeout),
        [this] { return frame_ready_ || !video_running_; });

    if (!got || !frame_ready_) return false;

    toMat(frame, front_buffer_.data(), vw_, vh_, vstr_);

    frame_ready_ = false;
    return true;
}

// ---------------------------------------------------------------------------
// Viewfinder mode
// ---------------------------------------------------------------------------

bool Camera::startViewfinder(std::function<void(cv::Mat &)> callback)
{
    if (viewfinder_active_) return false;

    viewfinder_cb_     = std::move(callback);
    viewfinder_active_ = true;

    if (video_running_) {
        // Video dispatcher already running — callback will be invoked from it.
        // Ensure video stream dimensions are set (they should be already).
        return true;
    }

    if (camera_started_) {
        // Photo mode active — upgrade to still+viewfinder
        stopDispatcher();   // no-op if not running
        app_->StopCamera();
        app_->Teardown();
        reconfigure();      // picks up camera_started_+viewfinder_active_
        app_->StartCamera();
        startDispatcher();
        return true;
    }

    // Nothing running — start viewfinder-only
    app_->OpenCamera();
    reconfigure();          // viewfinder-only (camera_started_=false)
    app_->StartCamera();
    startDispatcher();
    return true;
}

void Camera::stopViewfinder()
{
    if (!viewfinder_active_) return;

    viewfinder_active_ = false;
    viewfinder_cb_     = nullptr;

    if (video_running_) {
        // Video keeps the dispatcher alive — nothing else to do
        return;
    }

    if (camera_started_) {
        // Downgrade from still+viewfinder to still-only
        stopDispatcher();
        app_->StopCamera();
        app_->Teardown();
        reconfigure();      // still-only now (camera_started_=true, viewfinder=false)
        // Don't StartCamera — capturePhoto will do it on next call
        return;
    }

    // Viewfinder-only — shut everything down
    stopDispatcher();
    app_->StopCamera();
    app_->Teardown();
    app_->CloseCamera();
}

// ---------------------------------------------------------------------------
// Zoom
// ---------------------------------------------------------------------------

void Camera::ApplyZoomOptions()
{
    app_->ApplyZoom(options->zoom, options->pan_x, options->pan_y);
}
