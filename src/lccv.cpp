#include "lccv.hpp"

using namespace cv;
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
    if (shutter)
        std::cerr << "    shutter: " << shutter << std::endl;
    if (gain)
        std::cerr << "    gain: " << gain << std::endl;
    std::cerr << "    metering: " << getMeteringMode() << std::endl;
    std::cerr << "    exposure: " << getExposureMode() << std::endl;
    std::cerr << "    ev: " << ev << std::endl;
    std::cerr << "    awb: " << getWhiteBalance() << std::endl;
    if (awb_gain_r && awb_gain_b)
        std::cerr << "    awb gains: red " << awb_gain_r
                  << " blue " << awb_gain_b << std::endl;
    std::cerr << "    brightness: " << brightness << std::endl;
    std::cerr << "    contrast: " << contrast << std::endl;
    std::cerr << "    saturation: " << saturation << std::endl;
    std::cerr << "    sharpness: " << sharpness << std::endl;
    std::cerr << "    denoise: " << denoise << std::endl;
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

Camera::Camera()
{
    app_ = std::make_unique<LibcameraApp>(std::make_unique<Options>());
    options = static_cast<Options *>(app_->GetOptions());
    still_flags_ = LibcameraApp::FLAG_STILL_RGB;
    options->setMetering(Metering::MATRIX);
    options->setExposureMode(Exposure::NORMAL);
    options->setWhiteBalance(WhiteBalance::AUTO);
    running_.store(false, std::memory_order_release);
    camera_started_ = false;
}

Camera::~Camera()
{
    // unique_ptr members cleaned up automatically
}

void Camera::getImage(cv::Mat &frame, CompletedRequestPtr &payload)
{
    unsigned int w, h, stride;
    libcamera::Stream *stream = app_->StillStream();
    app_->StreamDimensions(stream, &w, &h, &stride);
    const std::vector<libcamera::Span<uint8_t>> mem = app_->Mmap(payload->buffers[stream]);
    frame.create(h, w, CV_8UC3);
    uint ls = w * 3;
    uint8_t *ptr = (uint8_t *)mem[0].data();
    for (unsigned int i = 0; i < h; i++, ptr += stride)
        memcpy(frame.ptr(i), ptr, ls);
}

bool Camera::startPhoto()
{
    app_->OpenCamera();
    app_->ConfigureStill(still_flags_);
    camera_started_ = true;
    return true;
}

bool Camera::stopPhoto()
{
    if (camera_started_) {
        camera_started_ = false;
        app_->Teardown();
        app_->CloseCamera();
    }
    return true;
}

bool Camera::capturePhoto(cv::Mat &frame)
{
    bool opened_here = false;
    if (!camera_started_) {
        app_->OpenCamera();
        app_->ConfigureStill(still_flags_);
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
        std::cerr << "Incorrect stream received" << std::endl;
        app_->StopCamera();
        if (opened_here) { app_->Teardown(); app_->CloseCamera(); }
        return false;
    }
    app_->StopCamera();
    getImage(frame, std::get<CompletedRequestPtr>(msg.payload));
    if (opened_here) { app_->Teardown(); app_->CloseCamera(); }
    return true;
}

bool Camera::startVideo()
{
    if (camera_started_) stopPhoto();
    if (running_.load(std::memory_order_relaxed)) {
        std::cerr << "Video already running" << std::endl;
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_ready_ = false;
    }
    app_->OpenCamera();
    app_->ConfigureViewfinder();
    app_->StartCamera();

    running_.store(true, std::memory_order_release);
    video_thread_ = std::thread(&Camera::videoThread, this);
    return true;
}

void Camera::stopVideo()
{
    if (!running_.load(std::memory_order_acquire)) return;

    running_.store(false, std::memory_order_release);
    frame_cv_.notify_all();

    if (video_thread_.joinable())
        video_thread_.join();

    app_->StopCamera();
    app_->Teardown();
    app_->CloseCamera();

    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_ready_ = false;
}

bool Camera::getVideoFrame(cv::Mat &frame, unsigned int timeout)
{
    if (!running_.load(std::memory_order_acquire)) return false;

    std::unique_lock<std::mutex> lock(frame_mutex_);
    bool got_frame = frame_cv_.wait_for(lock,
        std::chrono::milliseconds(timeout),
        [this] { return frame_ready_ || !running_.load(std::memory_order_relaxed); });

    if (!got_frame || !frame_ready_)
        return false;

    frame.create(vh_, vw_, CV_8UC3);
    uint ls = vw_ * 3;
    const uint8_t *ptr = front_buffer_.data();
    for (unsigned int i = 0; i < vh_; i++, ptr += vstr_)
        memcpy(frame.ptr(i), ptr, ls);

    frame_ready_ = false;
    return true;
}

void Camera::videoThread()
{
    libcamera::Stream *stream = app_->ViewfinderStream(&vw_, &vh_, &vstr_);
    size_t buffersize = (size_t)vh_ * vstr_;
    back_buffer_.resize(buffersize);
    front_buffer_.resize(buffersize);

    while (running_.load(std::memory_order_acquire)) {
        LibcameraApp::Msg msg = app_->Wait();
        if (msg.type == LibcameraApp::MsgType::Quit) {
            std::cerr << "Quit message received" << std::endl;
            running_.store(false, std::memory_order_release);
            break;
        }
        if (msg.type != LibcameraApp::MsgType::RequestComplete) {
            std::cerr << "Unrecognised message in video thread" << std::endl;
            break;
        }

        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
        auto mem = app_->Mmap(payload->buffers[stream]);
        memcpy(back_buffer_.data(), mem[0].data(), buffersize);

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            std::swap(front_buffer_, back_buffer_);
            frame_ready_ = true;
        }
        frame_cv_.notify_one();
    }
    frame_cv_.notify_all();
}

void Camera::ApplyZoomOptions()
{
    app_->ApplyRoiSettings();
}
