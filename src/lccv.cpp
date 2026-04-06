#include "lccv.hpp"

using namespace cv;
using namespace lccv;

PiCamera::PiCamera()
{
    app = std::make_unique<LibcameraApp>(std::make_unique<Options>());
    options = static_cast<Options *>(app->GetOptions());
    still_flags = LibcameraApp::FLAG_STILL_NONE;
    options->setMetering(Metering_Modes::METERING_MATRIX);
    options->setExposureMode(Exposure_Modes::EXPOSURE_NORMAL);
    options->setWhiteBalance(WhiteBalance_Modes::WB_AUTO);
    still_flags |= LibcameraApp::FLAG_STILL_RGB;
    running.store(false, std::memory_order_release);
    camerastarted = false;
}

PiCamera::~PiCamera()
{
    // unique_ptr members cleaned up automatically
}

void PiCamera::getImage(cv::Mat &frame, CompletedRequestPtr &payload)
{
    unsigned int w, h, stride;
    libcamera::Stream *stream = app->StillStream();
    app->StreamDimensions(stream, &w, &h, &stride);
    const std::vector<libcamera::Span<uint8_t>> mem = app->Mmap(payload->buffers[stream]);
    frame.create(h, w, CV_8UC3);
    uint ls = w * 3;
    uint8_t *ptr = (uint8_t *)mem[0].data();
    for (unsigned int i = 0; i < h; i++, ptr += stride)
        memcpy(frame.ptr(i), ptr, ls);
}

bool PiCamera::startPhoto()
{
    app->OpenCamera();
    app->ConfigureStill(still_flags);
    camerastarted = true;
    return true;
}

bool PiCamera::stopPhoto()
{
    if (camerastarted) {
        camerastarted = false;
        app->Teardown();
        app->CloseCamera();
    }
    return true;
}

bool PiCamera::capturePhoto(cv::Mat &frame)
{
    bool opened_here = false;
    if (!camerastarted) {
        app->OpenCamera();
        app->ConfigureStill(still_flags);
        opened_here = true;
    }
    app->StartCamera();
    LibcameraApp::Msg msg = app->Wait();
    if (msg.type != LibcameraApp::MsgType::RequestComplete) {
        app->StopCamera();
        if (opened_here) {
            app->Teardown();
            app->CloseCamera();
        }
        return false;
    }
    if (!app->StillStream()) {
        std::cerr << "Incorrect stream received" << std::endl;
        app->StopCamera();
        if (opened_here) {
            app->Teardown();
            app->CloseCamera();
        }
        return false;
    }
    app->StopCamera();
    getImage(frame, std::get<CompletedRequestPtr>(msg.payload));
    if (opened_here) {
        app->Teardown();
        app->CloseCamera();
    }
    return true;
}

bool PiCamera::startVideo()
{
    if (camerastarted) stopPhoto();
    if (running.load(std::memory_order_relaxed)) {
        std::cerr << "Video thread already running" << std::endl;
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame_ready_ = false;
    }
    app->OpenCamera();
    app->ConfigureViewfinder();
    app->StartCamera();

    running.store(true, std::memory_order_release);
    video_thread_ = std::thread(&PiCamera::videoThread, this);
    return true;
}

void PiCamera::stopVideo()
{
    if (!running.load(std::memory_order_acquire)) return;

    running.store(false, std::memory_order_release);
    frame_cv_.notify_all(); // unblock any waiting getVideoFrame call

    if (video_thread_.joinable())
        video_thread_.join();

    app->StopCamera();
    app->Teardown();
    app->CloseCamera();

    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_ready_ = false;
}

bool PiCamera::getVideoFrame(cv::Mat &frame, unsigned int timeout)
{
    if (!running.load(std::memory_order_acquire)) return false;

    std::unique_lock<std::mutex> lock(frame_mutex_);
    bool got_frame = frame_cv_.wait_for(lock,
        std::chrono::milliseconds(timeout),
        [this] { return frame_ready_ || !running.load(std::memory_order_relaxed); });

    if (!got_frame || !frame_ready_)
        return false;

    frame.create(vh, vw, CV_8UC3);
    uint ls = vw * 3;
    const uint8_t *ptr = front_buffer_.data();
    for (unsigned int i = 0; i < vh; i++, ptr += vstr)
        memcpy(frame.ptr(i), ptr, ls);

    frame_ready_ = false;
    return true;
}

void PiCamera::videoThread()
{
    libcamera::Stream *stream = app->ViewfinderStream(&vw, &vh, &vstr);
    size_t buffersize = (size_t)vh * vstr;
    back_buffer_.resize(buffersize);
    front_buffer_.resize(buffersize);

    while (running.load(std::memory_order_acquire)) {
        LibcameraApp::Msg msg = app->Wait();
        if (msg.type == LibcameraApp::MsgType::Quit) {
            std::cerr << "Quit message received" << std::endl;
            running.store(false, std::memory_order_release);
            break;
        }
        if (msg.type != LibcameraApp::MsgType::RequestComplete) {
            std::cerr << "Unrecognised message in video thread" << std::endl;
            break;
        }

        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
        auto mem = app->Mmap(payload->buffers[stream]);
        memcpy(back_buffer_.data(), mem[0].data(), buffersize);

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            std::swap(front_buffer_, back_buffer_);
            frame_ready_ = true;
        }
        frame_cv_.notify_one();
    }
    frame_cv_.notify_all(); // wake up any blocked getVideoFrame on exit
}

void PiCamera::ApplyZoomOptions()
{
    app->ApplyRoiSettings();
}
