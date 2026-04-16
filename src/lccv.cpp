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
        if (fmt == libcamera::formats::RGB888)
            cv::cvtColor(out, out, cv::COLOR_RGB2BGR);
        return;
    }

    std::cerr << "lccv: unsupported pixel format " << fmt.toString() << std::endl;
    out = cv::Mat();
}

} // namespace

PiCamera::PiCamera()
{
    app = new LibcameraApp(std::make_unique<Options>());
    options = static_cast<Options *>(app->GetOptions());
    still_flags = LibcameraApp::FLAG_STILL_NONE;
    options->photo_width = 4056;
    options->photo_height = 3040;
    options->video_width = 640;
    options->video_height = 480;
    options->framerate = 30;
    options->denoise = "auto";
    options->timeout = 1000;
    options->setMetering(Metering_Modes::METERING_MATRIX);
    options->setExposureMode(Exposure_Modes::EXPOSURE_NORMAL);
    options->setWhiteBalance(WhiteBalance_Modes::WB_AUTO);
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

PiCamera::~PiCamera()
{
    delete app;
}

void PiCamera::getImage(cv::Mat &frame, CompletedRequestPtr &payload)
{
    libcamera::Stream *stream = app->StillStream();
    auto *buffer = payload->buffers[stream];
    const auto mem = app->Mmap(buffer);
    const size_t bytesused = buffer->metadata().planes()[0].bytesused;
    bufferToMat(stream->configuration(), mem[0].data(), bytesused, frame);
}

bool PiCamera::startPhoto()
{
    app->OpenCamera();
    app->ConfigureStill(still_flags);
    camerastarted=true;
    return true;
}
bool PiCamera::stopPhoto()
{
    if(camerastarted){
        camerastarted=false;
        app->Teardown();
        app->CloseCamera();
    }
    return true;
}

bool PiCamera::capturePhoto(cv::Mat &frame)
{
    if(!camerastarted){
        app->OpenCamera();
        app->ConfigureStill(still_flags);
    }
    app->StartCamera();
    LibcameraApp::Msg msg = app->Wait();
    if (msg.type == LibcameraApp::MsgType::Quit)
        return false;
    else if (msg.type != LibcameraApp::MsgType::RequestComplete)
        return false;
    if (app->StillStream())
    {
        app->StopCamera();
        getImage(frame, std::get<CompletedRequestPtr>(msg.payload));
        app->Teardown();
        app->CloseCamera();
    } else {
        std::cerr<<"Incorrect stream received"<<std::endl;
        return false;
        app->StopCamera();
        if(!camerastarted){
            app->Teardown();
            app->CloseCamera();
        }
    }
    return true;
}

bool PiCamera::startVideo()
{
    if(camerastarted)stopPhoto();
    if(running.load(std::memory_order_relaxed)){
        std::cerr<<"Video thread already running";
        return false;
    }
    frameready.store(false, std::memory_order_release);
    app->OpenCamera();
    app->ConfigureViewfinder();
    app->StartCamera();

    int ret = pthread_create(&videothread, NULL, &videoThreadFunc, this);
    if (ret != 0) {
        std::cerr<<"Error starting video thread";
        return false;
    }
    // Wait until the thread has actually started before returning, so a
    // getVideoFrame() call right after startVideo() doesn't observe
    // running=false and bail out.
    timespec wait_step{0, 100000}; // 100us
    while (!running.load(std::memory_order_acquire))
        nanosleep(&wait_step, nullptr);
    return true;
}

void PiCamera::stopVideo()
{
    if(!running)return;

    running.store(false, std::memory_order_release);

    //join thread
    void *status;
    int ret = pthread_join(videothread, &status);
    if(ret<0)
        std::cerr<<"Error joining thread"<<std::endl;

    app->StopCamera();
    app->Teardown();
    app->CloseCamera();
    frameready.store(false, std::memory_order_release);
}

bool PiCamera::getVideoFrame(cv::Mat &frame, unsigned int timeout)
{
    if(!running.load(std::memory_order_acquire))return false;
    auto start_time = std::chrono::high_resolution_clock::now();
    bool timeout_reached = false;
    timespec req;
    req.tv_sec=0;
    req.tv_nsec=1000000;//1ms
    while((!frameready.load(std::memory_order_acquire))&&(!timeout_reached)){
        nanosleep(&req,NULL);
        timeout_reached = (std::chrono::high_resolution_clock::now() - start_time > std::chrono::milliseconds(timeout));
    }
    if(!frameready.load(std::memory_order_acquire))
        return false;

    {
        std::lock_guard<std::mutex> lock(mtx);
        bufferToMat(vstream->configuration(), framebuffer.data(), vbytesused, frame);
        frameready.store(false, std::memory_order_release);
    }
    return !frame.empty();
}

void *PiCamera::videoThreadFunc(void *p)
{
    PiCamera *t = (PiCamera *)p;
    t->vstream = t->app->ViewfinderStream(&t->vw,&t->vh,&t->vstr);
    t->running.store(true, std::memory_order_release);

    while(t->running.load(std::memory_order_acquire)){
        LibcameraApp::Msg msg = t->app->Wait();
        if (msg.type == LibcameraApp::MsgType::Quit){
            std::cerr<<"Quit message received"<<std::endl;
            t->running.store(false,std::memory_order_release);
            break;
        }
        if (msg.type != LibcameraApp::MsgType::RequestComplete){
            std::cerr<<"lccv: unrecognised message in video thread"<<std::endl;
            t->running.store(false,std::memory_order_release);
            break;
        }

        CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
        auto *buffer = payload->buffers[t->vstream];
        auto mem = t->app->Mmap(buffer);
        // bytesused is the actual data size in this frame; for fixed-size
        // formats (RGB/BGR) it equals plane.length, for MJPEG it varies.
        const size_t bytesused = buffer->metadata().planes()[0].bytesused;

        std::lock_guard<std::mutex> lock(t->mtx);
        t->framebuffer.assign(mem[0].data(), mem[0].data() + bytesused);
        t->vbytesused = bytesused;
        t->frameready.store(true, std::memory_order_release);
    }
    return NULL;
}

void PiCamera::ApplyZoomOptions()
{
    app->ApplyRoiSettings();
}
