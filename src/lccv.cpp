#include "lccv.hpp"
#include <libcamera/libcamera/stream.h>
#include <time.h>

using namespace cv;
using namespace lccv;

PiCamera::PiCamera()
{
    app = new LibcameraApp(std::make_unique<Options>());
    options = static_cast<Options *>(app->GetOptions());
    still_flags = LibcameraApp::FLAG_STILL_NONE;
    options->photo_width = 4056;
    options->photo_height = 3040;
    options->video_width = 640;
    options->video_height = 480;
    options->denoise = "auto";
    options->timeout = 1000;
    options->setMetering(Metering_Modes::METERING_MATRIX);
    options->setExposureMode(Exposure_Modes::EXPOSURE_NORMAL);
    options->setWhiteBalance(WhiteBalance_Modes::WB_AUTO);
    options->contrast = 1.0;
    options->saturation = 1.0;
    still_flags |= LibcameraApp::FLAG_STILL_RGB;
    running.store(false, std::memory_order_release);;
    frameready.store(false, std::memory_order_release);;
    framebuffer=nullptr;
}

PiCamera::~PiCamera()
{
    delete app;
}

void PiCamera::getImage(cv::Mat &frame, CompletedRequestPtr &payload)
{
    unsigned int w, h, stride;
    libcamera::Stream *stream = app->StillStream();
	app->StreamDimensions(stream, &w, &h, &stride);
    const std::vector<libcamera::Span<uint8_t>> mem = app->Mmap(payload->buffers[stream]);
    frame.create(h,w,CV_8UC3);
    uint ls = w*3;
    uint8_t *ptr = (uint8_t *)mem[0].data();
    for (unsigned int i = 0; i < h; i++, ptr += stride)
    {
        memcpy(frame.ptr(i),ptr,ls);
    }
}


bool PiCamera::capturePhoto(cv::Mat &frame)
{
    app->OpenCamera();
    app->ConfigureStill(still_flags);
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
        app->Teardown();
        app->CloseCamera();
    }
    return true;
}

bool PiCamera::startVideo()
{
    if(running.load(std::memory_order_release)){
        std::cerr<<"Video thread already running";
        return false;
    }
    frameready.store(false, std::memory_order_release);
    app->OpenCamera();
    app->ConfigureVideo();
    app->StartCamera();
    libcamera::Stream *stream = app->VideoStream(&vw,&vh,&vstr);
    bool ret = pthread_create(&videothread, NULL, &videoThreadFunc, stream);
    if (ret != 0) {
        std::cerr<<"Error starting video thread";
        return false;
    }
    return true;
}

void PiCamera::stopVideo()
{
    if(!running)return;

    running.store(false, std::memory_order_release);;

    //join thread
    void *status;
    bool ret = pthread_join(videothread, &status);
    if(!ret)
        std::cerr<<"Error joining thread"<<std::endl;

    app->StopCamera();
    app->Teardown();
    app->CloseCamera();
    frameready.store(false, std::memory_order_release);;
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
    if(frameready.load(std::memory_order_acquire)){
        frame.create(vh,vw,CV_8UC3);
        uint ls = vw*3;
        mtx.lock();
            uint8_t *ptr = framebuffer;
            for (unsigned int i = 0; i < vh; i++, ptr += vstr)
                memcpy(frame.ptr(i),ptr,ls);
        mtx.unlock();
        frameready.store(false, std::memory_order_release);;
        return true;
    }
    else
        return false;
}

void *PiCamera::videoThreadFunc(void *p)
{
    running.store(true, std::memory_order_release);

    //allocate framebuffer
    //libcamera::Stream *stream = (libcamera::Stream *)p;
    (void)p;
    int buffersize=vh*vstr;
    if(framebuffer)delete[] framebuffer;
    framebuffer=new uint8_t[buffersize];
    const std::vector<libcamera::Span<uint8_t>> mem;

    //main loop
    while(running.load(std::memory_order_acquire)){
        //grab buffer ***
        mtx.lock();
            memcpy(framebuffer,mem[0].data(),buffersize);
        mtx.unlock();
        frameready.store(true, std::memory_order_release);
    }
    if(framebuffer){
        delete[] framebuffer;
        framebuffer=nullptr;
    }
    return NULL;
}
/*void PiCamera::videoThread()
{
    cv::Mat frame;

    while(GoOn){
        LibcameraApp::Msg msg = app->Wait();
		if (msg.type == LibcameraApp::MsgType::Quit)
			GoOn=false;
		else if (msg.type != LibcameraApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
        getVideoFrame(frame,std::get<CompletedRequestPtr>(msg.payload));
    }
    app->StopCamera();
    app->CloseCamera();
}*/
