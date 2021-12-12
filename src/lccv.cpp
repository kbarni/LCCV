#include "lccv.hpp"
#include <libcamera/libcamera/stream.h>

using namespace cv;
using namespace lccv;

PiCamera::PiCamera()
{
    app = new LibcameraApp(std::make_unique<Options>());
    options = static_cast<Options *>(app->GetOptions());
    still_flags = LibcameraApp::FLAG_STILL_NONE;
    options->width = 4056;
    options->height = 3040;
    options->denoise = "auto";
    options->timeout = 1000;
    options->setMetering(Metering_Modes::METERING_MATRIX);
    options->setExposureMode(Exposure_Modes::EXPOSURE_NORMAL);
    options->setWhiteBalance(WhiteBalance_Modes::WB_AUTO);
    options->contrast = 1.0;
    options->saturation = 1.0;
    still_flags |= LibcameraApp::FLAG_STILL_RGB;
    GoOn=false;
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

void PiCamera::getVideoFrame(cv::Mat &frame, CompletedRequestPtr &payload)
{
    unsigned int w, h, stride;
    libcamera::Stream *stream = app->VideoStream();
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

bool PiCamera::captureFrame(cv::Mat &frame)
{
    app->OpenCamera();
    app->ConfigureStill(still_flags);
    app->StartCamera();
    LibcameraApp::Msg msg = app->Wait();
    if (msg.type == LibcameraApp::MsgType::Quit)
        return false;
    else if (msg.type != LibcameraApp::MsgType::RequestComplete)
        return false;
    if (app->ViewfinderStream())
        std::cerr << "Viewfinder frame "<<std::endl;
    else if (app->StillStream())
    {
        app->StopCamera();
        getImage(frame, std::get<CompletedRequestPtr>(msg.payload));
        app->CloseCamera();
    }
    return true;
}

void PiCamera::run()
{
    if(GoOn==true)return;

    app->OpenCamera();
    app->ConfigureVideo();
    app->StartCamera();

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
}
