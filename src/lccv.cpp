#include "lccv.hpp"
#include <libcamera/libcamera/stream.h>

using namespace cv;
using namespace lccv;

PiCamera::PiCamera() : PiCamera(0) {}

PiCamera::PiCamera(uint32_t id)
	: app(std::make_unique<LibcameraApp>(std::make_unique<Options>())), running_(false), frame_ready_(false),
	  camera_started_(false), viewfinder_callback_(nullptr)
{
	options = static_cast<Options *>(app->GetOptions());
	still_flags = LibcameraApp::FLAG_STILL_NONE;
	options->camera = id;
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
	still_flags |= LibcameraApp::FLAG_STILL_RGB;
}

PiCamera::~PiCamera()
{
	stop();
}

void PiCamera::getImage(cv::Mat &frame, CompletedRequestPtr &payload)
{
	unsigned int w, h, stride;
	libcamera::Stream *stream = app->StillStream();
	if (!stream)
		stream = app->ViewfinderStream();
	app->StreamDimensions(stream, &w, &h, &stride);
	const std::vector<libcamera::Span<uint8_t>> mem = app->Mmap(payload->buffers[stream]);
	frame.create(h, w, CV_8UC3);
	uint ls = w * 3;
	uint8_t *ptr = (uint8_t *)mem[0].data();
	for (unsigned int i = 0; i < h; i++, ptr += stride)
	{
		memcpy(frame.ptr(i), ptr, ls);
	}
}

bool PiCamera::startPhoto(std::function<void(cv::Mat &)> callback)
{
	viewfinder_callback_ = callback;
	app->OpenCamera();
	if (viewfinder_callback_)
	{
		app->ConfigureViewfinder();
	}
	else
	{
		app->ConfigureStill(still_flags);
	}
	app->StartCamera();
	camera_started_ = true;
	if (viewfinder_callback_)
	{
		running_ = true;
		camera_thread_ = std::thread(&PiCamera::run, this);
	}
	return true;
}

bool PiCamera::stopPhoto()
{
	stop();
	return true;
}

bool PiCamera::capturePhoto(cv::Mat &frame)
{
	if (!camera_started_)
	{
		app->OpenCamera();
		app->ConfigureStill(still_flags);
		app->StartCamera();
	}

	if (viewfinder_callback_)
	{
		app->QueueRequest(LibcameraApp::RequestType::Still);
	}

	LibcameraApp::Msg msg = app->Wait();
	if (msg.type == LibcameraApp::MsgType::Quit)
		return false;
	else if (msg.type != LibcameraApp::MsgType::RequestComplete)
		return false;

	if (app->StillStream())
	{
		getImage(frame, std::get<CompletedRequestPtr>(msg.payload));
	}
	else
	{
		std::cerr << "Incorrect stream received" << std::endl;
		return false;
	}

	if (!viewfinder_callback_)
	{
		app->StopCamera();
		app->Teardown();
		app->CloseCamera();
		camera_started_ = false;
	}
	return true;
}

bool PiCamera::startVideo(std::function<void(cv::Mat &)> callback)
{
	if (camera_started_)
		stop();
	if (running_)
	{
		std::cerr << "Video thread already running";
		return false;
	}
	viewfinder_callback_ = callback;
	app->OpenCamera();
	app->ConfigureViewfinder();
	app->StartCamera();
	camera_started_ = true;
	running_ = true;
	camera_thread_ = std::thread(&PiCamera::run, this);
	return true;
}

void PiCamera::stopVideo()
{
	stop();
}

void PiCamera::stop()
{
	if (running_)
	{
		running_ = false;
		if (camera_thread_.joinable())
			camera_thread_.join();
	}
	if (camera_started_)
	{
		app->StopCamera();
		app->Teardown();
		app->CloseCamera();
		camera_started_ = false;
	}
}

bool PiCamera::getVideoFrame(cv::Mat &frame, unsigned int timeout)
{
	if (!running_)
		return false;

	std::unique_lock<std::mutex> lock(camera_mutex_);
	if (frame_cv_.wait_for(lock, std::chrono::milliseconds(timeout), [this] { return frame_ready_; }))
	{
		frame_.copyTo(frame);
		frame_ready_ = false;
		return true;
	}
	return false;
}

void PiCamera::run()
{
	libcamera::Stream *stream = app->ViewfinderStream();
	unsigned int w, h, stride;
	app->StreamDimensions(stream, &w, &h, &stride);

	while (running_)
	{
		LibcameraApp::Msg msg = app->Wait();
		if (msg.type == LibcameraApp::MsgType::Quit)
		{
			running_ = false;
		}
		else if (msg.type != LibcameraApp::MsgType::RequestComplete)
		{
			throw std::runtime_error("unrecognised message!");
		}

		CompletedRequestPtr payload = std::get<CompletedRequestPtr>(msg.payload);
		if (payload->stream == app->ViewfinderStream())
		{
			cv::Mat frame;
			getImage(frame, payload);
			if (viewfinder_callback_)
			{
				viewfinder_callback_(frame);
			}
			else
			{
				std::unique_lock<std::mutex> lock(camera_mutex_);
				frame.copyTo(frame_);
				frame_ready_ = true;
				frame_cv_.notify_one();
			}
		}
	}
}

void PiCamera::ApplyZoomOptions()
{
	app->ApplyRoiSettings();
}