/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_app.cpp - base class for libcamera apps.
 */

#include "libcamera_app.hpp"
#include "libcamera_app_options.hpp"

LibcameraApp::LibcameraApp(std::unique_ptr<Options> opts)
	: options_(std::move(opts)), controls_(controls::controls)

{
	if (!options_)
		options_ = std::make_unique<Options>();
	controls_.clear();
}

LibcameraApp::~LibcameraApp()
{
	StopCamera();
	Teardown();
	CloseCamera();
}

std::string const &LibcameraApp::CameraId() const
{
	return camera_->id();
}

uint32_t LibcameraApp::GetNumberCameras() {
	return getCameraManager()->cameras().size();
}

void LibcameraApp::OpenCamera()
{

	if (options_->verbose)
		std::cerr << "Opening camera..." << std::endl;

	if (getCameraManager()->cameras().size() == 0)
		throw std::runtime_error("no cameras available");
	if (options_->camera >= getCameraManager()->cameras().size())
		throw std::runtime_error("selected camera is not available");

	std::string const &cam_id = getCameraManager()->cameras()[options_->camera]->id();
	camera_ = getCameraManager()->get(cam_id);
	if (!camera_)
		throw std::runtime_error("failed to find camera " + cam_id);

	if (camera_->acquire())
		throw std::runtime_error("failed to acquire camera " + cam_id);
	camera_acquired_ = true;

	if (options_->verbose)
		std::cerr << "Acquired camera " << cam_id << std::endl;

}

void LibcameraApp::CloseCamera()
{
	if (camera_acquired_)
		camera_->release();
	camera_acquired_ = false;

	camera_.reset();

	if (options_->verbose && !options_->help)
		std::cerr << "Camera closed" << std::endl;
}

void LibcameraApp::ConfigureStill(unsigned int flags)
{
	if (options_->verbose)
		std::cerr << "Configuring still capture..." << std::endl;

	StreamRoles stream_roles = { StreamRole::StillCapture, StreamRole::Viewfinder };
	configuration_ = camera_->generateConfiguration(stream_roles);
	if (!configuration_)
		throw std::runtime_error("failed to generate still capture configuration");

	if (flags & FLAG_STILL_BGR)
		configuration_->at(0).pixelFormat = libcamera::formats::BGR888;
	else if (flags & FLAG_STILL_RGB)
		configuration_->at(0).pixelFormat = libcamera::formats::RGB888;
	else
		configuration_->at(0).pixelFormat = libcamera::formats::YUV420;

	if ((flags & FLAG_STILL_BUFFER_MASK) == FLAG_STILL_DOUBLE_BUFFER)
		configuration_->at(0).bufferCount = 2;
	else if ((flags & FLAG_STILL_BUFFER_MASK) == FLAG_STILL_TRIPLE_BUFFER)
		configuration_->at(0).bufferCount = 3;

	if (options_->photo_width)
		configuration_->at(0).size.width = options_->photo_width;
	if (options_->photo_height)
		configuration_->at(0).size.height = options_->photo_height;

	configuration_->at(1).pixelFormat = libcamera::formats::RGB888;
	configuration_->at(1).size.width = options_->video_width;
	configuration_->at(1).size.height = options_->video_height;
	configuration_->at(1).bufferCount = 4;

	configureDenoise(options_->denoise == "auto" ? "cdn_hq" : options_->denoise);
	setupCapture();

	streams_["still"] = configuration_->at(0).stream();
	streams_["viewfinder"] = configuration_->at(1).stream();

	if (options_->verbose)
		std::cerr << "Still capture setup complete" << std::endl;
}

void LibcameraApp::ConfigureViewfinder()
{
    if (options_->verbose)
        std::cerr << "Configuring viewfinder..." << std::endl;

    StreamRoles stream_roles = { StreamRole::Viewfinder };
    configuration_ = camera_->generateConfiguration(stream_roles);
    if (!configuration_)
        throw std::runtime_error("failed to generate viewfinder configuration");

    configuration_->at(0).pixelFormat = libcamera::formats::RGB888;
    configuration_->at(0).size.width = options_->video_width;
    configuration_->at(0).size.height = options_->video_height;
    configuration_->at(0).bufferCount = 4;

    configureDenoise(options_->denoise == "auto" ? "cdn_off" : options_->denoise);
    setupCapture();

    streams_["viewfinder"] = configuration_->at(0).stream();

    if (options_->verbose)
        std::cerr << "Viewfinder setup complete" << std::endl;
}

void LibcameraApp::Teardown()
{
	if (options_->verbose && !options_->help)
		std::cerr << "Tearing down requests, buffers and configuration" << std::endl;

	for (auto &iter : mapped_buffers_)
	{
		for (auto &span : iter.second)
			munmap(span.data(), span.size());
	}
	mapped_buffers_.clear();

	delete allocator_;
	allocator_ = nullptr;

	configuration_.reset();

	frame_buffers_.clear();

	streams_.clear();
}

void LibcameraApp::StartCamera()
{
	makeRequests();

	if (!controls_.get(controls::ScalerCrop) && options_->roi_width != 0 && options_->roi_height != 0)
	{
		Rectangle sensor_area = *camera_->properties().get(properties::ScalerCropMaximum);
		int x = options_->roi_x * sensor_area.width;
		int y = options_->roi_y * sensor_area.height;
		int w = options_->roi_width * sensor_area.width;
		int h = options_->roi_height * sensor_area.height;
		Rectangle crop(x, y, w, h);
		crop.translateBy(sensor_area.topLeft());
		if (options_->verbose)
			std::cerr << "Using crop " << crop.toString() << std::endl;
		controls_.set(controls::ScalerCrop, crop);
	}

	if (!controls_.get(controls::FrameDurationLimits))
	{
		if (StillStream())
			controls_.set(controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({ INT64_C(100), INT64_C(1000000000) }));
		else if (options_->framerate > 0)
		{
			int64_t frame_time = 1000000 / options_->framerate; // in us
			controls_.set(controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
		}
	}

	if (!controls_.get(controls::ExposureTime) && options_->shutter)
		controls_.set(controls::ExposureTime, options_->shutter);
	if (!controls_.get(controls::AnalogueGain) && options_->gain)
		controls_.set(controls::AnalogueGain, options_->gain);
	if (!controls_.get(controls::AeMeteringMode))
		controls_.set(controls::AeMeteringMode, options_->getMeteringMode());
	if (!controls_.get(controls::AeExposureMode))
		controls_.set(controls::AeExposureMode, options_->getExposureMode());
	if (!controls_.get(controls::ExposureValue))
		controls_.set(controls::ExposureValue, options_->ev);
	if (!controls_.get(controls::AwbMode))
		controls_.set(controls::AwbMode, options_->getWhiteBalance());
	if (!controls_.get(controls::ColourGains) && options_->awb_gain_r && options_->awb_gain_b)
		controls_.set(controls::ColourGains, libcamera::Span<const float, 2>({ options_->awb_gain_r, options_->awb_gain_b }));
	if (!controls_.get(controls::Brightness))
		controls_.set(controls::Brightness, options_->brightness);
	if (!controls_.get(controls::Contrast))
		controls_.set(controls::Contrast, options_->contrast);
	if (!controls_.get(controls::Saturation))
		controls_.set(controls::Saturation, options_->saturation);
	if (!controls_.get(controls::Sharpness))
		controls_.set(controls::Sharpness, options_->sharpness);

	if (camera_->start(&controls_))
		throw std::runtime_error("failed to start camera");
	controls_.clear();
	camera_started_ = true;
	last_timestamp_ = 0;

	camera_->requestCompleted.connect(this, &LibcameraApp::requestComplete);

	for (std::unique_ptr<Request> &request : requests_)
	{
		if (camera_->queueRequest(request.get()) < 0)
			throw std::runtime_error("Failed to queue request");
	}

	if (options_->verbose)
		std::cerr << "Camera started!" << std::endl;
}

void LibcameraApp::StopCamera()
{
	{
		std::lock_guard<std::mutex> lock(camera_stop_mutex_);
		if (camera_started_)
		{
			if (camera_->stop())
				throw std::runtime_error("failed to stop camera");

			camera_started_ = false;
		}
	}

	if (camera_)
		camera_->requestCompleted.disconnect(this, &LibcameraApp::requestComplete);

	completed_requests_.clear();

	msg_queue_.Clear();
	request_queue_.Clear();

	while (!free_requests_.empty())
		free_requests_.pop();

	requests_.clear();

	controls_.clear();

	if (options_->verbose && !options_->help)
		std::cerr << "Camera stopped!" << std::endl;
}

void LibcameraApp::ApplyRoiSettings(){
    if (!controls_.get(controls::ScalerCrop) && options_->roi_width != 0 && options_->roi_height != 0)
    {
        Rectangle sensor_area = *camera_->properties().get(properties::ScalerCropMaximum);
        int x = options_->roi_x * sensor_area.width;
        int y = options_->roi_y * sensor_area.height;
        int w = options_->roi_width * sensor_area.width;
        int h = options_->roi_height * sensor_area.height;
        Rectangle crop(x, y, w, h);
        crop.translateBy(sensor_area.topLeft());
        if (options_->verbose)
            std::cerr << "Using crop " << crop.toString() << std::endl;
        controls_.set(controls::ScalerCrop, crop);
    }
}

LibcameraApp::Msg LibcameraApp::Wait()
{
	return msg_queue_.Wait();
}

void LibcameraApp::QueueRequest(RequestType type)
{
	request_queue_.Post(type);
}

void LibcameraApp::queueRequest(CompletedRequest *completed_request)
{
	BufferMap buffers(std::move(completed_request->buffers));

	Request *request = completed_request->request;
	assert(request);

	std::lock_guard<std::mutex> stop_lock(camera_stop_mutex_);
	if (!camera_started_)
		return;

	{
		std::lock_guard<std::mutex> lock(completed_requests_mutex_);
		auto it = completed_requests_.find(completed_request);
        delete completed_request;
		if (it == completed_requests_.end())
			return;
		completed_requests_.erase(it);
	}

	for (auto const &p : buffers)
	{
		if (request->addBuffer(p.first, p.second) < 0)
			throw std::runtime_error("failed to add buffer to request in QueueRequest");
	}

	{
		std::lock_guard<std::mutex> lock(control_mutex_);
		request->controls() = std::move(controls_);
	}

	if (camera_->queueRequest(request) < 0)
		throw std::runtime_error("failed to queue request");
}

void LibcameraApp::PostMessage(MsgType &t, MsgPayload &p)
{
	msg_queue_.Post(Msg(t, std::move(p)));
}

libcamera::Stream *LibcameraApp::GetStream(std::string const &name, unsigned int *w, unsigned int *h,
										   unsigned int *stride) const
{
	auto it = streams_.find(name);
	if (it == streams_.end())
		return nullptr;
	StreamDimensions(it->second, w, h, stride);
	return it->second;
}

libcamera::Stream *LibcameraApp::ViewfinderStream(unsigned int *w, unsigned int *h, unsigned int *stride) const
{
	return GetStream("viewfinder", w, h, stride);
}

libcamera::Stream *LibcameraApp::StillStream(unsigned int *w, unsigned int *h, unsigned int *stride) const
{
	return GetStream("still", w, h, stride);
}

libcamera::Stream *LibcameraApp::RawStream(unsigned int *w, unsigned int *h, unsigned int *stride) const
{
	return GetStream("raw", w, h, stride);
}

libcamera::Stream *LibcameraApp::VideoStream(unsigned int *w, unsigned int *h, unsigned int *stride) const
{
	return GetStream("video", w, h, stride);
}

libcamera::Stream *LibcameraApp::LoresStream(unsigned int *w, unsigned int *h, unsigned int *stride) const
{
	return GetStream("lores", w, h, stride);
}

libcamera::Stream *LibcameraApp::GetMainStream() const
{
	for (auto &p : streams_)
	{
		if (p.first == "viewfinder" || p.first == "still" || p.first == "video")
			return p.second;
	}

	return nullptr;
}

std::vector<libcamera::Span<uint8_t>> LibcameraApp::Mmap(FrameBuffer *buffer) const
{
	auto item = mapped_buffers_.find(buffer);
	if (item == mapped_buffers_.end())
		return {};
	return item->second;
}

void LibcameraApp::SetControls(ControlList &controls)
{
	std::lock_guard<std::mutex> lock(control_mutex_);
	controls_ = std::move(controls);
}

void LibcameraApp::StreamDimensions(Stream const *stream, unsigned int *w, unsigned int *h, unsigned int *stride) const
{
	StreamConfiguration const &cfg = stream->configuration();
	if (w)
		*w = cfg.size.width;
	if (h)
		*h = cfg.size.height;
	if (stride)
		*stride = cfg.stride;
}

void LibcameraApp::setupCapture()
{
	CameraConfiguration::Status validation = configuration_->validate();
	if (validation == CameraConfiguration::Invalid)
		throw std::runtime_error("failed to valid stream configurations");
	else if (validation == CameraConfiguration::Adjusted)
		std::cerr << "Stream configuration adjusted" << std::endl;

	if (camera_->configure(configuration_.get()) < 0)
		throw std::runtime_error("failed to configure streams");
	if (options_->verbose)
		std::cerr << "Camera streams configured" << std::endl;

	allocator_ = new FrameBufferAllocator(camera_);
	for (StreamConfiguration &config : *configuration_)
	{
		Stream *stream = config.stream();

		if (allocator_->allocate(stream) < 0)
			throw std::runtime_error("failed to allocate capture buffers");

		for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream))
		{
			size_t buffer_size = 0;
			for (unsigned i = 0; i < buffer->planes().size(); i++)
			{
				const FrameBuffer::Plane plane = buffer->planes()[i];
				buffer_size += plane.length;
				if (i == buffer->planes().size() - 1 || plane.fd.get() != buffer->planes()[i + 1].fd.get())
				{
					void *memory = mmap(NULL, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
					mapped_buffers_[buffer.get()].push_back(
						libcamera::Span<uint8_t>(static_cast<uint8_t *>(memory), buffer_size));
					buffer_size = 0;
				}
			}
			frame_buffers_[stream].push(buffer.get());
		}
	}
	if (options_->verbose)
		std::cerr << "Buffers allocated and mapped" << std::endl;
}

void LibcameraApp::makeRequests()
{
	auto free_buffers(frame_buffers_);
	while (true)
	{
		RequestType type = request_queue_.Wait();
		if (type == RequestType::None)
			break;

		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request)
			throw std::runtime_error("failed to make request");

		if (type == RequestType::Still)
		{
			Stream *stream = streams_["still"];
			if (free_buffers[stream].empty())
				throw std::runtime_error("no buffer for still stream");
			if (request->addBuffer(stream, free_buffers[stream].front()) < 0)
				throw std::runtime_error("failed to add buffer to request");
			free_buffers[stream].pop();
		}
		else if (type == RequestType::Video || type == RequestType::Viewfinder)
		{
			Stream *stream = streams_["viewfinder"];
			if (free_buffers[stream].empty())
				throw std::runtime_error("no buffer for viewfinder stream");
			if (request->addBuffer(stream, free_buffers[stream].front()) < 0)
				throw std::runtime_error("failed to add buffer to request");
			free_buffers[stream].pop();
		}
		requests_.push_back(std::move(request));
	}
	if (options_->verbose)
		std::cerr << "Requests created" << std::endl;
}

void LibcameraApp::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	CompletedRequest *r = new CompletedRequest(sequence_++, request);
	CompletedRequestPtr payload(r, [this](CompletedRequest *cr) { this->queueRequest(cr); });
	{
		std::lock_guard<std::mutex> lock(completed_requests_mutex_);
		completed_requests_.insert(r);
	}

	uint64_t timestamp = payload->buffers.begin()->second->metadata().timestamp;
	if (last_timestamp_ == 0 || last_timestamp_ == timestamp)
		payload->framerate = 0;
	else
		payload->framerate = 1e9 / (timestamp - last_timestamp_);
	last_timestamp_ = timestamp;

    payload->stream = const_cast<libcamera::Stream *>(request->buffers().begin()->first);
    msg_queue_.Post(Msg(MsgType::RequestComplete, std::move(payload)));
}

void LibcameraApp::configureDenoise(const std::string &denoise_mode)
{
	using namespace libcamera::controls::draft;

	static const std::map<std::string, NoiseReductionModeEnum> denoise_table = {
		{ "off", NoiseReductionModeOff },
		{ "cdn_off", NoiseReductionModeMinimal },
		{ "cdn_fast", NoiseReductionModeFast },
		{ "cdn_hq", NoiseReductionModeHighQuality }
	};
	NoiseReductionModeEnum denoise;

	auto const mode = denoise_table.find(denoise_mode);
	if (mode == denoise_table.end())
		throw std::runtime_error("Invalid denoise mode " + denoise_mode);
	denoise = mode->second;

	controls_.set(NoiseReductionMode, denoise);
}
