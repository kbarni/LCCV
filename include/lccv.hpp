#ifndef LCCV_HPP
#define LCCV_HPP

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include "libcamera_app.hpp"
#include "libcamera_app_options.hpp"

namespace lccv
{

class PiCamera
{
public:
	PiCamera();
	PiCamera(uint32_t id);
	~PiCamera();

	Options *options;

	// Photo mode
	bool startPhoto(std::function<void(cv::Mat &)> callback = nullptr);
	bool capturePhoto(cv::Mat &frame);
	bool stopPhoto();

	// Video mode
	bool startVideo(std::function<void(cv::Mat &)> callback = nullptr);
	bool getVideoFrame(cv::Mat &frame, unsigned int timeout);
	void stopVideo();

	// Applies new zoom options. Before invoking this func modify options->roi.
	void ApplyZoomOptions();

protected:
	void run();
	void stop();

	std::unique_ptr<LibcameraApp> app;
	void getImage(cv::Mat &frame, CompletedRequestPtr &payload);

	std::thread camera_thread_;
	std::mutex camera_mutex_;
	std::condition_variable frame_cv_;
	std::atomic<bool> running_;
	cv::Mat frame_;
	bool frame_ready_;

	unsigned int still_flags;
	bool camera_started_;

	std::function<void(cv::Mat &)> viewfinder_callback_;
};

} // namespace lccv
#endif
