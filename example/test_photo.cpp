// Headless test for lccv::PiCamera photo mode.
// Captures a single still and prints metadata to stdout. No display, no GUI deps.
// Returns 0 on success, non-zero on failure.

#include <chrono>
#include <iostream>

#include <lccv.hpp>

int main()
{
    lccv::PiCamera cam;
    cam.options->photo_width  = 1280;
    cam.options->photo_height = 960;
    cam.options->verbose      = true;

    std::cout << "Photo test: requesting "
              << cam.options->photo_width << "x" << cam.options->photo_height
              << std::endl;

    cv::Mat frame;
    auto t0 = std::chrono::steady_clock::now();
    bool ok = cam.capturePhoto(frame);
    auto t1 = std::chrono::steady_clock::now();
    auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    if (!ok) {
        std::cerr << "capturePhoto returned false" << std::endl;
        return 1;
    }
    if (frame.empty()) {
        std::cerr << "frame is empty" << std::endl;
        return 1;
    }

    cv::Vec3b px = frame.at<cv::Vec3b>(0, 0);
    cv::Scalar mean = cv::mean(frame);
    std::cout << "  ok in " << dt_ms << " ms"
              << "  size=" << frame.cols << "x" << frame.rows
              << "  channels=" << frame.channels()
              << "  type=" << frame.type()
              << "  raw[0,0]={" << (int)px[0] << "," << (int)px[1] << "," << (int)px[2] << "}"
              << "  mean=(" << mean[0] << "," << mean[1] << "," << mean[2] << ")"
              << std::endl;

    return 0;
}
