// Headless test for lccv::Camera viewfinder mode.
// Registers a callback, runs the viewfinder for N frames, prints per-frame
// metadata to stdout. No display, no GUI deps.
// Returns 0 on success, non-zero if no frames arrive.

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include <lccv.hpp>

int main()
{
    constexpr int N = 10;
    constexpr int TIMEOUT_MS = 2000;

    std::atomic<int> frames{0};

    lccv::Camera cam;
    cam.options->viewfinder_width  = 640;
    cam.options->viewfinder_height = 480;
    cam.options->framerate         = 30;
    cam.options->verbose           = true;

    cam.addViewfinder([&frames](cv::Mat &frame) {
        int n = ++frames;
        std::cout << "frame " << n
                  << "  " << frame.cols << "x" << frame.rows
                  << "  channels=" << frame.channels()
                  << "  first px BGR=("
                  << int(frame.at<cv::Vec3b>(0, 0)[0]) << ","
                  << int(frame.at<cv::Vec3b>(0, 0)[1]) << ","
                  << int(frame.at<cv::Vec3b>(0, 0)[2]) << ")"
                  << std::endl;
    });

    if (!cam.startViewfinder()) {
        std::cerr << "startViewfinder failed" << std::endl;
        return 1;
    }

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(TIMEOUT_MS);
    while (frames.load() < N && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    cam.stopViewfinder();

    const int n = frames.load();
    std::cout << "received " << n << " frames" << std::endl;
    return (n >= N) ? 0 : 1;
}
