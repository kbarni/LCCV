// Headless test for lccv::Camera video mode.
// Captures N frames and prints per-frame metadata to stdout. No display, no GUI deps.
// Returns 0 on success, non-zero if any frame timed out.

#include <chrono>
#include <iostream>

#include <lccv.hpp>

int main()
{
    constexpr int N = 10;
    constexpr unsigned int TIMEOUT_MS = 1000;

    lccv::Camera cam;
    cam.options->video_width  = 640;
    cam.options->video_height = 480;
    cam.options->framerate    = 30;
    cam.options->verbose      = true;

    std::cout << "Video test: requesting "
              << cam.options->video_width << "x" << cam.options->video_height
              << " @ " << cam.options->framerate << " fps, "
              << N << " frames" << std::endl;

    if (!cam.startVideo()) {
        std::cerr << "startVideo failed" << std::endl;
        return 1;
    }

    int errors = 0;
    cv::Mat frame;
    for (int i = 0; i < N; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        bool ok = cam.getVideoFrame(frame, TIMEOUT_MS);
        auto t1 = std::chrono::steady_clock::now();
        auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

        if (!ok) {
            std::cerr << "  frame " << i << ": timeout after " << dt_ms << " ms" << std::endl;
            ++errors;
            continue;
        }

        cv::Vec3b px = frame.at<cv::Vec3b>(0, 0);
        std::cout << "  frame " << i
                  << "  size=" << frame.cols << "x" << frame.rows
                  << "  raw[0,0]={" << (int)px[0] << "," << (int)px[1] << "," << (int)px[2] << "}"
                  << "  fetched in " << dt_ms << " ms"
                  << std::endl;
    }

    cam.stopVideo();
    std::cout << "Done. " << errors << " timeouts" << std::endl;
    return errors == 0 ? 0 : 1;
}
