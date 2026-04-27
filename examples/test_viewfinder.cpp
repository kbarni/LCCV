// Interactive viewfinder + photo test for lccv::Camera.
// Shows a live 640x480 viewfinder window for 5 seconds, then captures a
// full-res photo and shows it in a second window. Repeats 6 times.
// Press ESC at any time to quit early.

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <lccv.hpp>

int main()
{
    lccv::Camera cam;
    cam.options->viewfinder_width  = 640;
    cam.options->viewfinder_height = 480;
    cam.options->photo_width       = 1280;
    cam.options->photo_height      = 960;
    cam.options->framerate         = 30;

    cv::namedWindow("Viewfinder", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Photo",      cv::WINDOW_AUTOSIZE);

    // Frame shared between the viewfinder worker thread and the main thread.
    // The callback must not call imshow directly — OpenCV's GUI is main-thread only.
    cv::Mat vf_display;
    std::mutex vf_mutex;
    std::atomic<bool> vf_new{false};

    cam.addViewfinder([&](cv::Mat &frame) {
        std::lock_guard<std::mutex> lk(vf_mutex);
        frame.copyTo(vf_display);
        vf_new.store(true, std::memory_order_release);
    });

    bool quit = false;
    for (int i = 0; i < 6 && !quit; ++i) {
        std::cout << "Cycle " << (i + 1) << "/6: starting viewfinder" << std::endl;

        if (!cam.startViewfinder()) {
            std::cerr << "startViewfinder failed" << std::endl;
            return 1;
        }

        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline && !quit) {
            if (vf_new.exchange(false, std::memory_order_acq_rel)) {
                std::lock_guard<std::mutex> lk(vf_mutex);
                if (!vf_display.empty())
                    cv::imshow("Viewfinder", vf_display);
            }
            if (cv::waitKey(1) == 27)
                quit = true;
        }

        cam.stopViewfinder();
        if (quit) break;

        std::cout << "Cycle " << (i + 1) << "/6: capturing photo" << std::endl;
        cv::Mat photo;
        if (cam.capturePhoto(photo)) {
            std::cout << "Photo " << (i + 1) << ": "
                      << photo.cols << "x" << photo.rows << std::endl;
            cv::imshow("Photo", photo);
            cv::waitKey(1);
        } else {
            std::cerr << "capturePhoto failed" << std::endl;
        }
    }

    // Keep the last photo visible for a moment before closing.
    if (!quit)
        cv::waitKey(2000);

    cv::destroyAllWindows();
    return 0;
}
