// Interactive viewfinder + photo + video test for lccv::Camera.
// Each cycle: shows a live viewfinder for 5 s, then captures a full-res photo.
// During seconds 2-4 of the viewfinder phase the video gate is opened so that
// getVideoFrame() also works in parallel with the viewfinder callback.
// Repeats 6 times. Press ESC at any time to quit early.

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

        // Open the video gate after ~1 s, close it after ~4 s.
        // This exercises startVideo/stopVideo as software gates while the
        // viewfinder callback continues uninterrupted throughout.
        auto t0 = std::chrono::steady_clock::now();
        bool video_started = false;
        int video_frames = 0;

        auto deadline = t0 + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline && !quit) {
            auto elapsed = std::chrono::steady_clock::now() - t0;

            if (!video_started && elapsed > std::chrono::seconds(1)) {
                std::cout << "  opening video gate" << std::endl;
                cam.startVideo();
                video_started = true;
            }
            if (video_started && elapsed < std::chrono::seconds(4)) {
                cv::Mat vframe;
                if (cam.getVideoFrame(vframe, 50)) {
                    ++video_frames;
                    // On first video frame print resolution as a sanity check.
                    if (video_frames == 1)
                        std::cout << "  first video frame: "
                                  << vframe.cols << "x" << vframe.rows << std::endl;
                }
            }
            if (video_started && elapsed >= std::chrono::seconds(4)) {
                cam.stopVideo();
                video_started = false;
                std::cout << "  video gate closed, got " << video_frames << " frames" << std::endl;
                video_frames = -1; // sentinel so we don't print again
            }

            if (vf_new.exchange(false, std::memory_order_acq_rel)) {
                std::lock_guard<std::mutex> lk(vf_mutex);
                if (!vf_display.empty())
                    cv::imshow("Viewfinder", vf_display);
            }
            if (cv::waitKey(1) == 27)
                quit = true;
        }
        if (video_started)
            cam.stopVideo();

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
