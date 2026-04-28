#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>

// Viewfinder preview with on-demand photo capture.
// Press SPACE to capture a full-res photo while the viewfinder keeps running.
// Press ESC to quit.

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

    cv::Mat vf_display;
    std::mutex vf_mutex;
    std::atomic<bool> vf_new{false};

    cam.addViewfinder([&](cv::Mat &frame) {
        std::lock_guard<std::mutex> lk(vf_mutex);
        frame.copyTo(vf_display);
        vf_new.store(true, std::memory_order_release);
    });

    if (!cam.startViewfinder()) return 1;

    int key = 0;
    while (key != 27) {
        if (vf_new.exchange(false, std::memory_order_acq_rel)) {
            std::lock_guard<std::mutex> lk(vf_mutex);
            if (!vf_display.empty())
                cv::imshow("Viewfinder", vf_display);
        }
        key = cv::waitKey(10);
        if (key == ' ') {
            cv::Mat photo;
            if (cam.capturePhoto(photo))
                cv::imshow("Photo", photo);
        }
    }

    cam.stopViewfinder();
    cv::destroyAllWindows();
}
