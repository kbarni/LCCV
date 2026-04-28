#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>

// Viewfinder preview with simultaneous video stream.
// The viewfinder callback drives the preview window; the video frame count
// is overlaid to confirm both streams are running. Press ESC to quit.

int main()
{
    lccv::Camera cam;
    cam.options->viewfinder_width  = 640;
    cam.options->viewfinder_height = 480;
    cam.options->video_width       = 1280;
    cam.options->video_height      = 960;
    cam.options->framerate         = 30;

    cv::namedWindow("Viewfinder", cv::WINDOW_AUTOSIZE);

    cv::Mat vf_display;
    std::mutex vf_mutex;
    std::atomic<bool> vf_new{false};

    cam.addViewfinder([&](cv::Mat &frame) {
        std::lock_guard<std::mutex> lk(vf_mutex);
        frame.copyTo(vf_display);
        vf_new.store(true, std::memory_order_release);
    });

    if (!cam.startViewfinder()) return 1;
    cam.startVideo();

    int key = 0;
    int frames = 0;
    while (key != 27) {
        cv::Mat vframe;
        if (cam.getVideoFrame(vframe, 50))
            ++frames;

        if (vf_new.exchange(false, std::memory_order_acq_rel)) {
            std::lock_guard<std::mutex> lk(vf_mutex);
            if (!vf_display.empty()) {
                cv::putText(vf_display, "video frames: " + std::to_string(frames),
                            {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
                cv::imshow("Viewfinder", vf_display);
            }
        }
        key = cv::waitKey(1);
    }

    cam.stopVideo();
    cam.stopViewfinder();
    cv::destroyAllWindows();
}
