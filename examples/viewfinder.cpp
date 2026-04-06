/*
 * viewfinder.cpp — LCCV viewfinder example
 *
 * Demonstrates three usage patterns:
 *   Mode 1 (default): Viewfinder-only live preview via callback.
 *   Mode 2: Viewfinder + photo — press SPACE to capture a still while
 *           the preview runs.
 *   Mode 3: Viewfinder + video — preview via callback while simultaneously
 *           saving frames to an output file.
 *
 * Usage:
 *   ./viewfinder          — mode 1 (live preview)
 *   ./viewfinder photo    — mode 2 (preview + still capture on SPACE)
 *   ./viewfinder video    — mode 3 (preview + video save)
 *
 * Press ESC or Q to exit.
 */

#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include <string>

static const char *WINDOW_VF    = "Viewfinder";
static const char *WINDOW_PHOTO = "Captured photo";

// ---------------------------------------------------------------------------
// Mode 1 — Viewfinder only
// ---------------------------------------------------------------------------
static void runViewfinderOnly()
{
    lccv::Camera cam;
    cam.options->viewfinder_width  = 640;
    cam.options->viewfinder_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose   = false;

    cv::namedWindow(WINDOW_VF, cv::WINDOW_NORMAL);

    cam.startViewfinder([](cv::Mat &frame) {
        cv::imshow(WINDOW_VF, frame);
    });

    std::cout << "Viewfinder running — press ESC or Q to stop." << std::endl;

    while (true) {
        int key = cv::waitKey(10);
        if (key == 27 || key == 'q' || key == 'Q')
            break;
    }

    cam.stopViewfinder();
    cv::destroyAllWindows();
}

// ---------------------------------------------------------------------------
// Mode 2 — Viewfinder + photo capture
// ---------------------------------------------------------------------------
static void runViewfinderPhoto()
{
    lccv::Camera cam;
    cam.options->photo_width       = 2028;
    cam.options->photo_height      = 1520;
    cam.options->viewfinder_width  = 640;
    cam.options->viewfinder_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose   = false;

    cv::namedWindow(WINDOW_VF,    cv::WINDOW_NORMAL);
    cv::namedWindow(WINDOW_PHOTO, cv::WINDOW_NORMAL);

    // Start photo mode first, then attach viewfinder
    cam.startPhoto();
    cam.startViewfinder([](cv::Mat &frame) {
        cv::imshow(WINDOW_VF, frame);
    });

    std::cout << "Viewfinder + photo — press SPACE to capture, ESC/Q to quit." << std::endl;

    int photoCount = 0;
    while (true) {
        int key = cv::waitKey(10);
        if (key == 27 || key == 'q' || key == 'Q')
            break;

        if (key == ' ') {
            cv::Mat image;
            std::cout << "Capturing photo " << ++photoCount << "..." << std::flush;
            if (cam.capturePhoto(image)) {
                std::cout << " OK (" << image.cols << "x" << image.rows << ")" << std::endl;
                cv::imshow(WINDOW_PHOTO, image);
                std::string fname = "photo_" + std::to_string(photoCount) + ".jpg";
                cv::imwrite(fname, image);
                std::cout << "Saved to " << fname << std::endl;
            } else {
                std::cout << " FAILED" << std::endl;
            }
        }
    }

    cam.stopViewfinder();
    cam.stopPhoto();
    cv::destroyAllWindows();
}

// ---------------------------------------------------------------------------
// Mode 3 — Viewfinder + video
// ---------------------------------------------------------------------------
static void runViewfinderVideo()
{
    lccv::Camera cam;
    cam.options->video_width       = 1280;
    cam.options->video_height      = 720;
    cam.options->viewfinder_width  = 640;
    cam.options->viewfinder_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose   = false;

    cv::namedWindow(WINDOW_VF, cv::WINDOW_NORMAL);

    // Count frames via atomic so the callback (different thread) can update it
    std::atomic<int> frameCount{0};
    std::atomic<bool> saveFrames{false};

    cam.startVideo();
    cam.startViewfinder([&](cv::Mat &frame) {
        cv::imshow(WINDOW_VF, frame);
        if (saveFrames.load())
            ++frameCount;
    });

    std::cout << "Viewfinder + video — press R to start/stop recording, ESC/Q to quit."
              << std::endl;

    cv::VideoWriter writer;
    const std::string outfile = "recording.avi";

    while (true) {
        // Also poll getVideoFrame so the writer gets full-resolution frames
        if (saveFrames.load()) {
            cv::Mat frame;
            if (cam.getVideoFrame(frame, 100)) {
                if (!writer.isOpened()) {
                    writer.open(outfile,
                                cv::VideoWriter::fourcc('M','J','P','G'),
                                cam.options->framerate,
                                cv::Size(cam.options->video_width,
                                         cam.options->video_height));
                    std::cout << "Recording to " << outfile << std::endl;
                }
                writer.write(frame);
            }
        }

        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
            break;

        if (key == 'r' || key == 'R') {
            if (!saveFrames.load()) {
                saveFrames.store(true);
                std::cout << "Recording started." << std::endl;
            } else {
                saveFrames.store(false);
                if (writer.isOpened()) {
                    writer.release();
                    std::cout << "Recording stopped. "
                              << frameCount.load() << " callback frames." << std::endl;
                }
            }
        }
    }

    if (writer.isOpened())
        writer.release();

    cam.stopViewfinder();
    cam.stopVideo();
    cv::destroyAllWindows();
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    std::string mode = (argc > 1) ? argv[1] : "";

    if (mode == "photo") {
        runViewfinderPhoto();
    } else if (mode == "video") {
        runViewfinderVideo();
    } else {
        if (!mode.empty() && mode != "vf")
            std::cerr << "Unknown mode '" << mode << "'. Using viewfinder-only." << std::endl;
        runViewfinderOnly();
    }

    return 0;
}
