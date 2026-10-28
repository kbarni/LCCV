#include <lccv.hpp>
#include <opencv2/opencv.hpp>

// Streams video from the first camera, and from a second one if present.
int main()
{
    const uint32_t num_cams = LibcameraApp::GetNumberCameras();
    std::cout << "Found " << num_cams << " cameras." << std::endl;

    const uint32_t width = 640;
    const uint32_t height = 480;
    std::cout << "Sample program for LCCV video capture" << std::endl;
    std::cout << "Press ESC to stop." << std::endl;

    cv::Mat image, image2;
    lccv::Camera cam;
    cam.options->video_width = width;
    cam.options->video_height = height;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cv::namedWindow("Video1", cv::WINDOW_NORMAL);
    cam.startVideo();

    lccv::Camera cam2(1);
    if (num_cams > 1) {
        cam2.options->video_width = width;
        cam2.options->video_height = height;
        cam2.options->framerate = 30;
        cam2.options->verbose = true;
        cv::namedWindow("Video2", cv::WINDOW_NORMAL);
        cam2.startVideo();
    }

    int ch = 0;
    while (ch != 27) {
        if (!cam.getVideoFrame(image, 1000))
            std::cout << "Timeout error" << std::endl;
        else
            cv::imshow("Video1", image);

        if (num_cams > 1) {
            if (!cam2.getVideoFrame(image2, 1000))
                std::cout << "Timeout2 error" << std::endl;
            else
                cv::imshow("Video2", image2);
        }
        ch = cv::waitKey(5);
    }

    cam.stopVideo();
    cam2.stopVideo();
    cv::destroyAllWindows();
}
