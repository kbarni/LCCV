#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>

void viewfinder_callback(cv::Mat &frame)
{
    std::cout<<"*";
    cv::imshow("Viewfinder", frame);
    cv::waitKey(1);
}

int main()
{
    cv::Mat image;
    lccv::PiCamera cam;
    cam.options->photo_width = 2028;
    cam.options->photo_height = 1520;
    cam.options->verbose = true;

    cv::namedWindow("Viewfinder", cv::WINDOW_NORMAL);
    cam.startPhoto(viewfinder_callback);

    for (int i = 0; i < 10; i++)
    {
        std::cout << "Capturing photo " << i << std::endl;
        if (!cam.capturePhoto(image))
        {
            std::cout << "Camera error" << std::endl;
        }
        else
        {
            cv::imshow("Photo", image);
            cv::waitKey(1000);
        }
        sleep(5);
    }

    cam.stopPhoto();
    cv::destroyAllWindows();
}
