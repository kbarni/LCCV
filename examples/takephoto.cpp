#include <lccv.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

int main()
{
    cv::Mat image;
    lccv::Camera cam;
    cam.options->photo_width=2028;
    cam.options->photo_height=1520;
    cam.options->verbose=true;
    cv::namedWindow("Image",cv::WINDOW_NORMAL);
    cam.startPhoto();
    // Let AE/AWB settle on the first frames before we capture.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    for(int i=0;i<100;i++){
        std::cout<<i<<std::endl;
        if(!cam.capturePhoto(image)){
            std::cout<<"Camera error"<<std::endl;
        }
        cv::imshow("Image",image);
        cv::waitKey(30);
    }
    cam.stopPhoto();
    cv::waitKey();
    cv::destroyWindow("Image");
}
