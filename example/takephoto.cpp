#include <lccv.hpp>
#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat image;
    lccv::PiCamera cam;
    cam.options->width=4056;
    cam.options->height=3040;
    cam.options->verbose=true;
    if(!cam.captureFrame(image)){
        std::cout<<"Camera error"<<std::endl;
    }
    cv::namedWindow("Image",cv::WINDOW_NORMAL);
    cv::imshow("Image",image);
    cv::waitKey();
    cv::destroyWindow("Image");
}
