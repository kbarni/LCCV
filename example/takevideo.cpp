#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>

int main() {
	uint32_t num_cams = LibcameraApp::GetNumberCameras();
	std::cout << "Found " << num_cams << " cameras." << std::endl;

    uint32_t height = 480;
    uint32_t width = 640;
    std::cout<<"Sample program for LCCV video capture"<<std::endl;
    std::cout<<"Press ESC to stop."<<std::endl;
    cv::Mat image = cv::Mat(height, width, CV_8UC3);
    cv::Mat image2 = cv::Mat(height, width, CV_8UC3);
    lccv::PiCamera cam;
    cam.options->video_width=width;
    cam.options->video_height=height;
    cam.options->framerate=30;
    cam.options->verbose=true;
    cv::namedWindow("Video",cv::WINDOW_NORMAL);
    cam.startVideo();

	lccv::PiCamera cam2(1);
	cam2.options->video_width=width;
	cam2.options->video_height=height;
	cam2.options->framerate=30;
	cam2.options->verbose=true;
	if (1 < num_cams) {
		cam2.startVideo();
	}

    int ch=0;
    while(ch!=27){
        if (!cam.getVideoFrame(image,1000)){
            std::cout<<"Timeout error"<<std::endl;
        } else {
			cv::imshow("Video1",image);
		}
		if (1 < num_cams) {
			if (!cam2.getVideoFrame(image2,1000)){
				std::cout<<"Timeout2 error"<<std::endl;
			} else {
				cv::imshow("Video2",image2);
			}
		}
        ch=cv::waitKey(5);
    }

    cam.stopVideo();
	cam2.stopVideo();
	cv::destroyAllWindows();
}
