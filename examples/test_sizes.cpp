// Headless test for lccv::Camera resolution enumeration.
// Lists supported photo and video resolutions for the default camera.

#include <iostream>

#include <lccv.hpp>

int main()
{
    lccv::Camera cam;
    cam.options->verbose = true;

    std::cout << "Photo sizes:" << std::endl;
    auto photo = cam.supportedPhotoSizes();
    for (const auto &s : photo)
        std::cout << "  " << s.width << " x " << s.height << std::endl;
    std::cout << "  (" << photo.size() << " total)" << std::endl;

    std::cout << "Video sizes:" << std::endl;
    auto video = cam.supportedVideoSizes();
    for (const auto &s : video)
        std::cout << "  " << s.width << " x " << s.height << std::endl;
    std::cout << "  (" << video.size() << " total)" << std::endl;

    return (photo.empty() && video.empty()) ? 1 : 0;
}
