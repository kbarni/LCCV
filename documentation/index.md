# LCCV — libcamera bindings for OpenCV

LCCV is a C++ library that provides a simple, OpenCV-compatible interface to cameras accessed through libcamera. It is primarily intended for Raspberry Pi cameras but is not architecturally restricted to that platform.

---

## Contents

- [Installation](index.md#installation)
- [Quick start](index.md#quick-start)
- [API reference](api_reference.md)
- [Usage examples](examples.md)

---

## Installation

### Prerequisites

| Package | Purpose |
|---------|---------|
| `libcamera-dev` | Camera access library |
| `libopencv-dev` | Image processing and display |
| `cmake` (≥ 3.16) | Build system |
| C++17 compiler | `g++` or `clang++` |

```bash
sudo apt install build-essential cmake git libcamera-dev libopencv-dev
```

### Build and install

```bash
git clone https://github.com/kbarni/LCCV.git
cd LCCV
cmake -B build
cmake --build build
sudo cmake --install build
```

### Using in your project

After installation, downstream CMake projects can find the library with:

```cmake
find_package(lccv REQUIRED)
target_link_libraries(my_app PRIVATE lccv::liblccv)
```

Or link manually:

```bash
g++ -std=c++17 my_app.cpp -llccv $(pkg-config --libs libcamera) -lopencv_core -lopencv_highgui -lopencv_imgproc
```

### Build the examples

```bash
cmake -B build -DBUILD_EXAMPLES=ON
cmake --build build
```

---

## Quick start

### Capture a single photo

```cpp
#include <lccv.hpp>
#include <opencv2/opencv.hpp>

int main()
{
    lccv::Camera cam;
    cam.options->photo_width  = 1920;
    cam.options->photo_height = 1080;

    cv::Mat image;
    if (cam.capturePhoto(image))
        cv::imwrite("photo.jpg", image);
}
```

### Capture a burst of photos efficiently

```cpp
lccv::Camera cam;
cam.options->photo_width  = 1920;
cam.options->photo_height = 1080;
cam.startPhoto();               // open camera once

for (int i = 0; i < 10; i++) {
    cv::Mat image;
    cam.capturePhoto(image);    // reuses open camera — no latency
    cv::imwrite("photo_" + std::to_string(i) + ".jpg", image);
}

cam.stopPhoto();
```

### Live video feed

```cpp
lccv::Camera cam;
cam.options->video_width  = 1280;
cam.options->video_height = 720;
cam.options->framerate    = 30;

cam.startVideo();
cv::Mat frame;
while (true) {
    if (cam.getVideoFrame(frame, 1000))
        cv::imshow("Video", frame);
    if (cv::waitKey(1) == 27) break;  // ESC
}
cam.stopVideo();
```

### Live preview with a callback (viewfinder)

```cpp
lccv::Camera cam;
cam.options->viewfinder_width  = 640;
cam.options->viewfinder_height = 480;

cam.startViewfinder([](cv::Mat &frame) {
    cv::imshow("Preview", frame);
    cv::waitKey(1);
});

cv::waitKey(0);         // block main thread; callback runs in background
cam.stopViewfinder();
```

### Viewfinder + photo capture

```cpp
lccv::Camera cam;
cam.options->photo_width  = 4056;
cam.options->photo_height = 3040;
cam.options->viewfinder_width  = 640;
cam.options->viewfinder_height = 480;

cam.startPhoto();
cam.startViewfinder([](cv::Mat &frame) {
    cv::imshow("Preview", frame);
    cv::waitKey(1);
});

cv::Mat image;
cam.capturePhoto(image);    // preview keeps running
cv::imwrite("full_res.jpg", image);

cam.stopViewfinder();
cam.stopPhoto();
```
