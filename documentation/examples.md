# Usage Examples

---

## Photo capture

### Single one-shot capture

The simplest usage: no setup needed. `capturePhoto()` opens the camera, captures one frame, and closes it.

```cpp
#include <lccv.hpp>
#include <opencv2/opencv.hpp>

int main()
{
    lccv::Camera cam;
    cam.options->photo_width  = 1920;
    cam.options->photo_height = 1080;

    cv::Mat image;
    if (!cam.capturePhoto(image)) {
        std::cerr << "Capture failed" << std::endl;
        return 1;
    }
    cv::imwrite("shot.jpg", image);
}
```

### Burst capture (efficient)

`startPhoto()` opens and configures the camera once. Each subsequent `capturePhoto()` call reuses the session — no open/configure overhead.

```cpp
lccv::Camera cam;
cam.options->photo_width  = 4056;
cam.options->photo_height = 3040;
cam.options->verbose      = true;

cam.startPhoto();

for (int i = 0; i < 5; i++) {
    cv::Mat image;
    if (cam.capturePhoto(image)) {
        cv::imwrite("burst_" + std::to_string(i) + ".jpg", image);
        std::cout << "Captured " << i << std::endl;
    }
}

cam.stopPhoto();
```

### Custom exposure settings

```cpp
lccv::Camera cam;
cam.options->photo_width  = 2028;
cam.options->photo_height = 1520;
cam.options->shutter      = 10000;  // 10 ms (microseconds)
cam.options->gain         = 4.0f;
cam.options->ev           = -0.5f;
cam.options->setWhiteBalance(lccv::WhiteBalance::DAYLIGHT);
cam.options->setMetering(lccv::Metering::SPOT);

cv::Mat image;
cam.capturePhoto(image);
cv::imwrite("custom_exposure.jpg", image);
```

### Grayscale capture

```cpp
lccv::Camera cam;
cam.options->photo_width  = 1920;
cam.options->photo_height = 1080;
cam.options->format       = lccv::PixelFormat::GRAYSCALE;

cv::Mat image;
cam.capturePhoto(image);   // image.type() == CV_8UC1
cv::imwrite("gray.png", image);
```

---

## Video

### Live display

```cpp
lccv::Camera cam;
cam.options->video_width  = 1280;
cam.options->video_height = 720;
cam.options->framerate    = 30;

cv::namedWindow("Video", cv::WINDOW_NORMAL);
cam.startVideo();

cv::Mat frame;
while (cv::waitKey(1) != 27) {           // ESC to quit
    if (cam.getVideoFrame(frame, 1000))
        cv::imshow("Video", frame);
    else
        std::cerr << "Timeout" << std::endl;
}

cam.stopVideo();
cv::destroyAllWindows();
```

### Recording to file

```cpp
lccv::Camera cam;
cam.options->video_width  = 1920;
cam.options->video_height = 1080;
cam.options->framerate    = 25;

cam.startVideo();

cv::VideoWriter writer("out.avi",
    cv::VideoWriter::fourcc('M','J','P','G'),
    cam.options->framerate,
    cv::Size(cam.options->video_width, cam.options->video_height));

cv::Mat frame;
for (int i = 0; i < 250; i++) {        // record 10 seconds at 25 fps
    if (cam.getVideoFrame(frame, 1000))
        writer.write(frame);
}

cam.stopVideo();
```

### Applying a filter pipeline

`getVideoFrame()` returns the frame by value — modify it freely before displaying or saving.

```cpp
cam.startVideo();
cv::Mat frame, edges;

while (cv::waitKey(1) != 27) {
    if (cam.getVideoFrame(frame, 1000)) {
        cv::Canny(frame, edges, 50, 150);
        cv::imshow("Edges", edges);
    }
}
cam.stopVideo();
```

---

## Zoom and pan

### Static zoom

Set before starting any mode:

```cpp
lccv::Camera cam;
cam.options->video_width  = 1280;
cam.options->video_height = 720;
cam.options->zoom  = 2.0f;   // 2× zoom
cam.options->pan_x = 0.5f;   // centred horizontally
cam.options->pan_y = 0.3f;   // slightly above centre

cam.startVideo();
// ...
```

### Smooth zoom-in while streaming

Update `zoom` at any time — the dispatcher applies it on the next frame automatically:

```cpp
cam.startVideo();
cv::Mat frame;

for (float z = 1.0f; z <= 4.0f; z += 0.02f) {
    cam.options->zoom = z;
    if (cam.getVideoFrame(frame, 100))
        cv::imshow("Zoom", frame);
    cv::waitKey(1);
}
cam.stopVideo();
```

### Pan across the image

```cpp
cam.startVideo();
cam.options->zoom = 3.0f;

cv::Mat frame;
for (float p = 0.0f; p <= 1.0f; p += 0.005f) {
    cam.options->pan_x = p;   // pan left → right
    if (cam.getVideoFrame(frame, 100))
        cv::imshow("Pan", frame);
    cv::waitKey(1);
}
cam.stopVideo();
```

---

## Viewfinder

### Viewfinder-only (live preview)

```cpp
lccv::Camera cam;
cam.options->viewfinder_width  = 640;
cam.options->viewfinder_height = 480;
cam.options->framerate = 30;

cv::namedWindow("Preview", cv::WINDOW_NORMAL);

cam.startViewfinder([](cv::Mat &frame) {
    cv::imshow("Preview", frame);
    cv::waitKey(1);  // needed to update the OpenCV window
});

cv::waitKey(0);   // block until any key
cam.stopViewfinder();
cv::destroyAllWindows();
```

### Viewfinder alongside photo capture

The preview runs continuously while `capturePhoto()` captures full-resolution stills.

```cpp
lccv::Camera cam;
cam.options->photo_width  = 4056;  cam.options->photo_height = 3040;
cam.options->viewfinder_width = 640; cam.options->viewfinder_height = 480;

cam.startPhoto();
cam.startViewfinder([](cv::Mat &frame) {
    cv::imshow("Preview", frame);
});

int photoIdx = 0;
while (true) {
    int key = cv::waitKey(10);
    if (key == 27) break;                // ESC → quit
    if (key == ' ') {                    // SPACE → capture
        cv::Mat image;
        if (cam.capturePhoto(image)) {
            cv::imwrite("photo_" + std::to_string(photoIdx++) + ".jpg", image);
            std::cout << "Captured" << std::endl;
        }
    }
}

cam.stopViewfinder();
cam.stopPhoto();
cv::destroyAllWindows();
```

### Viewfinder alongside video (shared stream)

When video and viewfinder are both active, they share the same camera stream — no extra overhead.

```cpp
lccv::Camera cam;
cam.options->video_width  = 1280; cam.options->video_height = 720;
cam.options->framerate = 30;

cam.startVideo();
cam.startViewfinder([](cv::Mat &frame) {
    // This callback fires on every video frame
    cv::imshow("Preview", frame);
    cv::waitKey(1);
});

// Meanwhile, getVideoFrame() delivers the same frames for processing
cv::Mat frame;
cv::VideoWriter writer("output.avi",
    cv::VideoWriter::fourcc('M','J','P','G'), 30,
    cv::Size(1280, 720));

while (cv::waitKey(1) != 27) {
    if (cam.getVideoFrame(frame, 100))
        writer.write(frame);
}

cam.stopViewfinder();
cam.stopVideo();
```

### Processing in the viewfinder callback

The callback receives a `cv::Mat` by reference. You can process it in-place or read it — but do not retain a reference after the callback returns.

```cpp
std::atomic<double> last_brightness{0.0};

cam.startViewfinder([&](cv::Mat &frame) {
    // Compute mean brightness and display
    cv::Scalar mean = cv::mean(frame);
    last_brightness.store((mean[0] + mean[1] + mean[2]) / 3.0);

    // Draw brightness on frame (in-place modification is fine)
    cv::putText(frame,
        "Brightness: " + std::to_string((int)last_brightness.load()),
        {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);

    cv::imshow("Preview", frame);
    cv::waitKey(1);
});
```

---

## Multiple cameras

Use `options->camera` to select which physical camera to open. Create one `Camera` instance per camera for simultaneous use.

```cpp
lccv::Camera cam0, cam1;
cam0.options->camera = 0;
cam1.options->camera = 1;

cam0.startVideo();
cam1.startVideo();

cv::Mat frame0, frame1;
while (cv::waitKey(1) != 27) {
    if (cam0.getVideoFrame(frame0, 100)) cv::imshow("Camera 0", frame0);
    if (cam1.getVideoFrame(frame1, 100)) cv::imshow("Camera 1", frame1);
}

cam0.stopVideo();
cam1.stopVideo();
```

---

## Raw Bayer output

The `BAYER` pixel format delivers the raw sensor data as a single-channel `CV_16UC1` matrix. This is useful for custom demosaicing algorithms or HDR processing.

```cpp
lccv::Camera cam;
cam.options->photo_width  = 4056;
cam.options->photo_height = 3040;
cam.options->format       = lccv::PixelFormat::BAYER;

cv::Mat raw;
cam.capturePhoto(raw);     // raw.type() == CV_16UC1

// Demosaic with OpenCV (adjust BayerBG/RG/GB/GR to match your sensor)
cv::Mat color;
cv::cvtColor(raw, color, cv::COLOR_BayerBG2BGR);
cv::imwrite("demosaiced.jpg", color);
```

---

## Verbose logging

Set `verbose = true` to see lifecycle events on `stderr`:

```cpp
cam.options->verbose = true;
cam.startVideo();
// → "Opening camera..."
// → "Acquired camera /base/soc/i2c0mux/..."
// → "Configuring viewfinder..."
// → "Camera streams configured"
// → "Buffers allocated and mapped"
// → "Camera started!"
```
