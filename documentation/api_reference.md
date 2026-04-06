# API Reference

All public symbols are in the `lccv` namespace. Include `<lccv.hpp>` — no other header is needed.

---

## Enums

### `lccv::PixelFormat`

Controls the pixel format of images delivered by the library.

| Value | Description |
|-------|-------------|
| `BGR` | 8-bit per channel, BGR order — OpenCV native format. **Default.** |
| `RGB` | 8-bit per channel, RGB order |
| `GRAYSCALE` | 8-bit single channel, converted from the colour stream |
| `BAYER` | Raw Bayer data from the sensor's raw stream, delivered as `CV_16UC1`. The user is responsible for demosaicing. |

Set via `cam.options->format`:

```cpp
cam.options->format = lccv::PixelFormat::GRAYSCALE;
```

---

### `lccv::Metering`

Controls the AE metering mode.

| Value | Description |
|-------|-------------|
| `CENTRE` | Centre-weighted metering |
| `SPOT` | Spot metering |
| `MATRIX` | Matrix / evaluative metering. **Default.** |
| `CUSTOM` | Custom metering |

```cpp
cam.options->setMetering(lccv::Metering::SPOT);
```

---

### `lccv::Exposure`

Controls the AE exposure mode.

| Value | Description |
|-------|-------------|
| `NORMAL` | Normal auto-exposure. **Default.** |
| `SHORT` | Prefer shorter exposures |
| `CUSTOM` | Custom exposure mode |

```cpp
cam.options->setExposureMode(lccv::Exposure::SHORT);
```

---

### `lccv::WhiteBalance`

Controls the AWB mode.

| Value | Description |
|-------|-------------|
| `AUTO` | Automatic white balance. **Default.** |
| `INCANDESCENT` | Incandescent / tungsten-A lighting |
| `TUNGSTEN` | Tungsten lighting |
| `FLUORESCENT` | Fluorescent lighting |
| `INDOOR` | General indoor lighting |
| `DAYLIGHT` | Daylight |
| `CLOUDY` | Overcast / cloudy |
| `CUSTOM` | Manual gains via `awb_gain_r` / `awb_gain_b` |

```cpp
cam.options->setWhiteBalance(lccv::WhiteBalance::DAYLIGHT);

// Manual gains (use with WhiteBalance::CUSTOM)
cam.options->setWhiteBalance(lccv::WhiteBalance::CUSTOM);
cam.options->awb_gain_r = 2.0f;
cam.options->awb_gain_b = 1.5f;
```

---

## `lccv::Options`

Plain data class holding all camera parameters. Accessible as `cam.options`.

Options take effect when a mode is started. Exceptions:
- `zoom`, `pan_x`, `pan_y` — applied automatically on the next captured frame (no restart needed).
- `verbose` — takes effect immediately for any subsequent log output.

### Resolution

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `photo_width` | `unsigned int` | 4056 | Still capture width in pixels |
| `photo_height` | `unsigned int` | 3040 | Still capture height in pixels |
| `video_width` | `unsigned int` | 1280 | Video stream width |
| `video_height` | `unsigned int` | 720 | Video stream height |
| `viewfinder_width` | `unsigned int` | 640 | Viewfinder stream width |
| `viewfinder_height` | `unsigned int` | 480 | Viewfinder stream height |

The camera driver may adjust these to the nearest supported resolution.

### Capture

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `framerate` | `float` | 30 | Target frames per second (video/viewfinder) |
| `format` | `PixelFormat` | `BGR` | Output pixel format |
| `verbose` | `bool` | false | Log lifecycle events to `stderr` |
| `timeout` | `uint64_t` | 1000 | Internal timeout in ms |
| `denoise` | `std::string` | `"auto"` | Noise reduction: `"auto"`, `"off"`, `"cdn_off"`, `"cdn_fast"`, `"cdn_hq"` |
| `camera` | `unsigned int` | 0 | Camera index when multiple cameras are connected |

### Exposure and image

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `shutter` | `float` | 0 | Exposure time in microseconds. 0 = auto |
| `gain` | `float` | 0 | Analogue gain. 0 = auto |
| `ev` | `float` | 0 | Exposure value compensation (EV steps) |
| `brightness` | `float` | 0 | Image brightness, −1.0 to 1.0 |
| `contrast` | `float` | 1.0 | Image contrast multiplier |
| `saturation` | `float` | 1.0 | Colour saturation multiplier |
| `sharpness` | `float` | 1.0 | Sharpness multiplier |
| `awb_gain_r` | `float` | 0 | Red channel gain for manual AWB |
| `awb_gain_b` | `float` | 0 | Blue channel gain for manual AWB |

### Zoom and pan

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `zoom` | `float` | 1.0 | Zoom factor. 1.0 = full sensor, 2.0 = 2× zoom. Must be ≥ 1.0 |
| `pan_x` | `float` | 0.5 | Horizontal centre of the crop window, 0.0 (left) to 1.0 (right) |
| `pan_y` | `float` | 0.5 | Vertical centre of the crop window, 0.0 (top) to 1.0 (bottom) |

Zoom and pan are translated to a libcamera `ScalerCrop` control. When a dispatcher thread is running (video or viewfinder active), changes to these fields are detected automatically on each frame and applied without restarting the camera. In photo-only mode, the zoom is applied when the camera is started.

```cpp
// Smooth zoom-in while video is running
cam.startVideo();
for (float z = 1.0f; z <= 4.0f; z += 0.02f) {
    cam.options->zoom = z;
    cv::Mat frame;
    cam.getVideoFrame(frame, 100);
    cv::imshow("Zoom", frame);
    cv::waitKey(1);
}
```

### Image transform

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `transform` | `libcamera::Transform` | `Identity` | Image flip/rotation transform |

```cpp
#include <libcamera/transform.h>
cam.options->transform = libcamera::Transform::HFlip;
```

### Mode setters/getters

```cpp
void setMetering(Metering m);
void setExposureMode(Exposure e);
void setWhiteBalance(WhiteBalance w);

int getMeteringMode() const;  // returns raw libcamera enum value
int getExposureMode() const;
int getWhiteBalance() const;
```

### `Options::print()`

```cpp
cam.options->print();   // logs all option values to stderr
```

---

## `lccv::Camera`

### Constructor / destructor

```cpp
lccv::Camera cam;
```

No arguments. Default options are applied. The camera hardware is **not** opened at construction time. The destructor cleanly stops any active mode and releases all resources.

---

### Photo mode

#### `bool startPhoto()`

Opens the camera and configures the still stream. Must be called before a series of `capturePhoto()` calls to avoid the open/configure overhead on each capture.

Returns `false` if:
- Photo mode is already active
- Video mode is active (photo and video are mutually exclusive)
- The camera cannot be opened or configured (throws `std::runtime_error` for hardware errors)

If viewfinder mode is already active, `startPhoto()` reconfigures the camera to a combined still+viewfinder session transparently.

#### `bool capturePhoto(cv::Mat &frame)`

Captures one still image into `frame`.

- If called after `startPhoto()`: reuses the open camera session.
- If called without prior `startPhoto()`: opens, captures, and closes in one call (convenient for single shots, but slower).
- If a viewfinder is active: the viewfinder callback continues to be called during the capture; `capturePhoto()` blocks until the next full-resolution frame is ready (up to 5 seconds).

Returns `false` on timeout or hardware error.

#### `bool stopPhoto()`

Stops photo mode and releases the camera (unless viewfinder is still active, in which case the camera is reconfigured to viewfinder-only). Returns `false` if photo mode was not active.

---

### Video mode

#### `bool startVideo()`

Starts the video stream. Launches an internal dispatcher thread that continuously delivers frames.

Returns `false` if:
- Video mode is already active
- Photo mode is active

If viewfinder mode is already active, `startVideo()` shares the viewfinder stream — no extra stream or reconfiguration needed.

#### `bool getVideoFrame(cv::Mat &frame, unsigned int timeout_ms)`

Blocks until a new frame is available or `timeout_ms` milliseconds elapse.

- Returns `true` and writes the frame to `frame` if a new frame arrived in time.
- Returns `false` on timeout or if video is not running.
- Each frame is delivered at most once; consecutive calls wait for the next new frame.

#### `void stopVideo()`

Stops the video stream. If viewfinder is still active, the dispatcher continues running for the viewfinder; the camera is not closed.

---

### Viewfinder mode

#### `bool startViewfinder(std::function<void(cv::Mat &)> callback)`

Starts a continuous preview stream and calls `callback` on each new frame from a background thread.

| Active mode at call time | Behaviour |
|--------------------------|-----------|
| Nothing | Opens camera with viewfinder stream; starts dispatcher |
| Photo | Reconfigures to still+viewfinder; dispatcher delivers both |
| Video | Shares the video stream; no extra stream or overhead |

Returns `false` if a viewfinder is already active.

**Callback contract:**
- Called from the dispatcher thread — not the main thread.
- The `cv::Mat &frame` is valid only for the duration of the call; do not store a reference to it.
- Must return promptly. If it takes longer than one frame period, frames are dropped (not queued).
- Any exception thrown from the callback is caught, logged to `stderr`, and the stream continues.

#### `void stopViewfinder()`

Stops the viewfinder callback. Behaviour depends on what else is running:

| Other active mode | Behaviour |
|-------------------|-----------|
| Video | Dispatcher keeps running; video delivery continues |
| Photo | Reconfigures to still-only; dispatcher stops |
| Nothing | Camera stopped and closed |

---

### Zoom / pan

#### `void ApplyZoomOptions()`

Immediately applies the current `zoom`, `pan_x`, `pan_y` option values to the running camera as a `ScalerCrop` control. Useful in photo-only mode (no dispatcher) to update the crop between shots.

When a dispatcher is running (video or viewfinder active), zoom/pan changes are applied automatically — calling this method explicitly is not necessary.

---

### Valid mode combinations

| Photo | Video | Viewfinder | Supported |
|-------|-------|------------|-----------|
| ✓ | ✗ | ✗ | Photo-only |
| ✓ | ✗ | ✓ | Photo + viewfinder |
| ✗ | ✓ | ✗ | Video-only |
| ✗ | ✓ | ✓ | Video + viewfinder |
| ✗ | ✗ | ✓ | Viewfinder-only |
| ✓ | ✓ | any | **Not supported** |

---

### Thread safety

A single `Camera` instance is **not thread-safe**. Do not call public methods from multiple threads concurrently without external synchronisation.

The viewfinder callback and video dispatcher thread are managed internally. The callback is always invoked from the dispatcher thread, never the main thread.

---

### Error handling

- `startPhoto()`, `startVideo()`, `startViewfinder()` return `bool` (true = success).
- `capturePhoto()` and `getVideoFrame()` return `bool` (true = frame available).
- Fatal hardware errors (camera open failure, stream configuration failure) throw `std::runtime_error`.
- The camera is always returned to a clean state when a `Camera` object is destroyed, even if modes were left running.
