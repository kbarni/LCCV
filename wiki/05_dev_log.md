# Development Log

---

## Phase 1 — Cleanup

### T01 — Rename example/ → examples/
**Files:** `example/` → `examples/`, `README.md`  
**Result:** Build successful. Directory renamed; README updated.  
**Commit:** a726631

### T05 — Fix hard-coded libcamera include path
**Files:** `src/lccv.cpp`  
**Note:** This was blocking the build — `#include <libcamera/libcamera/stream.h>` caused a `StreamFormats` redefinition error. Removed the include; type is already available via `libcamera_app.hpp`.  
**Result:** Build successful.  
**Commit:** a4ee881

### T02 — Remove unused FrameInfo/Metadata
**Files:** `include/libcamera_app.hpp`  
**Result:** Removed 180 lines of dead code copied from libcamera-apps. Build successful.  
**Commit:** 69f3253

### T03 — Clean up Options class
**Files:** `include/libcamera_app_options.hpp`, `src/libcamera_app_options.cpp`, `src/libcamera_app.cpp`  
**Changes:** Removed CLI-only fields (`help`, `version`, `list_cameras`, `info_text`, `rawfull`). Fixed WB_CUSTOM → `AwbCustom`. Initialized all fields in constructor. Removed `!options_->help` guards in `libcamera_app.cpp`.  
**Result:** Build successful.  
**Commit:** 3b4b10e

### T04 — Smart pointer cleanup
**Files:** `include/lccv.hpp`, `include/libcamera_app.hpp`, `src/lccv.cpp`, `src/libcamera_app.cpp`  
**Changes:** `allocator_` → `unique_ptr`. `PiCamera::app` → `unique_ptr`. `framebuffer` → `std::vector<uint8_t>`. Also fixed video thread: `break` on Quit/error instead of `throw` across pthread boundary.  
**Result:** Build successful.  
**Commit:** 0909a04

### T06 — Fix capturePhoto() bugs B1/B2/B3
**Files:** `src/lccv.cpp`  
**Changes:** `opened_here` flag tracks whether this call opened the camera; only tears down when it did. StopCamera() now called in all error paths. Removed dead code after `return false`.  
**Result:** Build successful. Manual test (camera required) needed for full verification.  
**Commit:** 6172210

---

## Phase 2 — Refactor existing modes

### T07 — std::thread + condition_variable for video

**Files:** `include/lccv.hpp`, `src/lccv.cpp`  
**Changes:** Replaced `pthread_t` + `static void*` trampoline with `std::thread`. Replaced 1 ms nanosleep busy-wait in `getVideoFrame()` with `condition_variable::wait_for`. Added double-buffer (`front_buffer_` / `back_buffer_`) swapped under mutex. `stopVideo()` notifies CV to unblock callers immediately.  
**Result:** Build successful. Manual test (camera required) needed.  
**Commit:** 8f8ee21

### T08 — Rename PiCamera→Camera, scoped enums, merge Options into lccv.hpp
**Files:** `include/lccv.hpp`, `include/libcamera_app.hpp`, `src/lccv.cpp`, `src/libcamera_app.cpp`, `examples/*.cpp`, `CMakeLists.txt`  
**Changes:** `PiCamera` → `lccv::Camera`. Old C-style enums replaced with `enum class lccv::Exposure/Metering/WhiteBalance`. `Options` class moved into `lccv.hpp`; `libcamera_app_options.hpp/.cpp` deleted. `LibcameraApp` updated to use `lccv::Options` via forward declaration + include. `pthread` removed from CMake link deps.  
**Result:** Build successful.  
**Commit:** c94a8f9

---

## Phase 3 — Options extensions

### T09 — Add zoom/pan/PixelFormat to Options; remove roi_*
**Files:** `include/lccv.hpp`, `src/lccv.cpp`, `src/libcamera_app.cpp`  
**Changes:** New `lccv::PixelFormat` enum. Options gains `zoom`, `pan_x`, `pan_y`, `viewfinder_width/height`, `format`. `roi_x/y/width/height` removed. Both `StartCamera()` and `ApplyRoiSettings()` in `libcamera_app.cpp` updated to compute ScalerCrop from zoom/pan.  
**Note:** Initialiser list ordering triggered `-Wreorder` error; fixed by reordering to match declaration order.  
**Result:** Build successful.  
**Commit:** 11f5809

---

## Phase 4 — LibcameraApp extensions

### T10 — ConfigureStillWithViewfinder() + ApplyZoom()
**Files:** `include/libcamera_app.hpp`, `src/libcamera_app.cpp`, `src/lccv.cpp`  
**Changes:** `ConfigureStillWithViewfinder()` configures StillCapture+Raw+Viewfinder in one session. `ApplyZoom(zoom, pan_x, pan_y)` replaces `ApplyRoiSettings()` with proper clamping to sensor bounds. `StartCamera()` delegates to `ApplyZoom()`. `Camera::ApplyZoomOptions()` updated.  
**Result:** Build successful.  
**Commit:** 6e45520

---

## Phase 5 — New features

### T11 / T12 / T13 — Dispatcher thread, viewfinder mode, PixelFormat conversion
**Files:** `include/lccv.hpp`, `src/lccv.cpp`, `src/libcamera_app.cpp`  
**Changes (T11):** Single `dispatcherThread()` owns `app_->Wait()` loop. Routes still frames to `still_pending_`+`still_cv_`, video/viewfinder frames to double-buffer+`frame_cv_` and viewfinder callback. Auto-detects zoom/pan changes per frame and calls `ApplyZoom()`. `capturePhoto()` waits on `still_cv_` when dispatcher is running. `reconfigure()` helper selects the right `Configure*` call.  
**Changes (T12):** `startViewfinder(callback)` / `stopViewfinder()`. Viewfinder-only (own stream+dispatcher), viewfinder+photo (ConfigureStillWithViewfinder), viewfinder+video (shares video dispatcher, zero extra overhead). Callback exceptions caught and logged.  
**Changes (T13):** `toMat()` handles BGR/RGB (memcpy), GRAYSCALE (cvtColor), BAYER (CV_16UC1). Configure methods select pixel format from `options->format`.  
**Tests:** Both examples compile. Manual test with camera required.  
**Commit:** 3695c21

---

## Phase 6 — Build system and examples

### T14 — CMake package config + BUILD_EXAMPLES
**Files:** `CMakeLists.txt`, `cmake/lccvConfig.cmake.in`, `examples/CMakeLists.txt`  
**Changes:** CMake minimum 3.16. Version 1.0.0 + SOVERSION 1. `BUILD_EXAMPLES` option. `find_package(lccv)` support via installed CMake package config files. Examples CMakeLists works both as part of main build and standalone.  
**Result:** Build successful; library is now `liblccv.so.1.0.0`.  
**Commit:** 7dc4252

### T15 — Viewfinder example
**Files:** `examples/viewfinder.cpp`  
**Changes:** Three modes via argv: viewfinder-only, viewfinder+photo (SPACE to capture), viewfinder+video (R to record). Compiles against the new library.  
**Result:** Compiles cleanly. Manual run required for full test.  
**Commit:** fb1c115


