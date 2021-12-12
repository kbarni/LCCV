LibCamera bindings for OpenCV
=============================

LCCV (*LibCamera bindings for OpenCV*) is a small wrapper library that provides access to the Raspberry Pi camera in OpenCV.

Context
-------

In Raspbian Bullseye, the Raspberry Pi camera framework was completely rebased from MMAL to the LibCamera library - thus breaking most of the previous camera dependencies.

Raspbian comes with the handy `libcamera-apps` package that duplicates the old `raspistill` and `raspivid` applications, with some added functionnality, like the possibility of adding postprocessing routines to the capturing process.

However this is still limited, as it doesn't allow full integration of the camera in your software.

LCCV aims to provide a simple to use wrapper library that allows you to access the camera from a C++ program and capture images in cv::Mat format.

Features and limitations
------------------------

LCCV is heavily based on Raspbian's `libcamera-apps` source code. It is aimed to offer full control over the camera, so the original configuration class was kept instead of a new one based on OpenCV's VideoCapture class. Note that only the camera parameters are available, other parameters and functions, like previewing, cropping and post-processing were stripped from the library.

This first version gives access to the high resolution photo mode; continuous acquisition (video mode) will follow soon.

This is a very early version, so expect to have some bugs.

Prerequisites
-------------

- Raspbian Bullseye
- libcamera (with development packages)
- OpenCV (with development packages)
- cmake

    sudo apt install build-essential cmake git libcamera-dev libopencv-dev

Building and installing
-----------------------

	git clone https://github.com/kbarni/LCCV.git
	cd LCCV
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    
Using the library
-----------------

### Still image capture

First declare the camera:

    lccv::PiCamera cam;
    
You can set the camera parameters in the `cam.options` class. See the `libcamera_app_options.hpp` file for the list of the available parameters.

Capture the image using the `cam.captureFrame(cv::Mat &frame)` function.

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).
