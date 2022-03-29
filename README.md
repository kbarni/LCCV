libcamera bindings for OpenCV
=============================

LCCV (*libcamera bindings for OpenCV*) is a small wrapper library that provides access to the Raspberry Pi camera in OpenCV.

### WARNING: 

This is still an early version of the project, so expect to have some bugs.

Please help with the development by reporting the bugs and issues you encounter, committing bugfixes, and proposing ideas!

Context
-------

In Raspbian Bullseye, the Raspberry Pi camera framework was completely rebased from MMAL to the libcamera library - thus breaking most of the previous camera dependencies.

Raspbian comes with the handy `libcamera-apps` package that duplicates the old `raspistill` and `raspivid` applications, with some added functionnality, like the possibility of adding postprocessing routines to the capturing process.

However this is still limited, as it doesn't allow full integration of the camera in your software.

LCCV aims to provide a simple to use wrapper library that allows you to access the camera from a C++ program and capture images in cv::Mat format.

Features and limitations
------------------------

LCCV is heavily based on Raspbian's `libcamera-apps` source code. It is aimed to offer full control over the camera, so the original options class was kept instead of a new one based on OpenCV's VideoCapture class. Note that only the camera parameters are available, other parameters and functions, like previewing, cropping and post-processing were stripped from the library.

Prerequisites
-------------

- Raspbian Bullseye
- Development libraries (gcc/clang, cmake, git)
- libcamera (with development packages)
- OpenCV (with development packages)

Install everything using the following command:

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

Please refer to the [wiki](https://github.com/kbarni/LCCV/wiki)

Also see some example code in the `examples` folder.

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).
