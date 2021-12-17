project(takephoto)
cmake_minimum_required(VERSION 3.0.0)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
add_executable(takephoto takephoto.cpp)
add_executable(takevideo takevideo.cpp)

target_link_libraries(takephoto -llccv ${OpenCV_LIBS})
target_link_libraries(takevideo -llccv ${OpenCV_LIBS})
