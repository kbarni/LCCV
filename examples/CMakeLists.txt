cmake_minimum_required(VERSION 3.13)
project(lccv_examples)

set(CMAKE_CXX_STANDARD 20)

# Build the in-tree library as a subproject so changes to lccv pick up here
# without a `sudo make install` round-trip. liblccv propagates its include
# dirs (lccv, libcamera, OpenCV) and link libraries publicly.
if(NOT TARGET liblccv)
    add_subdirectory(.. lccv_build)
endif()

add_executable(takephoto takephoto.cpp)
add_executable(takevideo takevideo.cpp)
add_executable(test_photo test_photo.cpp)
add_executable(test_video test_video.cpp)
add_executable(test_sizes test_sizes.cpp)
add_executable(test_viewfinder test_viewfinder.cpp)
add_executable(viewfinder_photo viewfinder_photo.cpp)
add_executable(viewfinder_video viewfinder_video.cpp)

target_link_libraries(takephoto PRIVATE liblccv)
target_link_libraries(takevideo PRIVATE liblccv)
target_link_libraries(test_photo PRIVATE liblccv)
target_link_libraries(test_video PRIVATE liblccv)
target_link_libraries(test_sizes PRIVATE liblccv)
target_link_libraries(test_viewfinder PRIVATE liblccv)
target_link_libraries(viewfinder_photo PRIVATE liblccv)
target_link_libraries(viewfinder_video PRIVATE liblccv)

# Qt smartphone demo — only built when Qt Widgets is available
find_package(Qt6 QUIET COMPONENTS Widgets)
if(NOT Qt6_FOUND)
    find_package(Qt5 QUIET COMPONENTS Widgets)
endif()
if(Qt6_FOUND OR Qt5_FOUND)
    add_subdirectory(qt_smartphone)
else()
    message(STATUS "qt_smartphone: skipped (Qt Widgets not found)")
endif()
