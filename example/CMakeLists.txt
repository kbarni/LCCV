cmake_minimum_required(VERSION 3.13)
project(lccv_examples)

set(CMAKE_CXX_STANDARD 17)

# Build the in-tree library as a subproject so changes to lccv pick up here
# without a `sudo make install` round-trip. liblccv propagates its include
# dirs (lccv, libcamera, OpenCV) and link libraries publicly.
if(NOT TARGET liblccv)
    add_subdirectory(.. lccv_build)
endif()

add_executable(takephoto takephoto.cpp)
add_executable(takevideo takevideo.cpp)

target_link_libraries(takephoto PRIVATE liblccv)
target_link_libraries(takevideo PRIVATE liblccv)
