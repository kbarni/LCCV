# Examples are built as part of the main project when BUILD_EXAMPLES=ON.
# They can also be built standalone after installing the library:
#   cmake -Dlccv_DIR=<install_prefix>/lib/cmake/lccv ..

cmake_minimum_required(VERSION 3.16)

if(NOT TARGET liblccv)
    # Standalone build — find the installed library
    project(lccv_examples CXX)
    find_package(lccv REQUIRED)
    set(LCCV_TARGET lccv::liblccv)
else()
    # Part of the main build — use the build-tree target directly
    set(LCCV_TARGET liblccv)
endif()

find_package(OpenCV REQUIRED)

foreach(example takephoto takevideo viewfinder)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${example}.cpp)
        add_executable(${example} ${example}.cpp)
        target_link_libraries(${example} PRIVATE ${LCCV_TARGET} ${OpenCV_LIBS})
    endif()
endforeach()
