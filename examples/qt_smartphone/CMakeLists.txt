cmake_minimum_required(VERSION 3.16)

# Can be built standalone after installing the library:
#   cmake -Dlccv_DIR=<install_prefix>/lib/cmake/lccv ..
#
# Or as part of the main project (via examples/CMakeLists.txt).

if(NOT TARGET liblccv)
    project(qt_smartphone CXX)
    find_package(lccv REQUIRED)
    set(LCCV_TARGET lccv::liblccv)
else()
    set(LCCV_TARGET liblccv)
endif()

find_package(OpenCV REQUIRED)

# Try Qt6 first, fall back to Qt5
find_package(Qt6 QUIET COMPONENTS Widgets)
if(Qt6_FOUND)
    set(QT_TARGET Qt6::Widgets)
    message(STATUS "qt_smartphone: using Qt6")
else()
    find_package(Qt5 REQUIRED COMPONENTS Widgets)
    set(QT_TARGET Qt5::Widgets)
    message(STATUS "qt_smartphone: using Qt5")
endif()

set(CMAKE_AUTOMOC ON)

add_executable(qt_smartphone
    main.cpp
    mainwindow.cpp
    settingsdialog.cpp
)

target_link_libraries(qt_smartphone PRIVATE
    ${LCCV_TARGET}
    ${OpenCV_LIBS}
    ${QT_TARGET}
)
