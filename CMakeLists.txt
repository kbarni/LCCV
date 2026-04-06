cmake_minimum_required(VERSION 3.16)

project(lccv VERSION 1.0.0 LANGUAGES CXX)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
        message(STATUS "No previous build - default to Release build")
    endif()
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -pedantic -Wno-unused-parameter -faligned-new -Werror -Wfatal-errors)
add_definitions(-D_FILE_OFFSET_BITS=64)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wno-psabi)
endif()

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
option(BUILD_EXAMPLES    "Build example programs"       OFF)

IF(NOT ENABLE_COMPILE_FLAGS_FOR_TARGET)
    # On a Pi this will give us armhf or arm64.
    execute_process(COMMAND dpkg-architecture -qDEB_HOST_ARCH
        OUTPUT_VARIABLE ENABLE_COMPILE_FLAGS_FOR_TARGET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
message(STATUS "Platform: ${ENABLE_COMPILE_FLAGS_FOR_TARGET}")
if("${ENABLE_COMPILE_FLAGS_FOR_TARGET}" STREQUAL "arm64")
    add_definitions(-ftree-vectorize)
elseif("${ENABLE_COMPILE_FLAGS_FOR_TARGET}" STREQUAL "armv8-neon")
    add_definitions(-mfpu=neon-fp-armv8 -ftree-vectorize)
endif()

# Source package generation setup.
set(CPACK_GENERATOR "TXZ")
set(CPACK_PACKAGE_FILE_NAME "lccv-build")
set(CPACK_SOURCE_GENERATOR "TXZ")
set(CPACK_INSTALL_SCRIPTS ${CMAKE_SOURCE_DIR}/package.cmake)
set(CPACK_SOURCE_PACKAGE_FILE_NAME "lccv-src")
set(CPACK_SOURCE_IGNORE_FILES "/\\.git.*;/build;")
include(CPack)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(LIBCAMERA REQUIRED libcamera)
message(STATUS "libcamera library found:")
message(STATUS "    version: ${LIBCAMERA_VERSION}")
message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------

set(SOURCES
    src/lccv.cpp
    src/libcamera_app.cpp
)
set(HEADERS
    include/lccv.hpp
    include/libcamera_app.hpp
)

add_library(liblccv ${SOURCES} ${HEADERS})

target_include_directories(liblccv
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        ${LIBCAMERA_INCLUDE_DIRS}
)

# OpenCV include dirs via imported target if available, fallback to variable
if(TARGET opencv_core)
    target_link_libraries(liblccv PUBLIC opencv_core opencv_imgproc opencv_highgui)
else()
    target_include_directories(liblccv PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(liblccv PUBLIC ${OpenCV_LIBS})
endif()

target_link_libraries(liblccv PUBLIC ${LIBCAMERA_LINK_LIBRARIES})

set_target_properties(liblccv PROPERTIES
    PREFIX       ""
    IMPORT_PREFIX ""
    VERSION      ${PROJECT_VERSION}
    SOVERSION    1
)

# ---------------------------------------------------------------------------
# Examples (optional)
# ---------------------------------------------------------------------------

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ---------------------------------------------------------------------------
# Install + CMake package config
# ---------------------------------------------------------------------------

install(TARGETS liblccv
    EXPORT  lccvTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES ${HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Generate and install lccvTargets.cmake
install(EXPORT lccvTargets
    FILE        lccvTargets.cmake
    NAMESPACE   lccv::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lccv
)

# Generate lccvConfig.cmake and lccvConfigVersion.cmake
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/lccvConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/lccvConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lccv
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/lccvConfigVersion.cmake
    VERSION      ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/lccvConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/lccvConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lccv
)
