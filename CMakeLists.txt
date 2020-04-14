# camera-v4l2/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(camera-v4l2 VERSION 0.1.0
        DESCRIPTION "Camera V4L2 library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_sources(${PROJECT_NAME}
    PUBLIC
        camera-v4l2/camera.h
    PRIVATE
        camera-v4l2/viewer.cpp
        camera-v4l2/grab.cpp
        camera-v4l2/camera.cpp
)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_include_directories(${PROJECT_NAME} PUBLIC ./)

