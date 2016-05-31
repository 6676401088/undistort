cmake_minimum_required(VERSION 2.8)
project(undistort)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES main.cpp)
find_package(OpenCV REQUIRED)

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        .
)
add_executable(undistort ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(undistort ${OpenCV_LIBS})