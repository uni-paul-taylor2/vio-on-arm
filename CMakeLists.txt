cmake_minimum_required(VERSION 3.28)
project(Dev)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR /home/zakareeyah/opencv/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Dev main.cpp)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)


target_link_libraries(Dev ${OpenCV_LIBS} rerun_sdk)