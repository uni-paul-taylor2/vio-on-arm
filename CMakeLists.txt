cmake_minimum_required(VERSION 3.28)
project(Dev)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_LIBRARIES "-lstdc++fs") #https://github.com/borglab/gtsam/issues/1495
add_compile_definitions("SOURCE_ROOT=${CMAKE_SOURCE_DIR}")

# Find Packages
find_package(kimera_vio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost 1.65.0 REQUIRED system filesystem serialization REQUIRED)

# Include Directories
include_directories(${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${GTSAM_INCLUDE_DIRS})

add_executable(Dev main.cpp
        rerunLogger.h
        opencvProjTest.h
        opencvProjTest.cpp)

add_executable(gtsamExample GTSAMVisualISAMExample.cpp )

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        #https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip) # for the latest version of rerun
        https://github.com/rerun-io/rerun/releases/download/0.17.0/rerun_cpp_sdk.zip) # for rerun 0.17
FetchContent_MakeAvailable(rerun_sdk)


target_link_libraries(Dev ${OpenCV_LIBRARIES} rerun_sdk ${Boost_LIBRARIES} ${GTSAM_LIBRARIES} metis-gtsam gtsam gtsam_unstable)

target_link_libraries(gtsamExample ${Boost_LIBRARIES} ${GTSAM_LIBRARIES} metis-gtsam gtsam gtsam_unstable)
