#include <iostream>

#include "markerDetection.cpp"
#include "poseDetection.cpp"
#include "poseDetectionRerunLogging.cpp"
#include "LogPoseAprilTagsVid.cpp"
#include "VSLAM.h"


int main() {
    std::cout << "Hello, World!" << std::endl;

    VSLAM s{};


    std::cout << "END" << std::endl;
    return 0;
}
