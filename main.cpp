#include <iostream>

#include "markerDetection.cpp"
#include "poseDetection.cpp"
#include "poseDetectionRerunLogging.cpp"
#include "LogPoseAprilTagsVid.cpp"


int main()
{

    std::cout << "Hello, World!" << std::endl;
    detectPose();

    return 0;
}
