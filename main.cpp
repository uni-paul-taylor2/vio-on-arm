#include <iostream>

#include "markerDetection.cpp"
#include "poseDetection.cpp"
#include "poseDetectionRerunLogging.cpp"
#include "LogPoseAprilTagsVid.cpp"

#include "VSLAM.h"
#include "slamProto.cpp"
#include "GTSAMviSAMExample2.cpp"
#include "selectiveDataTest.cpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    // prototype::slam();
    // detectPoseVid();
    /*
     * Defaults:
     *  8 (camera) poses
     *  8 landmarks
     *
     * Each landmark NEEDS to be seen at least TWICE before putting into ISAM else gtsam::IndeterminantLinearSystemException thrown near landmark variable
     *      i.e.: initial estimate and at least 2 projection factors
     * Above implies at least 2 poses are needed (with the same landmarks)
     * At least 5 landmarks are needed else gtsam::IndeterminantLinearSystemException thrown near pose variable
     * Increasing the number poses to 20 also worked (safe to assume on upper limit on poses?)
     */
    adj(20, -1);

    // selective_data::createPoints();
    // selective_data::createPoses();
    std::cout << "====================================SLAM+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    // selective_data::slam();

    std::cout << "END" << std::endl;
    return 0;
}
