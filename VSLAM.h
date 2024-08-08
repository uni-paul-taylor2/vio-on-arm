#ifndef SLAM_H
#define SLAM_H

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>


#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


/**
 * Pose detection and vSLAM from video feed
 * Pose detection:
 *      tag family
 *      camera intrinsics
 *      distortion coeffecients
 *      video
 * SLAM (GTSAM)
 *      ?
 *
 * Process:
 *      name of dir passed as cmd arg
 *      info.json found and parsed
 *      VSLAM obj created and passed the camInts, distCoeffs, tag family
 *      each frame of the vid asscced, transformed to cv::Mat/cvFrame? and passed to VSLAM obj
 *      for each frame passed, VLSAM obj:
 *          detects markers
 *          sets world (if first) or check if at least one of the tags were seen before
 *          detects pose of tags
 *          for redetected tags:
 *              passes (u,v) to factor graph
 *          for new tags:
 *              calcs pose wrt to world and passes to factor graph
 *          calcs cameara pose wrt world and passes to factor graph
 *          updates factor graph (perhaps more than once?)
 *
 *
 *
 *  (REMEMBER: data validation)
 *
 */

class VSLAM {
public:
    /**
     * @param tag: aruco tag used
     * @param cameraIntrinsics
     * @param distCoeffs
     * ...?
     */
    cv::Mat intrinsicsMatrix = (cv::Mat_<double>(3, 3) <<
        1025.26513671875, 0.0, 642.6650390625,
        0.0, 1025.26513671875, 359.37811279296875,
        0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 14) << 2.0888574, -82.303825,
        -0.00071347022, 0.0020022474, 315.66144, 1.8588818,
        -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);

    float markerLength = 0.15;
    cv::Mat objPoints;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    gtsam::Values currentEstimate; // for LM batch opt

    std::map<int, gtsam::Pose3> observedTags;

    int currentPose;
    int currentFrame;



    VSLAM() {
        objPoints = cv::Mat(4, 1, CV_32FC3);

        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0); // top left
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0); // top right
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0); // bottom right
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0); // bottom left


    };


};


#endif //SLAM_H
