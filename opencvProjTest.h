//
// Created by zakareeyah on 8/7/24.
//

#ifndef OPENCVPROJTEST_H
#define OPENCVPROJTEST_H
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

#include "rerunLogger.h"

class opencvProjTest {
public:
    cv::Mat intrinsicsMatrix = (cv::Mat_<double>(3, 3) <<
       1025.26513671875, 0.0, 642.6650390625,
       0.0, 1025.26513671875, 359.37811279296875,
       0, 0, 1);
    /*
    cv::Mat intrinsicsMatrix = (cv::Mat_<double>(3, 3) <<
    3075.7952, 0, 1927.995,
     0, 3075.7952, 1078.1343,
     0, 0, 1);
     */

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 14) << 2.0888574, -82.303825,
        -0.00071347022, 0.0020022474, 315.66144, 1.8588818,
        -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);

    float markerLength = 0.15;
    cv::Mat objPoints;




    opencvProjTest() {
        objPoints = cv::Mat(4, 1, CV_32FC3);
        // x down, y across, z up (to facilitate placing world in corner)
        /*objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0, 0, 0); // top left
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(0, markerLength, 0); // top right
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength, markerLength, 0); // bottom right
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(markerLength, 0, 0); // bottom left*/
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
    }

    void TestOneImage();
};



#endif //OPENCVPROJTEST_H
