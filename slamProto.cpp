#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Cal3DS2.h>

#include <vector>

void slam() {
    // may have to read in as float (check if OpenCV needs it as floats)
    // consider reading in as vectors instead of Mat
    // https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#mat-mat
    cv::Mat intrinsicsMatrix = (cv::Mat_<double>(3,3) <<
       3075.7952, 0, 1927.995,
       0, 3075.7952, 1078.1343,
       0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 14) << 2.0888574, -82.303825, -0.00071347022, 0.0020022474, 315.66144,
        1.8588818, -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);


    auto cam = intrinsicsMatrix.ptr(0);


    // Define the camera calibration parameters
    // TODO: find correct model for our camera.
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3DS2(
        intrinsicsMatrix.at<double>(0,0),
        intrinsicsMatrix.at<double>(1,2),
        0.0,
        intrinsicsMatrix.at<double>(0,2),
        intrinsicsMatrix.at<double>(1, 2),
        distCoeffs.at<double>(0),
        distCoeffs.at<double>(1),
        distCoeffs.at<double>(2),
        distCoeffs.at<double>(3)
        )
    );

    // Define the camera observation noise model (will leave the same for now).
    // TODO: Also try with gaussian noise.
    auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v



}


/*
* *      f_x   0    c_x
 *      0     f_y  c_y
 *      0     0    1
 *
 */