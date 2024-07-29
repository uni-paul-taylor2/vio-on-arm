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


float markerLength = 0.15;

void getCorners(const cv::Mat& image, std::vector<int> ids, std::vector<std::vector<cv::Point2f> > corners) {
    cv::aruco::detectMarkers(image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), corners, ids);
}

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

    cv::Mat objPoints(4, 1, CV_32FC3);
    // x down, y across, z up (to facilitate placing world in corner)
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0,0, 0); // top left
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(0, markerLength, 0); // top right
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength, markerLength, 0); // bottom right
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(markerLength, 0, 0); // bottom left

    // Define the camera calibration parameters
    // [what i initially used that was causing error]
    gtsam::Cal3DS2::shared_ptr K(new gtsam::Cal3DS2(
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
    // [however declaring it as Cal3_S2 worked]
    // gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

    // [but when I changed the way I declare it to explicitly match what one of the constructors was looking for it worked]
    // [seems there is a bug? with this particular version of gtsam where Cal3DS2::shared_ptr resolves to boost::shared_ptr<gtsam::Cal3> instead of boost::shared_ptr<gtsam::Cal3DS2> like it should have]
    // [the commented out code below seems to work]
    /*const boost::shared_ptr<gtsam::Cal3DS2> K(new gtsam::Cal3DS2(  // Cal
            3075.7952,
            3075.7952,
            0.0,
            1927.995,
            1078.1343,
            2.0888574,
            -82.303825,
            -0.00071347022,
            0.0020022474
            )
    );*/

    // Define the camera observation noise model (will leave the same for now).
    // TODO: Also try with gaussian noise.
    auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

    // Create a NonlinearISAM object which will relinearize and reorder the variables
    // every "relinearizeInterval" updates
    constexpr int relinearizeInterval = 3;
    gtsam::NonlinearISAM isam(relinearizeInterval);

    // Create a Factor Graph and Values to hold the new data
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;

    // Loop over the different poses, adding the observations to iSAM incrementally
    // In our case - these are images from the camera
    constexpr int numImages = 171;
    for (size_t i = 0; i < numImages; ++i) { // looping through frames
        gtsam::Values currentEstimate = isam.estimate();

        // get (u,v) of tags in frame
        auto imgPath = std::string("/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image") + std::to_string(i) + std::string(".png");
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        getCorners(cv::imread(imgPath, cv::IMREAD_COLOR), ids, corners);
        // only using top left corner for now (point 0) of each tag; using all corners may be better
        if (!ids.empty()) {
            // if only one tag verify it's world ?
            // Add factors for each landmark observation
            // In our case - these are the apriltags - note - not all tags are visible in all frames.
            // this "j" loop is only for the tags seen in THIS current frame "i"
            for (size_t j = 0; j < ids.size(); ++j) { // looping through tags in frame
                // Create ground truth measurement
                // In our case, we have the "measurement" from the aruco detection (u, v).
                // Not sure if to use all 4 corners of the april tag or just 1 and how that will affect the optimization.
                gtsam::Point2 measurement = gtsam::Point2(corners[j][0].x, corners[j][0].y); // only using top left crnr for now
                // Add measurement
                graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> >  (measurement, noise,
                                                                                                                   gtsam::Symbol('x', i), gtsam::Symbol('l', j), K);
                // TODO: If apriltag has not been seen yet, then add it to the initial estimate and initialize using solvePnP
                if (!currentEstimate.exists(gtsam::Symbol('l', ids[j]))) {    //gtsam::Symbol(ids[i], [corner])
                    // get cam pose wrt world; get pts wrt world; add to graph (deal with x2 sighting later
                }

            }


            // If this is the first iteration, add a prior on the first pose to set the coordinate frame
            // and a prior on the first landmark to set the scale (and denote the first landmark as world)
            // Also, as iSAM solves incrementally, we must wait until each is observed at least twice ? before
            // adding it to iSAM.
            if (i == 0) {
                int worldTagID = ids[0]; // let the first tag detected be world
                // get cTw
                cv::Vec3d rvec, tvec;
                cv::solvePnP(objPoints, corners[0], intrinsicsMatrix, distCoeffs, rvec, tvec);
                cv::Mat rot; // 3x3 rot matrix for
                cv::Rodrigues(rvec, rot);

                gtsam::Pose3 cTw(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)), gtsam::Point3(tvec.val));



                /*// Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
                auto poseNoise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
                graph.addPrior(Symbol('x', 0), poses[0], poseNoise);

                // Add a prior on landmark l0
                auto pointNoise =
                    noiseModel::Isotropic::Sigma(3, 0.1);
                graph.addPrior(Symbol('l', 0), points[0], pointNoise);

                // Add initial guesses to all observed landmarks
                // We don't do this here, we need to do this in the inner "j" loop from above. We add apriltags as we see new ones
                Point3 noise(-0.25, 0.20, 0.15);
                for (size_t j = 0; j < points.size(); ++j) {
                    // Intentionally initialize the variables off from the ground truth
                    Point3 initial_lj = points[j] + noise;
                    initialEstimate.insert(Symbol('l', j), initial_lj);*/
            }
        }

        }
    }


/*
* *      f_x   0    c_x
 *      0     f_y  c_y
 *      0     0    1
 *
 */