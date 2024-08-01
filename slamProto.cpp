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

#include <rerun.hpp>

#include <vector>



float markerLength = 0.15;

void getCorners(const cv::Mat& image, std::vector<int>& ids, std::vector<std::vector<cv::Point2f> >& corners) {
    cv::aruco::detectMarkers(image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), corners, ids);
}

void slam() {
    std::cout << "initializing variables\n";
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
    const boost::shared_ptr<gtsam::Cal3DS2> K(new gtsam::Cal3DS2( // explicily using boost::shared_ptr<gtsam::Cal3DS2> instead of gtsam::Cal3DS2::share_ptr TODO: explain
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
    auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v (TODO: see if OpenCV says anything about the error in detecting tag corners)

    // Create a NonlinearISAM object which will relinearize and reorder the variables
    // every "relinearizeInterval" updates
    constexpr int relinearizeInterval = 3;
    gtsam::NonlinearISAM isam(relinearizeInterval);

    // Create a Factor Graph and Values to (temp) hold the new data
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;

    std::cout << "Starting loop...\n";

    const auto rec = rerun::RecordingStream("UnoptimizedMap");
    rec.spawn().exit_on_failure();


    std::array<float, 9> matPtr{};
    int index = 0;
    // travers cv::Mat in col-major order
    for (int j = 0; j < 3; ++j)     // cols
      for (int i = 0; i < 3; ++i)   // rows
        matPtr[index++] = static_cast<float>(intrinsicsMatrix.at<double>(i, j));
    rerun::Mat3x3 colMajIntrinsics(matPtr);

    rec.log("world/camera/image",
                rerun::Pinhole(rerun::components::PinholeProjection(colMajIntrinsics)));

    // Loop over the different poses, adding the observations to iSAM incrementally
    // In our case - these are images from the camera
    constexpr int numImages = 171;
    int worldTagID;
    std::map<int, gtsam::Pose3> observedTags; // stores all already observed tags and their poses wrt to world
    // array/map ? of (seen) tag poses wrt wrld
    for (size_t i = 0; i <= numImages; ++i) { // looping through frames
        std::cout << "Frame: " << i << std::endl;
        // get (u,v) of tags in frame
        std::cout << "Reading image...\n";
        auto imgPath = std::string("/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image") + std::to_string(i) + std::string(".png");
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        std::cout << "detecting marker(s)...";
        auto image = cv::imread(imgPath, cv::IMREAD_COLOR);
        getCorners(image, ids, corners);

        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        rec.log("world/camera/image",
                rerun::Image({static_cast<size_t>(image.rows), static_cast<size_t>(image.cols), static_cast<size_t>(image.channels())}, reinterpret_cast<const uint8_t*>(image.data)));

        std::vector<rerun::Position2D> corner_positions;
        for (const auto& marker : corners) {
            for (auto corner : marker) {
                corner_positions.emplace_back(corner.x, corner.y);
            }
        }
        // logging all corners as one point cloud
        rec.log("world/camera/image/corners", rerun::Points2D(corner_positions).with_radii(2.0f));


        gtsam::Values currentEstimate = isam.estimate();

        gtsam::Pose3 wTc; // camera pose wrt wrld

        // only using top left corner for now (point 0) of each tag; using all corners may be better
        // TODO: must also check that if only one tag is present that that tag is world (or no world chosen yet) [could also just simply ignore frames with less than 2 tags]
        // todo: and that at least one of the tags observed was seen before
        // if (!ids.empty()) {
        if (ids.size() >= 2) {
            // If this is the first iteration, add a prior on the first pose to set the coordinate frame
            // and a prior on the first landmark to set the scale (and denote the first landmark as world)
            // Also, as iSAM solves incrementally, we must wait until each is observed at least twice ? before
            // adding it to iSAM.
            if (currentEstimate.empty()) { // first tag seen
                std::cout << "First tag detectd. Setting world...\n";
                worldTagID = ids[0]; // let the first tag detected be world
                observedTags.insert({worldTagID, gtsam::Pose3()});

                std::cout << "World tag ID: " << worldTagID << std::endl;
                // get cTw - pose of world wrt camera
                cv::Vec3d rvec, tvec;
                cv::solvePnP(objPoints, corners[0], intrinsicsMatrix, distCoeffs, rvec, tvec);
                gtsam::Pose3 cTw(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)), gtsam::Point3(tvec.val));
                wTc = cTw.inverse(); // pose of camaer wrt world
                std::cout << "Got pose of camer wrt world.";

                // Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
                auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished());
                graph.addPrior(gtsam::Symbol('x', i), wTc, poseNoise); // since we're using j, not every increment of the pose will have a value in the factor graph; only those at which a marker was detected
                std::cout << "Added PRIOR to temp GRAPH to first camera pose: x" << i << std::endl;
                // Add a prior on landmark l0
                auto pointNoise =
                    gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
                graph.addPrior(gtsam::Symbol('l', ids[0]), gtsam::Point3(0,0,0), pointNoise);
                std::cout << "Added prior to temp GRAPH to first landmark: l" << ids[0] << std::endl;
            }

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
                                                                                                                   gtsam::Symbol('x', i), gtsam::Symbol('l', ids[j]), K);
                std::cout << "Found tag l" << ids[j] << ". Adding pojection to temp GRAPH at pose x" << i << std::endl;
            }

            std::cout << "Calc camera pose...\n";
            for (int j = 0; j < ids.size(); ++j) { // calc wTc
                // calc camera pose wrt world
                if (observedTags.count(ids[j]) == 1) { // tag was seen before and we know its pose wrt world
                    gtsam::Pose3 wTt = observedTags[ids[j]]; // pose of tag wrt wrld (transforms pts in tag crdnt frm to world)
                    std::cout << "camera has prev seen tag l" << ids[j] << std::endl;
                    cv::Vec3d rvec, tvec;
                    cv::solvePnP(objPoints, corners[j], intrinsicsMatrix, distCoeffs, rvec, tvec);
                    gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)), gtsam::Point3(tvec.val));

                    wTc = wTt.compose(cTt.inverse()); // pose of cam wrt world
                    // float * data = reinterpret_cast<float *>(wTc.rotation().r1().data()); // really bad; ideally should not point a float to a double
                    rec.log("world/camera",
                        rerun::Transform3D(rerun::Vec3D(reinterpret_cast<const float *>(wTc.translation().data())),
                            {rerun::Vec3D(reinterpret_cast<float *>(wTc.rotation().r1().data())),
                                    rerun::Vec3D(reinterpret_cast<float *>(wTc.rotation().r2().data())),
                                    rerun::Vec3D(reinterpret_cast<float *>(wTc.rotation().r3().data())),
                                    })
                        // rerun::Transform3D(rerun::RotationAxisAngle({0.0f, 0.0f, 0.0f}, rerun::Angle::radians(0)), 1.0f)
                        );
                    std::cout << "Calculated camera pose wrt wrld\n";
                    break;
                }
            }

            // Add an initial guess for the current camera pose
            // Use solvePnP and the apriltag map you are building up.
            std::cout << "Adding to VALUES initial guess for pose x" << i << std::endl;
            initialEstimate.insert(gtsam::Symbol('x', i), wTc);

            std::cout << "Adding guesses for newly seen landmarks...";
            // Add initial guesses to all newly observed landmarks
            for (size_t j = 0; j < ids.size(); ++j) {
                // Intentionally initialize the variables off from the ground truth
                // TODO: If apriltag has not been seen yet, then add it to the initial estimate and initialize using solvePnP
                // if (!currentEstimate.exists(gtsam::Symbol('l', ids[j]))) {    //gtsam::Symbol(ids[i], [corner])
                if (observedTags.count(ids[j]) == 0) {    //gtsam::Symbol(ids[i], [corner])
                    std::cout << "Found new tag l" << ids[j] << std::endl;
                    cv::Vec3d rvec, tvec;
                    cv::solvePnP(objPoints, corners[0], intrinsicsMatrix, distCoeffs, rvec, tvec);
                    gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)), gtsam::Point3(tvec.val));
                    auto wTt = wTc.compose(cTt);
                    rec.log(
                        "world/marker" + std::to_string(ids[j]),
                        rerun::Transform3D(rerun::Vec3D(reinterpret_cast<const float *>(wTc.translation().data())),
                            {rerun::Vec3D(reinterpret_cast<float *>(wTt.rotation().r1().data())),
                                    rerun::Vec3D(reinterpret_cast<float *>(wTt.rotation().r2().data())),
                                    rerun::Vec3D(reinterpret_cast<float *>(wTt.rotation().r3().data())),
                                    })
                    );

                    // wTc.
                    // observedTags.insert({ids[j], wTt});
                    std::cout << "For tag l" << ids[j] << std::endl;
                    wTc.print("wTc");
                    cTt.print("SolvePnP output: cTt");
                    wTt.print("wTt");
                    gtsam::Point3 initial_lj = wTt.transformFrom(gtsam::Point3(0, 0, 0)); // coordiantes of top left crnr of tag (assuming tag origin is top left)
                    std::cout << "Top left crnr wrt wrld: " << initial_lj << std::endl;
                    // get cam pose wrt world; get pts wrt world; add to graph (deal with x2 sighting later)
                    initialEstimate.insert(gtsam::Symbol('l', ids[j]), initial_lj);
                    std::cout << "Added to VALUES init estimate for tag l" << ids[j] << std::endl;
                }
            }

            /*initialEstimate.print("Initial Estimate");
            std::cout << "Updating iSAM...\n";
            // Update iSAM with the new factors
            isam.print();
            isam.printStats();
            isam.update(graph, initialEstimate);
            isam.saveGraph("graph.txt");
            // gtsam::Values currentEstimate = isam.estimate();
            currentEstimate = isam.estimate();
            std::cout << "****************************************************" << std::endl;
            std::cout << "Frame " << i << ": " << std::endl;
            currentEstimate.print("Current estimate: ");*/

            /*auto pose = currentEstimate.exists<gtsam::Pose3>(gtsam::Symbol('x', 0)); // current estimate of initial pose
            std::cout << pose.get();*/

            // Clear the factor graph and values for the next iteration
            std::cout << "Clearing temp graph and vals...\n";
            graph.resize(0);
            initialEstimate.clear();
        }
    }
}
/*
* *      f_x   0    c_x
 *      0     f_y  c_y
 *      0     0    1
 *
 */