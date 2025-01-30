#include <iostream>

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


#include <rerun.hpp>

#include <vector>

#include "rerunLogger.h"

#define STRING(x) #x
#define XSTRING(x) STRING(x)



namespace iSAMslam {
    struct Tag {
        gtsam::Pose3 pose;
        int count{};
    };

    // auto dirPath = std::string("/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/");
    // auto dirPath = std::string("/home/zakareeyah/CLionProjects/Dev/original order/vid_APRILTAG_36h11_720p/");
    std::string dirPath = std::string(XSTRING(SOURCE_ROOT)).append("/original order/vid_APRILTAG_36h11_720p/");

    float markerLength = 0.111;

    // may have to read in as float (check if OpenCV needs it as floats); consider reading in as vectors instead of Mat; https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#mat-mat
    cv::Mat intrinsicsMatrix = (cv::Mat_<double>(3, 3) <<
        1025.26513671875, 0.0, 642.6650390625,
        0.0, 1025.26513671875, 359.37811279296875,
        0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 14) << 2.0888574, -82.303825,
        -0.00071347022, 0.0020022474, 315.66144, 1.8588818,
        -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);


    // Define the camera calibration parameters (for GTSAM)
    /*const boost::shared_ptr<gtsam::Cal3DS2> K(
        new gtsam::Cal3DS2( // explicily using boost::shared_ptr<gtsam::Cal3DS2>
            // instead of gtsam::Cal3DS2::share_ptr TODO: explain
            intrinsicsMatrix.at<double>(0, 0), intrinsicsMatrix.at<double>(1, 2),
            0.0, intrinsicsMatrix.at<double>(0, 2),
            intrinsicsMatrix.at<double>(1, 2), distCoeffs.at<double>(0),
            distCoeffs.at<double>(1), distCoeffs.at<double>(2),
            distCoeffs.at<double>(3)));*/
    /*const boost::shared_ptr<gtsam::Cal3DS2> K(
        new gtsam::Cal3DS2(
           intrinsicsMatrix.at<double>(0, 0),
           intrinsicsMatrix.at<double>(1, 1),
           0.0,
           intrinsicsMatrix.at<double>(0, 2),
           intrinsicsMatrix.at<double>(1, 2),
           distCoeffs.at<double>(0),
           distCoeffs.at<double>(1),
           distCoeffs.at<double>(2),
           distCoeffs.at<double>(3)));*/
    const boost::shared_ptr<gtsam::Cal3DS2> K(
        new gtsam::Cal3DS2(
           intrinsicsMatrix.at<double>(0, 0),
           intrinsicsMatrix.at<double>(1, 1),
           0.0,
           intrinsicsMatrix.at<double>(0, 2),
           intrinsicsMatrix.at<double>(1, 2),
           0,0));

    // Define the camera observation noise model (will leave the same for now).
    // TODO: Also try with gaussian noise.
    auto noise = gtsam::noiseModel::Isotropic::Sigma(
        2, 1.0); // one pixel in u and v (TODO: see if OpenCV says anything about
    // the error in detecting tag corners)

    cv::Mat objPoints(4, 1, CV_32FC3);


    void initObjPoints(cv::Mat objPoints) {
        // x down, y across, z up (to facilitate placing world in corner)
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0, 0, 0); // top left
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(0, markerLength, 0); // top right
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength, markerLength, 0); // bottom right
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(markerLength, 0, 0); // bottom left
    }

    void getCorners(const cv::Mat& image, std::vector<int>& ids,
                    std::vector<std::vector<cv::Point2f>>& corners) {
        cv::aruco::detectMarkers(
            image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),
            corners, ids);
    }

    cv::Mat getCVImage(const int frameNum) {
        const std::string imgPath = dirPath + "image" + std::to_string(frameNum) + std::string(".png");
        return cv::imread(imgPath, cv::IMREAD_COLOR);
    }


    void slam() {
        initObjPoints(objPoints);

        // Create a NonlinearISAM object which will relinearize and reorder the
        // variables every "relinearizeInterval" updates
        constexpr int relinearizeInterval = 3;
        gtsam::NonlinearISAM isam(relinearizeInterval);

        // Create a Factor Graph and Values to (temp) hold the new data
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialEstimate;

        std::cout << "Starting loop...\n";

        const auto rec = rerun::RecordingStream("UnoptimizedMap");
        rec.spawn().exit_on_failure();
        rec.set_time_sequence("Frame", 0);

        // rec.log("world/camera", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);

        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> intrinsics_Eigen(
            intrinsicsMatrix.ptr<double>(), intrinsicsMatrix.rows, intrinsicsMatrix.cols);
        rec.log(
            "world/camera/image",
            rerun::Pinhole(rerun::components::PinholeProjection(
                rerun::Mat3x3(static_cast<Eigen::Matrix3f>(intrinsics_Eigen.cast<float>()).data())))
        );


        // Loop over the different poses, adding the observations to iSAM
        // incrementally In our case - these are images from the camera
        constexpr int numImages = 171;
        int worldTagID;
        // std::map<int, gtsam::Pose3> observedTags; // stores all already observed tags
        std::map<int, Tag> observedTags; // stores all already observed tags

        // for (size_t frame_num = 35; frame_num <= numImages; ++frame_num) {
        int poseNum = 0;
        for (size_t frame_num = 0; frame_num <= numImages; frame_num += 1) {
        // for (size_t frame_num = 18; frame_num <= numImages; frame_num++) {
        std::vector<int> frames = {35, 39, 65, 78, 85, 128, 142};
        // for (int frame_num : frames) {
            // looping through frames
            std::cout << "\nFrame " << frame_num <<
                "=============================================================================\n";
            rec.set_time_sequence("Frame", static_cast<int64_t>(frame_num));
            /*std::cout << "OBSERVED TAGS......................................\n";
            for (const auto& [id, pose] : observedTags) {
                std::cout << id << ":" << pose << std::endl;
            }
            std::cout << "..................................................\n\n";*/
            std::cout << "Frame: " << frame_num << std::endl;
            // get (u,v) of tags in frame
            std::cout << "Reading image...\n";
            cv::Mat image = getCVImage(static_cast<int>(frame_num));

            std::cout << "detecting marker(s)...";
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            getCorners(image, ids, corners);
            std::cout << "\nIDS:++++++++++++++++++++++++++++++++++++++++++++\n";
            for (auto num : ids) {
                std::cout << num << " ";
            }
            std::cout << std::endl;
            std::cout << "IDS:++++++++++++++++++++++++++++++++++++++++++++\n";

            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
            rec.log("world/camera/image",
                    rerun::Image({
                                     static_cast<size_t>(image.rows),
                                     static_cast<size_t>(image.cols),
                                     static_cast<size_t>(image.channels())
                                 },
                                 reinterpret_cast<const uint8_t*>(image.data)));

            // logging all corners as one point cloud
            std::vector<rerun::Position2D> corner_positions;
            for (const auto& marker : corners)
                for (auto corner : marker)
                    corner_positions.emplace_back(corner.x, corner.y);
            rec.log("world/camera/image/corners",
                    rerun::Points2D(corner_positions).with_radii(4.0f));


            gtsam::Values currentEstimate = isam.estimate();

            gtsam::Pose3 wTc; // camera pose wrt wrld

            // only using top left corner for now (point 0) of each tag; using all
            // corners may be better
            // TODO: must also check that if only one tag is present that that tag is
            // world (or no world chosen yet) [could also just simply ignore frames with
            // less than 2 tags] todo: and that at least one of the tags observed was
            // seen before if (!ids.empty()) {
            // if (ids.size() >= 2) {
            if (ids.size() > 2) {
                // If this is the first iteration, add a prior on the first pose to set
                // the coordinate frame and a prior on the first landmark to set the scale
                // (and denote the first landmark as world) Also, as iSAM solves
                // incrementally, we must wait until each is observed at least twice ?
                // before adding it to iSAM.
                if (observedTags.empty()) {
                    std::cout << "\n\n\n\n\n\n\n\n\nsetting world\n\n\n\n\n\n\n\n\n";
                    // first tag seen
                    std::cout << "First tag detectd. Setting world...\n";
                    worldTagID = ids[0]; // let the first tag detected be world
                    // observedTags.insert({worldTagID, });
                    Tag tag = {gtsam::Pose3(), 0};
                    observedTags.insert({worldTagID, tag});
                    rec.log("world/marker" + std::to_string(ids[0]),
                            rerun::Transform3D(
                                rerun::Vec3D(static_cast<Eigen::Vector3f>(
                                        gtsam::Pose3().translation().matrix().cast<float>())
                                    .data()),
                                rerun::Mat3x3(static_cast<Eigen::Matrix3f>(
                                        gtsam::Pose3().rotation().matrix().cast<float>())
                                    .data())));

                    std::cout << "World tag ID: " << worldTagID << std::endl;
                    // get cTw - pose of world wrt camera
                    cv::Vec3d rvec, tvec;
                    // use corners[0] to access the set of corners that coressoponds to
                    // world tag (ids[0])
                    cv::solvePnP(objPoints, corners[0], intrinsicsMatrix, distCoeffs, rvec,
                                 tvec);
                    gtsam::Pose3 cTw(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),
                                     gtsam::Point3(tvec.val));
                    wTc = cTw.inverse(); // pose of camaer wrt world
                    std::cout << "Got pose of camer wrt world.";

                    rerun::Mat3x3 rot(static_cast<Eigen::Matrix3f>(
                                    wTc.rotation().matrix().cast<float>())
                                .data());
                    rerun::Vec3D trans(static_cast<Eigen::Vector3f>(
                            wTc.translation().matrix().cast<float>())
                        .data());
                    rec.log("world/camera", rerun::Transform3D(trans, rot));
                    // Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on
                    // roll,pitch,yaw
                    auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1),
                            gtsam::Vector3::Constant(0.1))
                        .finished());
                    graph.addPrior(
                        gtsam::Symbol('x', poseNum), wTc,
                        poseNoise); // since we're using j, not every increment of the pose
                    // will have a value in the factor graph; only those at
                    // which a marker was detected
                    std::cout << "Added PRIOR to temp GRAPH to first camera pose: x" << frame_num
                        << std::endl;
                    // Add a prior on landmark l0
                    auto pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
                    graph.addPrior(gtsam::Symbol('l', ids[0]), gtsam::Point3(0, 0, 0),
                                   pointNoise);

                    initialEstimate.insert(gtsam::Symbol('l', ids[0]), gtsam::Point3(0, 0, 0));

                    std::cout << "Added prior to temp GRAPH to first landmark: l" << ids[0]
                        << std::endl;
                }
                else {
                    std::cout << "Calc camera pose...\n";
                    for (int j = 0; j < ids.size(); ++j) {
                        // calc wTc
                        // calc camera pose wrt world
                        if (observedTags.count(ids[j]) ==
                            1) {
                            // tag was seen before and we know its pose wrt world
                            gtsam::Pose3 wTt =
                                observedTags[ids[j]].pose; // pose of tag wrt wrld (transforms pts in
                            // tag crdnt frm to world)
                            std::cout << "camera has prev seen tag l" << ids[j] << std::endl;
                            cv::Vec3d rvec, tvec;
                            cv::solvePnP(objPoints, corners[j], intrinsicsMatrix, distCoeffs,
                                         rvec, tvec);
                            gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),
                                             gtsam::Point3(tvec.val));



                            wTc = wTt.compose(cTt.inverse()); // pose of cam wrt world
                            wTc.print("wTc");

                            rerun::Mat3x3 rot(static_cast<Eigen::Matrix3f>(
                                    wTc.rotation().matrix().cast<float>())
                                .data());
                            rerun::Vec3D trans(static_cast<Eigen::Vector3f>(
                                    wTc.translation().matrix().cast<float>())
                                .data());
                            rec.log("world/camera", rerun::Transform3D(trans, rot));

                            /*rec.log("world/camera/image",
                                rerun::Pinhole(rerun::components::PinholeProjection(colMajIntrinsics)));
                            rec.log("world/camera/image",
                                           rerun::Image({static_cast<size_t>(image.rows),
                            static_cast<size_t>(image.cols),
                            static_cast<size_t>(image.channels())}, reinterpret_cast<const
                            uint8_t*>(image.data)));

                            rec.log("world/camera/image/corners",
                            rerun::Points2D(corner_positions).with_radii(4.0f));*/

                            break;
                        }
                    }
                }



                // Add an initial guess for the current camera pose
                // Use solvePnP and the apriltag map you are building up.
                std::cout << "Adding to VALUES initial guess for pose x" << frame_num
                    << std::endl;
                initialEstimate.insert(gtsam::Symbol('x', poseNum), wTc);

                std::cout << "Adding guesses for newly seen landmarks...";
                // Add initial guesses to all newly observed landmarks
                std::vector<rerun::Position3D> ptEstimates;
                for (size_t j = 0; j < ids.size(); ++j) {
                    // looping through ids for new tags
                    // Intentionally initialize the variables off from the ground truth
                    // TODO: If apriltag has not been seen yet, then add it to the initial
                    // estimate and initialize using solvePnP if
                    // (!currentEstimate.exists(gtsam::Symbol('l', ids[j]))) {
                    // //gtsam::Symbol(ids[i], [corner])

                    if (observedTags.count(ids[j]) == 0) {
                        // gtsam::Symbol(ids[i],
                        // [corner])
                        std::cout << "Found new tag l" << ids[j] << std::endl;
                        cv::Vec3d rvec, tvec;
                        cv::solvePnP(objPoints, corners[j], intrinsicsMatrix, distCoeffs,
                                     rvec, tvec);
                        gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),
                                         gtsam::Point3(tvec.val));
                        auto wTt = wTc.compose(cTt);

                        // std::string info = "Tag ID: " + std::to_string(ids[j]) + "\nrvec:";
                        /*for (auto num : rvec.row(0)) {
                            info += std::to_string(num) + " ";
                        }
                        info += "\ntvec";
                        for (auto num : tvec.row(0)) {
                            info += std::to_string(num) + " ";
                        }*/
                        // rec.log("logs", rerun::TextLog(info).with_level(rerun::TextLogLevel::Trace));


                        rec.log("world/marker" + std::to_string(ids[j]),
                                rerun::Transform3D(
                                    rerun::Vec3D(static_cast<Eigen::Vector3f>(
                                            wTt.translation().matrix().cast<float>())
                                        .data()),
                                    rerun::Mat3x3(static_cast<Eigen::Matrix3f>(
                                            wTt.rotation().matrix().cast<float>())
                                        .data())));

                        observedTags.insert({ids[j], {wTt, 0}});
                        std::cout << "For tag l" << ids[j] << std::endl;
                        wTc.print("wTc");
                        cTt.print("SolvePnP output: cTt");
                        wTt.print("wTt");
                        gtsam::Point3 initial_lj = wTt.transformFrom(
                            gtsam::Point3(0, 0, 0)); // coordiantes of top left crnr of tag TODO
                        ptEstimates.push_back(rerun::Position3D(initial_lj.x(),initial_lj.y(),initial_lj.z()));

                        /*std::vector<cv::Point2d> imgPts;
                        cv::projectPoints(cvPts, rvec, tvec, intrinsicsMatrix, distCoeffs, imgPts);
                        rec.set_time_sequence("Frame", 0);
                        std::vector<rerun::Position2D> rrPts;
                        for (auto pt : imgPts) {
                            rrPts.push_back(rerun::Position2D(pt.x, pt.y));
                        }*/

                        // (assuming tag origin is top left)
                        std::cout << "Top left crnr wrt wrld: " << initial_lj << std::endl;
                        // get cam pose wrt world; get pts wrt world; add to graph (deal with
                        // x2 sighting later)
                        initialEstimate.insert(gtsam::Symbol('l', ids[j]), initial_lj);
                        std::cout << "Added to VALUES init estimate for tag l" << ids[j]
                            << std::endl;
                    }
                    // log inital pts
                    rec.log("world/initpts", rerun::Points3D(ptEstimates).with_colors(rerun::Color(255, 0, 0)).with_radii(0.05f));

                }

                // if only one tag verify it's world ?
                // Add factors for each landmark observation
                // In our case - these are the apriltags - note - not all tags are visible
                // in all frames. this "j" loop is only for the tags seen in THIS current
                // frame "i"
                std::vector<rerun::Position2D> meansurments;
                for (size_t j = 0; j < ids.size(); ++j) { // emplacing (u,v)
                    // looping through tags in frame
                    // Create ground truth measurement
                    // In our case, we have the "measurement" from the aruco detection (u,
                    // v). Not sure if to use all 4 corners of the april tag or just 1 and
                    // how that will affect the optimization.

                    observedTags[ids[j]].count += 1;

                    gtsam::Point2 measurement =
                        gtsam::Point2(corners[j][0].x,
                                      corners[j][0].y); // only using top left crnr for now
                    meansurments.push_back(rerun::Position2D(measurement.x(), measurement.y()));

                    /*gtsam::PinholeCamera<gtsam::Cal3DS2> camera(wTc, *K);
                    // gtsam::Point2 projMeasure = camera.project(observedTags[ids[j]].pose);
                    log2DPoint(rec, measurement, i, "world/camera/image/projpoint" + std::to_string(j), 1);
                    */

                    // Add measurement
                    graph.emplace_shared<gtsam::GenericProjectionFactor<
                        gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
                        measurement, noise, gtsam::Symbol('x', poseNum),
                        gtsam::Symbol('l', ids[j]), K);
                    std::cout << "Found tag l" << ids[j]
                        << ". Adding pojection to temp GRAPH at pose x" << frame_num
                        << std::endl;

                    std::string info = "Tag ID: " + std::to_string(ids[j]) + "\nrvec:";
                    rec.log("logs", rerun::TextLog(info).with_level(rerun::TextLogLevel::Trace));
                }
                rec.log("world/camera/image/measurments", rerun::Points2D(meansurments).with_colors(rerun::Color(0, 255, 0)).with_radii(5.0f));

                initialEstimate.print("Initial Estimate");

                // Update iSAM with the new factors
                bool update = false;
                for (int id : ids) {
                    if (observedTags[id].count < 2) {
                        update = true;
                    }
                }


                if(update) {
                    graph.print("***********************************GRAPH***********************************");
                    std::cout << "Updating iSAM...\n";
                    isam.print();
                    isam.printStats();
                    isam.update(graph, initialEstimate);
                    isam.saveGraph("graph.txt");

                    // Clear the factor graph and values for the next iteration
                    std::cout << "Clearing temp graph and vals...\n";
                    graph.resize(0);
                    initialEstimate.clear();
                    break;
                }

                // gtsam::Values currentEstimate = isam.estimate();
                currentEstimate = isam.estimate();
                std::cout << "****************************************************" <<
                std::endl; std::cout << "Frame " << frame_num << ": " << std::endl;
                currentEstimate.print("Current estimate: ");

                /*auto pose = currentEstimate.exists<gtsam::Pose3>(gtsam::Symbol('x', 0));
                // current estimate of initial pose std::cout << pose.get();*/
                poseNum++;

            }
        }

        // batch optimization using Levenberg-Marquardt
        std::cout << "-----------------------------------------------------------------------------------------------------------------------------------------------\n\n";
        gtsam::LevenbergMarquardtParams param;
        param.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
        param.lambdaUpperBound = 1e10;
        auto result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate, param).optimize();
        result.print("Result");

        std::cout << "-----------------------------------------------------------------------------------------------------------------------------------------------\n\n";

        for (auto &[key, value] : observedTags) {
            std::cout << result.at<gtsam::Point3>(gtsam::Symbol('l', key)) << std::endl<<std::endl;

            log3DPoint(rec, result.at<gtsam::Point3>(gtsam::Symbol('l', key)), 173, "world/l" + std::to_string(key), 3);
        }



        // auto pose = result.at<gtsam::Pose3>(0);
        // printf("first pose: %f,%f,%f\n",pose.z(), pose.x(), pose.y());
        // pose = result.at<gtsam::Pose3>(1);
        // printf("second pose: %f,%f,%f\n",pose.z(), pose.x(), pose.y());
        /*graph.print("***********************************GRAPH***********************************");
        isam.update(graph, initialEstimate);*/
    }
} // namespace prototype

/*
 * *      f_x   0    c_x
 *      0     f_y  c_y
 *      0     0    1
 *
 */
