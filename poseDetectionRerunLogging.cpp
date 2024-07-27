/**
 * Detect pose of markers in single image and log results via rerun
 * The image(s) used are of DICT_4x4_50 (an aruco calibration board)
 */
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/ximgproc.hpp>
#include <rerun.hpp>


void detectPoseLog () {
    std::string imgPath = "/home/zakareeyah/CLionProjects/Dev/media/OAK_DICT_4x4_50_1080p_obstructed.jpeg";
    std::string outPath = "/home/zakareeyah/CLionProjects/Dev/media/out_OAK_DICT_4x4_50_1080p_pose.png";
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::Mat intrinsicsMatrix = (cv::Mat_<float>(3,3) <<
        3075.7952, 0, 1927.995,
        0, 3075.7952, 1078.1343,
        0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<float>(1, 14) << 2.0888574, -82.303825, -0.00071347022, 0.0020022474, 315.66144,
        1.8588818, -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);




    cv::Mat image = cv::imread(imgPath, cv::IMREAD_COLOR);
    if(!image.data) {
        std::cout << "Could not find image " + imgPath;
        exit(1);
    }

    // set coordinate system
    float markerLength = 0.019;
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);



    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners, rejected;

    // detect markers and estimate pose
    cv::aruco::detectMarkers(image, dictionary, corners, ids);

    size_t nMarkers = corners.size();
    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    if(!ids.empty()) {
        // Calculate pose for each marker
        for (size_t i = 0; i < nMarkers; i++) {
            cv::solvePnP(objPoints, corners.at(i), intrinsicsMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        }
    }

    /*if(!ids.empty()) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
        for(unsigned int i = 0; i < ids.size(); i++)
            cv::drawFrameAxes(image, intrinsicsMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
    }*/

    // log results
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);     // Rerun expects RGB format

    const auto rec = rerun::RecordingStream("OpenCV4.6");
    rec.spawn().exit_on_failure();
    rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);



    //                                  --------------- logging camera ---------------
    // identity transformation so that the axes show up
    rec.log(
        "world/camera",
        rerun::Transform3D(rerun::RotationAxisAngle({0.0f, 0.0f, 0.0f}, rerun::Angle::radians(0)), 1.0f)
        );

    cv::Mat intsMatTrans;
    transpose(intrinsicsMatrix, intsMatTrans);
    const auto cam = rerun::Mat3x3(intsMatTrans.ptr<float>(0));
    rec.log("world/camera/image",
            rerun::Pinhole(rerun::components::PinholeProjection(cam)));


    //                          ---------------  logging image---------------


    rec.log("world/camera/image",
            rerun::Image({image.rows, image.cols, image.channels()}, reinterpret_cast<const uint8_t*>(image.data)));



    //                         ---------------  logging corners ---------------

    // logging all corners as one point cloud
    std::vector<rerun::Position2D> corner_positions;
    for (const auto& marker : corners) {
        for (auto corner : marker) {
            corner_positions.emplace_back(corner.x, corner.y);
        }
    }
    rec.log("world/camera/image/corners", rerun::Points2D(corner_positions).with_radii(2.0f));


    //                               ---------------  logging poses ---------------
    for (size_t i = 0; i < ids.size(); ++i)
    {
        cv::Mat rotMat, rotMatTrans;
        cv::Rodrigues(rvecs[i], rotMat);
        cv::transpose(rotMat, rotMatTrans);
        std::array<float, 9> rotVals{};
        for (int j = 0; j < 9; ++j) {
            rotVals.at(j) = static_cast<float>(rotMatTrans.at<double>(j));
        }
        rerun::Mat3x3 rot(rotVals);
        rerun::Vec3D trans(tvecs[i][0], tvecs[i][1], tvecs[i][2]);

        rec.log(
            "world/marker" + std::to_string(ids[i]),
            rerun::Transform3D(trans, rot)
        );
    }

}
