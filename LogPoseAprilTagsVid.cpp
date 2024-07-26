/**
 * Detect poses of April Tags (36h11) in a video (saved as a dir of pics) and log results via rerun
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/ximgproc.hpp>
#include <rerun.hpp>

auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);


cv::Mat intrinsicsMatrix = (cv::Mat_<float>(3,3) <<
1025.26513671875, 0.0, 642.6650390625,
0.0, 1025.26513671875, 359.37811279296875,
0.0, 0.0, 1.0);

cv::Mat distCoeffs = (cv::Mat_<float>(1, 14) << 2.0888574, -82.303825, -0.00071347022, 0.0020022474, 315.66144,
    1.8588818, -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);



void detectPose () {

    const auto rec = rerun::RecordingStream("OpenCV4.6");
    rec.spawn().exit_on_failure();
    rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);

    // set coordinate system
    float markerLength = 0.15;
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    std::string dir = "/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p";

    for (int k = 0; k <= 171; ++k) {
        std::string imgPath = dir;
        imgPath.append("/image").append(std::to_string(k)).append(".png");
        cv::Mat image = cv::imread(imgPath, cv::IMREAD_COLOR);
        if(!image.data) {
            std::cout << "Could not find image " + imgPath;
            exit(1);
        }

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


        // log results
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);     // Rerun expects RGB format

        //                                  --------------- logging camera ---------------
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
        std::vector<rerun::Position2D> corner_positions;
        for (const auto& marker : corners) {
            for (auto corner : marker) {
                corner_positions.emplace_back(corner.x, corner.y);
            }
        }
        // logging all corners as one point cloud
        rec.log("world/camera/image/corners", rerun::Points2D(corner_positions).with_radii(2.0f));


        //                               ---------------  logging poses ---------------
        for (size_t i = 0; i < ids.size(); ++i) {
            cv::Mat rotMat, rotMatTrans;
            cv::Rodrigues(rvecs[i], rotMat);
            cv::transpose(rotMat, rotMatTrans);
            std::array<float, 9> rotVals = std::array<float, 9>();

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

}
