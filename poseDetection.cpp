/**
* Detect pose of markers in single image and save results to file new image file using cv::write
* The image(s) used are of DICT_4x4_50 (an aruco calibration board)
*/
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>


std::string imgPath = "/home/zakareeyah/CLionProjects/Dev/media/OAK_DICT_4x4_50_1080p.jpeg";
std::string outPath = "/home/zakareeyah/CLionProjects/Dev/media/out_OAK_DICT_4x4_50_1080p_pose.png";
auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);


cv::Mat intrinsicsMatrix = (cv::Mat_<float>(3,3) <<
    3075.7952, 0, 1927.995,
    0, 3075.7952, 1078.1343,
    0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<float>(1, 14) << 2.0888574, -82.303825, -0.00071347022, 0.0020022474, 315.66144,
    1.8588818, -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);


void detectPose () {
    cv::Mat image = cv::imread(imgPath, cv::IMREAD_COLOR);
    if(!image.data) {
        std::cout << "Could not find image " + imgPath;
        exit(1);
    }

    // set coordinate system
    float markerLength = 0.15;
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

    // draw results
    cv::Mat imageCopy;
    image.copyTo(imageCopy);
    if(!ids.empty()) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        for(unsigned int i = 0; i < ids.size(); i++)
            cv::drawFrameAxes(imageCopy, intrinsicsMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
    }

    cv::imwrite(outPath, imageCopy);
}
