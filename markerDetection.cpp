/**
* Detect and outline markers in a single image and save result to file using cv::imwrite();
* Image(s) are of DICT_4x4_50 (an aruco calibration board)
*/

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>


//TODO: see below
//utility function to get path: via cmd line or using relative paths based on assumptions.
//func should take in image/path to image

std::string imgPath = "/home/zakareeyah/CLionProjects/Dev/media/OAK_DICT_4x4_50_1080p.jpeg";
std::string outPath = "/home/zakareeyah/CLionProjects/Dev/media/out_OAK_DICT_4x4_50_1080p_markersDectected.png";
auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

void detectMarkers() {
    cv::Mat inputImage = cv::imread(imgPath, cv::IMREAD_COLOR);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    cv::Mat outputImage = inputImage.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    cv::imwrite(outPath, outputImage);
}