//
// Created by zakareeyah on 8/7/24.
//

#include "opencvProjTest.h"

void opencvProjTest::TestOneImage() {
    /*float markerLength = 0.15;
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
    */

    // get image 35
    auto rec = startLogger();
    rec.set_time_sequence("Frame", 0);

    cv::Mat image = cv::imread("/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image35.png");

    // detect corners in image 35
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), corners, ids);

    for (int i = 0; i < corners.size(); ++i) {
        for (int j = 0; j < corners[0].size(); ++j) {
            log2DPoint(rec, gtsam::Point2(corners[i][j].x, corners[i][j].y), 0,
                       "world/camera/image/cornerl" + std::to_string(i) + "c" + std::to_string(j));
        }
    }

    // select world tag  and calc wTc (also saving cTw in OpenCV form)
    const int worldTagID = ids[0];
    cv::Vec3d rvec_cTw, tvec_cTw;
    cv::solvePnP(objPoints, corners[0], intrinsicsMatrix, distCoeffs, rvec_cTw, tvec_cTw);  // use corners[0] to access the set of corners that coressoponds to world tag (ids[0])
    gtsam::Pose3 cTw(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec_cTw.val)), gtsam::Point3(tvec_cTw.val));
    auto wTc = cTw.inverse(); // pose of camaer wrt world

    //log pinhole and camera image
    logPose(rec, wTc, 0, "world/camera");
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> intrinsics_Eigen(
            intrinsicsMatrix.ptr<double>(), intrinsicsMatrix.rows, intrinsicsMatrix.cols);
    rec.log(
        "world/camera/image",
        rerun::Pinhole(rerun::components::PinholeProjection(
            rerun::Mat3x3(static_cast<Eigen::Matrix3f>(intrinsics_Eigen.cast<float>()).data())))
    );
    logCVImage(rec, image, 0, "world/camera/image");

    // calc 3d pts (lndmrks) wrt world
    std::map<int, gtsam::Pose3> observedTagsPoses;
    observedTagsPoses.insert({worldTagID, gtsam::Pose3()});

    std::vector<gtsam::Point3> points;
    std::vector<cv::Point3d> cv3DPts;
    for (auto &[id, pose] : observedTagsPoses) {
        points.push_back(pose.transformFrom(gtsam::Point3(-markerLength/2.f, markerLength/2.f, 0)));
        points.push_back(pose.transformFrom(gtsam::Point3(markerLength/2.f, markerLength/2.f, 0)));
        points.push_back(pose.transformFrom(gtsam::Point3(markerLength/2.f, -markerLength/2.f, 0)));
        points.push_back(pose.transformFrom(gtsam::Point3(-markerLength/2.f, -markerLength/2.f, 0)));
    }
    for (const auto& point : points) cv3DPts.push_back(cv::Point3d(point.x(), point.y(), point.z()));

    for (int i =0; i<points.size(); ++i) {
        log3DPoint(rec, points[i], 0, "world/l" + std::to_string(i), 2);
    }


    // project pts back using OpenCV
    std::vector<cv::Point2d> imgPts;
    cv::projectPoints(cv3DPts, rvec_cTw, tvec_cTw, intrinsicsMatrix, distCoeffs, imgPts);
    rec.set_time_sequence("Frame", 0);
    std::vector<rerun::Position2D> rrPts;
    std::cout << "OpenCV projected:\n";
    for (auto pt : imgPts) {
        std::cout << imgPts << std::endl;
        rrPts.push_back(rerun::Position2D(pt.x, pt.y));
    }
    rec.log("world/camera/image/opencvProjPts", rerun::Points2D(rrPts).with_colors(rerun::Color(255,255,255)));

    const boost::shared_ptr<gtsam::Cal3DS2> K(
       new gtsam::Cal3DS2(
          intrinsicsMatrix.at<double>(0, 0),
          intrinsicsMatrix.at<double>(1, 1),
          0.0,
          intrinsicsMatrix.at<double>(0, 2),
          intrinsicsMatrix.at<double>(1, 2),
          distCoeffs.at<double>(0),
          distCoeffs.at<double>(1),
          distCoeffs.at<double>(2),
          distCoeffs.at<double>(3)));

    K->print("K");

    std::cout << "\nGTSAM Projected:\n";
    for (size_t j = 0; j < points.size(); ++j) {
        gtsam::PinholeCamera<gtsam::Cal3DS2> camera(wTc, *K);
        gtsam::Point2 measurement = camera.project(points[j]);
        // std::cout << points[j];
        std::cout << measurement << std::endl;
        log2DPoint(rec, measurement, 0, "world/camera/image/projpoint" + std::to_string(j), 3);
    }


}



