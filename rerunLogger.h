//
// Created by zakareeyah on 8/4/24.
//

#ifndef RERUNLOGGER_H
#define RERUNLOGGER_H

#include <rerun.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>


inline rerun::RecordingStream startLogger(const std::string& stream="Logger") {
    auto rec = rerun::RecordingStream(stream);
    rec.spawn().exit_on_failure();
    rec.set_time_sequence("Frame", 0);
    return rec;
}

inline void logCVImage(const rerun::RecordingStream& rec, const cv::Mat& image, const int frameNum, const std::string& entity="world/camera/image") {
    rec.set_time_sequence("Frame", frameNum);
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    rec.log(entity, rerun::Image({
                                     static_cast<size_t>(image.rows),
                                     static_cast<size_t>(image.cols),
                                     static_cast<size_t>(image.channels())
                                 },
                                 image.data));
}

inline void log2DPoint(const rerun::RecordingStream& rec, const gtsam::Point2& point, const int frameNum, const std::string& entity, int color=1) {
    rerun::Color c;
    switch (color) {
    case 0:
        c = rerun::Color(0, 0, 0);
        break;
    case 1:
            c = rerun::Color(255, 0, 0); break;
    case 2:
            c = rerun::Color(0, 255, 0); break;
    case 3:
            c = rerun::Color(0, 0, 255); break;
    default:
            c = rerun::Color(255, 255, 255); break;
    }

    rec.set_time_sequence("Frame", frameNum);
    // rec.log(entity, rerun::Position2D(point.x(), point.y()));
    std::vector<rerun::Position2D> points = {rerun::Position2D(point.x(), point.y())};
    rec.log(entity, rerun::Points2D(points).with_colors(c).with_radii(5.0f));
}

inline void log3DPoint(const rerun::RecordingStream& rec, const gtsam::Point3& point, const int frameNum, const std::string& entity, const int color=1) {
    rerun::Color c;
    switch (color) {
    case 0:
        c = rerun::Color(0, 0, 0);
        break;
    case 1:
            c = rerun::Color(255, 0, 0); break;
    case 2:
            c = rerun::Color(0, 255, 0); break;
    case 3:
            c = rerun::Color(0, 0, 255); break;
    default:
            c = rerun::Color(255, 255, 255); break;
    }

    rec.set_time_sequence("Frame", frameNum);
    std::vector<rerun::Position3D> points = {rerun::Position3D(point.x(), point.y(), point.z())};
    rec.log(entity, rerun::Points3D(points).with_colors(c));
}

inline void logTextTrace(const rerun::RecordingStream& rec, const std::string& text, const int frameNum) {
    rec.set_time_sequence("Frame", frameNum);
    rec.log("logs", rerun::TextLog(text).with_level(rerun::TextLogLevel::Trace));
}

inline void logPose(const rerun::RecordingStream& rec, gtsam::Pose3 pose, const int frameNum, const std::string& entity) {
    const rerun::Mat3x3 rot(static_cast<Eigen::Matrix3f>(
                                    pose.rotation().matrix().cast<float>())
                                .data());
    const rerun::Vec3D trans(static_cast<Eigen::Vector3f>(
            pose.translation().matrix().cast<float>())
        .data());
    rec.log(entity, rerun::Transform3D(trans, rot));
}

#endif //RERUNLOGGER_H
