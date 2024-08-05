#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

#include "rerunLogger.h"

namespace selective_data {
    int startFrame = 54,
        endFrame = 63;

    // may have to read in as float (check if OpenCV needs it as floats); consider reading in as vectors instead of Mat; https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#mat-mat
    cv::Mat intrinsicsMatrix = (cv::Mat_<double>(3, 3) <<
        1025.26513671875, 0.0, 642.6650390625,
        0.0, 1025.26513671875, 359.37811279296875,
        0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 14) << 2.0888574, -82.303825,
        -0.00071347022, 0.0020022474, 315.66144, 1.8588818,
        -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);

    /*cv::Mat intrinsicsMatrix = (cv::Mat_<float>(3,3) <<
       3075.7952, 0, 1927.995,
       0, 3075.7952, 1078.1343,
       0, 0, 1);*/



    cv::Mat objPoints(4, 1, CV_32FC3);

    std::string partPath = "/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image";


    std::map<int, gtsam::Pose3> observedTagsPoses;

    /* ************************************************************************* */
    /**
     * Parse images using OpenCV and return the points of
     * @return
     */
    std::vector<gtsam::Point3> createPoints() {

        // Create the set of ground-truth landmarks
        std::vector<gtsam::Point3> points;

        constexpr float markerLength = 0.15;
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

        // const std::string imgPath = "/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image34.png"; // ids: 21
        // const std::string imgPath = "/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image35.png"; // 21, 25
        // const std::string imgPath2 = "/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image36.png"; // ids: 25
        const std::string imgPath2 = "/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image54.png"; // ids: 25, 22 (from frame 54-63 inclusive)


        // std::string partPath = "/home/zakareeyah/CLionProjects/Dev/vid_APRILTAG_36h11_720p/image";
        // for (int i = 0; i < 1; ++i) {
        // for (size_t frame_num = 54; frame_num <= 171; frame_num++) {
            // const cv::Mat image = cv::imread(paths[i], cv::IMREAD_COLOR);
            // const cv::Mat image = cv::imread(partPath + std::to_string(frame_num) + ".png", cv::IMREAD_COLOR);
        const cv::Mat image = cv::imread(imgPath2, cv::IMREAD_COLOR);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), corners, ids);
            // std::cout << frame_num << ": ";
        for (const auto id : ids) std::cout << id << " ";
        std::cout << std::endl;
        // }

        const int worldTagID = ids[0];
        observedTagsPoses.insert({worldTagID, gtsam::Pose3()});
        std::cout << "World Tag ID: " << worldTagID << std::endl;

        cv::Vec3d rvec, tvec;
        cv::solvePnP(objPoints, corners[0], prototype::intrinsicsMatrix, distCoeffs, rvec, tvec);  // use corners[0] to access the set of corners that coressoponds to world tag (ids[0])
        gtsam::Pose3 cTw(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)), gtsam::Point3(tvec.val));
        auto wTc = cTw.inverse(); // pose of camaer wrt world

        cv::solvePnP(objPoints, corners[1], intrinsicsMatrix, distCoeffs,rvec, tvec);
        gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),gtsam::Point3(tvec.val));
        auto wTt = wTc.compose(cTt);
        observedTagsPoses.insert({ids[1], wTt});


        for (auto &[id, pose] : observedTagsPoses) {
            points.push_back(pose.transformFrom(gtsam::Point3(-markerLength/2.f, markerLength/2.f, 0)));
            points.push_back(pose.transformFrom(gtsam::Point3(markerLength/2.f, markerLength/2.f, 0)));
            points.push_back(pose.transformFrom(gtsam::Point3(markerLength/2.f, -markerLength/2.f, 0)));
            points.push_back(pose.transformFrom(gtsam::Point3(-markerLength/2.f, -markerLength/2.f, 0)));
        }

        for (const auto& point : points) {
            std::cout << point << std::endl << std::endl;
        }
        return points;
    }

    /* ************************************************************************* */
    std::vector<gtsam::Pose3> createPoses() {
        // Create the set of ground-truth poses
        // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
        std::vector<gtsam::Pose3> poses;

        // for (size_t frame_num = 54; frame_num <= 61; frame_num++) { // 8 frames with only tags 25 and 22
        for (size_t frame_num = startFrame; frame_num <= endFrame; frame_num++) {

            const cv::Mat image = cv::imread(partPath + std::to_string(frame_num) + ".png", cv::IMREAD_COLOR);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), corners,
                                     ids);

            for (int j = 0; j < ids.size(); ++j) {
                if (observedTagsPoses.count(ids[j]) == 1) {
                    // tag was seen before and we know its pose wrt world
                    gtsam::Pose3 wTt = observedTagsPoses[ids[j]];
                    // pose of tag wrt wrld (transforms pts in tag crdnt to world)
                    std::cout << "using tag " << ids[j] << " to calc cam pose." << std::endl;
                    cv::Vec3d rvec, tvec;
                    cv::solvePnP(objPoints, corners[j], intrinsicsMatrix, distCoeffs,
                                 rvec, tvec);
                    gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),
                                     gtsam::Point3(tvec.val));

                    auto wTc = wTt.compose(cTt.inverse()); // pose of cam wrt world
                    poses.push_back(wTc);
                    break;
                }
            }
        }

        for (auto pose : poses) std::cout << pose << "\n\n";

        return poses;
    }


    void slam() {
        auto rec = startLogger();
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> intrinsics_Eigen(
            intrinsicsMatrix.ptr<double>(), intrinsicsMatrix.rows, intrinsicsMatrix.cols);
        rec.log(
            "world/camera/image",
            rerun::Pinhole(rerun::components::PinholeProjection(
                rerun::Mat3x3(static_cast<Eigen::Matrix3f>(intrinsics_Eigen.cast<float>()).data())))
        );
        // Define the camera calibration parameters
        // Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

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

        cout << endl;
        cout << intrinsicsMatrix << endl;
        cout << distCoeffs << endl;
        K->print("K");
        cout << endl;
        // Define the camera observation noise model.
        auto noise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

        // Create the set of ground-truth landmarks - apriltags
        vector<Point3> points = createPoints();

        // Create the set of ground-truth poses
        vector<Pose3> poses = createPoses();

        // Create a NonlinearISAM object which will relinearize and reorder the variables
        // every "relinearizeInterval" updates
        int relinearizeInterval = 3;
        // int relinearizeInterval = 4;
        NonlinearISAM isam(relinearizeInterval);

        // Create a Factor Graph and Values to hold the new data
        NonlinearFactorGraph graph;
        Values initialEstimate;

        /*if(numPoses < 0) numPoses = poses.size();
        if(numLandmarks < 0) numLandmarks = points.size();*/
        int currentFrame = 54;
        // Loop over the different poses, adding the observations to iSAM incrementally
        for (size_t i = 0; i < poses.size(); ++i) {
            logTextTrace(rec, "Current loop: " + to_string(i), i);
            logPose(rec, poses[i], i, "world/camera");
            // Add factors for each landmark observation
            for (size_t j = 0; j < points.size(); ++j) {
                // Create ground truth measurement
                // PinholeCamera<Cal3DS2> camera(poses[i]);
                PinholeCamera<Cal3DS2> camera(poses[i], *K);
                Point2 measurement = camera.project(points[j]);
                log2DPoint(rec, measurement, i, "world/camera/image/projpoint" + std::to_string(j), 1);
                // Add measurement
                /*graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3DS2>>(measurement, noise,
                                                                                      Symbol('x', i), Symbol('l', j), K);*/
            }
        // for (size_t i = 0; i < numPoses; ++i) {
            // Add factors for each landmark observation
            const cv::Mat image = cv::imread(partPath + std::to_string(currentFrame++) + ".png", cv::IMREAD_COLOR);
            logCVImage(rec, image, i, "world/camera/image");
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), corners, ids);
            // for (size_t j = 0; j < points.size(); ++j) {
            // for (size_t j = 0; j < numLandmarks; ++j) {
            int j=0;
            for (const auto& marker : corners) {
                // Create ground truth measurement
                for (auto corner: marker){
                    auto measurement = gtsam::Point2(corner.x, corner.y);
                    log2DPoint(rec, measurement, i, "world/camera/image/PnPpoint" + std::to_string(j), 3);
                    // Add measurement
                    graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3DS2>>(measurement, noise,
                                                                                          Symbol('x', i), Symbol('l', j), K);
                    j++;
                }
            }

            // Intentionally initialize the variables off from the ground truth
            // Pose3 noise(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
            // Pose3 initial_xi = poses[i].compose(noise);


            // Add an initial guess for the current pose
            // initialEstimate.insert(Symbol('x', i), initial_xi);
            initialEstimate.insert(Symbol('x', i), poses[i]);

            // If this is the first iteration, add a prior on the first pose to set the coordinate frame
            // and a prior on the first landmark to set the scale
            // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
            // adding it to iSAM.
            if (i == 0) {
                // Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
                auto poseNoise = noiseModel::Diagonal::Sigmas(
                  (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
                graph.addPrior(Symbol('x', 0), poses[0], poseNoise);

                // Add a prior on landmark l0
                auto pointNoise =
                  noiseModel::Isotropic::Sigma(3, 0.1);
                graph.addPrior(Symbol('l', 0), points[0], pointNoise);

                // Add initial guesses to all observed landmarks
                // Point3 noise(-0.25, 0.20, 0.15);
                for (size_t j = 0; j < points.size(); ++j) {
                    log3DPoint(rec, points[j], i, "world/l" + std::to_string(j), 2);
                // for (size_t j = 0; j < numLandmarks; ++j) {
                    // Intentionally initialize the variables off from the ground truth
                    // Point3 initial_lj = points[j] + noise;
                    // initialEstimate.insert(Symbol('l', j), initial_lj);
                    initialEstimate.insert(Symbol('l', j), points[j]);
                }
            }
            else {
                // Update iSAM with the new factors
                cout << "First update call\n";
                // isam.print("ISAM_)(*&^%^&*()_$%^&*()*&^%$^&*())(^&%$#$%^&*)()*&$%#$%^&*()&*^$%^#%$^&*(*(&^%$#%^&*^%#$%^&**&$^#%%$%&*&^%$#%$^%&*&*^$");
                isam.update(graph, initialEstimate);
                Values currentEstimate = isam.estimate();
                cout << "****************************************************" << endl;
                cout << "Frame " << i << ": " << endl;
                currentEstimate.print("Current estimate: ");
                // Clear the factor graph and values for the next iteration
                graph.resize(0);
                initialEstimate.clear();
            }
        }
    }

}

/*
 * Tag (ids) per frame
54: 25 22
55: 25 22
56: 25 22
57: 25 22
58: 25 22
59: 25 22
60: 25 22
61: 25 22
62: 25 22
63: 25 22
64: 25 22 24
65: 25 22 24
66: 25 22 24
67: 22 25 24
68: 25 22 24
69: 25 22 24
70: 25 24 22
71: 22 24 25
72: 25 22 24
73: 25 22 24
74: 25 22
75: 25 22
76: 25 22
77: 25 22
78: 25 22
79: 25 22
80: 21 25 22
81: 21 22
82: 21 22
83: 21 22
84: 21 22
85: 21 22
86: 21 22
87: 21 22
88: 21 22
89: 22
90: 26 22
91: 26 22
92: 26 22
93: 26 22
94: 26
95: 26
96:
97:
98:
99: 26
100: 23
101: 23
102: 23
103:
104:
105:
106: 23
107: 23
108: 23
109: 23
110: 23
111: 23
112: 23
113: 23
114: 23
115: 23
116: 23
117: 26
118: 26
119: 26
120: 26
121: 26
122: 26
123: 26
124: 26
125:
126:
127: 20 22
128: 22 20
129: 20 22
130: 22 20
131: 22 20
132: 22 20
133: 22 20
134: 20 22
135: 20 22
136: 22 20
137: 22
138: 22
139: 21 22
140: 21 26 22
141: 21 26 22
142: 21 26 22
143: 21 26
144: 21 26
145: 21 26
146: 21 26
147: 21
148: 21
149: 21
150: 21
151: 21
152: 21
153: 21
154: 21
155: 21
156: 21
157: 21
158: 21
159: 21
160: 21
161: 21
162: 21
163: 21
164: 21
165: 21
166: 21
167: 21
168: 21
169: 21
170: 21
171: 21
 */