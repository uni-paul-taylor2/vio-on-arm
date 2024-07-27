#ifndef SLAM_H
#define SLAM_H


/**
 * Pose detection and vSLAM from video feed
 * Pose detection:
 *      tag family
 *      camera intrinsics
 *      distortion coeffecients
 *      video
 * SLAM (GTSAM)
 *      ?
 *
 * Process:
 *      name of dir passed as cmd arg
 *      info.json found and parsed
 *      VSLAM obj created and passed the camInts, distCoeffs, tag family
 *      each frame of the vid asscced, transformed to cv::Mat/cvFrame? and passed to VSLAM obj
 *      for each frame passed, VLSAM obj:
 *          detects markers
 *          sets world (if first) or check if at least one of the tags were seen before
 *          detects pose of tags
 *          for redetected tags:
 *              passes (u,v) to factor graph
 *          for new tags:
 *              calcs pose wrt to world and passes to factor graph
 *          calcs cameara pose wrt world and passes to factor graph
 *          updates factor graph (perhaps more than once?)
 *
 *
 *
 *  (REMEMBER: data validation)
 *
 */
class VSLAM {
public:
    /**
     * @param tag: aruco tag used
     * @param cameraIntrinsics
     * @param distCoeffs
     * ...?
     */
    VSLAM();

};



#endif //SLAM_H
