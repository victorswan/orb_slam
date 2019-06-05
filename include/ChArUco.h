//
// Created by parallels on 7/16/18.
//

#ifndef DETECTION_CPP_CHARUCO_H
#define DETECTION_CPP_CHARUCO_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <string>

// #define VIZ_ARUCO

namespace ORB_SLAM2
{
class ChArUco {
public:
    //ChArUco(int squaresX_, int squareY_, int dictionaryId_, float squareLength_, float markerLength_,
    //      bool showRejected_, bool refindStrategy_, string cameraFile_, string paramsFile_)
//    ChArUco(int squaresX_ = 5, int squaresY_ = 7, int dictionaryId_= 14, float squareLength_ = 0.078, float markerLength_ = 0.058,
//            bool showRejected_ = false, bool refindStrategy_ = false,
//            std::string cameraFile_ = "/home/yipuzhao/ros_workspace/package_dir/ORB_Data/charuco_calib.yml",
 //           std::string paramsFile_ = "/home/yipuzhao/ros_workspace/package_dir/ORB_Data/charuco_param.yml");
        ChArUco(const cv::Mat &camMatrix_, 
		 const cv::Mat &distCoeffs_, 
		 const std::string &paramsFile_ = std::string("/home/yipuzhao/ros_workspace/package_dir/ORB_Data/charuco_param.yml"), 
		 const bool &showRejected_ = false, 
		 const bool &refindStrategy_ = false);

    ~ChArUco() {};

    bool loadParameters(const std::string &filename);

    bool process(const cv::Mat& image, cv::Mat & Twc);
    
private:
  //
    int squaresX, squaresY, dictionaryId, totalIterations = 0;
    float squareLength, markerLength, totalTime = 0;
    bool showRejected, refindStrategy;
    std::string paramsFile;
    cv::Mat camMatrix, distCoeffs;
    cv::Mat image_viz;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard;
    cv::Ptr<cv::aruco::Board> board;
};

}// namespace ORB_SLAM

#endif //DETECTION_CPP_CHARUCO_H

