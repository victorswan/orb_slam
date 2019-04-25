/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include"../../../include/System.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 9)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings budget_per_frame "
             << " do_rectify do_viz "
             << " topic_img_l topic_img_r path_to_traj" << endl;
        ros::shutdown();
        return 1;
    }

    bool do_viz;
    stringstream s1(argv[5]);
    s1 >> boolalpha >> do_viz;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,do_viz);

    SLAM.SetBudgetPerFrame(std::atoi(argv[3]));

    std::string fNameRealTimeTrack = std::string(argv[8]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack);


    ImageGrabber igb(&SLAM);

    stringstream s2(argv[4]);
    s2 >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

#ifdef USE_FISHEYE_DISTORTION
        cv::fisheye::initUndistortRectifyMap(K_l,D_l,R_l,P_l,cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::fisheye::initUndistortRectifyMap(K_r,D_r,R_r,P_r,cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
        cout << "finish creating equidistant rectification map!" << endl;
#else
        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
        cout << "finish creating rad-tan rectification map!" << endl;
#endif

    }

    ros::NodeHandle nh;

    // message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, argv[6], 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, argv[7], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

while(ros::ok()) 
    ros::spin();
    // ros::spin();

    cout << "ros_stereo: done with spin!" << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM( std::string(argv[8]) + "_KeyFrameTrajectory.txt" );
    SLAM.SaveTrackingLog( std::string(argv[8]) + "_Log.txt" );

    //
    // SLAM.SaveGFLog( std::string(argv[6]) + "_LogGF.txt" );
    //
    // SLAM.SaveLmkLog( std::string(argv[8]) + "_Log_lmk.txt" );

    std::cout << "Finished saving!" << std::endl;

    ros::shutdown();

    cout << "ros_stereo: done with ros Shutdown!" << endl;

    // Stop all threads
    SLAM.Shutdown();
    cout << "ros_stereo: done with SLAM Shutdown!" << endl;

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose;
    //
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        //        cv::Mat out;
        //        cv::hconcat(imLeft, imRight, out);
        //        cv::imwrite( "./stereo_input.png", out );

        pose = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
        //        cv::Mat image_match;
        //        mpSLAM->mpTracker->mCurrentFrame.plotStereoMatching(imLeft, imRight, image_match);
        //        cv::imwrite( "./stereo_matching.png", image_match );
    }
    else
    {
        pose = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    if (pose.empty())
        return;

    //    std::cout << "broadcast pose!" << std::endl;

    /* global left handed coordinate system */
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                     -1, 1,-1, 1,
                                     -1,-1, 1, 1,
                                     1, 1, 1, 1);

    //prev_pose * T = pose
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();

    tf::Matrix3x3 tf3d;
    tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
                  pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
                  pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                            0, 0, 1,
                                            1, 0, 0);

    static tf::TransformBroadcaster br;

    tf::Matrix3x3 globalRotation_rh = tf3d;
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;

    tf::Quaternion tfqt;
    globalRotation_rh.getRotation(tfqt);

    double aux = tfqt[0];
    tfqt[0]=-tfqt[2];
    tfqt[2]=tfqt[1];
    tfqt[1]=aux;

    tf::Transform transform;
    transform.setOrigin(globalTranslation_rh);
    transform.setRotation(tfqt);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_pose"));
}


