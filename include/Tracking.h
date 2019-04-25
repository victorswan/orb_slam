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

#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include "Observability.h"

#include <set>
#include <utility>
#include <algorithm>

#include <Eigen/Dense>
using namespace Eigen;

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <mutex>


//#define SIMU_MOTION_BLUR
//#define TIMECOST_VERBOSE
//#define LMKNUM_VERBOSE


/* --- options of baseline methods --- */
//#define ORB_SLAM_BASELINE
//#define DISABLE_RELOC

/* --- options of non-necessary viz codes --- */
// when running on long-term large-scale dataset, this will save us a lot of time!
#define DISABLE_MAP_VIZ

/* --- options of key-frame insert condition --- */
//#define SPARSE_KEYFRAME_COND

/* --- options to priortize feature matching wrt local map --- */
#ifndef ORB_SLAM_BASELINE

    /* --- options of additional search after pose estimation --- */
    #define DELAYED_MAP_MATCHING

    /* --- options to reduce the size of local map with good ones only --- */
    // TODO add map hash macros here

    // #define RANDOM_FEATURE_MAP_MATCHING
    // #define LONGLIVE_FEATURE_MAP_MATCHING

    #define GOOD_FEATURE_MAP_MATCHING
    // include frame-by-frame matchings as prior term in good matching
    // #define FRAME_MATCHING_INFO_PRIOR
    // pre-compute Jacobian for next frame at the end of tracking
    // TODO disable it when using map hash; check the latency vs. performance
    #define PRECOMPUTE_WITH_MOTION_MODEL

    #define USE_INFO_MATRIX
    //#define USE_HYBRID_MATRIX
    //#define USE_OBSERVABILITY_MATRIX

    // limit the budget of computing matrices of existing matches at current frame to 2ms
    #define MATRIX_BUDGET_REALTIME  0.002 // 0.1 // 
    // limit the budget of predicting matrices at next frame to 2ms
    #define MATRIX_BUDGET_PREDICT   0.002

    // For low-power devices with 2-cores, disable multi-thread matrix building
    #define USE_MULTI_THREAD        true // false //

#endif

/* --- options to fair comparison wrt other VO pipelines --- */
// time to init tracking with full feature set
#define TIME_INIT_TRACKING      5 // 10 //

#define THRES_INIT_MPT_NUM          100 // 50
#define SRH_WINDOW_SIZE_INIT        100

#define MAX_FRAME_LOSS_DURATION     5

/* --- options of feature subset selection methods --- */
//#define RANDOM_FAIR_COMPARISON
//#define BUCKETING_FAIR_COMPARISON
#define BUCKET_WIDTH                50
#define BUCKET_HEIGHT               50
//#define LONGEST_TRACK_FAIR_COMPARISON
//#define QUALITY_FAIR_COMPARISON
//#define GOOD_FEATURE_FAIR_COMPARISON
//#define RANDOM_ENHANCEMENT
//#define QUALITY_ENHANCEMENT

//#define WITH_IMU_PREINTEGRATION
//#define MIN_NUM_MATCHES           30 // 50
//#define MIN_NUM_GOOD_FEATURES     40 // For KITTI
//#define MIN_NUM_GOOD_FEATURES     30 // For TUM
//#define MIN_NUM_GOOD_FEATURES       20 // 30 // 50 // 80 // (with 80 the performance of GF is closed to ALL) // 100 // For EuRoC
// 40 is used when insert GF at trackMotionModel

/* --- options to good feature selection; discarded since good feature 2.0 --- */
//#define ENABLE_EXPLICIT_OUTLIER_REJ
//#define RANSAC_POOL_FOR_SUBSET
//#define RANSAC_ITER_NUMBER      150 // 100 // 50 //

/* --- options of candidate pooling methods; discarded since good feature 2.0 --- */
//#define QUALITY_POOL_FOR_SUBSET
//#define QUALITY_POOL_SCALE         2 // 1.25 // 2.5 // 1.0 // 1.5 // 1.75 //
//#define QUALITY_POOL_PERCENTILE     0.75 // 0.5 // 0.3 // 0.85 //
//#define QUALITY_POOL_MAXSIZE        250

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // for test only
    Tracking(const cv::Mat K, const cv::Mat DistCoef) {
        K.copyTo(mK);
        DistCoef.copyTo(mDistCoef);
    }


    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


    ~Tracking();

    // init file stream for writing real time tracking result
    void SetRealTimeFileStream(string fNameRealTimeTrack);

    void updateORBExtractor();

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    arma::mat mCurrentInfoMat;
    int mnInitStereo;

    // Observability computation
    Observability * mObsHandler;

    size_t num_good_constr_predef;
    //    double ratio_good_inlier_predef;
    //    size_t num_good_feature_found;


    //    int mToMatchMeasurement;
    //    int mMatchedLocalMapPoint;

    //
    bool first_hit_tracking;
    double time_frame_init;
    double camera_fps;

    //    vector<LmkSelectionInfo> obs_thres_arr;
    //    vector<FramePose> mFramePoseSeq;
    //    vector<std::pair<double, int> > mFrameInlierSeq;

    // Time log
    vector<TimeLog> mFrameTimeLog;
    TimeLog logCurrentFrame;

    //
    void BucketingMatches(const Frame *pFrame, vector<GoodPoint> & mpBucketed);
    void LongLivedMatches(const Frame *pFrame, vector<GoodPoint> & mpLongLived);
    void RanSACMatches(const Frame *pFrame, vector<GoodPoint> & mpRanSAC);

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();

    int SearchLocalPoints();
    void SearchAdditionalMatchesInFrame(const double time_for_search, Frame & F);

    void PredictJacobianNextFrame(const double time_for_predict, const size_t pred_horizon);

    bool NeedNewKeyFrame();
    //
    bool NeedNewKeyFrame_Experimental();
    void CreateNewKeyFrame();


    void PlotFrameWithPointMatches();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mK_ori, mDistCoef, mR, mP;
    float mbf;
    //
    // for right camera undistortion
    cv::Mat mK_right, mDistCoef_right, mR_right, mP_right;

    //
    cv::Mat mMap1_l, mMap2_l, mMap1_r, mMap2_r;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // frame counter after initialization
    size_t mFrameAfterInital;

    size_t mbTrackLossAlert;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    //
    //    int budget_matching_in_track = 150; // 60; // 100; //

    std::ofstream f_realTimeTrack;

};

} //namespace ORB_SLAM

#endif // TRACKING_H
