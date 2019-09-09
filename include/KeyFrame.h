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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/serialization/split_member.hpp>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

#ifdef ENABLE_MAP_IO
    // For Map IO only; init keyframe from cv::FileStorage
    KeyFrame(Map *mMap, ORBVocabulary *mVocabulary, KeyFrameDatabase *mKeyFrameDatabase);
    KeyFrame(cv::FileStorage &fs, Map *mMap, ORBVocabulary *mVocabulary, KeyFrameDatabase *mKeyFrameDatabase);
#endif

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
    // For virtual KF creation only; cannot be used in regular KF processing (mapping; loop closing)
    KeyFrame(const double &timeStamp, const cv::Mat &Tcw,
             const float &fx_, const float &fy_, const float &cx_, const float &cy_, const float &mb_);
#endif


    void ExportToYML(cv::FileStorage &fs);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    cv::Mat GetOw();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame *pKF, const int &weight);
    void EraseConnection(KeyFrame *pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame *pKF);

    // Spanning tree functions
    void AddChild(KeyFrame *pKF);
    void EraseChild(KeyFrame *pKF);
    void ChangeParent(KeyFrame *pKF);
    std::set<KeyFrame *> GetChilds();
    KeyFrame *GetParent();
    bool hasChild(KeyFrame *pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame *pKF);
    std::set<KeyFrame *> GetLoopEdges();

    // MapPoint observation functions
    void AppendMapPoint(MapPoint *pMP);
    void AddMapPoint(MapPoint *pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint *pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);
    std::set<MapPoint *> GetMapPoints();
    std::vector<MapPoint *> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint *GetMapPoint(const size_t &idx);
    size_t GetMatchNum();

    //
    void AddCovisibleKeyFrames(KeyFrame *pKF);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp(const int a, const int b)
    {
        return a > b;
    }

    static bool idComp(const KeyFrame *pKF1, const KeyFrame *pKF2)
    {
        return pKF1->mnId < pKF2->mnId;
    }

    static bool timeStampComp(const KeyFrame *pKF1, const KeyFrame *pKF2)
    {
        return pKF1->mTimeStamp < pKF2->mTimeStamp;
    }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    static long unsigned int nNextId;
    long unsigned int mnId;

#if defined ENABLE_MAP_IO || defined ENABLE_ANTICIPATION_IN_GRAPH
    long unsigned int mnFrameId;

    double mTimeStamp;

    // Grid (to speed up feature matching)
    int mnGridCols;
    int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;
#else
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;
#endif

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKFCand;
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKFCand;
    long unsigned int mnBAFixedForKF;
    //
    long unsigned int mnBALocalCount;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

#if defined ENABLE_MAP_IO || defined ENABLE_ANTICIPATION_IN_GRAPH
    // Calibration parameters
    float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<float> mvuRight; // negative value for monocular points
    std::vector<float> mvDepth;  // negative value for monocular points
    cv::Mat mDescriptors;
#else
    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth;  // negative value for monocular points
    const cv::Mat mDescriptors;
#endif

    std::vector<cv::Mat> mvTrel;
    double mNumVisibleMpt;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

#if defined ENABLE_MAP_IO || defined ENABLE_ANTICIPATION_IN_GRAPH
    // Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    int mnMinX;
    int mnMinY;
    int mnMaxX;
    int mnMaxY;
    cv::Mat mK;
#else
    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;
#endif

    // The following variables need to be accessed trough a mutex to be thread safe.
public:
    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint *> mvpMapPoints;   //#1 need external setting

    // BoW
    KeyFrameDatabase *mpKeyFrameDB;
    ORBVocabulary *mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector<std::vector<std::vector<size_t>>> mGrid;

    std::map<KeyFrame *, int> mConnectedKeyFrameWeights;   //#2 need external setting
    std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames;   //#3 need external setting
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame *mpParent;   //#4 need external setting
    std::set<KeyFrame *> mspChildrens;   //#5 need external setting
    std::set<KeyFrame *> mspLoopEdges;   //#6 need external setting

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline; // Only for visualization

    Map *mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;

private:
    friend class boost::serialization::access;

    template<class Archive>
    void saveMat(Archive & ar, const cv::Mat &m) const
    {
        bool empty_flag = m.empty();
        ar << empty_flag;

        if(!empty_flag)
        {
            size_t elem_size = m.elemSize();
            int elem_type = m.type();

            ar << m.cols;
            ar << m.rows;
            ar << elem_size;
            ar << elem_type;

            const size_t data_size = m.cols * m.rows * elem_size;
            ar << boost::serialization::make_array(m.ptr(), data_size);
        }
    }

    template<class Archive>
    void loadMat(Archive & ar, cv::Mat &m)
    {
        bool empty_flag;
        ar >> empty_flag;

        if(!empty_flag)
        {
            int cols, rows;
            size_t elem_size;
            int elem_type;

            ar >> cols;
            ar >> rows;
            ar >> elem_size;
            ar >> elem_type;

            m.create(rows, cols, elem_type);
            size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }
    }

    template<class Archive, class Type>
    void saveVector(Archive & ar, const std::vector<Type> & vec) const
    {
        int size = vec.size();
        ar << size;

        std::vector<Type> vec_copy = vec;

        if(size > 0)
        {
            ar << boost::serialization::make_binary_object(vec_copy.data(), sizeof(Type) * size);
        }
    }

    template<class Archive, class Type>
    void loadVector(Archive & ar, std::vector<Type> & vec)
    {
        int size;
        ar >> size;
        vec.resize(size);
        if(size > 0)
        {
            ar >> boost::serialization::make_binary_object(vec.data(), sizeof(Type) * size);
        }
    }

    template<class Archive>
    void save(Archive &ar, unsigned int) const
    {
        ar << mnId;
        ar << mnFrameId;

        ar << mTimeStamp;

        ar << mnGridCols;
        ar << mnGridRows;
        ar << mfGridElementWidthInv;
        ar << mfGridElementHeightInv;

        ar << mnTrackReferenceForFrame;
        ar << mnFuseTargetForKF;

        ar << mnBALocalForKFCand;
        ar << mnBALocalForKF;
        ar << mnBAFixedForKFCand;
        ar << mnBAFixedForKF;

        ar << mnBALocalCount;

        ar << mnLoopQuery;
        ar << mnLoopWords;
        ar << mLoopScore;
        ar << mnRelocQuery;
        ar << mnRelocWords;
        ar << mRelocScore;

        ar << fx;
        ar << fy;
        ar << cx;
        ar << cy;
        ar << invfx;
        ar << invfy;
        ar << mbf;
        ar << mb;
        ar << mThDepth;

        int tmp_N = mvpMapPoints.size();
        ar << tmp_N;

        saveVector(ar, mvKeys);
        saveVector(ar, mvKeysUn);

        saveVector(ar, mvuRight);
        saveVector(ar, mvDepth);

        saveMat(ar, mDescriptors);

//        ar << mBowVec;
//        ar << mFeatVec;

        saveMat(ar, mTcp);

        ar << mnScaleLevels;
        ar << mfScaleFactor;
        ar << mfLogScaleFactor;
        ar << mvScaleFactors;
        ar << mvLevelSigma2;
        ar << mvInvLevelSigma2;

        ar << mnMinX;
        ar << mnMinY;
        ar << mnMaxX;
        ar << mnMaxY;
        saveMat(ar, mK);

        saveMat(ar, Tcw);
        saveMat(ar, Twc);
        saveMat(ar, Ow);

        saveMat(ar, Cw);

        ar << mGrid;

        ar << mvOrderedWeights;

        ar << mbFirstConnection;

        ar << mbNotErase;
        ar << mbToBeErased;
        ar << mbBad;

        ar << mHalfBaseline;
    }

    template<class Archive>
    void load(Archive &ar, unsigned int)
    {
        ar >> mnId;
        ar >> mnFrameId;

        ar >> mTimeStamp;

        ar >> mnGridCols;
        ar >> mnGridRows;
        ar >> mfGridElementWidthInv;
        ar >> mfGridElementHeightInv;

        ar >> mnTrackReferenceForFrame;
        ar >> mnFuseTargetForKF;

        ar >> mnBALocalForKFCand;
        ar >> mnBALocalForKF;
        ar >> mnBAFixedForKFCand;
        ar >> mnBAFixedForKF;

        ar >> mnBALocalCount;

        ar >> mnLoopQuery;
        ar >> mnLoopWords;
        ar >> mLoopScore;
        ar >> mnRelocQuery;
        ar >> mnRelocWords;
        ar >> mRelocScore;

        ar >> fx;
        ar >> fy;
        ar >> cx;
        ar >> cy;
        ar >> invfx;
        ar >> invfy;
        ar >> mbf;
        ar >> mb;
        ar >> mThDepth;

        ar >> N;

        loadVector(ar, mvKeys);
        loadVector(ar, mvKeysUn);

        loadVector(ar, mvuRight);
        loadVector(ar, mvDepth);

        loadMat(ar, mDescriptors);

//        ar >> mBowVec;
//        ar >> mFeatVec;

        loadMat(ar, mTcp);

        ar >> mnScaleLevels;
        ar >> mfScaleFactor;
        ar >> mfLogScaleFactor;
        ar >> mvScaleFactors;
        ar >> mvLevelSigma2;
        ar >> mvInvLevelSigma2;

        ar >> mnMinX;
        ar >> mnMinY;
        ar >> mnMaxX;
        ar >> mnMaxY;
        loadMat(ar, mK);

        loadMat(ar, Tcw);
        loadMat(ar, Twc);
        loadMat(ar, Ow);

        loadMat(ar, Cw);

        ar >> mGrid;

        ar >> mvOrderedWeights;

        ar >> mbFirstConnection;

        ar >> mbNotErase;
        ar >> mbToBeErased;
        ar >> mbBad;

        ar >> mHalfBaseline;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

public:
    void saveExtern(boost::archive::binary_oarchive &ar) const;
    void loadExtern(boost::archive::binary_iarchive &ar, std::map<long unsigned int, ORB_SLAM2::KeyFrame *> &map_ID_2_KF, std::map<long unsigned int, ORB_SLAM2::MapPoint *> &map_ID_2_Pt);
};

} // namespace ORB_SLAM2

#endif // KEYFRAME_H
