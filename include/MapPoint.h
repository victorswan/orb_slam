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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#define ARMA_NO_DEBUG
#include "armadillo"

#include <opencv2/core/core.hpp>
#include <mutex>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/serialization/split_member.hpp>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap);
    MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF);

    MapPoint(Map *pMap);
    MapPoint(cv::FileStorage &fs, Map *pMap);

    void ExportToYML(cv::FileStorage &fs);

    static bool isLessID(MapPoint *pMP1, MapPoint *pMP2)
    {
        return pMP1->mnId < pMP2->mnId;
    }

    // for unit test only; not used in actual application
    MapPoint(){};

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame *GetReferenceKeyFrame();

    //
    void SetReferenceKeyFrame(KeyFrame *mRKF);

    std::map<KeyFrame *, size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame *pKF, size_t idx);
    void EraseObservation(KeyFrame *pKF);

    int GetIndexInKeyFrame(KeyFrame *pKF);
    bool IsInKeyFrame(KeyFrame *pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint *pMP);
    MapPoint *GetReplaced();

    void IncreaseVisible(int n = 1);
    void IncreaseFound(int n = 1);
    float GetFoundRatio();
    inline int GetFound()
    {
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame *pKF);
    int PredictScale(const float &currentDist, Frame *pF);

public:
    static long unsigned int nNextId;
    long unsigned int mnId;    
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKFCand;
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

    // XXX: Changed from protected to public!
    // Tracking counters
    int mnVisible;
    int mnFound;

    std::vector<size_t> mvMatchCandidates;

    // Observability
    arma::mat H_meas;
    arma::mat H_proj;
    arma::mat ObsMat;
    arma::vec ObsVector;
    double ObsScore;
    int ObsRank;
    //
    float u_proj, v_proj;
    //
    long unsigned int matchedAtFrameId;
    long unsigned int updateAtFrameId;
    long unsigned int goodAtFrameId;
    long unsigned int mnUsedForLocalMap;

    static std::mutex mGlobalMutex;

protected:
    // Position in absolute coordinates
    cv::Mat mWorldPos;

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame *, size_t> mObservations; //#1 need extern setting

    // Mean viewing direction
    cv::Mat mNormalVector;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;

    // Reference KeyFrame
    KeyFrame *mpRefKF;  //#2 need extern setting

    //  // Tracking counters
    //  int mnVisible;
    //  int mnFound;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    MapPoint *mpReplaced;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    Map *mpMap;

    std::mutex mMutexPos;
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
            ar >> boost::serialization::make_array(m.ptr(), data_size);
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

    friend class boost::serialization::access;
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
        ar << mnFirstKFid;
        ar << mnFirstFrame;
        ar << nObs;

        ar << mTrackProjX;
        ar << mTrackProjY;
        ar << mTrackProjXR;
        ar << mbTrackInView;
        ar << mnTrackScaleLevel;
        ar << mTrackViewCos;
        ar << mnTrackReferenceForFrame;
        ar << mnLastFrameSeen;

        ar << mnBALocalForKFCand;
        ar << mnBALocalForKF;
        ar << mnFuseCandidateForKF;

        ar << mnLoopPointForKF;
        ar << mnCorrectedByKF;
        ar << mnCorrectedReference;

        saveMat(ar, mPosGBA);

        ar << mnBAGlobalForKF;

        ar << mnVisible;
        ar << mnFound;

        saveVector(ar, mvMatchCandidates);

        saveMat(ar, mWorldPos);

        saveMat(ar, mNormalVector);
        saveMat(ar, mDescriptor);

        ar << mnVisible;
        ar << mnFound;

        ar << mbBad;

        ar << mfMinDistance;
        ar << mfMaxDistance;
    }

    friend class boost::serialization::access;
    template<class Archive>
    void load(Archive &ar, unsigned int)
    {
        ar >> mnId;
        ar >> mnFirstKFid;
        ar >> mnFirstFrame;
        ar >> nObs;

        ar >> mTrackProjX;
        ar >> mTrackProjY;
        ar >> mTrackProjXR;
        ar >> mbTrackInView;
        ar >> mnTrackScaleLevel;
        ar >> mTrackViewCos;
        ar >> mnTrackReferenceForFrame;
        ar >> mnLastFrameSeen;

        ar >> mnBALocalForKFCand;
        ar >> mnBALocalForKF;
        ar >> mnFuseCandidateForKF;

        ar >> mnLoopPointForKF;
        ar >> mnCorrectedByKF;
        ar >> mnCorrectedReference;

        loadMat(ar, mPosGBA);

        ar >> mnBAGlobalForKF;

        ar >> mnVisible;
        ar >> mnFound;

        loadVector(ar, mvMatchCandidates);

        loadMat(ar, mWorldPos);

        loadMat(ar, mNormalVector);
        loadMat(ar, mDescriptor);

        ar >> mnVisible;
        ar >> mnFound;

        ar >> mbBad;

        ar >> mfMinDistance;
        ar >> mfMaxDistance;
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();

public:
    void saveExtern(boost::archive::binary_oarchive &ar) const;
    void loadExtern(boost::archive::binary_iarchive &ar, std::map<long unsigned int, ORB_SLAM2::KeyFrame *> &map_ID_2_KF, std::map<long unsigned int, ORB_SLAM2::MapPoint *> &map_ID_2_Pt);
};

} // namespace ORB_SLAM2

#endif // MAPPOINT_H
