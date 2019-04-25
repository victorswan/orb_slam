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

#include <unistd.h>

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
            "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cout << "Try loading yaml from " << strSettingsFile.c_str() << endl;
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    clock_t tStart = clock();

    mpVocabulary = new ORBVocabulary();
    // bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    // cout << "Vocabulary loaded!" << endl << endl;
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    mFrameLossTrack = 0;

}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;


    //
    if (mTrackingState == Tracking::LOST) {
        mFrameLossTrack ++;
        std::cout << "Track Lost!" << std::endl;
    }
    else {
        // reset the frame counter
        mFrameLossTrack = 0;
    }

    // terminate the whole pipeline if certain amount of frames being loss track
    if (mFrameLossTrack > MAX_FRAME_LOSS_DURATION * mpTracker->camera_fps) {
        std::cout << "Too many frames loss track, terminate the SLAM pipeline!" << std::endl;
        Shutdown();
    }


    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    //
    if (mTrackingState == Tracking::LOST) {
        mFrameLossTrack ++;
        std::cout << "Track Lost!" << std::endl;
    }
    else {
        // reset the frame counter
        mFrameLossTrack = 0;
    }

    // terminate the whole pipeline if certain amount of frames being loss track
    if (mFrameLossTrack > MAX_FRAME_LOSS_DURATION * mpTracker->camera_fps) {
        std::cout << "Too many frames loss track, terminate the SLAM pipeline!" << std::endl;
        Shutdown();
    }

    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;


    //
    if (mTrackingState == Tracking::LOST) {
        mFrameLossTrack ++;
        std::cout << "Track Lost!" << std::endl;
    }
    else {
        // reset the frame counter
        mFrameLossTrack = 0;
    }

    // terminate the whole pipeline if certain amount of frames being loss track
    if (mFrameLossTrack > MAX_FRAME_LOSS_DURATION * mpTracker->camera_fps) {
        std::cout << "Too many frames loss track, terminate the SLAM pipeline!" << std::endl;
        Shutdown();
    }

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    cout << "At the stage 1 of system Shutdown!" << endl;

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    cout << "At the stage 2 of system Shutdown!" << endl;

    // NOTE
    // for some reason the shutdown func gets blocked at this line;
    // disabled the text change for smooth termination & logging
    //    if(mpViewer)
    //        pangolin::BindToContext("ORB-SLAM2: Map Viewer");

    cout << "At the end of system Shutdown!" << endl;
}

void System::SetRealTimeFileStream(const string &filename)
{
    mpTracker->SetRealTimeFileStream(filename);
    std::cout << "mpTracker real time output to " << filename << std::endl;
}

void System::SetBudgetPerFrame(const size_t budget_per_frame)
{
    mpTracker->num_good_constr_predef = budget_per_frame;
#ifdef ORB_SLAM_BASELINE
    // baseline only
    // re-create orb extractor with the defined budget per frame
    mpTracker->updateORBExtractor();
#endif
    std::cout << "mpTracker budget adjusted to " << mpTracker->num_good_constr_predef << std::endl;
}

void System::GrabAllLmkLog() {

    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    if(vpMPs.empty())
        return;

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        MapPoint * pMP = vpMPs[i];
        if(!pMP || pMP->isBad())
            continue;
        //
        const map<KeyFrame*,size_t> observations = pMP->GetObservations();
        size_t N = observations.size();
        if (N == 0)
            continue ;
        size_t id = pMP->mnId;
        size_t life = observations.rbegin()->first->mnFrameId - observations.begin()->first->mnFrameId + 1;
        logLmkLife.push_back(LmkLog(id, life));
    }
}

void System::SaveLmkLog(const std::string &filename) {

    GrabAllLmkLog();

    size_t N = logLmkLife.size();
    std::cout << std::endl << "Saving " << N << " records to lmk log file " << filename << " ..." << std::endl;

    std::ofstream fLmkLog;
    fLmkLog.open(filename.c_str());
    fLmkLog << std::fixed;
    fLmkLog << "#id life" << std::endl;
    for(size_t i=0; i<N; i++)
    {
        fLmkLog << std::setprecision(0)
                << logLmkLife[i].id << " "
                << logLmkLife[i].life << std::endl;
    }
    fLmkLog.close();

    std::cout << "Finished saving lmk log! " << std::endl;
}

void System::SaveGFLog(const string &filename) {
    //
    std::cout << endl << "Saving running log to " << filename << " ..." << std::endl;
    ofstream fFrameGFLog;
    fFrameGFLog.open(filename.c_str());
    fFrameGFLog << fixed;
    fFrameGFLog << "#frame_time_stamp time_match lmk_num_initTrack  lmk_num_refTrack lmk_num_refInlier log_det_frame" << std::endl;
    for(size_t i=0; i<mpTracker->mFrameTimeLog.size(); i++)
    {
        fFrameGFLog << setprecision(6)
                    << mpTracker->mFrameTimeLog[i].frame_time_stamp << " "
                    << mpTracker->mFrameTimeLog[i].time_match << " "
                    << mpTracker->mFrameTimeLog[i].log_det_frame << " "
                    << setprecision(0)
                    << mpTracker->mFrameTimeLog[i].lmk_num_motion << " "
                    << mpTracker->mFrameTimeLog[i].lmk_num_map << " "
                    << mpTracker->mFrameTimeLog[i].lmk_num_BA << std::endl;
    }
    fFrameGFLog.close();
}

void System::SaveTrackingLog(const string &filename) {
    //
    std::cout << endl << "Saving running log to " << filename << " ..." << std::endl;
    ofstream fFrameTimeLog;
    fFrameTimeLog.open(filename.c_str());
    fFrameTimeLog << fixed;
    fFrameTimeLog << "#frame_time_stamp time_ORB_extraction time_track_motion time_track_frame time_track_map time_match ..." << std::endl;
    for(size_t i=0; i<mpTracker->mFrameTimeLog.size(); i++)
    {

        fFrameTimeLog << setprecision(6)
                      << mpTracker->mFrameTimeLog[i].frame_time_stamp << " "
                      << mpTracker->mFrameTimeLog[i].time_rectification + mpTracker->mFrameTimeLog[i].time_ORB_extraction << " "
                      << mpTracker->mFrameTimeLog[i].time_track_motion << " "
                      << mpTracker->mFrameTimeLog[i].time_track_frame << " "
                      << mpTracker->mFrameTimeLog[i].time_track_map << " "
                      << mpTracker->mFrameTimeLog[i].time_match << " "
                      << mpTracker->mFrameTimeLog[i].time_select << " "
                      << mpTracker->mFrameTimeLog[i].time_optim << " "
                      << mpTracker->mFrameTimeLog[i].time_stereo_motion << " "
                      << mpTracker->mFrameTimeLog[i].time_stereo_frame << " "
                      << mpTracker->mFrameTimeLog[i].time_stereo_map << " "
                      << mpTracker->mFrameTimeLog[i].time_stereo_post << " "
                      << mpTracker->mFrameTimeLog[i].time_post_proc << " "
                      << mpTracker->mFrameTimeLog[i].time_mat_online << " "
                      << setprecision(0)
                      << mpTracker->mFrameTimeLog[i].lmk_num_motion << " "
                      << mpTracker->mFrameTimeLog[i].lmk_num_frame << " "
                      << mpTracker->mFrameTimeLog[i].lmk_num_map << " "
                      << mpTracker->mFrameTimeLog[i].lmk_num_BA << std::endl;
    }
    fFrameTimeLog.close();
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

/*
static void FindAllFilesInDir( const std::string & dir, const std::string & ext,
                               std::list<std::string> * filename_list,
                               size_t & filename_maxlen ) {
    // get a sorted list of files in the img directories
    boost::filesystem::path files_dir(dir.c_str());
    if (!boost::filesystem::exists(files_dir))
    {
        std::cout << std::endl << "Left image directory does not exist: \t" << files_dir << std::endl;
        return ;
    }

    // get all files in the img directories
    filename_maxlen = 0;
    filename_list->clear();
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator file(files_dir); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        if (boost::filesystem::is_regular_file(file->status()) &&
                filename_path.extension() == ext.c_str() )
        {
            std::string filename(filename_path.string());
            filename_list->push_back(filename);
            filename_maxlen = max(filename_maxlen, filename.length());
        }
    }
    std::cout << std::endl << "Found " << filename_list->size() << " files under the given dir with extension " << ext << std::endl;
}
*/

//// Load & Save Function
//void System::LoadMap(const std::string & map_path) {
//    //loading KeyFrames..
//    cout<<"loading KeyFrames.."<<endl;
//    vector<ORB_SLAM2::KeyFrame*> vpKFs;
//    vector<vector<double> > mvMapPointsVector;
//    vector<vector<double> > mConnectedKeyFrameWeights_firstVector;
//    vector<vector<int> > mConnectedKeyFrameWeights_secondVector;
//    vector<vector<double> >mvOrderedConnectedKeyFramesVector;
//    vector<double> mParentVector;
//    vector<vector<double> > msChildrensVector;
//    vector<vector<double> > msLoopEdgesVector;
//    boost::unordered_map<long unsigned int, ORB_SLAM2::KeyFrame*> id2pKFmap;

//    std::list<std::string> KF_file_list;
//    size_t KF_file_maxlen;
//    FindAllFilesInDir(map_path + "/KeyFrames", "yml", KF_file_list, KF_file_maxlen);
//    //
//    for(int i = 0; i < KF_file_list.size(); i++){
//        ORB_SLAM2::KeyFrame* pKF = new ORB_SLAM2::KeyFrame;
//        //        string KFfileName = ros::package::getPath("ORB_SLAM")+"/bin/KeyFrames/car_sample/"+"keyframe_" + boost::to_string(i) + ".yml";
//        //        std::string KFfileName = map_path + "/KeyFrames/keyframe_" + boost::to_string(i) + ".yml";
//        std::string KFfileName = map_path + "/KeyFrames/" + KF_file_list[i];

//        cv::FileStorage fs(KFfileName.c_str(), cv::FileStorage::READ);
//        if(!fs.isOpened()) cout<<"no such file!"<<endl;

//        double nNextId; fs["nNextId"] >> nNextId; pKF->nNextId = nNextId;// automatically transform
//        double mnId; fs["mnId"] >> mnId; pKF->mnId = mnId;
//        double mnFrameId; fs["mnFrameId"] >> mnFrameId; pKF->mnFrameId = mnFrameId;

//        fs["mTimeStamp"] >> pKF->mTimeStamp;

//        fs["mnGridCols"] >> pKF->mnGridCols;
//        fs["mnGridRows"] >> pKF->mnGridRows;
//        fs["mfGridElementWidthInv"] >> pKF->mfGridElementWidthInv;
//        fs["mfGridElementHeightInv"] >> pKF->mfGridElementHeightInv;

//        double mnTrackReferenceForFrame; fs["mnTrackReferenceForFrame"] >> mnTrackReferenceForFrame; pKF->mnTrackReferenceForFrame = mnTrackReferenceForFrame;
//        double mnFuseTargetForKF; fs["mnFuseTargetForKF"] >> mnFuseTargetForKF; pKF->mnFuseTargetForKF = mnFuseTargetForKF;

//        double mnBALocalForKF; fs["mnBALocalForKF"] >> mnBALocalForKF; pKF->mnBALocalForKF = mnBALocalForKF;
//        double mnBAFixedForKF; fs["mnBAFixedForKF"] >> mnBAFixedForKF; pKF->mnBAFixedForKF = mnBAFixedForKF;

//        double mnLoopQuery; fs["mnLoopQuery"] >> mnLoopQuery; pKF->mnLoopQuery = mnLoopQuery;
//        fs["mnLoopWords"] >> pKF->mnLoopWords;
//        fs["mLoopScore"] >> pKF->mLoopScore;
//        double mnRelocQuery; fs["mnRelocQuery"] >> mnRelocQuery; pKF->mnRelocQuery = mnRelocQuery;
//        fs["mnRelocWords"] >> pKF->mnRelocWords;
//        fs["mRelocScore"] >> pKF->mRelocScore;

//        fs["fx"] >> pKF->fx;
//        fs["fy"] >> pKF->fy;
//        fs["cx"] >> pKF->cx;
//        fs["cy"] >> pKF->cy;

//        fs["Tcw"] >> pKF->Tcw;
//        fs["Ow"] >> pKF->Ow;

//        fs["im"] >> pKF->im;
//        fs["mnMinX"] >> pKF->mnMinX;
//        fs["mnMinY"] >> pKF->mnMinY;
//        fs["mnMaxX"] >> pKF->mnMaxX;
//        fs["mnMaxY"] >> pKF->mnMaxY;
//        fs["mK"] >> pKF->mK;

//        cv::FileNode mvKeysFileNode = fs["mvKeys"];//>> no operator overloading for KeyPoint
//        std::read(mvKeysFileNode, pKF->mvKeys);
//        cv::FileNode mvKeysUnFileNode = fs["mvKeysUn"];
//        std::read(mvKeysUnFileNode, pKF->mvKeysUn);
//        fs["mDescriptors"] >> pKF->mDescriptors;

//        vector<double> mvMapPoints;
//        fs["mvMapPoints"] >> mvMapPoints;
//        mvMapPointsVector.push_back(mvMapPoints);

//        pKF->mpKeyFrameDB = mpVocabulary;
//        pKF->mpORBvocabulary = mpKeyFrameDatabase;
//        vector<cv::Mat> vCurrentDesc = ORB_SLAM2::Converter::toDescriptorVector(pKF->mDescriptors);
//        pKF->mpORBvocabulary->transform(vCurrentDesc,pKF->mBowVec,pKF->mFeatVec,4);

//        std::vector<int> mGrid_serialized;
//        std::vector<int> mGridSize;
//        fs["mGrid"] >> mGrid_serialized;
//        fs["mGridSize"] >> mGridSize;
//        for(size_t i = 0; i < 64; i++){
//            std::vector<std::vector<size_t> > mGridyz;
//            for(size_t j = 0; j < 48; j++){
//                std::vector<size_t> mGridz;
//                int sz = mGridSize.front();
//                mGridSize.erase(mGridSize.begin());
//                for(int k = 0; k < sz; k++){
//                    mGridz.push_back(size_t(mGrid_serialized.front()));
//                    mGrid_serialized.erase(mGrid_serialized.begin());
//                }
//                mGridyz.push_back(mGridz);
//            }
//            pKF->mGrid.push_back(mGridyz);
//        }

//        vector<double> mConnectedKeyFrameWeights_first;
//        vector<int> mConnectedKeyFrameWeights_second;
//        fs["mConnectedKeyFrameWeights_first"] >> mConnectedKeyFrameWeights_first;
//        fs["mConnectedKeyFrameWeights_second"] >> mConnectedKeyFrameWeights_second;
//        mConnectedKeyFrameWeights_firstVector.push_back(mConnectedKeyFrameWeights_first);
//        mConnectedKeyFrameWeights_secondVector.push_back(mConnectedKeyFrameWeights_second);

//        vector<double> mvOrderedConnectedKeyFrames;
//        fs["mvOrderedConnectedKeyFrames"] >> mvOrderedConnectedKeyFrames;
//        mvOrderedConnectedKeyFramesVector.push_back(mvOrderedConnectedKeyFrames);

//        fs["mvOrderedWeights"] >> pKF->mvOrderedWeights;

//        fs["mbFirstConnection"] >> pKF->mbFirstConnection;
//        double mParent;
//        fs["mParent"] >> mParent;
//        mParentVector.push_back(mParent);
//        vector<double> msChildrens;
//        fs["msChildrens"] >> msChildrens;
//        msChildrensVector.push_back(msChildrens);
//        vector<double> msLoopEdges;
//        fs["msLoopEdges"] >> msLoopEdges;
//        msLoopEdgesVector.push_back(msLoopEdges);

//        fs["mbNotErase"] >> pKF->mbNotErase;
//        fs["mbToBeErased"] >> pKF->mbToBeErased;
//        fs["mbBad"] >> pKF->mbBad;

//        fs["mnScaleLevels"] >> pKF->mnScaleLevels;
//        fs["mvScaleFactors"] >> pKF->mvScaleFactors;
//        fs["mvLevelSigma2"] >> pKF->mvLevelSigma2;
//        fs["mvInvLevelSigma2"] >> pKF->mvInvLevelSigma2;

//        pKF->mpMap = mpMap;

//        id2pKFmap[pKF->mnId] = pKF;
//        vpKFs.push_back(pKF);

//        fs.release();
//    }//end of creating KeyFrames


//    //loading MapPoints..
//    cout<<"loading MapPoints.."<<endl;
//    vector<ORB_SLAM2::MapPoint*> vpMPs;
//    vector<vector<double> > mObservations_firstVector;
//    vector<vector<int> > mObservations_secondVector;
//    vector<double> mRefKFVector;
//    boost::unordered_map<long unsigned int, ORB_SLAM2::MapPoint*> id2pMPmap;

//    std::list<std::string> MP_file_list;
//    size_t MP_file_maxlen;
//    FindAllFilesInDir(map_path + "/MapPoints", "yml", MP_file_list, MP_file_maxlen);

//    for(size_t i=0; i<MP_file_list.size(); i++){
//        ORB_SLAM2::MapPoint* pMP = new ORB_SLAM2::MapPoint;
//        //        string MPfileName = ros::package::getPath("ORB_SLAM")+"/bin/MapPoints/car_sample/"+"mappoint_" + boost::to_string(i) + ".yml";
//        //        std::string MPfileName = map_path + "/MapPoints/mappoint_" + boost::to_string(i) + ".yml";
//        std::string MPfileName = map_path + "/MapPoints/" + MP_file_list[i];

//        cv::FileStorage fs(MPfileName.c_str(), cv::FileStorage::READ);
//        if(!fs.isOpened()) cout<<"no such file!"<<endl;

//        double mnId; fs["mnId"] >> mnId; pMP->mnId = mnId;
//        double nNextId; fs["nNextId"] >> nNextId; pMP->nNextId = nNextId;
//        double mnFirstKFid; fs["mnFirstKFid"] >> mnFirstKFid; pMP->mnFirstKFid = mnFirstKFid;

//        fs["mTrackProjX"] >> pMP->mTrackProjX;
//        fs["mTrackProjY"] >> pMP->mTrackProjY;
//        fs["mbTrackInView"] >> pMP->mbTrackInView;
//        fs["mnTrackScaleLevel"] >> pMP->mnTrackScaleLevel;
//        fs["mTrackViewCos"] >> pMP->mTrackViewCos;
//        double mnTrackReferenceForFrame; fs["mnTrackReferenceForFrame"] >> mnTrackReferenceForFrame; pMP->mnTrackReferenceForFrame = mnTrackReferenceForFrame;
//        double mnLastFrameSeen; fs["mnLastFrameSeen"] >> mnLastFrameSeen; pMP->mnLastFrameSeen = mnLastFrameSeen;

//        double mnBALocalForKF; fs["mnBALocalForKF"] >> mnBALocalForKF; pMP->mnBALocalForKF = mnBALocalForKF;
//        double mnFuseCandidateForKF; fs["mnFuseCandidateForKF"] >> mnFuseCandidateForKF; pMP->mnFuseCandidateForKF = mnFuseCandidateForKF;

//        double mnLoopPointForKF; fs["mnLoopPointForKF"] >> mnLoopPointForKF; pMP->mnLoopPointForKF = mnLoopPointForKF;
//        double mnCorrectedByKF; fs["mnCorrectedByKF"] >> mnCorrectedByKF; pMP->mnCorrectedByKF = mnCorrectedByKF;
//        double mnCorrectedReference; fs["mnCorrectedReference"] >> mnCorrectedReference; pMP->mnCorrectedReference = mnCorrectedReference;

//        fs["mWorldPos"] >> pMP->mWorldPos;

//        vector<double> mObservations_first;
//        vector<int> mObservations_second;
//        fs["mObservations_first"] >> mObservations_first;
//        fs["mObservations_second"] >> mObservations_second;
//        mObservations_firstVector.push_back(mObservations_first);
//        mObservations_secondVector.push_back(mObservations_second);

//        fs["mNormalVector"] >> pMP->mNormalVector;
//        fs["mDescriptor"] >> pMP->mDescriptor;

//        //TODO mpRefKF
//        double mRefKF;
//        fs["mRefKF"] >> mRefKF;
//        mRefKFVector.push_back(mRefKF);

//        fs["mnVisible"] >> pMP->mnVisible;
//        fs["mnFound"] >> pMP->mnFound;

//        fs["mbBad"] >> pMP->mbBad;

//        fs["mfMinDistance"] >> pMP->mfMinDistance;
//        fs["mfMaxDistance"] >> pMP->mfMaxDistance;

//        pMP->mpMap = &World;

//        id2pMPmap[pMP->mnId] = pMP;
//        vpMPs.push_back(pMP);

//        fs.release();
//    }//end of creating MapPoints


//    //loading Reference MapPoints..
//    cout<<"loading Reference MapPoints.."<<endl;
//    vector<ORB_SLAM2::MapPoint*> vpRMPs;
//    vector<vector<double> > mObservations_firstVectorR;//reference
//    vector<vector<int> > mObservations_secondVectorR;//reference
//    vector<double> mRefKFVectorR;//reference
//    boost::unordered_map<long unsigned int, ORB_SLAM2::MapPoint*> id2pRMPmap;

//    std::list<std::string> RM_file_list;
//    size_t RM_file_maxlen;
//    FindAllFilesInDir(map_path + "/RMapPoints", "yml", RM_file_list, RM_file_maxlen);

//    for(size_t i=0; i<RM_file_list.size(); i++){
//        ORB_SLAM2::MapPoint* pRMP = new ORB_SLAM2::MapPoint;
//        //        string MPfileName = ros::package::getPath("ORB_SLAM")+"/bin/RMapPoints/car_sample/"+"rmappoint_" + boost::to_string(i) + ".yml";
//        //        std::string MPfileName = map_path + "/RMapPoints/rmappoint_" + boost::to_string(i) + ".yml";
//        std::string MPfileName = map_path + "/RMapPoints/" + RM_file_list[i];

//        cv::FileStorage fs(MPfileName.c_str(), cv::FileStorage::READ);
//        if(!fs.isOpened()) cout<<"no such file!"<<endl;

//        double mnId; fs["mnId"] >> mnId; pRMP->mnId = mnId;
//        double nNextId; fs["nNextId"] >> nNextId; pRMP->nNextId = nNextId;
//        double mnFirstKFid; fs["mnFirstKFid"] >> mnFirstKFid; pRMP->mnFirstKFid = mnFirstKFid;

//        fs["mTrackProjX"] >> pRMP->mTrackProjX;
//        fs["mTrackProjY"] >> pRMP->mTrackProjY;
//        fs["mbTrackInView"] >> pRMP->mbTrackInView;
//        fs["mnTrackScaleLevel"] >> pRMP->mnTrackScaleLevel;
//        fs["mTrackViewCos"] >> pRMP->mTrackViewCos;
//        double mnTrackReferenceForFrame; fs["mnTrackReferenceForFrame"] >> mnTrackReferenceForFrame; pRMP->mnTrackReferenceForFrame = mnTrackReferenceForFrame;
//        double mnLastFrameSeen; fs["mnLastFrameSeen"] >> mnLastFrameSeen; pRMP->mnLastFrameSeen = mnLastFrameSeen;

//        double mnBALocalForKF; fs["mnBALocalForKF"] >> mnBALocalForKF; pRMP->mnBALocalForKF = mnBALocalForKF;
//        double mnFuseCandidateForKF; fs["mnFuseCandidateForKF"] >> mnFuseCandidateForKF; pRMP->mnFuseCandidateForKF = mnFuseCandidateForKF;

//        double mnLoopPointForKF; fs["mnLoopPointForKF"] >> mnLoopPointForKF; pRMP->mnLoopPointForKF = mnLoopPointForKF;
//        double mnCorrectedByKF; fs["mnCorrectedByKF"] >> mnCorrectedByKF; pRMP->mnCorrectedByKF = mnCorrectedByKF;
//        double mnCorrectedReference; fs["mnCorrectedReference"] >> mnCorrectedReference; pRMP->mnCorrectedReference = mnCorrectedReference;

//        fs["mWorldPos"] >> pRMP->mWorldPos;

//        vector<double> mObservations_first;
//        vector<int> mObservations_second;
//        fs["mObservations_first"] >> mObservations_first;
//        fs["mObservations_second"] >> mObservations_second;
//        mObservations_firstVectorR.push_back(mObservations_first);
//        mObservations_secondVectorR.push_back(mObservations_second);

//        fs["mNormalVector"] >> pRMP->mNormalVector;
//        fs["mDescriptor"] >> pRMP->mDescriptor;

//        double mRefKF;
//        fs["mRefKF"] >> mRefKF;
//        mRefKFVectorR.push_back(mRefKF);

//        fs["mnVisible"] >> pRMP->mnVisible;
//        fs["mnFound"] >> pRMP->mnFound;

//        fs["mbBad"] >> pRMP->mbBad;

//        fs["mfMinDistance"] >> pRMP->mfMinDistance;
//        fs["mfMaxDistance"] >> pRMP->mfMaxDistance;

//        pRMP->mpMap = mpMap;

//        id2pRMPmap[pRMP->mnId] = pRMP;
//        vpRMPs.push_back(pRMP);

//        fs.release();
//    }//end of creating Reference MapPoints

//    //Assigning KeyFrames..
//    cout<<"Assigning KeyFrames.."<<endl;
//    for(size_t i = 0; i < vpKFs.size(); i++){
//        //loading mvpMapPoints
//        vector<double> mvMapPoints = mvMapPointsVector[i];
//        for(size_t j = 0; j < mvMapPoints.size(); j++){
//            if(mvMapPoints[j] != -1) vpKFs[i]->mvpMapPoints.push_back(id2pMPmap[mvMapPoints[j]]);
//            else vpKFs[i]->mvpMapPoints.push_back(NULL);
//        }
//        //loading mConnectedKeyFrameWeights
//        vector<double> mConnectedKeyFrameWeights_first = mConnectedKeyFrameWeights_firstVector[i];
//        vector<int> mConnectedKeyFrameWeights_second = mConnectedKeyFrameWeights_secondVector[i];
//        for(size_t j = 0; j < mConnectedKeyFrameWeights_second.size(); j++){
//            vpKFs[i]->mConnectedKeyFrameWeights[id2pKFmap[mConnectedKeyFrameWeights_first[j]]] = mConnectedKeyFrameWeights_second[j];
//        }
//        //loading mvpOrderedConnectedKeyFrames
//        vector<double> mvOrderedConnectedKeyFrames =  mvOrderedConnectedKeyFramesVector[i];
//        for(size_t j = 0; j < mvOrderedConnectedKeyFrames.size(); j++){
//            vpKFs[i]->mvpOrderedConnectedKeyFrames.push_back(id2pKFmap[mvOrderedConnectedKeyFrames[j]]);
//        }
//        //loading mParent
//        double mParent = mParentVector[i];
//        vpKFs[i]->mpParent = id2pKFmap[mParent];
//        //loading msChildrens
//        vector<double> msChildrens = msChildrensVector[i];
//        for(size_t j = 0; j < msChildrens.size(); j++){
//            vpKFs[i]->mspChildrens.insert(id2pKFmap[msChildrens[j]]);
//        }
//        //loading msLoopEdges
//        vector<double> msLoopEdges = msLoopEdgesVector[i];
//        for(size_t j = 0; j < msLoopEdges.size(); j++){
//            vpKFs[i]->mspLoopEdges.insert(id2pKFmap[msLoopEdges[j]]);
//        }
//    }

//    //Assigning MapPoints..
//    cout<<"Assigning MapPoints.."<<endl;
//    for(size_t i = 0; i < vpMPs.size(); i++){
//        //loading mObservations
//        vector<double> mObservations_first = mObservations_firstVector[i];
//        vector<int> mObservations_second = mObservations_secondVector[i];
//        for(size_t j = 0; j < mObservations_second.size(); j++){
//            vpMPs[i]->mObservations[id2pKFmap[mObservations_first[j]]] = mObservations_second[j];
//        }
//        //loading mpRefKF
//        double mRefKF = mRefKFVector[i];
//        vpMPs[i]->mpRefKF = id2pKFmap[mRefKF];
//    }

//    //Assigning Reference MapPoints..
//    cout<<"Assigning Reference MapPoints.."<<endl;
//    for(size_t i = 0; i < vpRMPs.size(); i++){
//        //loading mObservations
//        vector<double> mObservations_first = mObservations_firstVectorR[i];
//        vector<int> mObservations_second = mObservations_secondVectorR[i];
//        for(size_t j = 0; j < mObservations_second.size(); j++){
//            vpRMPs[i]->mObservations[id2pKFmap[mObservations_first[j]]] = mObservations_second[j];
//        }
//        //loading mpRefKF
//        double mRefKF = mRefKFVectorR[i];
//        vpRMPs[i]->mpRefKF = id2pKFmap[mRefKF];
//    }

//    //cout<<"size = "<<vpKFs.size()<<endl;
//    //ORB_SLAM2::KeyFrame* pKF = vpKFs[3];
//    //ORB_SLAM2::MapPoint* pRMP = vpRMPs[380];
//    //std::map<ORB_SLAM2::KeyFrame*,int>::iterator iteckfw = pKF->mConnectedKeyFrameWeights.begin();
//    //for(; iteckfw != pKF->mConnectedKeyFrameWeights.end(); iteckfw++){
//    //    cout<<iteckfw->first->mnId << " ";
//    //    cout<<iteckfw->second<<endl;
//    //}

//    //std::map<ORB_SLAM2::KeyFrame*,size_t>::iterator iteo = pRMP->mObservations.begin();
//    //for(; iteo != pRMP->mObservations.end(); iteo++){
//    //    cout<<iteo->first->mnId << " ";
//    //    cout<<iteo->second<<endl;
//    //}
//    //cout<<pRMP->mnId<<endl;
//    //cout<<pRMP->mpRefKF->mnId<<endl;

//    //std::set<ORB_SLAM2::KeyFrame*>::iterator itec = pKF->mspLoopEdges.begin();
//    //for(; itec != pKF->mspLoopEdges.end(); itec++){
//    //    cout<<(*itec)->mnId<<endl;
//    //}

//    //for(size_t k = 0; k < pKF->mvpOrderedConnectedKeyFrames.size(); k++){
//    //    cout<<pKF->mvpOrderedConnectedKeyFrames[k]->mnId<<" ";
//    //    cout<<pKF->mvOrderedWeights[k]<<endl;
//    //}

//    //for(size_t k = 0; k < pKF->mvpMapPoints.size(); k++)
//    //    if(pKF->mvpMapPoints[k] == NULL) cout<<"-1 ";
//    //    else cout<<pKF->mvpMapPoints[k]->mnId<<" ";
//    //cout<<endl;

//    //load KerFrames, MapPoints, Reference MapPoints, Database
//    cout<<"loading KeyFrames to World..";
//    for(size_t i = 0; i < vpKFs.size(); i++)
//        mpMap->AddKeyFrame(vpKFs[i]);
//    for(size_t i = 0; i < vpMPs.size(); i++)
//        mpMap->AddMapPoint(vpMPs[i]);
//    mpMap->SetReferenceMapPoints(vpRMPs);

//    for(size_t i = 1; i < vpKFs.size(); i++)
//        mpKeyFrameDatabase->add(vpKFs[i]);

//    long unsigned int lastKFId = vpKFs[vpKFs.size()-1]->mnId;
//    //    ORB_SLAM2::Frame Frame(lastKFId);

//    mpTracker->mnLastKeyFrameId = lastKFId;
//    mpTracker->mpLastKeyFrame = vpKFs[vpKFs.size()-1];
//}

//void System::SaveMap(const std::string & map_path) {
//    // Save keyframe poses at the end of the execution
//    ofstream f;

//    vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
//    sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM2::KeyFrame::lId);

//    cout << endl << "Saving MapPoint mnIds to mnId_list.txt" << endl;
////    string strFile = ros::package::getPath("ORB_SLAM")+"/bin/KeyFrames/"+"mnId_list.txt";
//    string strFile = map_path + "/KeyFrames/mnId_list.txt";

//    f.open(strFile.c_str());
//    f << fixed;
//    f << "keyframe mnId = "<<endl;
//    //save keyframes///////////////////////////////////////////////////////////////////////
//    cout << endl << "Saving KeyFrames.." << endl;
//    cout<<"KeyFrame size = "<<vpKFs.size()<<endl;
//    int count = 0;
//    for(size_t i=0; i<vpKFs.size(); i++)//vpKFs.size()
//    {
//        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

//        //if(pKF->isBad())//might cause missing KF in covisibility graph
//        //    continue;
//        f<<pKF->mnId<<" ";
////        string KFfileName = ros::package::getPath("ORB_SLAM")+"/bin/KeyFrames/"+"keyframe_"+boost::to_string(count++) + ".yml";
//        string KFfileName = map_path + "/KeyFrames/keyframe_" + boost::to_string(count++) + ".yml";

//        cv::FileStorage fs(KFfileName.c_str(), cv::FileStorage::WRITE);

//        write(fs, "nNextId", double(pKF->nNextId));
//        write(fs, "mnId", double(pKF->mnId));
//        write(fs, "mnFrameId", double(pKF->mnFrameId));

//        write(fs, "mTimeStamp", pKF->mTimeStamp);

//        write(fs, "mnGridCols", pKF->mnGridCols);
//        write(fs, "mnGridRows", pKF->mnGridRows);
//        write(fs, "mfGridElementWidthInv", pKF->mfGridElementWidthInv);
//        write(fs, "mfGridElementHeightInv", pKF->mfGridElementHeightInv);

//        write(fs, "mnTrackReferenceForFrame", double(pKF->mnTrackReferenceForFrame));
//        write(fs, "mnFuseTargetForKF", double(pKF->mnFuseTargetForKF));

//        write(fs, "mnBALocalForKF", double(pKF->mnBALocalForKF));
//        write(fs, "mnBAFixedForKF", double(pKF->mnBAFixedForKF));

//        write(fs, "mnLoopQuery", double(pKF->mnLoopQuery));
//        write(fs, "mnLoopWords", pKF->mnLoopWords);
//        write(fs, "mLoopScore", pKF->mLoopScore);
//        write(fs, "mnRelocQuery", double(pKF->mnRelocQuery));
//        write(fs, "mnRelocWords", pKF->mnRelocWords);
//        write(fs, "mRelocScore", pKF->mRelocScore);

//        write(fs, "fx", pKF->fx);
//        write(fs, "fy", pKF->fy);
//        write(fs, "cx", pKF->cx);
//        write(fs, "cy", pKF->cy);

//        write(fs, "Tcw", pKF->Tcw);
//        write(fs, "Ow", pKF->Ow);

//        write(fs, "im", pKF->im);
//        write(fs, "mnMinX", pKF->mnMinX);
//        write(fs, "mnMinY", pKF->mnMinY);
//        write(fs, "mnMaxX", pKF->mnMaxX);
//        write(fs, "mnMaxY", pKF->mnMaxY);
//        write(fs, "mK", pKF->mK);

//        write(fs, "mvKeys", pKF->mvKeys);
//        write(fs, "mvKeysUn", pKF->mvKeysUn);
//        write(fs, "mDescriptors", pKF->mDescriptors);

//        vector<double> mvMapPoints;
//        for(size_t i = 0; i < pKF->mvpMapPoints.size(); i++){
//            if(pKF->mvpMapPoints[i] != NULL) mvMapPoints.push_back(double(pKF->mvpMapPoints[i]->mnId));//might point to NULL?
//            else mvMapPoints.push_back(-1);
//        }
//        write(fs, "mvMapPoints", mvMapPoints);

//        std::vector<int> mGrid_serialized;
//        std::vector<int> mGridSize;
//        for(size_t i = 0; i < pKF->mGrid.size(); i++){
//            for(size_t j = 0; j < pKF->mGrid[i].size(); j++){
//                mGridSize.push_back(pKF->mGrid[i][j].size());
//                if(pKF->mGrid[i][j].size() == 0) continue;
//                else{
//                    for(size_t k = 0; k < pKF->mGrid[i][j].size(); k++){
//                        mGrid_serialized.push_back(int(pKF->mGrid[i][j][k]));
//                    }
//                }
//            }
//        }
//        write(fs, "mGrid", mGrid_serialized);
//        write(fs, "mGridSize", mGridSize);
//        /*
//            for(size_t i = 0; i < pKF->mGrid.size(); i++){
//                for(size_t j = 0; j < pKF->mGrid[i].size(); j++){
//                    f<<endl<<i<<" "<<j<<endl;
//                    for(size_t k = 0; k < pKF->mGrid[i][j].size(); k++){
//                        f<<pKF->mGrid[i][j][k];
//                    }
//                }
//            }
//            */
//        vector<double> mConnectedKeyFrameWeights_first;
//        vector<int> mConnectedKeyFrameWeights_second;
//        std::map<ORB_SLAM2::KeyFrame*,int>::iterator iteckfw = pKF->mConnectedKeyFrameWeights.begin();
//        for(; iteckfw != pKF->mConnectedKeyFrameWeights.end(); iteckfw++){
//            mConnectedKeyFrameWeights_first.push_back(double(iteckfw->first->mnId));
//            mConnectedKeyFrameWeights_second.push_back(iteckfw->second);
//        }
//        write(fs, "mConnectedKeyFrameWeights_first", mConnectedKeyFrameWeights_first);
//        write(fs, "mConnectedKeyFrameWeights_second", mConnectedKeyFrameWeights_second);

//        vector<double> mvOrderedConnectedKeyFrames;
//        for(size_t i = 0; i < pKF->mvpOrderedConnectedKeyFrames.size(); i++){
//            mvOrderedConnectedKeyFrames.push_back(double(pKF->mvpOrderedConnectedKeyFrames[i]->mnId));
//        }
//        write(fs, "mvOrderedConnectedKeyFrames", mvOrderedConnectedKeyFrames);

//        write(fs, "mvOrderedWeights", pKF->mvOrderedWeights);

//        write(fs, "mbFirstConnection", pKF->mbFirstConnection);
//        write(fs, "mParent", (pKF->mpParent == NULL)? -1:double(pKF->mpParent->mnId));

//        vector<double> msChildrens;
//        std::set<ORB_SLAM2::KeyFrame*>::iterator itec = pKF->mspChildrens.begin();
//        for(; itec != pKF->mspChildrens.end(); itec++){
//            msChildrens.push_back(double((*itec)->mnId));
//        }
//        write(fs, "msChildrens", msChildrens);

//        vector<double> msLoopEdges;
//        std::set<ORB_SLAM2::KeyFrame*>::iterator itele = pKF->mspLoopEdges.begin();
//        for(; itele != pKF->mspLoopEdges.end(); itele++){
//            msLoopEdges.push_back(double((*itele)->mnId));
//        }
//        write(fs, "msLoopEdges", msLoopEdges);

//        write(fs, "mbNotErase", pKF->mbNotErase);
//        write(fs, "mbToBeErased", pKF->mbToBeErased);
//        write(fs, "mbBad", pKF->mbBad);

//        write(fs, "mnScaleLevels", pKF->mnScaleLevels);
//        write(fs, "mvScaleFactors", pKF->mvScaleFactors);
//        write(fs, "mvLevelSigma2", pKF->mvLevelSigma2);
//        write(fs, "mvInvLevelSigma2", pKF->mvInvLevelSigma2);
//        fs.release();
//    }

//    f<<endl<<"databaseId = "<<endl;
//    for (std::set<ORB_SLAM2::KeyFrame*>::iterator it=mpKeyFrameDatabase->mvpKFset.begin(); it != mpKeyFrameDatabase->mvpKFset.end(); ++it){
//        f<<" "<<(*it)->mnId;
//    }


//    //save mappoints////////////////////////////////////////////////////////////////////////////////////
//    vector<ORB_SLAM2::MapPoint*> vpMPs = mpMap->GetAllMapPoints();
//    sort(vpMPs.begin(),vpMPs.end(),ORB_SLAM2::MapPoint::lId);
//    cout << endl << "Saving MapPoints.." << endl;

//    count = 0;
//    cout<<"MapPoint size = "<<vpMPs.size()<<endl;
//    for(size_t i=0; i<vpMPs.size(); i++)//vpMPs.size()
//    {
//        ORB_SLAM2::MapPoint* pMP = vpMPs[i];

//        //if(pMP->isBad())
//        //    continue;

//        //cout<<"mappoint mnId = "<<pMP->mnId<<endl;
////        string MPfileName = ros::package::getPath("ORB_SLAM")+"/bin/MapPoints/"+"mappoint_"+boost::to_string(count++) + ".yml";
//        string MPfileName = map_path + "/MapPoints/mappoint_" + boost::to_string(count++) + ".yml";

//        cv::FileStorage fs(MPfileName.c_str(), cv::FileStorage::WRITE);

//        write(fs, "mnId", double(pMP->mnId));
//        write(fs, "nNextId", double(pMP->nNextId));
//        write(fs, "mnFirstKFid", double(pMP->mnFirstKFid));

//        write(fs, "mTrackProjX", pMP->mTrackProjX);
//        write(fs, "mTrackProjY", pMP->mTrackProjY);
//        write(fs, "mbTrackInView", pMP->mbTrackInView);
//        write(fs, "mnTrackScaleLevel", pMP->mnTrackScaleLevel);
//        write(fs, "mTrackViewCos", pMP->mTrackViewCos);
//        write(fs, "mnTrackReferenceForFrame", double(pMP->mnTrackReferenceForFrame));
//        write(fs, "mnLastFrameSeen", double(pMP->mnLastFrameSeen));

//        write(fs, "mnBALocalForKF", double(pMP->mnBALocalForKF));
//        write(fs, "mnFuseCandidateForKF", double(pMP->mnFuseCandidateForKF));

//        write(fs, "mnLoopPointForKF", double(pMP->mnLoopPointForKF));
//        write(fs, "mnCorrectedByKF", double(pMP->mnCorrectedByKF));
//        write(fs, "mnCorrectedReference", double(pMP->mnCorrectedReference));

//        write(fs, "mWorldPos", pMP->mWorldPos);

//        vector<double> mObservations_first;
//        vector<int> mObservations_second;
//        std::map<ORB_SLAM2::KeyFrame*,size_t>::iterator iteo = pMP->mObservations.begin();
//        for(; iteo != pMP->mObservations.end(); iteo++){
//            mObservations_first.push_back(double(iteo->first->mnId));
//            mObservations_second.push_back(int(iteo->second));
//        }
//        write(fs, "mObservations_first", mObservations_first);
//        write(fs, "mObservations_second", mObservations_second);

//        write(fs, "mNormalVector", pMP->mNormalVector);
//        write(fs, "mDescriptor", pMP->mDescriptor);

//        write(fs, "mRefKF", double(pMP->mpRefKF->mnId));

//        write(fs, "mnVisible", pMP->mnVisible);
//        write(fs, "mnFound", pMP->mnFound);

//        write(fs, "mbBad", pMP->mbBad);

//        write(fs, "mfMinDistance", pMP->mfMinDistance);
//        write(fs, "mfMaxDistance", pMP->mfMaxDistance);
//        fs.release();
//    }

//    //save reference mappoints////////////////////////////////////////////////////////////////////////////////////
//    vector<ORB_SLAM2::MapPoint*> vpRMPs = mpMap->GetReferenceMapPoints();
//    sort(vpRMPs.begin(),vpRMPs.end(),ORB_SLAM2::MapPoint::lId);
//    cout << endl << "Saving Reference MapPoints.." << endl;

//    count = 0;
//    cout<<"Reference MapPoint size = "<<vpRMPs.size()<<endl;
//    for(size_t i=0; i<vpRMPs.size(); i++)//vpRMPs.size()
//    {
//        ORB_SLAM2::MapPoint* pRMP = vpRMPs[i];

//        //if(pRMP->isBad())
//        //    continue;

//        //cout<<"mappoint mnId = "<<pRMP->mnId<<endl;
////        string RMPfileName = ros::package::getPath("ORB_SLAM")+"/bin/RMapPoints/"+"rmappoint_"+boost::to_string(count++) + ".yml";
//        string RMPfileName = map_path + "/RMapPoints/rmappoint_" + boost::to_string(count++) + ".yml";

//        cv::FileStorage fs(RMPfileName.c_str(), cv::FileStorage::WRITE);

//        write(fs, "mnId", double(pRMP->mnId));
//        write(fs, "nNextId", double(pRMP->nNextId));
//        write(fs, "mnFirstKFid", double(pRMP->mnFirstKFid));

//        write(fs, "mTrackProjX", pRMP->mTrackProjX);
//        write(fs, "mTrackProjY", pRMP->mTrackProjY);
//        write(fs, "mbTrackInView", pRMP->mbTrackInView);
//        write(fs, "mnTrackScaleLevel", pRMP->mnTrackScaleLevel);
//        write(fs, "mTrackViewCos", pRMP->mTrackViewCos);
//        write(fs, "mnTrackReferenceForFrame", double(pRMP->mnTrackReferenceForFrame));
//        write(fs, "mnLastFrameSeen", double(pRMP->mnLastFrameSeen));

//        write(fs, "mnBALocalForKF", double(pRMP->mnBALocalForKF));
//        write(fs, "mnFuseCandidateForKF", double(pRMP->mnFuseCandidateForKF));

//        write(fs, "mnLoopPointForKF", double(pRMP->mnLoopPointForKF));
//        write(fs, "mnCorrectedByKF", double(pRMP->mnCorrectedByKF));
//        write(fs, "mnCorrectedReference", double(pRMP->mnCorrectedReference));

//        write(fs, "mWorldPos", pRMP->mWorldPos);

//        vector<double> mObservations_first;
//        vector<int> mObservations_second;
//        std::map<ORB_SLAM2::KeyFrame*,size_t>::iterator iteo = pRMP->mObservations.begin();
//        for(; iteo != pRMP->mObservations.end(); iteo++){
//            mObservations_first.push_back(double(iteo->first->mnId));
//            mObservations_second.push_back(int(iteo->second));
//        }
//        write(fs, "mObservations_first", mObservations_first);
//        write(fs, "mObservations_second", mObservations_second);

//        write(fs, "mNormalVector", pRMP->mNormalVector);
//        write(fs, "mDescriptor", pRMP->mDescriptor);

//        write(fs, "mRefKF", double(pRMP->mpRefKF->mnId));

//        write(fs, "mnVisible", pRMP->mnVisible);
//        write(fs, "mnFound", pRMP->mnFound);

//        write(fs, "mbBad", pRMP->mbBad);

//        write(fs, "mfMinDistance", pRMP->mfMinDistance);
//        write(fs, "mfMaxDistance", pRMP->mfMaxDistance);
//        fs.release();

//    }

//    f.close();
//}

} //namespace ORB_SLAM
