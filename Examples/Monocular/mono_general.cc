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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <boost/filesystem.hpp>

#include <gflags/gflags.h>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

// General sequences, e.g. EuRoC, TUM-VI, New College, Hololens, Oxford RobotCar
#define TIME_LONG_TO_SEC    1e-9

DEFINE_string(path_to_vocab,    "",     "Dir to ORB vocab file, in txt or bin format");
DEFINE_string(path_to_setting,  "",     "Dir to ORB-SLAM setting file, in yaml format");
DEFINE_string(path_to_images,   "",     "Dir to image data");
DEFINE_string(path_to_times,    "",     "Dir to time stamp data");
DEFINE_string(path_to_planned_track,"", "Dir to planned trajectory, in TUM track format");
DEFINE_string(path_to_track,    "",     "Dir to save pose tracking result");
DEFINE_string(path_to_map,      "",     "Dir to save final mapping result");
DEFINE_uint64(constr_per_frame, 1600,   "Max number of constraints from a mono/stereo frame (keypoint nubmer x 2))");
DEFINE_double(time_start,       0,      "Offset of first frame, in sec");
DEFINE_double(time_duration,    999,    "Duration to last frame, in sec");
DEFINE_double(budget_per_frame, 30,     "Total amount of time to process a frame, in ms");
DEFINE_bool(do_viz,             false,   "Visualize real-time SLAM or not");

// comparison, not case sensitive.
bool compareTimestamp (const std::string& first, const std::string& second)
{
    long time_stamp_l = std::stol( first );
    long time_stamp_r = std::stol( second );
    return ( time_stamp_l < time_stamp_r );
}

bool LoadImages(const string &strImagePath,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    // Retrieve paths to images
    vector<string> vstrImage_full;
    vector<double> vTimeStamp_full;
    LoadImages(FLAGS_path_to_images, vstrImage_full, vTimeStamp_full);

    // Clean images with input duration
    size_t i_st = 0;
    for (; i_st<vTimeStamp_full.size(); ++i_st) {
        if (vTimeStamp_full[i_st] >= vTimeStamp_full[0] + FLAGS_time_start)
            break ;
    }
    size_t i_ed = vTimeStamp_full.size()-1;
    for (; i_ed>=0; --i_ed) {
        if (vTimeStamp_full[i_ed] <= vTimeStamp_full[0] + FLAGS_time_start + FLAGS_time_duration)
            break ;
    }
    if (i_st >= vTimeStamp_full.size() || i_ed <= i_st)
    {
        cerr << "ERROR: No images in provided duration." << endl;
        return 1;
    }
    // Update all images logs
    vector<string> vstrImage(vstrImage_full.begin()+i_st, vstrImage_full.begin()+i_ed+1);
    vector<double> vTimeStamp(vTimeStamp_full.begin()+i_st, vTimeStamp_full.begin()+i_ed+1);
    if(vstrImage.empty() || vTimeStamp.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    const int nImages = vstrImage.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(FLAGS_path_to_vocab, FLAGS_path_to_setting, ORB_SLAM2::System::MONOCULAR, FLAGS_do_viz);

    SLAM.SetConstrPerFrame(FLAGS_constr_per_frame);

    // convert budget input from ms to sec
    SLAM.SetBudgetPerFrame(FLAGS_budget_per_frame*1e-3);

#if defined LOGGING_KF_LIST && defined REALTIME_TRAJ_LOGGING

    std::string fNameRealTimeTrack = FLAGS_path_to_track + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    std::string fNameRealTimeBA = FLAGS_path_to_track + "_Log_BA.txt";
    std::cout << std::endl << "Saving BA Log to Log_BA.txt" << std::endl;
    //
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack, fNameRealTimeBA);

#elif defined REALTIME_TRAJ_LOGGING

    std::string fNameRealTimeTrack = FLAGS_path_to_track + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack);

#endif

#ifdef ENABLE_MAP_IO
    cout << endl << "-------" << endl;
    cout << "Loading the pre-built map ..." << endl;
    SLAM.LoadMap(FLAGS_path_to_map);
    // SLAM.LoadMap(std::string(argv[8]) + "_Map/");
    SLAM.ForceRelocTracker();
    // SLAM.ForceInitTracker();
#endif


#if defined ENABLE_ANTICIPATION_IN_GRAPH || defined PRED_WITH_ODOM
    cout << endl << "-------" << endl;
    cout << "Loading the planned trajectory ..." << endl;
    SLAM.LoadOdomPlanned(FLAGS_path_to_planned_track);
#endif

    cout << endl << "-------" << endl;
    cout << "Pre-loading all data from the sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    cv::Mat im;
    vector<cv::Mat> vIm;
    //
#ifdef ENABLE_DETECTION_IO
    vector<ORB_SLAM2::KeyPointLog> vKptLog;
    cout << "Also loading pre-extracted FAST keypoints from file ..." << endl;
#endif
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    for(int ni=0; ni<nImages; ni++) {
        // Read images from file
        im = cv::imread(vstrImage[ni], CV_LOAD_IMAGE_GRAYSCALE);
        vIm.push_back(im);

        if(vIm[ni].empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImage[ni]) << endl;
            return 1;
        }

#ifdef ENABLE_DETECTION_IO
        // Read feature detection results from file
        std::string strDetectLeft = FLAGS_path_to_images + "/../kpt/" + std::to_string(vTimeStamp[ni]) + ".yml";
        // cout << "path to load kpt file: " << strDetectLeft << endl;
        ORB_SLAM2::KeyPointLog kptLog;
        if (!kptLog.ReadFromYAML(strDetectLeft))
        {
            cerr << endl << "Failed to load keypoints at: "
                 << strDetectLeft << endl;
            return 1;
        }
        //
        vKptLog.push_back(kptLog);
#endif
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    //    cout << "Images in the sequence: " << nImages << endl << endl;
    double time_rest = 0;

    // Main loop
    for(int ni=0; ni<nImages; ni++)
    {
        cout << "-------- Frame " << ni << " / " << nImages << " --------" << endl;

        // TODO
        // grab data from pre-load structure
        im = vIm[ni];

#ifdef ENABLE_DETECTION_IO 
        SLAM.mpTracker->mTimeStamp_loaded  = vKptLog[ni].mTimeStamp;
        SLAM.mpTracker->mvKeysLeft  = vKptLog[ni].mvKeysLeft;
        SLAM.mpTracker->mDiscLeft   = vKptLog[ni].mDiscLeft;
#endif

        double tframe = vTimeStamp[ni];

        if (time_rest < -FLAGS_budget_per_frame*1e-3 * 0.5) {
            time_rest += FLAGS_budget_per_frame*1e-3;
            cout << "Skip frame " << ni << endl;
            continue ;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        // Pass the images to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        //#ifdef ENABLE_DETECTION_IO
        //        // Save feature detection results to file
        //        std::string strDetectLeft = FLAGS_path_to_sequence + "/cam0/kpt/" + std::to_string(vTimeStamp[ni]) + ".yml";
        //        cout << "path to save kpt file: " << strDetectLeft << endl;
        //        SLAM.mpTracker->SaveKeyPoints(strDetectLeft);
        //#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        time_rest = FLAGS_budget_per_frame*1e-3 - ttrack;
        if(time_rest > 0)
            usleep(time_rest*1e6);
    }

    cout << "before calling SLAM.Shutdown !!!" << endl;

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM( FLAGS_path_to_track + "_KeyFrameTrajectory.txt" );
    SLAM.SaveTrackingLog( FLAGS_path_to_track + "_Log.txt" );
#ifdef LOCAL_BA_TIME_LOGGING
    SLAM.SaveMappingLog( FLAGS_path_to_track + "_Log_Mapping.txt" );
#endif
    //
    //    SLAM.SaveLmkLog( FLAGS_path_to_track + "_Log_lmk.txt" );

    /*
#ifdef ENABLE_MAP_IO
 // #ifdef MAP_PUBLISH
 // publish map points
 //  for (size_t i=0; i<10; ++i)
 //   igb.mpMapPub->Refresh();
 // #endif

   SLAM.SaveMap(FLAGS_path_to_map);
   // SLAM.SaveMap(std::string(argv[8]) + "_Map/");

#endif
*/

    std::cout << "Finished saving!" << std::endl;

    return 0;
}

bool LoadImages(const string &strImagePath,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{

    // get a sorted list of files in the img directories
    boost::filesystem::path img_dir_path(strImagePath.c_str());
    if (!boost::filesystem::exists(img_dir_path))
    {
        cout << endl << "Image directory does not exist: \t" << img_dir_path << endl;
        return false;
    }

    // get all files in the img directories
    size_t max_len_ = 0;
    std::list<std::string> imgs_;
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator file(img_dir_path); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        if (boost::filesystem::is_regular_file(file->status()) &&
                filename_path.extension() == ".png")
        {
            std::string filename(filename_path.string());
            //            cout << filename << endl;
            imgs_.push_back(filename.substr(0, filename.length()-4));
            //            cout << filename.substr(0, filename.length()-4) << endl;
            max_len_ = max(max_len_, filename.length());
        }
    }

    imgs_.sort(compareTimestamp);

    for (std::list<std::string>::iterator it_ = imgs_.begin(); it_ != imgs_.end(); ++ it_) {
        long time_stamp_ = std::stol( *it_ );
        vstrImages.push_back( strImagePath + '/' + *it_ + ".png" );
        vTimeStamps.push_back( (double)((long double)time_stamp_ * TIME_LONG_TO_SEC) );

    }
    assert(vstrImages.size() == vTimeStamps.size());
    std::cout << std::endl << "Number of monocular images: " << vstrImages.size() << std::endl;

    return true;
}

/*
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
*/
