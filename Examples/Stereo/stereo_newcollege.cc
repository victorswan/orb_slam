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
#include<iomanip>
#include<chrono>
#include <boost/filesystem.hpp>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

// comparison, not case sensitive.
bool compareTimestamp (const std::string& first, const std::string& second)
{
    double time_stamp_l = std::stod( first );
    double time_stamp_r = std::stod( second );
    return ( time_stamp_l < time_stamp_r );
}

bool LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv) {

    if(argc != 8)
    {
        cerr << endl << "Usage: ./stereo_general path_to_vocabulary path_to_settings budget_per_frame "
             << " do_viz play_back_speed path_to_sequence path_to_traj" << endl;

        return 1;
    }

    bool do_viz;
    stringstream s1(argv[4]);
    s1 >> boolalpha >> do_viz;

    double play_back_speed = std::stod(argv[5]);
    cout << play_back_speed << endl;
    assert(play_back_speed > 0);

    cout << string(argv[6]) << endl;

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    LoadImages(string(argv[6]), vstrImageLeft, vstrImageRight, vTimeStamp);

    if(vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    if(vstrImageLeft.size()!=vstrImageRight.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,do_viz);

    SLAM.SetBudgetPerFrame(std::atoi(argv[3]));

    std::string fNameRealTimeTrack = std::string(argv[7]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack);


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        cout << "-------- Frame " << ni << " / " << nImages << " --------" << endl;

        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_GRAYSCALE);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_GRAYSCALE);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }

        double tframe = vTimeStamp[ni];

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni < nImages-1)
            T = vTimeStamp[ni+1]-tframe;
        else if (ni>0)
            T = tframe-vTimeStamp[ni-1];

        T /= play_back_speed;

        if(ttrack < T)
            usleep((T-ttrack)*1e6);
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
    SLAM.SaveKeyFrameTrajectoryTUM( std::string(argv[7]) + "_KeyFrameTrajectory.txt" );
    SLAM.SaveTrackingLog( std::string(argv[7]) + "_Log.txt" );

    std::cout << "Finished saving!" << std::endl;

    return 0;
}


bool LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    // setup image directories
    string img_dir_l = strPathToSequence; // + "/cam0/data/";
    string img_dir_r = strPathToSequence; // + "/cam1/data/";

    // get a sorted list of files in the img directories
    boost::filesystem::path img_dir_path_l(img_dir_l.c_str());
    if (!boost::filesystem::exists(img_dir_path_l))
    {
        cout << endl << "Left image directory does not exist: \t" << img_dir_l << endl;
        return false;
    }
    boost::filesystem::path img_dir_path_r(img_dir_r.c_str());
    if (!boost::filesystem::exists(img_dir_path_r))
    {
        cout << endl << "Right image directory does not exist: \t" << img_dir_r << endl;
        return false;
    }

    // get all files in the img directories
    size_t max_len_l = 0;
    std::list<std::string> imgs_l;
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator file(img_dir_path_l); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        // cout << filename_path << endl;
        if (boost::filesystem::is_regular_file(file->status()) &&
                filename_path.extension() == ".pnm" &&
                filename_path.string().find("-left") != string::npos)
        {
            std::string filename(filename_path.string());
            //            cout << filename << endl;
            imgs_l.push_back(filename.substr(23, 17));
            //            cout << filename.substr(0, filename.length()-4) << endl;
            // cout << filename.substr(23, 17) << endl;
            max_len_l = max(max_len_l, filename.length());
        }
    }
    size_t max_len_r = 0;
    std::list<std::string> imgs_r;
    for (boost::filesystem::directory_iterator file(img_dir_path_r); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        if (boost::filesystem::is_regular_file(file->status()) &&
                filename_path.extension() == ".pnm" &&
                filename_path.string().find("-right") != string::npos)
        {
            std::string filename(filename_path.string());
            imgs_r.push_back(filename.substr(23, 17));
            // cout << filename.substr(23, 17) << endl;
            max_len_r = max(max_len_r, filename.length());
        }
    }

    imgs_l.sort(compareTimestamp);
    imgs_r.sort(compareTimestamp);

    cout << imgs_l.size() << "; " << imgs_l.size() << endl;

    //    // extract the image files with common name
    //    std::vector<std::string> img_lr_pair; // (n_imgs_l + n_imgs_r);
    //    std::set_intersection (imgs_l.begin(), imgs_l.end(),
    //                           imgs_r.begin(), imgs_r.end(),
    //                           std::back_inserter(img_lr_pair));
    //    std::cout << std::endl << "Number of stereo image pairs: " << img_lr_pair.size() << std::endl;

    std::list<std::string>::iterator it_r = imgs_r.begin();
    for (std::list<std::string>::iterator it_l = imgs_l.begin(); it_l != imgs_l.end(); ++ it_l) {
        // cout << (*it_l) << endl;
        while (it_r != imgs_r.end()) {
            // compare the timestamp between i & j
            // cout << (*it_r) << endl;
            // rectified_StereoImage__1225720188.298853-left.pnm
            double time_stamp_l = std::stod( (*it_l) );
            double time_stamp_r = std::stod( (*it_r) );
            if (fabs((double)(time_stamp_l) - (double)(time_stamp_r)) < 0.003) {
                // pair left & right frames
                vstrImageLeft.push_back( img_dir_l + "/rectified_StereoImage__" + *it_l + "-left.pnm" );
                vstrImageRight.push_back( img_dir_r + "/rectified_StereoImage__" + *it_r + "-right.pnm" );
                //                vTimeStamps.push_back( (double)((long double)time_stamp_l * 1e-9) );
                vTimeStamps.push_back( ((double)time_stamp_l) );
                //                cout << vTimeStamps.back() << endl;
                // move on to the next left index
                it_r ++;
                break ;
            }
            else if ((double)(time_stamp_l) < (double)(time_stamp_r)) {
                // skip current left index
                break ;
            }
            else {
                // move on to the next right index
                it_r ++;
            }
        }
    }
    assert(vstrImageLeft.size() == vstrImageRight.size());
    std::cout << std::endl << "Number of stereo image pairs: " << vstrImageRight.size() << std::endl;

    return true;
}
