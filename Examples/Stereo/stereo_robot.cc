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

#include <dirent.h>
#include <sys/stat.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include "System.h"

using namespace std;

bool LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        cerr << endl <<
            "Usage: ./stereo_robot path_to_sequence iBegin iEnd"
            << endl;
        return 1;
    }
    int iBegin = 0, iEnd = -1;
    if (argc >= 3) {
        sscanf(argv[2], "%d", &iBegin);
    }
    if (argc >= 4) {
        sscanf(argv[3], "%d", &iEnd);
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    if (!LoadImages(string(argv[1]), vstrImageLeft, vstrImageRight, vTimestamps))
        return 1;
    const std::string strSeqPath(argv[1]);
    const std::string outputDir = strSeqPath.substr(0, strSeqPath.find("stereo"));
    int nImages = vstrImageLeft.size();

    std::string vocFile = "/prj/3DV-AD/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    std::string settingFile = "/prj/3DV-AD/ORB_SLAM2/Examples/Stereo/Robot.yaml";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocFile,settingFile,ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "input seq path: " << strSeqPath << endl;
    cout << "output dir: " << outputDir << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Save camera trajectory
    const std::string trjFile = outputDir + "ORB-stereo-robot.txt";
    ofstream f;
    f.open(trjFile.c_str());
    f << fixed;
    // record track time of each frame
    const std::string timeFile = outputDir + "ORB-stereo-robot-time.txt";
    FILE *fp = fopen(timeFile.c_str(), "w");
    // Main loop
    cv::Mat imLeft, imRight;
    if (iEnd != -1 && iEnd < nImages)
        nImages = iEnd;
    std::cout << "test sequence:" << iBegin << "-->" << nImages << std::endl;
    for(int ni=iBegin; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(strSeqPath + "image_0/" + vstrImageLeft[ni],
            CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(strSeqPath + "image_1/" + vstrImageRight[ni],
            CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        cv::Mat Tcw = SLAM.TrackStereo(imLeft,imRight,tframe);

        if( ! Tcw.empty() )
        {
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            f << setprecision(9) << tframe<<" "<<
              Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
              Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
              Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        fprintf(fp, "%lf\n", ttrack);
        vTimesTrack[ni]=ttrack;

    }
    f.close();
    fclose(fp);

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

    // stat info
    const std::string statFile = outputDir + "ORB-stereo-robot-stat.txt";
    fp = fopen(statFile.c_str(), "w");
    fprintf(fp, "KeyFrame ratio: %d / %d = %lf\n", SLAM.GetKeyFrameNumber(), nImages, SLAM.GetKeyFrameNumber() * 1.0 / nImages);
    fclose(fp);    

    return 0;
}

void GetFileNames(std::string path, std::vector<std::string> &filenames) {
    if (path.empty()) {
        std::cout << "path is NULL" << std::endl;
        return;
    }
#ifdef _WIN32
    long hFile = 0;
    struct _finddata_t fileinfo;
    std::string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {  
        do {
            //如果是目录,迭代之（即文件夹内还有文件夹）
            if ((fileinfo.attrib &  _A_SUBDIR)) {
                //文件名不等于"."&&文件名不等于".."
                //.表示当前目录
                //..表示当前目录的父目录
                //判断时，两者都要忽略，不然就无限递归跳不出去了！
                // if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                //     getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
            }
            //如果不是,加入列表    
            else {
                // 文件名
                filenames.push_back(fileinfo.name);
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        //_findclose函数结束查找
        _findclose(hFile);
    }
#else   // linux
    DIR *dp;
    struct dirent *dirp;
    std::string file;
    
    // check if dir is valid
    struct stat s;    
    lstat(path.c_str(), &s);
    if (!S_ISDIR(s.st_mode)) {
        std::cout << "dir is not valid" << std::endl;
        return;
    }    

    if ((dp = opendir(path.c_str())) == NULL) {
        std::cout << "can't open " << path << std::endl;
        return;
    }

    while((dirp = readdir(dp)) != NULL) {
        if (dirp->d_type == 8) {
            file = std::string(dirp->d_name);
            // 文件名
            filenames.push_back(file);
        }
    }

    closedir(dp);
#endif
    // sort with file name ASCII
    std::sort(filenames.begin(), filenames.end());
}

bool LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    if (strPathToSequence.empty()) {
        std::cout << "path is null" << std::endl;
        return false;
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    GetFileNames(strPrefixLeft, vstrImageLeft);
    GetFileNames(strPrefixRight, vstrImageRight);

    if (vstrImageLeft.size() != vstrImageRight.size()) {
        std::cout << "image number not equal!" << std::endl;
        return false;        
    }

    double time;
    for (auto file : vstrImageLeft) {
        sscanf(file.c_str(), "%lf", &time);
        vTimestamps.emplace_back(time / 1000000.0);
    }

    return true;
}

