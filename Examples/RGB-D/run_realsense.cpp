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
#include<chrono>

#include<opencv2/core/core.hpp>

#include <System.h>
#include <Config.h>
#include <unistd.h>
//#include <Timer.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
using namespace std;


int main()
{
    int argc = 5;
    string argv[6];
    argv[0] = "";
    argv[1] = "/ORBvoc.txt";
    argv[2] = "/SP-SLAM/Examples/RGB-D/realsense.yaml";

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    ORB_SLAM2::Config::SetParameterFile(argv[2]);
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
//    ORB_SLAM2::Timer::StartTimer(7);

    // Main loop
    cv::Mat imRGB, imD;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    while(true)
    {

        rs2::frameset frames = pipe.wait_for_frames();
        auto rgb = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();

        cv::Mat image_rgb(cv::Size(640, 480), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat image_depth(cv::Size(640, 480), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        auto tstamp = std::chrono::duration<double>(now - start).count();
        SLAM.TrackRGBD(image_rgb, image_depth, tstamp);
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
