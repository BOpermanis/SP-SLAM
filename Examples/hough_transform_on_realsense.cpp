//
// Created by root on 8/21/20.
//

//
// Created by root on 8/5/20.
//

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
#include "opencv2/opencv.hpp"
#include <unistd.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>

typedef cv::Vec3f PointT;
typedef std::vector<PointT> PointCloud;



using namespace std;
using namespace cv;
int main()
{

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    cv::FileStorage fSettings("/SP-SLAM/Examples/RGB-D/realsense.yaml", cv::FileStorage::READ);

    // Main loop
    cv::Mat imRGB, imD, edges;
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.

    while(true)
    {

        rs2::frameset frames = pipe.wait_for_frames();
        auto rgb = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();

        cv::Mat image_rgb(cv::Size(640, 480), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat image_depth(cv::Size(640, 480), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        image_depth.convertTo(image_depth, CV_32F);


        // Edge detection
        cv::Canny(image_rgb, edges, 50, 200, 3);


        // Standard Hough Line Transform
        vector<Vec2f> lines; // will hold the results of the detection
        HoughLines(edges, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
        // Draw the lines
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
//            line( image_rgb, pt1, pt2, Scalar(0,0,255), 3);
        }
        // Probabilistic Line Transform
        vector<Vec4i> linesP; // will hold the results of the detection
        HoughLinesP(edges, linesP, 1, CV_PI/180, 150, 100, 10 ); // runs the actual detection
        // Draw the lines
        for( size_t i = 0; i < linesP.size(); i++ )
        {
            Vec4i l = linesP[i];
            line( image_rgb, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3);
        }

//        imshow( "Display window",  mask);
        imshow( "Display window",  image_rgb);

        // Press  ESC on keyboard to  exit
        char c = (char) cv::waitKey(1);
        if( c == 27 )
            break;

    }
    return 0;
}
