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

#include "CAPE/capewrap.cpp"

cv::Mat plane_mask(const cv::Mat &seg, uchar i_plane){
    cv::Mat mask = cv::Mat::zeros(seg.rows, seg.cols, CV_8U);
    for(int i=0; i<seg.rows; i++)
    {
        for(int j=0; j<seg.cols; j++)
        {
            if (seg.at<uchar>(i,j) == i_plane){
                mask.at<uchar>(i,j) = 1;
            }
        }
    }
    return mask;
}

using namespace std;
int main()
{

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    cv::FileStorage fSettings("/SP-SLAM/Examples/RGB-D/realsense.yaml", cv::FileStorage::READ);
    capewrap cape(fSettings);

    // Main loop
    cv::Mat imRGB, imD;
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.

    while(true)
    {

        rs2::frameset frames = pipe.wait_for_frames();
        auto rgb = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();

        cv::Mat image_rgb(cv::Size(640, 480), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat image_depth(cv::Size(640, 480), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        image_depth.convertTo(image_depth, CV_32F);
        auto plane_extracts = cape.process(image_depth);


        std::vector<cv::Vec3b> color_code;
        // Populate with random color codes
        for(int i=0; i<100;i++){
            cv::Vec3b color;
            color[0]=rand()%255;
            color[1]=rand()%255;
            color[2]=rand()%255;
            color_code.push_back(color);
        }

        // Add specific colors for planes
        color_code[0][0] = 0; color_code[0][1] = 0; color_code[0][2] = 255;
        color_code[1][0] = 255; color_code[1][1] = 0; color_code[1][2] = 204;
        color_code[2][0] = 255; color_code[2][1] = 100; color_code[2][2] = 0;
        color_code[3][0] = 0; color_code[3][1] = 153; color_code[3][2] = 255;
        // Add specific colors for cylinders
        color_code[50][0] = 178; color_code[50][1] = 255; color_code[50][2] = 0;
        color_code[51][0] = 255; color_code[51][1] = 0; color_code[51][2] = 51;
        color_code[52][0] = 0; color_code[52][1] = 255; color_code[52][2] = 51;
        color_code[53][0] = 153; color_code[53][1] = 0; color_code[53][2] = 255;

        uchar * sCode;
        uchar * dColor;
        uchar * srgb;
        int code;
        cv::Mat_<cv::Vec3b> seg_rz = cv::Mat_<cv::Vec3b>(480,640,cv::Vec3b(0,0,0));
        for(int r=0; r<  480; r++) {
            dColor = seg_rz.ptr<uchar>(r);
            sCode = plane_extracts.seg_output.ptr<uchar>(r);
            srgb = image_rgb.ptr<uchar>(r);
            for (int c = 0; c < 640; c++) {
                code = *sCode;
                if (code > 0) {
                    dColor[c * 3] = color_code[code - 1][0] / 2 + srgb[0] / 2;
                    dColor[c * 3 + 1] = color_code[code - 1][1] / 2 + srgb[1] / 2;
                    dColor[c * 3 + 2] = color_code[code - 1][2] / 2 + srgb[2] / 2;;
                } else {
                    dColor[c * 3] = srgb[0];
                    dColor[c * 3 + 1] = srgb[1];
                    dColor[c * 3 + 2] = srgb[2];
                }
                sCode++;
                srgb++;
                srgb++;
                srgb++;
            }
        }

        auto mask = plane_mask(plane_extracts.seg_output, 1);

        float s = cv::sum(mask)[0];
        float s1 = plane_extracts.plane_params[0].normal[0] + plane_extracts.plane_params[0].normal[1] + plane_extracts.plane_params[0].normal[2] + plane_extracts.plane_params[0].d;
        cout << plane_extracts.nr_planes << " " << s << " " << s1 << endl;
//        imshow( "Display window", seg_rz );
        imshow( "Display window", mask * 255 );

        // Press  ESC on keyboard to  exit
        char c = (char) cv::waitKey(1);
        if( c == 27 )
            break;

    }


    return 0;
}
