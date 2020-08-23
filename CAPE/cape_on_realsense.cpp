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


typedef cv::Vec3f PointT;
typedef std::vector<PointT> PointCloud;

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

void Erosion( cv::Mat &src, cv::Mat &erosion_dst)
{
    int erosion_size = 10;
    cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                                             cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
    cv::erode( src, erosion_dst, element );
//    imshow( "Erosion Demo", erosion_dst );
}

void Dilate( cv::Mat &src, cv::Mat &dilation_dst)
{
    int dilation_size = 10;

    cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                                             cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                             cv::Point( dilation_size, dilation_size ) );
    cv::dilate( src, dilation_dst, element );
}


bool extract_border_points(uchar c_plane, PointCloud &boundaryPoints, cv::Mat &coef, vector<cv::Point> &border, const cape_output &capeout, const capewrap &cape, const cv::Mat &imDepth){
    int i_plane = (int)c_plane - 1;

    coef.at<float>(0) = (float)capeout.plane_params[i_plane].normal[0];
    coef.at<float>(1) = (float)capeout.plane_params[i_plane].normal[1];
    coef.at<float>(2) = (float)capeout.plane_params[i_plane].normal[2];
    coef.at<float>(3) = (float)capeout.plane_params[i_plane].d;
    float s = cv::sum(coef)[0];
    bool flag_ok = false;

    if (s != 0 && !std::isinf(s)){
        auto mask = plane_mask(capeout.seg_output, c_plane);

        Dilate(mask, mask);
        Erosion(mask, mask);
        vector<vector<cv::Point> > contours;
        vector<cv::Point> contour0;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, -1, 2);

        int maxlen = 0;
        for(auto &cnt: contours){
            if (cnt.size() > maxlen){
                contour0 = cnt;
                maxlen = cnt.size();
            }
        }
        double rel_area = cv::contourArea(contour0) / (imDepth.cols * imDepth.rows);
        if (rel_area > 0.1){
            double epsilon = 0.01 * cv::arcLength(contour0,true);
            while (true){
                cv::approxPolyDP(contour0, border, epsilon, false);
                if(border.size() > 3) break;
                border.clear();
                epsilon *= 0.7;
            }

            if(coef.at<float>(3) < 0) {
                coef = -coef;
            }
            for(auto &pt: border){

                float y = (pt.y - cape.cy_ir) / cape.fy_ir;
                float x = (pt.x - cape.cx_ir) / cape.fx_ir;

                float theta = - coef.at<float>(3) / (x * coef.at<float>(0) + y * coef.at<float>(1) + coef.at<float>(2));

                boundaryPoints.push_back(PointT(x * theta, y * theta, theta));
            }
//            mvBoundaryPoints.push_back(boundaryPoints);
//            mvPlaneCoefficients.push_back(coef);
            flag_ok = true;
        }
    }
    return flag_ok;
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
        std::vector<cv::Mat> lines;
        auto plane_extracts = cape.process(image_depth, image_rgb, lines);


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

//        float s = cv::sum(mask)[0];
//        float s1 = plane_extracts.plane_params[0].normal[0] + plane_extracts.plane_params[0].normal[1] + plane_extracts.plane_params[0].normal[2] + plane_extracts.plane_params[0].d;
//        cout << plane_extracts.nr_planes << " " << s << " " << s1 << endl;
//        imshow( "Display window", seg_rz );
        mask *= 255;
        cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
        for(uchar c_plane = 1; (int)c_plane<=plane_extracts.nr_planes; c_plane++){
            cv::Mat coef = cv::Mat_<float>(4,1);
            PointCloud boundaryPoints;
            vector<cv::Point> border;
            if (extract_border_points(c_plane, boundaryPoints, coef, border, plane_extracts, cape, image_depth)){
//            boundaryPoints.push_back(PointT(boundaryPoints[0]));
                cv::Scalar color( 0, 255, 0 );
//                border.push_back(border[0]);
                cout << border[0] << " " << border[border.size()-1] << endl;
                vector<vector<cv::Point> > contours111;
                contours111.push_back(border);
                cv::drawContours( mask, contours111, 0, color, 2, 8);
            }
        }

//        imshow( "Display window",  mask);
        imshow( "Display window",  seg_rz);

        // Press  ESC on keyboard to  exit
        char c = (char) cv::waitKey(1);
        if( c == 27 )
            break;

    }


    return 0;
}
