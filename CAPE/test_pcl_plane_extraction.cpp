//
// Created by root on 7/15/20.
//

/*
 * Copyright 2018 Pedro Proenza <p.proenca@surrey.ac.uk> (University of Surrey)
 *
 */

#include <iostream>
#include <cstdio>
#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "CAPE/capewrap.cpp"
#include <cstdlib>
#include<zconf.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>


using namespace std;

std::vector<cv::Vec3b> color_code;



std::map<string, float> loadCalibParameters(string filepath){

    std::map<string, float> params;
    cv::Mat K_rgb, K_ir;

    cv::FileStorage fs(filepath,cv::FileStorage::READ);
    if (fs.isOpened()){
        fs["RGB_intrinsic_params"]>>K_rgb;
        fs["IR_intrinsic_params"]>>K_ir;

        float fx_ir = K_ir.at<double>(0,0); float fy_ir = K_ir.at<double>(1,1);
        float cx_ir = K_ir.at<double>(0,2); float cy_ir = K_ir.at<double>(1,2);
        float fx_rgb = K_rgb.at<double>(0,0); float fy_rgb = K_rgb.at<double>(1,1);
        float cx_rgb = K_rgb.at<double>(0,2); float cy_rgb = K_rgb.at<double>(1,2);

        params["Camera.fx"] = fx_ir;
        params["Camera.fy"] = fy_ir;
        params["Camera.cx"] = cx_ir;
        params["Camera.cy"] = cy_ir;
        params["Camera.fx"] = fx_rgb;
        params["Camera.fy"] = fy_rgb;
        params["Camera.cx"] = cx_rgb;
        params["Camera.cy"] = cy_rgb;
        params["Camera.width"] = 640.0;
        params["Camera.height"] = 480.0;

        fs.release();
        return params;
    }else{
        cerr<<"Calibration file missing"<<endl;
        return params;
    }
}

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud <PointT> PointCloud;

void apply_pcl(const cv::Mat &imDepth, string filepath){

    std::map<string, float> params;
    cv::Mat K_rgb, K_ir;

    cv::FileStorage fs(filepath,cv::FileStorage::READ);
    fs["RGB_intrinsic_params"]>>K_rgb;
    fs["IR_intrinsic_params"]>>K_ir;

    float fx = K_ir.at<double>(0,0); float fy = K_ir.at<double>(1,1);
    float cx = K_ir.at<double>(0,2); float cy = K_ir.at<double>(1,2);
    float invfx = 1.0f/fx;
    float invfy = 1.0f/fy;

    int cloudDis = 3;
    int min_plane = 500;
    float AngTh = 3.0;
    float DisTh = 0.05;

    PointCloud::Ptr inputCloud( new PointCloud() );
    for ( int m=0; m<imDepth.rows; m+=cloudDis )
    {
        for ( int n=0; n<imDepth.cols; n+=cloudDis )
        {
            float d = imDepth.ptr<float>(m)[n];
            PointT p;
            p.z = d;
            p.x = ( n - cx) * p.z / fx;
            p.y = ( m - cy) * p.z / fy;
            p.r = 0;
            p.g = 0;
            p.b = 250;

            inputCloud->points.push_back(p);
        }
    }
    inputCloud->height = ceil(imDepth.rows/float(cloudDis));
    inputCloud->width = ceil(imDepth.cols/float(cloudDis));

    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(inputCloud);
    ne.compute(*cloud_normals);

    vector<pcl::ModelCoefficients> coefficients;
    vector<pcl::PointIndices> inliers;
    pcl::PointCloud<pcl::Label>::Ptr labels ( new pcl::PointCloud<pcl::Label> );
    vector<pcl::PointIndices> label_indices;
    vector<pcl::PointIndices> boundary;

    pcl::OrganizedMultiPlaneSegmentation< PointT, pcl::Normal, pcl::Label > mps;
    mps.setMinInliers (min_plane);
    mps.setAngularThreshold (0.017453 * AngTh);
    mps.setDistanceThreshold (DisTh);
    mps.setInputNormals (cloud_normals);
    mps.setInputCloud (inputCloud);
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
    mps.segmentAndRefine (regions, coefficients, inliers, labels, label_indices, boundary);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(inputCloud);
    extract.setNegative(false);
}


int main(int argc, char ** argv){

    string sequence;
    int PATCH_SIZE;
    if (argc>1){
        PATCH_SIZE = atoi(argv[1]);
        sequence = argv[2];
    }else{
        PATCH_SIZE = 20;
        sequence = "seq_example";
    }

    stringstream string_buff;
    string data_path = "/CAPE/Data/";
    string_buff<<data_path<<sequence;

    // Get intrinsics
    cv::Mat K_rgb, K_ir, dist_coeffs_rgb, dist_coeffs_ir, R_stereo, t_stereo;
    stringstream calib_path;
    calib_path<<string_buff.str()<<"/calib_params.xml";
    auto params = loadCalibParameters(calib_path.str());

    auto plane_detector = capewrap(params);

    // Read frame 1 to allocate and get dimension
    cv::Mat rgb_img, d_img;
    int width, height;
    stringstream image_path;
    stringstream depth_img_path;
    stringstream rgb_img_path;
    rgb_img_path<<string_buff.str()<<"/rgb_0.png";
    depth_img_path<<string_buff.str()<<"/depth_0.png";

    rgb_img = cv::imread(rgb_img_path.str(),cv::IMREAD_COLOR);

    if(rgb_img.data){
        width = rgb_img.cols;
        height = rgb_img.rows;
    }else{
        cout<<"Error loading file";
        return -1;
    }

    cv::namedWindow("Seg");

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


    int i=0;
    while(true){

        // Read frame i
        rgb_img_path.str("");
        rgb_img_path<<string_buff.str()<<"/rgb_"<<i<<".png";
        rgb_img = cv::imread(rgb_img_path.str(),cv::IMREAD_COLOR);

        if (!rgb_img.data)
            break;

        cout<<"Frame: "<<i<<endl;

        // Read depth image
        depth_img_path.str("");
        depth_img_path<<string_buff.str()<<"/depth_"<<i<<".png";

        d_img = cv::imread(depth_img_path.str(), cv::IMREAD_ANYDEPTH);
        d_img.convertTo(d_img, CV_32F);

        cv::Size s = d_img.size();
        cout << d_img.dims << " " << s.width << " " << s.height << endl;

        cv::Mat_<cv::Vec3b> seg_rz = cv::Mat_<cv::Vec3b>(height,width,cv::Vec3b(0,0,0));
        cv::Mat_<uchar> seg_output = cv::Mat_<uchar>(height,width,uchar(0));

        // Get intrinsics
        auto output = plane_detector.process(d_img);

        apply_pcl(d_img, calib_path.str());

        // Map segments with color codes and overlap segmented image w/ RGB
        uchar * sCode;
        uchar * dColor;
        uchar * srgb;
        int code;
        for(int r=0; r<  height; r++){
            dColor = seg_rz.ptr<uchar>(r);
            sCode = seg_output.ptr<uchar>(r);
            srgb = rgb_img.ptr<uchar>(r);
            for(int c=0; c< width; c++){
                code = *sCode;
                if (code>0){
                    dColor[c*3] =   color_code[code-1][0]/2 + srgb[0]/2;
                    dColor[c*3+1] = color_code[code-1][1]/2 + srgb[1]/2;
                    dColor[c*3+2] = color_code[code-1][2]/2 + srgb[2]/2;;
                }else{
                    dColor[c*3] =  srgb[0];
                    dColor[c*3+1] = srgb[1];
                    dColor[c*3+2] = srgb[2];
                }
                sCode++; srgb++; srgb++; srgb++;
            }
        }

        // Show frame rate and labels
        cv::rectangle(seg_rz,  cv::Point(0,0),cv::Point(width,20), cv::Scalar(0,0,0),-1);
        std::stringstream fps;
        cv::putText(seg_rz, fps.str(), cv::Point(15,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,1));

        cout<<"Nr cylinders:"<<output.nr_cylinders<<endl;
        cout<<"Nr planes:"<<output.nr_planes<<endl;
        int cylinder_code_offset = 50;
        // show cylinder labels
        if (output.nr_cylinders>0){
            std::stringstream text;
            text<<"Cylinders:";
            cv::putText(seg_rz, text.str(), cv::Point(width/2,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,1));
            for(int j=0;j<output.nr_cylinders;j++){
                cv::rectangle(seg_rz,  cv::Point(width/2 + 80+15*j,6),cv::Point(width/2 + 90+15*j,16), cv::Scalar(color_code[cylinder_code_offset+j][0],color_code[cylinder_code_offset+j][1],color_code[cylinder_code_offset+j][2]),-1);
            }
        }
//        cv::imshow("Seg", seg_rz);
//        cv::waitKey(1);
//        sleep(10);
        i++;

    }
    return 0;
}

