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
//#include <opencv2/imgproc.hpp>
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
#include <iostream>
#include <fstream>

using namespace std;

void writeMatToFile(cv::Mat& m, const char* filename)
{
    ofstream fout(filename);
    if(!fout)
    {
        cout<<"File Not Opened"<<endl;  return;
    }
    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            if (m.at<uchar>(i,j) == 0){
                fout<<"0,";
            } else {
                fout<<"1,";
            }
        }
        fout<<endl;
    }
    fout.close();
}

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
    for (auto coef: coefficients){
        cout << "222 ";
        for(auto a: coef.values){
            cout << a << " ";
        }
        cout << endl;
    }

    for (int i = 0; i < inliers.size(); ++i) {
        PointCloud::Ptr planeCloud(new PointCloud());
        cv::Mat coef = (cv::Mat_<float>(4,1) << coefficients[i].values[0],
                coefficients[i].values[1],
                coefficients[i].values[2],
                coefficients[i].values[3]);
        if(coef.at<float>(3) < 0)
            coef = -coef;

//            extract.setIndices(pcl::PointIndicesConstPtr(inliers[i]));
        extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers[i]));
        extract.filter(*planeCloud);


        PointCloud::Ptr boundaryPoints(new PointCloud());
        // boundaryPoints->points ir std::vector<PointT, Eigen::aligned_allocator<PointT> > points;
        boundaryPoints->points = regions[i].getContour();
//        for(auto &a: *boundaryPoints){
//            cout << a << endl;
//        }
//        mvBoundaryPoints.push_back(*boundaryPoints);
//        mvPlaneCoefficients.push_back(coef);
    }

}
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

//    cv::namedWindow("Seg");

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
        for (const auto out: output.plane_params){
            cout << "111 ";
            for(auto n: out.normal){
                cout << n << " ";
            }
            cout << " " << out.d << endl;
        }

//        cv::FileStorage file("/home/slam_data/data_sets/seg_output.csv", cv::FileStorage::WRITE);
//        file << "mat" << output.seg_output;
//        cv::Mat a = output.seg_output(cv::Rect(0, 0, 160, 480));



        for(uchar i_plane = 1; i_plane<=output.nr_planes; i_plane++){
            auto mask = plane_mask(output.seg_output, i_plane);
//            writeMatToFile(mask, "/home/slam_data/data_sets/seg_output.csv");
//            cv::Mat contour;

//            if(coef.at<float>(3) < 0)
//                coef = -coef;

            Dilate(mask, mask);
            Erosion(mask, mask);
            vector<vector<cv::Point> > contours;
            vector<cv::Point> contour0;
            vector<cv::Point> border;
            vector<cv::Vec4i> hierarchy;
            cv::findContours(mask, contours, hierarchy, -1, 2);

            int maxlen = 0;
            for(auto &cnt: contours){
                if (cnt.size() > maxlen){
                    contour0 = cnt;
                    maxlen = cnt.size();
                }
            }

            cout << contours.size() << " " << contours[0].size() << endl;
            cv::Scalar color( 255, 255, 255 );
            cv::Scalar color1( 0, 127, 0 );

            double epsilon = 0.1*cv::arcLength(contour0,true);
//            approx = cv2.approxPolyDP(contour,epsilon,True)
            cv::approxPolyDP(contour0, border, epsilon, false);
            vector<vector<cv::Point> > contours111;
            contours111.push_back(border);
            cv::drawContours( mask, contours, 0, color, CV_FILLED, 8);
            cv::drawContours( mask, contours111, 0, color1, 3, 8);

            for (auto &pt: border){
                float d = d_img.at<float>(pt.x, pt.y);
                cout << "33 " << pt << " " << d_img.at<float>(pt.x, pt.y) << endl;
            }




//            PointCloud::Ptr boundaryPoints(new PointCloud());
//
//            std::vector<PointT, Eigen::aligned_allocator<PointT> > points;

            cv::Mat coef = (cv::Mat_<float>(4,1) <<
                    output.plane_params[i_plane].normal[0],
                    output.plane_params[i_plane].normal[1],
                    output.plane_params[i_plane].normal[2],
                    output.plane_params[i_plane].d);

            if(coef.at<float>(3) < 0)
                coef = -coef;

            std::vector<PointT, Eigen::aligned_allocator<PointT> > points;
            for(auto &pt: border){

                float y = (pt.y - plane_detector.cy_ir) / plane_detector.fy_ir;
                float x = (pt.x - plane_detector.cx_ir) / plane_detector.fx_ir;

                float theta = coef.at<float>(3) / (x * coef.at<float>(0) + y * coef.at<float>(1) + coef.at<float>(2));

                PointT p;
                p.z = theta;
                p.x = x * theta;
                p.y = y * theta;

                p.r = 0;
                p.g = 0;
                p.b = 250;
                points.push_back(p);
            }
            PointCloud::Ptr boundaryPoints(new PointCloud());
            boundaryPoints->points = points;


            cv::imshow("test", mask);
            cv::waitKey(0);
        }
        apply_pcl(d_img, calib_path.str());

        cout<<"Nr planes:"<<output.nr_planes<<endl;
//        cv::imshow("Seg", seg_rz);
//        cv::waitKey(1);
//        sleep(10);
        i++;

    }
    return 0;
}

