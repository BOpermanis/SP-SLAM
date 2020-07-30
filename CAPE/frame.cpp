//
// Created by root on 7/30/20.
//
#include <math.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
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
#include <string>
#include <iostream>
#include <fstream>
#include <cstdio>
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud <PointT> PointCloud;

class Frame {
public:
        long unsigned int nNextId=0;
        bool mbInitialComputations=true;
        float cx, cy, fx, fy, invfx, invfy;
        float mnMinX, mnMinY, mnMaxX, mnMaxY;
        float mfGridElementWidthInv, mfGridElementHeightInv;
        int FRAME_GRID_ROWS = 48;
        int FRAME_GRID_COLS = 64;
        cv::Mat mK;
    cv::Mat mDistCoef;

    vector<PointCloud> mvPlanePoints;
    vector<PointCloud> mvNotSeenPlanePoints;

    vector<PointCloud> mvBoundaryPoints;
    vector<PointCloud> mvNotSeenBoundaryPoints;

    vector<cv::Mat> mvPlaneCoefficients;
    vector<cv::Mat> mvNotSeenPlaneCoefficients;

    int cloudDis = 3;
    int min_plane = 500;
    float AngTh = 3.0;
    float DisTh = 0.05;

    double lineRatio = 0.2;
    float disTh = 0.01;
    int mMinFrames, mMaxFrames;
    float mbf;
    cv::FileStorage fs;
        Frame(){
            cv::Mat K_rgb, K_ir;
            cv::FileStorage fs("/CAPE/Data/seq_example/calib_params.xml",cv::FileStorage::READ);
            fs["RGB_intrinsic_params"]>>K_rgb;
            fs["IR_intrinsic_params"]>>K_ir;

            fx = K_ir.at<double>(0,0);
            fy = K_ir.at<double>(1,1);
            cx = K_ir.at<double>(0,2);
            cy = K_ir.at<double>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mK = K_ir;

            cv::Mat DistCoef(4,1,CV_32F);
            DistCoef.at<float>(0) = fs["Camera.k1"];
            DistCoef.at<float>(1) = fs["Camera.k2"];
            DistCoef.at<float>(2) = fs["Camera.p1"];
            DistCoef.at<float>(3) = fs["Camera.p2"];
            const float k3 = fs["Camera.k3"];
            if(k3!=0)
            {
                DistCoef.resize(5);
                DistCoef.at<float>(4) = k3;
            }
            DistCoef.copyTo(mDistCoef);

            mbf = 40.0;

            float fps = fs["Camera.fps"];
            if(fps==0)
                fps=30;

            // Max/Min Frames to insert keyframes and to check relocalisation
            mMinFrames = 0;
            mMaxFrames = fps;

        }


    void apply_pcl(const cv::Mat &imDepth){


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
        mvBoundaryPoints.push_back(*boundaryPoints);
        mvPlaneCoefficients.push_back(coef);
        }

    }


        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
        {
            posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
            posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

            //Keypoint's coordinates are undistorted, which could cause to go out of the image
            if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
                return false;

            return true;
        }

        void ComputeImageBounds(const cv::Mat &imLeft)
        {
            if(mDistCoef.at<float>(0)!=0.0)
            {
                cv::Mat mat(4,2,CV_32F);
                mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
                mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
                mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
                mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

                // Undistort corners
                mat=mat.reshape(2);
                cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
                mat=mat.reshape(1);

                mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
                mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
                mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
                mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

            }
            else
            {
                mnMinX = 0.0f;
                mnMaxX = imLeft.cols;
                mnMinY = 0.0f;
                mnMaxY = imLeft.rows;
            }

        }



        void ComputePlanesFromOrganizedPointCloud(const cv::Mat &imDepth) {
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

//        srand(time(0));
            for (int i = 0; i < inliers.size(); ++i) {
                PointCloud::Ptr planeCloud(new PointCloud());
                cv::Mat coef = (cv::Mat_<float>(4,1) << coefficients[i].values[0],
                        coefficients[i].values[1],
                        coefficients[i].values[2],
                        coefficients[i].values[3]);
                if(coef.at<float>(3) < 0)
                    coef = -coef;

                if(!PlaneNotSeen(coef)){
                    continue;
                }
//            extract.setIndices(pcl::PointIndicesConstPtr(inliers[i]));
                extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers[i]));
                extract.filter(*planeCloud);

                mvPlanePoints.push_back(*planeCloud);

                PointCloud::Ptr boundaryPoints(new PointCloud());
                boundaryPoints->points = regions[i].getContour();
                mvBoundaryPoints.push_back(*boundaryPoints);
                mvPlaneCoefficients.push_back(coef);
            }

        }

        void GeneratePlanesFromBoundries(const cv::Mat &imDepth) {
            pcl::SACSegmentation<PointT> segLine;
            pcl::ExtractIndices<PointT> extract;

            segLine.setOptimizeCoefficients(true);
            segLine.setModelType(pcl::SACMODEL_LINE);
            segLine.setMaxIterations(1000);
            segLine.setDistanceThreshold(disTh);
            PointCloud::Ptr boundPoints(new pcl::PointCloud<PointT>);
            PointCloud::Ptr tempPoints(new pcl::PointCloud<PointT>);
            PointCloud::Ptr linePoints(new pcl::PointCloud<PointT>);
            pcl::PointIndices::Ptr lineins (new pcl::PointIndices ());
            pcl::ModelCoefficients::Ptr coeffline (new pcl::ModelCoefficients ());

            int iend = mvBoundaryPoints.size() - 1;
            for(int i=iend; i >= 0; --i){
                boundPoints->points = mvBoundaryPoints[i].points;
                int boundSize = boundPoints->points.size();
                if(boundSize < 50){
                    if(boundSize == 0)
                        GenerateBoundaryPoints(i);
                    continue;
                }
                for(int j=0; j < 4; j++) {
                    segLine.setInputCloud(boundPoints);
                    segLine.segment(*lineins, *coeffline);
                    if (lineins->indices.size() < lineRatio * boundSize){
                        break;
                    }

                    extract.setInputCloud(boundPoints);
                    extract.setIndices(lineins);
                    extract.setNegative(false);
                    extract.filter(*linePoints);
                    cv::Mat pc = (cv::Mat_<float>(3,1) << coeffline->values[0], coeffline->values[1], coeffline->values[2]);

                    if(LineInRange(pc) && IsBorderLine(linePoints,imDepth)) {
                        cv::Mat coef = (cv::Mat_<float>(6,1) << coeffline->values[0],
                                coeffline->values[1],
                                coeffline->values[2],
                                coeffline->values[3],
                                coeffline->values[4],
                                coeffline->values[5]);
                        if(CaculatePlanes(mvPlaneCoefficients[i], coef)){
                            for(auto &p : linePoints->points){
                                p.r = 255;
                                p.g = 0;
                                p.b = 0;
                            }
                            mvPlanePoints[mvPlanePoints.size()-1] += *linePoints;
                            mvBoundaryPoints.push_back(*linePoints);
                        }else {
                        }
                    }
                    extract.setNegative(true);
                    extract.filter(*tempPoints);
                    boundPoints.swap(tempPoints);
                }
            }

        }

        void GenerateBoundaryPoints(int i) {
            for(int j=0;j<mvPlanePoints[i].points.size();j+=20){
                const PointT &pPlane = mvPlanePoints[i].points[j];
                PointT p;
                p.z = pPlane.z;
                p.x = pPlane.x;
                p.y = pPlane.y;
                mvBoundaryPoints[i].points.push_back(p);
            }

        }

        bool IsBorderLine(const PointCloud::Ptr line, const cv::Mat &imDepth) {
            int s = line->points.size();
            int res = 0;
            for(auto p:line->points){
                cv::Mat pc = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
                if(!IsBorderPoint(pc,imDepth))
                    res ++;
                if(res > s/4)
                    return false;
            }
            return true;
        }

        bool IsBorderPoint(const cv::Mat &Pc, const cv::Mat &imDepth) {
            const float &PcX = Pc.at<float>(0);
            const float &PcY = Pc.at<float>(1);
            const float &PcZ = Pc.at<float>(2);
            if(PcZ<0.0f)
                return false;

            const float invz = 1.0f/PcZ;
            const float u=fx*PcX*invz+cx;
            const float v=fy*PcY*invz+cy;
            int num = 0, nan = 0;
            float res = 0;
            int b = 10;
            for(int j = v - b; j < v + b; ++j){
                for(int i = u - b; i < u + b; ++i){
                    if(imDepth.ptr<float>(j)[i] > 0.05){
                        res += imDepth.ptr<float>(j)[i];
                        num++;
                    }else{
                        nan++;
                        if(nan > b*b){
                            return false;
                        }
                    }
                }
            }
            if(PcZ - res/num > 0.1){
                return false;
            }
            return true;
        }

        bool LineInRange(const cv::Mat &Pc) {
            const float &PcX = Pc.at<float>(0);
            const float &PcY = Pc.at<float>(1);
            const float &PcZ = Pc.at<float>(2);
            if(PcZ<0.0f)
                return false;

            const float invz = 1.0f/PcZ;
            const float u=fx*PcX*invz+cx;
            const float v=fy*PcY*invz+cy;
            if(u<(mnMinX+50) || u>(mnMaxX-50))
                return false;
            if(v<(mnMinY+50) || v>(mnMaxY-50))
                return false;

            return true;
        }


        bool CaculatePlanes(const cv::Mat &inputplane, const cv::Mat &inputline) {
            //(l,m,n) × (o,p,q) = (mq-np,no-lq,lp-mo)

            float a,b,c,d;
            a = inputplane.at<float>(1)*inputline.at<float>(5) - inputplane.at<float>(2)*inputline.at<float>(4);
            b = inputplane.at<float>(2)*inputline.at<float>(3) - inputplane.at<float>(0)*inputline.at<float>(5);
            c = inputplane.at<float>(0)*inputline.at<float>(4) - inputplane.at<float>(1)*inputline.at<float>(3);
            d = a*inputline.at<float>(0) + b*inputline.at<float>(1) + c*inputline.at<float>(2);
            float v = sqrt(a*a + b*b + c*c);

            cv::Mat coef = (cv::Mat_<float>(4,1) << a/v, b/v, c/v, -d/v);
            if(coef.at<float>(3) < 0)
                coef = -coef;
            if(PlaneNotSeen(coef)){

                PointCloud cloud;
                PointT p;

                for(float i = -0.25; i< 0.25;){
                    for(float j = -0.25; j < 0.25;){
                        p.x = inputline.at<float>(0) + i * inputline.at<float>(3) + j * inputplane.at<float>(0);
                        p.y = inputline.at<float>(1) + i * inputline.at<float>(4) + j * inputplane.at<float>(1);
                        p.z = (coef.at<float>(0)*p.x + coef.at<float>(1)*p.y + coef.at<float>(3)) / (-coef.at<float>(2));
                        p.r = 0;
                        p.g = 255;
                        p.b = 0;
                        cloud.points.push_back(p);
                        j = j + 0.01;
                    }
                    i = i + 0.01;
                }

                mvPlaneCoefficients.push_back(coef);
                mvPlanePoints.push_back(cloud);
                return true;
            }
            return false;
        }

        bool PlaneNotSeen(const cv::Mat &coef) {
            for (int j = 0; j < mvPlaneCoefficients.size(); ++j) {
                cv::Mat pM = mvPlaneCoefficients[j];
                float d = pM.at<float>(3,0) - coef.at<float>(3,0);
                float angle = pM.at<float>(0,0) * coef.at<float>(0,0) +
                              pM.at<float>(1,0) * coef.at<float>(1,0) +
                              pM.at<float>(2,0) * coef.at<float>(2,0);

                if(d > 0.2 || d < -0.2)
                    continue;

                if(angle < 0.9397 && angle > -0.9397)
                    continue;
                return false;
            }
            for (int j = 0; j < mvNotSeenPlaneCoefficients.size(); ++j) {
                cv::Mat pM = mvNotSeenPlaneCoefficients[j];
                float d = pM.at<float>(3,0) - coef.at<float>(3,0);
                float angle = pM.at<float>(0,0) * coef.at<float>(0,0) +
                              pM.at<float>(1,0) * coef.at<float>(1,0) +
                              pM.at<float>(2,0) * coef.at<float>(2,0);
                if(d > 0.2 || d < -0.2)
                    continue;
                if(angle < 0.9397 && angle > -0.9397)
                    continue;
                return false;
            }
            return true;
        }

//        cv::Mat ComputePlaneWorldCoeff(const int &idx) {
//            cv::Mat temp;
//            cv::transpose(mTcw, temp);
//            return temp*mvPlaneCoefficients[idx];
//        }
//
//        cv::Mat ComputeNotSeenPlaneWorldCoeff(const int &idx) {
//            cv::Mat temp;
//            cv::transpose(mTcw, temp);
//            return temp*mvNotSeenPlaneCoefficients[idx];
//        }

};