//
// Created by root on 7/1/20.
//

/*
 * Copyright 2018 Pedro Proenza <p.proenca@surrey.ac.uk> (University of Surrey)
 *
 */


#ifndef capewrap_cpp
#define capewrap_cpp

#include <iostream>

#include <math.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "CAPE/CAPE.h"

using namespace std;

class cape_output {
public:
    int nr_planes, nr_cylinders;
    cv::Mat_<uchar> seg_output;
    vector<PlaneSeg> plane_params;
    vector<CylinderSeg> cylinder_params;
    cape_output(int nr_planes1, int nr_cylinders1, cv::Mat_<uchar> &seq_output1, vector<PlaneSeg> &plane_params1, vector<CylinderSeg> &cylinder_params1){
        nr_planes = nr_planes1;
        nr_cylinders = nr_cylinders1;
        seg_output = seq_output1;
        plane_params = plane_params1;
        cylinder_params = cylinder_params1;
    }
};

template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst)
{
    if (!(src.Flags & Eigen::RowMajorBit))
    {
        cv::Mat _src(src.cols(), src.rows(), cv::DataType<_Tp>::type,
                     (void*)src.data(), src.stride() * sizeof(_Tp));
        cv::transpose(_src, dst);
    }
    else
    {
        cv::Mat _src(src.rows(), src.cols(), cv::DataType<_Tp>::type,
                     (void*)src.data(), src.stride() * sizeof(_Tp));
        _src.copyTo(dst);}
}

class capewrap {
public:
    void projectPointCloud(cv::Mat &X, cv::Mat &Y, cv::Mat &Z, cv::Mat &U, cv::Mat &V, float fx_rgb, float fy_rgb, float cx_rgb,
                           float cy_rgb, Eigen::MatrixXf &cloud_array) {

        int width = X.cols;
        int height = X.rows;

        // Project to image coordinates
        cv::divide(X, Z, U, 1);
        cv::divide(Y, Z, V, 1);
        U = U * fx_rgb + cx_rgb;
        V = V * fy_rgb + cy_rgb;
        // Reusing U as cloud index
        //U = V*width + U + 0.5;

        float *sz, *sx, *sy, *u_ptr, *v_ptr, *id_ptr;
        float z, u, v;
        int id;
        for (int r = 0; r < height; r++) {
            sx = X.ptr<float>(r);
            sy = Y.ptr<float>(r);
            sz = Z.ptr<float>(r);
            u_ptr = U.ptr<float>(r);
            v_ptr = V.ptr<float>(r);
            for (int c = 0; c < width; c++) {
                z = sz[c];
                u = u_ptr[c];
                v = v_ptr[c];
                if (u > 0 && v > 0 && u < width && v < height) {
                    id = floor(v) * width + u;
                    cloud_array(id, 0) = sx[c];
                    cloud_array(id, 1) = sy[c];
                    cloud_array(id, 2) = z;
                }
            }
        }
    }

    void organizePointCloudByCell(Eigen::MatrixXf &cloud_in, Eigen::MatrixXf &cloud_out, cv::Mat &cell_map1) {

        int width1 = cell_map1.cols;
        int height1 = cell_map1.rows;
        int mxn = width1 * height1;
        int mxn2 = 2 * mxn;

        int id, it(0);
        int *cell_map_ptr;
        for (int r = 0; r < height1; r++) {
            cell_map_ptr = cell_map1.ptr<int>(r);
            for (int c = 0; c < width1; c++) {
                id = cell_map_ptr[c];
                *(cloud_out.data() + id) = *(cloud_in.data() + it);
                *(cloud_out.data() + mxn + id) = *(cloud_in.data() + mxn + it);
                *(cloud_out.data() + mxn2 + id) = *(cloud_in.data() + mxn2 + it);
                it++;
            }
        }
    }

    bool done = false;
    float COS_ANGLE_MAX = cos(M_PI / 12);
    float MAX_MERGE_DIST = 50.0f;
    bool cylinder_detection = false;

    std::vector<cv::Vec3b> color_code;

    CAPE *plane_detector;

    int PATCH_SIZE = 20;
    cv::Mat_<int> cell_map;

    float fx_ir, fy_ir, cx_ir, cy_ir, fx_rgb, fy_rgb, cx_rgb, cy_rgb;
    int width, height;
    cv::Mat d_img;
    // d_img.convertTo(d_img, CV_32F);

    cv::Mat_<float> X, Y, X_t, Y_t, X_pre, Y_pre, U, V;
    Eigen::MatrixXf cloud_array, cloud_array_organized;

    capewrap(cv::FileStorage fSettings){
        std::map<string, float> map;
        map["Camera.fx"] = fSettings["Camera.fx"];
        map["Camera.fy"] = fSettings["Camera.fy"];
        map["Camera.cx"] = fSettings["Camera.cx"];
        map["Camera.cy"] = fSettings["Camera.cy"];
        map["Camera.fx"] = fSettings["Camera.fx"];
        map["Camera.fy"] = fSettings["Camera.fy"];
        map["Camera.cx"] = fSettings["Camera.cx"];
        map["Camera.cy"] = fSettings["Camera.cy"];
        map["Camera.width"]  = float(fSettings["Camera.width"]);
        map["Camera.height"] = float(fSettings["Camera.height"]);
        _capewrap(map);
    }

    capewrap(std::map<string, float> fSettings){
        _capewrap(fSettings);
    }

    bool loadCalibParameters(string filepath, cv:: Mat & intrinsics_rgb, cv::Mat & dist_coeffs_rgb, cv:: Mat & intrinsics_ir, cv::Mat & dist_coeffs_ir, cv::Mat & R, cv::Mat & T){

        cv::FileStorage fs(filepath,cv::FileStorage::READ);
        if (fs.isOpened()){
            fs["RGB_intrinsic_params"]>>intrinsics_rgb;
            fs["RGB_distortion_coefficients"]>>dist_coeffs_rgb;
            fs["IR_intrinsic_params"]>>intrinsics_ir;
            fs["IR_distortion_coefficients"]>>dist_coeffs_ir;
            fs["Rotation"]>>R;
            fs["Translation"]>>T;
            fs.release();
            return true;
        }else{
            cerr<<"Calibration file missing"<<endl;
            return false;
        }
    }

    void _capewrap(std::map<string, float> fSettings){

        // Get intrinsics
        fx_ir = fSettings["Camera.fx"];
        fy_ir = fSettings["Camera.fy"];
        cx_ir = fSettings["Camera.cx"];
        cy_ir = fSettings["Camera.cy"];
        fx_rgb = fSettings["Camera.fx"];
        fy_rgb = fSettings["Camera.fy"];
        cx_rgb = fSettings["Camera.cx"];
        cy_rgb = fSettings["Camera.cy"];
        width  = int(fSettings["Camera.width"]);
        height = int(fSettings["Camera.height"]);

        X = cv::Mat_<float>(height, width);
        Y = cv::Mat_<float>(height, width);
        X_pre = cv::Mat_<float>(height, width);
        Y_pre = cv::Mat_<float>(height, width);
        U = cv::Mat_<float>(height, width);
        V = cv::Mat_<float>(height, width);
        X_t = cv::Mat_<float>(height, width);
        Y_t = cv::Mat_<float>(height, width);
        cloud_array = Eigen::MatrixXf(width * height, 3);
        cloud_array_organized = Eigen::MatrixXf(width * height, 3);

        int nr_horizontal_cells = width / PATCH_SIZE;
        int nr_vertical_cells = height / PATCH_SIZE;

        for (int r = 0; r < height; r++) {
            for (int c = 0; c < width; c++) {
                // Not efficient but at this stage doesn t matter
                X_pre.at<float>(r, c) = (c - cx_ir) / fx_ir;
                Y_pre.at<float>(r, c) = (r - cy_ir) / fy_ir;
            }
        }
        // Pre-computations for maping an image point cloud to a cache-friendly array where cell's local point clouds are contiguous
        cell_map = cv::Mat_<int>(height, width);

        for (int r = 0; r < height; r++) {
            int cell_r = r / PATCH_SIZE;
            int local_r = r % PATCH_SIZE;
            for (int c = 0; c < width; c++) {
                int cell_c = c / PATCH_SIZE;
                int local_c = c % PATCH_SIZE;
                cell_map.at<int>(r, c) =
                        (cell_r * nr_horizontal_cells + cell_c) * PATCH_SIZE * PATCH_SIZE + local_r * PATCH_SIZE + local_c;
            }
        }

        plane_detector = new CAPE(height, width, PATCH_SIZE, PATCH_SIZE, cylinder_detection, COS_ANGLE_MAX, MAX_MERGE_DIST);
    }

    static float norm(const cv::Vec3f &x){
        return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    }

    void extract_lines(const Eigen::MatrixXf &cloud, const cv::Mat &imD, const cv::Mat &imGray, std::vector<cv::Mat> &lines){
        int margin = 50;
        cv::Mat edges;
        std::vector<cv::Vec4i> line_vecs;
        cv::Canny(imGray, edges, 50, 200, 3);
        cv::HoughLinesP(edges, line_vecs, 1, CV_PI/180, 150, 100, 10 );
        int d = 5;
        int ix, iy, i_cloud;
        float mad1, mad2;
        cv::Vec3f d1, d2;
        std::vector<cv::Vec3f> vec1, vec2;

        for(auto &line: line_vecs){
            int x1 = line[0];
            int y1 = line[1];
            int x2 = line[2];
            int y2 = line[3];

            if (x1 < margin or x1 > imGray.cols - margin or y1 < margin or y1 > imGray.rows - margin) continue;
            if (x2 < margin or x2 > imGray.cols - margin or y2 < margin or y2 > imGray.rows - margin) continue;

            vec1.clear();
            vec2.clear();
            d1 = cv::Vec3f(0, 0, 0); d2 = cv::Vec3f(0, 0, 0);
            mad1 = 0; mad2 = 0;

            for (ix=x1-d;ix<=x1+d;ix++)
                for (iy=y1-d;iy<=y1+d;iy++)
                    if (imGray.at<float>(ix, iy) > 0){
                        i_cloud = iy * width + ix;
                        vec1.emplace_back(cloud_array(i_cloud, 0), cloud_array(i_cloud, 1), cloud_array(i_cloud, 2));
                    }


            for (ix=x2-d;ix<=x2+d;ix++)
                for (iy=y2-d;iy<=y2+d;iy++)
                    if (imGray.at<float>(ix, iy) > 0){
                        i_cloud = iy * width + ix;
                        vec2.emplace_back(cloud_array(i_cloud, 0), cloud_array(i_cloud, 1), cloud_array(i_cloud, 2));
                    }

            if (vec1.size() < 5 || vec2.size() < 5) continue;

            for(auto &v: vec1) d1 += v;
            for(auto &v: vec2) d2 += v;
            d1 /= float(vec1.size());
            d2 /= float(vec2.size());

            for(const auto &v: vec1) mad1 += norm(d1 - v);
            for(const auto &v: vec2) mad2 += norm(d2 - v);

            mad1 /= float(vec1.size());
            mad2 /= float(vec2.size());

            if (mad1 > 0.1 || mad2 > 0.1) continue;

            auto x = cv::Mat(cv::Size(2, 4), CV_32F);
            x.at<cv::Vec4f>(0) = cv::Vec4f(d1[0], d1[1], d1[2], 1.0);
            x.at<cv::Vec4f>(1) = cv::Vec4f(d2[0], d2[1], d2[2], 1.0);
            lines.push_back(x);
        }
    }

    cape_output process(const cv::Mat &imD, const cv::Mat &imGray, std::vector<cv::Mat> &lines) {
        d_img = imD.clone();


//        cv::HoughLines(edges, lines, 1, CV_PI/180, 150, 100, 10 );

        X = X_pre.mul(d_img);
        Y = Y_pre.mul(d_img);
        cloud_array.setZero();

        X_t = ((float)1.0)*X;
        Y_t = ((float)1.0)*Y;
        d_img = ((float)1.0)*d_img;

        projectPointCloud(X_t, Y_t, d_img, U, V, fx_rgb, fy_rgb, cx_rgb, cy_rgb, cloud_array);

        extract_lines(cloud_array, imD, imGray, lines);


        cv::Mat_<uchar> seg_output = cv::Mat_<uchar>(height, width, uchar(0));

        int nr_planes, nr_cylinders;
        vector<PlaneSeg> plane_params;
        vector<CylinderSeg> cylinder_params;

        organizePointCloudByCell(cloud_array, cloud_array_organized, cell_map);

        plane_detector->process(cloud_array_organized, nr_planes, nr_cylinders, seg_output, plane_params,
                                cylinder_params);
        return cape_output(nr_planes, nr_cylinders, seg_output, plane_params, cylinder_params);
    }

    cv::Mat get_cloud(){
        cv::Mat cloud_cv;
        eigen2cv(cloud_array, cloud_cv);
        return cloud_cv;
    }
        };

#endif // capewrap_cpp
