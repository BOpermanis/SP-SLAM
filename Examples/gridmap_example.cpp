#include <iostream>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

#include "grid_map_core/GridMap.hpp"
#include "grid_map_cv/grid_map_cv.hpp"
using namespace std;
using namespace grid_map;

typedef cv::Vec3f PointT;

float dot(PointT a, PointT b){
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

PointT crossProduct(PointT &v_A, PointT &v_B) {
    PointT c_P;
    c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
    c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
    c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
    return c_P;
}

cv::Vec2f to_plane_coords(PointT &pt, cv::Mat3f &A1){
    auto a = cv::Mat(pt);
    cv::Mat b = A1 * a;
    return cv::Vec2f(b.at<float>(1), b.at<float>(2));
}

cv::Mat3f projector_matrix(cv::Mat &coef){
    PointT plane_norm(coef.at<float>(0), coef.at<float>(1), coef.at<float>(2));
    PointT e1(-plane_norm[1], plane_norm[0], 0.0);
    e1 /= cv::sum(e1)[0];
    PointT e2 = crossProduct(plane_norm, e1);
    e2 /= cv::sum(e2)[0];

    cv::Mat A = cv::Mat::zeros(cv::Size(3, 3),CV_32F);
    A.at<cv::Vec3f>(0) = plane_norm;
    A.at<cv::Vec3f>(1) = e1;
    A.at<cv::Vec3f>(2) = e2;

    auto A1 = A.t().inv();

//    cout << "plane_norm " << plane_norm << endl;
//    cout << "e1 " << e1 << endl;
//    cout << "e2 " << e2 << endl;
//    cout << "A1" << endl << A1 << endl;

    return A1;
}

int main() {

//    cv::Mat coef = cv::Mat_<float>(4,1);
////    coef.at<float>(0) = 0.0;
////    coef.at<float>(1) = 1.0;
////    coef.at<float>(2) = 0.0;
////    coef.at<float>(3) = 1.0;
//
//    coef.at<float>(0) = -0.75097949;
//    coef.at<float>(1) = 0.61990957;
//    coef.at<float>(2) = 0.22746853;
//    coef.at<float>(3) = -0.13157494;
//
////    coef.push_back(-0.75097949);
////    coef.push_back(0.61990957);
////    coef.push_back(0.22746853);
////    coef.push_back(-0.13157494);
//
////    PointT pt(-1.22541887, 0.50870463, 1.18027016);
//    PointT pt(2.0, 100, 2.0);
//
//    float v = coef.at<float>(0) * coef.at<float>(0) + coef.at<float>(1) * coef.at<float>(1) + coef.at<float>(2) * coef.at<float>(2);
//    v = sqrt(v);
//    coef.at<float>(0) = coef.at<float>(0) / v;
//    coef.at<float>(1) = coef.at<float>(1) / v;
//    coef.at<float>(2) = coef.at<float>(2) / v;
//
//    auto A1 = projector_matrix(coef);
//    auto coord = to_plane_coords(pt, A1);
//    cout << "result " << coord << endl;


    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    GridMap map;
    map = GridMap( { "layer" });
    map.setGeometry(Length(3.0, 3.0), 0.01, Position(0.0, 0.0));
    map["layer"].setConstant(0.0);
    cout << map.getSize() << endl;

    Polygon polygon;
    polygon.addVertex(Position(1.0, 1.0));
    polygon.addVertex(Position(2.0, 1.0));
    polygon.addVertex(Position(2.0, 2.0));
    polygon.addVertex(Position(1.0, 2.0));


    Polygon polygon2;
    polygon2.addVertex(Position(1.2, 1.2));
    polygon2.addVertex(Position(2.2, 1.2));
    polygon2.addVertex(Position(2.2, 2.2));
    polygon2.addVertex(Position(1.2, 2.2));

//    PolygonIterator iterator(map, polygon);

    for (PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) map.at("layer", *iterator) += 1;
    for (PolygonIterator iterator(map, polygon2); !iterator.isPastEnd(); ++iterator) map.at("layer", *iterator) += 1;

//    cout << map.at("layer", Index(1,1)) << endl;
//    cout << map.at("layer", Index(51,51)) << endl;
//    cout << map.at("layer", Index(100,100)) << endl;
//    cout << map.at("layer", Index(219,219)) << endl;
//    cout << map.at("layer", Index(-130,120)) << endl;

    cout << map.atPosition("layer", Position(1.5,1.5)) << endl;
    cout << map.atPosition("layer", Position(1.01,1.01)) << endl;
    cout << map.atPosition("layer", Position(0.51,0.51)) << endl;
    cout << map.atPosition("layer", Position(0.0,0.0)) << endl;
//    cout << map.atPosition("layer", Position(-0.51,-0.51)) << endl;
//    cout << map.atPosition("layer", Position(2.02,2.02)) << endl;
//    cout << map.atPosition("layer", Position(2.99,2.99)) << endl;
    Position p;
    Index i;
    map.getPosition(Index(100,100), p);
    map.getIndex(Position(0.51,0.51), i);
    cout << p << endl;
    cout << i << endl;

    const float minValue = 0.0;
    const float maxValue = 2.0;
//    replaceNan(map.get("layer"), minValue); // When we move `mapIn`, new areas are filled with NaN. As `toImage` does not support NaN, we replace NaN with `minValue` instead.
    cv::Mat image;
    GridMapCvConverter::toImage<unsigned short, 1>(map, "layer", CV_16UC1, minValue, maxValue, image);
    cv::imshow( "Display window",  image);
    cv::waitKey(1000000);
    return 0;
}