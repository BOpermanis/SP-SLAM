#include <iostream>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

#include "grid_map_core/GridMap.hpp"
#include "grid_map_cv/grid_map_cv.hpp"
using namespace std;
using namespace grid_map;

typedef cv::Vec3f PointT;

PointT crossProduct(PointT &v_A, PointT &v_B) {
    PointT c_P;
    c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
    c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
    c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
    return c_P;
}

float dot(PointT a, PointT b){
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

int main() {

    cv::Mat coef;
    coef.push_back(-0.75097949);
    coef.push_back(0.61990957);
    coef.push_back(0.22746853);
    coef.push_back(-0.13157494);

    PointT pt(-1.22541887, 0.50870463, 1.18027016);

    PointT plane_norm(coef.at<float>(0), coef.at<float>(1), coef.at<float>(2));
    PointT e1(-plane_norm[1], plane_norm[0], 0.0);
    e1 /= cv::sum(e1)[0];
    PointT e2 = crossProduct(plane_norm, e1);
    e2 /= cv::sum(e2)[0];
    cout << cv::sum(e1)[0] << " " << cv::sum(e2)[0] << endl;


//    cout << dot(e1, e2) << endl;
//    cout << dot(plane_norm, e2) << endl;
//    cout << dot(e1, plane_norm) << endl;

    cv::Mat A;
    A.push_back(plane_norm);
    A.push_back(e1);
    A.push_back(e2);

    cout << "e1 " << e1 << endl;
    cout << "A " << A << endl;

    cv::Mat A1 = A.t().inv();
    A1.reshape(9, 1);

//    cout << A1 * pt.reshape(3) << endl;


//    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//    GridMap map( { "layer" });
//    map.setGeometry(Length(3.0, 3.0), 0.01, Position(0.0, 0.0));
//    map["layer"].setConstant(0.0);
//    cout << map.getSize() << endl;
//
//    Polygon polygon;
//    polygon.addVertex(Position(1.0, 1.0));
//    polygon.addVertex(Position(2.0, 1.0));
//    polygon.addVertex(Position(2.0, 2.0));
//    polygon.addVertex(Position(1.0, 2.0));
//
//
//    Polygon polygon2;
//    polygon2.addVertex(Position(1.2, 1.2));
//    polygon2.addVertex(Position(2.2, 1.2));
//    polygon2.addVertex(Position(2.2, 2.2));
//    polygon2.addVertex(Position(1.2, 2.2));
//
////    PolygonIterator iterator(map, polygon);
//
//    for (PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) map.at("layer", *iterator) += 1;
//    for (PolygonIterator iterator(map, polygon2); !iterator.isPastEnd(); ++iterator) map.at("layer", *iterator) += 1;
//
////    cout << map.at("layer", Index(1,1)) << endl;
////    cout << map.at("layer", Index(51,51)) << endl;
////    cout << map.at("layer", Index(100,100)) << endl;
////    cout << map.at("layer", Index(219,219)) << endl;
////    cout << map.at("layer", Index(-130,120)) << endl;
//
//    cout << 11111111111 << endl;
//    cout << map.atPosition("layer", Position(1.5,1.5)) << endl;
//    cout << map.atPosition("layer", Position(1.01,1.01)) << endl;
//    cout << map.atPosition("layer", Position(0.51,0.51)) << endl;
//    cout << map.atPosition("layer", Position(0.0,0.0)) << endl;
//    cout << map.atPosition("layer", Position(-0.51,-0.51)) << endl;
//    cout << map.atPosition("layer", Position(2.02,2.02)) << endl;
////    cout << map.atPosition("layer", Position(2.99,2.99)) << endl;
//    Position p;
//    Index i;
//    map.getPosition(Index(100,100), p);
//    map.getIndex(Position(0.51,0.51), i);
//    cout << p << endl;
//    cout << i << endl;

    // Convert to image.
//    const float minValue = 0.0;
//    const float maxValue = 2.0;
//    replaceNan(map.get("layer"), minValue); // When we move `mapIn`, new areas are filled with NaN. As `toImage` does not support NaN, we replace NaN with `minValue` instead.
//    cv::Mat image;
//    GridMapCvConverter::toImage<unsigned short, 1>(map, "layer", CV_16UC1, minValue, maxValue, image);
//    cv::imshow( "Display window",  image);
//    cv::waitKey(1000000);
    return 0;
}