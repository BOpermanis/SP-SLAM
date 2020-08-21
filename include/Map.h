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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "MapPlane.h"
#include "MapLine.h"
#include <set>
#include <string>
#include <map>
#include <mutex>
//#include <pcl/common/transforms.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/ModelCoefficients.h>


namespace ORB_SLAM2
{

class MapPoint;
class MapLine;
class KeyFrame;
class MapPlane;
class Frame;

class Map
{
public:
//    typedef pcl::PointXYZRGB PointT;
//    typedef pcl::PointCloud <PointT> PointCloud;
    typedef cv::Vec3f PointT;
    typedef std::vector<PointT> PointCloud;
    Map(const std::string &strSettingPath);

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void AddMapPlane(MapPlane* pMP);
    void AddNotSeenMapPlane(MapPlane* pMP);
    void EraseNotSeenMapPlane(MapPlane* pMP);

    std::vector<MapPlane*> GetAllMapPlanes();
    std::vector<MapPlane*> GetNotSeenMapPlanes();

    void EraseMapPoint(MapPoint* pMP);
    void EraseMapPlane(MapPlane* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    void AddMapLine(MapLine* pMP);
    void EraseMapLine(MapLine* pMP);
    void SetReferenceMapLines(const std::vector<MapLine*> &vpMPs);


    void AssociatePlanesByBoundary(Frame &pF, bool out = false);
    void SearchMatchedPlanes(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPlane*> &vpPlanes,
                             std::vector<MapPlane*> &vpMatched,std::vector<MapPlane*> &vpMatchedPar,std::vector<MapPlane*> &vpMatchedVer,
                             bool out = false);

    double PointDistanceFromPlane(const cv::Mat& plane, const PointCloud &boundry, bool out = false);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<long unsigned int> GetRemovedPlanes();

    std::vector<MapLine*> GetAllMapLines();
    std::vector<MapLine*> GetReferenceMapLines();


    long unsigned int MapPointsInMap();
    long unsigned int MapPlanesInMap();
    long unsigned int MapLinesInMap();
    long unsigned int NotSeenMapPlanesInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;
    std::mutex mMutexGridmapping;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::set<MapLine*> mspMapLines;
    std::vector<MapLine*> mvpReferenceMapLines;

    std::set<MapPlane*> mspNotSeenMapPlanes;
    std::set<MapPlane*> mspMapPlanes;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    std::vector<long unsigned int> mvnRemovedPlanes;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    float mfDisTh;
    float mfAngleTh;
    float mfVerTh;
    float mfParTh;

};

} //namespace ORB_SLAM

#endif // MAP_H
