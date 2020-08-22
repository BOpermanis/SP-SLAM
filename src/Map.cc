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

#include "Map.h"


#include<mutex>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

namespace ORB_SLAM2
{

Map::Map(const string &strSettingPath):mnMaxKFid(0),mnBigChangeIdx(0)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mfDisTh = fSettings["Plane.AssociationDisRef"];
    mfAngleTh = fSettings["Plane.AssociationAngRef"];
    mfVerTh = fSettings["Plane.VerticalThreshold"];
    mfParTh = fSettings["Plane.ParallelThreshold"];
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

    void Map::AddMapLine(MapLine *pML)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapLines.insert(pML);
    }

void Map::AddMapPlane(MapPlane *pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapPlanes.insert(pMP);
}

void Map::AddNotSeenMapPlane(ORB_SLAM2::MapPlane *pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspNotSeenMapPlanes.insert(pMP);
}

void Map::EraseNotSeenMapPlane(ORB_SLAM2::MapPlane *pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mspNotSeenMapPlanes.erase(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

    void Map::EraseMapLine(MapLine *pML)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapLines.erase(pML);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

void Map::EraseMapPlane(MapPlane *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPlanes.erase(pMP);

    mvnRemovedPlanes.push_back(pMP->mnId);
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

    void Map::SetReferenceMapLines(const vector<MapLine *> &vpMLs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapLines = vpMLs;
    }

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

    vector<MapLine*> Map::GetAllMapLines()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapLine*>(mspMapLines.begin(),mspMapLines.end());
    }

vector<MapPlane*> Map::GetAllMapPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPlane*>(mspMapPlanes.begin(),mspMapPlanes.end());;
}

vector<MapPlane*> Map::GetNotSeenMapPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPlane*>(mspNotSeenMapPlanes.begin(),mspNotSeenMapPlanes.end());;
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

    long unsigned int Map::MapLinesInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapLines.size();
    }

long unsigned int Map::MapPlanesInMap() {
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPlanes.size();
}

long unsigned int Map::NotSeenMapPlanesInMap() {
    unique_lock<mutex> lock(mMutexMap);
    return mspNotSeenMapPlanes.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

    vector<MapLine*> Map::GetReferenceMapLines()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapLines;
    }

vector<long unsigned int> Map::GetRemovedPlanes()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvnRemovedPlanes;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<MapLine*>::iterator sit=mspMapLines.begin(), send=mspMapLines.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    unique_lock<mutex> lock(mMutexGridmapping);
    for(set<MapPlane*>::iterator sit=mspMapPlanes.begin(), send=mspMapPlanes.end(); sit!=send; sit++)
        delete *sit;

    mspMapLines.clear();
    mspMapPoints.clear();
    mspKeyFrames.clear();
    mspMapPlanes.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpReferenceMapLines.clear();
    mvpKeyFrameOrigins.clear();
    mvnRemovedPlanes.clear();
}

    void Map::AssociateLines(ORB_SLAM2::Frame &pF) {
        for (int i = 0; i < pF.mvLines.size(); ++i) {
            cv::Mat pM = pF.ComputeLineWorldCoeff(i);
            for(auto sit=mspMapLines.begin(), send=mspMapLines.end(); sit!=send; sit++){
                cv::Mat pW = (*sit)->GetWorldPos();
                // TODO
            }
        }
    }

    void Map::AssociatePlanesByBoundary(ORB_SLAM2::Frame &pF, bool out) {
//        out = true;
        unique_lock<mutex> lock(mMutexMap);
        pF.mbNewPlane = false;

        int num_associated = 0;
        float sum_dis = 0.0;
        int num_total = 0;

        for (int i = 0; i < pF.mnPlaneNum; ++i) {

            cv::Mat pM = pF.ComputePlaneWorldCoeff(i);

            float ldTh = mfDisTh;
            float lverTh = mfVerTh;
            float lparTh = mfParTh;
            for(auto sit=mspMapPlanes.begin(), send=mspMapPlanes.end(); sit!=send; sit++){
                cv::Mat pW = (*sit)->GetWorldPos();

                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);

                num_total += 1;
                if ((angle > mfAngleTh || angle < -mfAngleTh)) // associate plane
                {
                    double dis = PointDistanceFromPlane(pM, (*sit)->mvBoundaryPoints, out);
                    sum_dis += dis;
                    num_associated += 1;
                    if(dis < ldTh) {

                        ldTh = dis;

                        pF.mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
                        pF.mvpMapPlanes[i] = (*sit);
                        continue;
                    }
                }

                // vertical planes
                if (angle < lverTh && angle > -lverTh) {

                    lverTh = abs(angle);
                    pF.mvpVerticalPlanes[i] = static_cast<MapPlane*>(nullptr);
                    pF.mvpVerticalPlanes[i] = (*sit);
                    continue;
                }

                //parallel planes
                if ((angle > lparTh || angle < -lparTh)) {

                    lparTh = abs(angle);
                    pF.mvpParallelPlanes[i] = static_cast<MapPlane*>(nullptr);
                    pF.mvpParallelPlanes[i] = (*sit);
                }else{
                    if(out)
                        cout << endl;
                }
            }

            if(ldTh == mfDisTh){ // associate in not seen planes
                for(auto it = mspNotSeenMapPlanes.begin(),iend = mspNotSeenMapPlanes.end(); it!=iend; ++it){
                    cv::Mat pW = (*it)->GetWorldPos();

                    float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                                  pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                                  pM.at<float>(2, 0) * pW.at<float>(2, 0);

                    if(out)
                        cout  << ":  angle : " << angle << endl;

                    if ((angle > mfAngleTh || angle < -mfAngleTh)) // associate plane
                    {

                        double dis = PointDistanceFromPlane(pM, (*it)->mvBoundaryPoints, out);
                        if(dis < ldTh) {
                            ldTh = dis;
                            pF.mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
                            pF.mvpMapPlanes[i] = (*it);
                        }
                    }
                }
            }
        }


        for (int i = 0; i < pF.mnNotSeenPlaneNum; ++i) {
            cv::Mat pM = pF.ComputeNotSeenPlaneWorldCoeff(i);
            float ldTh = mfDisTh;
            for(auto sit=mspMapPlanes.begin(), send=mspMapPlanes.end(); sit!=send; sit++){
                cv::Mat pW = (*sit)->GetWorldPos();

                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);

                if ((angle > mfAngleTh || angle < -mfAngleTh)) // associate plane
                {
                    double dis = PointDistanceFromPlane(pM, (*sit)->mvBoundaryPoints, out);
                    if (dis < ldTh) {
                        ldTh = dis;

                        pF.mvpNotSeenMapPlanes[i] = static_cast<MapPlane *>(nullptr);
                        pF.mvpNotSeenMapPlanes[i] = (*sit);
                    }
                }
            }

            if(ldTh == mfDisTh){ // associate in not seen planes
                for(auto it = mspNotSeenMapPlanes.begin(),iend = mspNotSeenMapPlanes.end(); it!=iend; ++it){
                    cv::Mat pW = (*it)->GetWorldPos();

                    float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                                  pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                                  pM.at<float>(2, 0) * pW.at<float>(2, 0);

                    if(out)
                        cout  << ":  angle : " << angle << endl;

                    if ((angle > mfAngleTh || angle < -mfAngleTh)) // associate plane
                    {

                        double dis = PointDistanceFromPlane(pM, (*it)->mvBoundaryPoints, out);
                        if(dis < ldTh) {
                            ldTh = dis;

                            pF.mvpNotSeenMapPlanes[i] = static_cast<MapPlane*>(nullptr);
                            pF.mvpNotSeenMapPlanes[i] = (*it);
                        }
                    }
                }
            }
        }

        for(auto p : pF.mvpMapPlanes){
            if(p== nullptr)
                pF.mbNewPlane = true;
        }
        float ratio = (num_total > 0) ? float(num_associated) / float(num_total) : 0.0;
        float average_dis = (num_total > 0) ? sum_dis / float(num_total) : 100;

//        cout << "association ratio: " << ratio << endl;
//        cout << "average distance: " << average_dis << endl;
    }


    double Map::PointDistanceFromPlane(const cv::Mat &plane, const PointCloud &boundry, bool out) {
    double res = 100;

    for(auto &p : boundry){
        double dis = abs(plane.at<float>(0, 0) * p[0] +
                   plane.at<float>(1, 0) * p[1] +
                   plane.at<float>(2, 0) * p[2] +
                   plane.at<float>(3, 0));
        if(dis < res)
            res = dis;
    }

    return res;
}

    void Map::SearchMatchedPlanes(ORB_SLAM2::KeyFrame *pKF, cv::Mat Scw, const vector<ORB_SLAM2::MapPlane *> &vpPlanes,
                                  vector<ORB_SLAM2::MapPlane *> &vpMatched, vector<ORB_SLAM2::MapPlane *> &vpMatchedPar,
                                  vector<ORB_SLAM2::MapPlane *> &vpMatchedVer, bool out) {

        unique_lock<mutex> lock(mMutexMap);

        cv::Mat ScwT;
        cv::transpose(Scw, ScwT);

        vpMatched = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));
        vpMatchedPar = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));
        vpMatchedVer = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));

        for (int i = 0; i < pKF->mnPlaneNum; ++i) {

            cv::Mat pM = ScwT * pKF->mvPlaneCoefficients[i];

            float ldTh = mfDisTh;
            float lverTh = mfVerTh;
            float lparTh = mfParTh;
            for (int j = 0;j < vpPlanes.size(); ++j) {
                cv::Mat pW = vpPlanes[j]->GetWorldPos();

                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);

                if ((angle > mfAngleTh || angle < -mfAngleTh)) // associate plane
                {

                    double dis = PointDistanceFromPlane(pM, vpPlanes[j]->mvBoundaryPoints, out);
                    if(dis < ldTh) {
                        ldTh = dis;
                        vpMatched[i] = static_cast<MapPlane*>(nullptr);
                        vpMatched[i] = vpPlanes[j];
                        continue;
                    }
                }

                // vertical planes
                if (angle < lverTh && angle > -lverTh) {
                    lverTh = abs(angle);
                    vpMatchedVer[i] = static_cast<MapPlane*>(nullptr);
                    vpMatchedVer[i] = vpPlanes[j];
                    continue;
                }

                //parallel planes
                if ((angle > lparTh || angle < -lparTh)) {
                    lparTh = abs(angle);
                    vpMatchedPar[i] = static_cast<MapPlane*>(nullptr);
                    vpMatchedPar[i] = vpPlanes[j];
                }
            }

        }
}

} //namespace ORB_SLAM










