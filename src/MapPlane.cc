//
// Created by fishmarch on 19-5-24.
//


#include "MapPlane.h"

#include <mutex>
#include <time.h>

namespace ORB_SLAM2{

    typedef cv::Vec3f PointT;
    typedef std::vector<PointT> PointCloud;

    void Transformation(const PointCloud &cloud_in, PointCloud &cloud_out, const cv::Mat4f &T){
        cv::Mat vec1 = cv::Mat::ones(4, 1, CV_32F);
        cv::Mat vec2;

        for (auto &pt: cloud_in){
            vec1.at<float>(0, 0) = pt[0];
            vec1.at<float>(1, 0) = pt[1];
            vec1.at<float>(2, 0) = pt[2];
            vec2 = T * vec1;
            float v = vec2.at<float>(3, 0);
            cloud_out.push_back(PointT(vec2.at<float>(0, 0) / v, vec2.at<float>(1, 0) / v, vec2.at<float>(2, 0) / v));
        }
    }

    long unsigned int MapPlane::nLastId = 0;
    mutex MapPlane::mGlobalMutex;

    MapPlane::MapPlane(const cv::Mat &Pos, ORB_SLAM2::KeyFrame *pRefKF, int idx, Map* pMap, bool s):
    mnBALocalForKF(0), mpMap(pMap), mbSeen(s), mpRefKF(pRefKF), mbBad(false) {
        Pos.copyTo(mWorldPos);
        mnId = nLastId++;
        if(mnId == 1)
            srand(time(0));

        mRed = rand() % 255;
        mBlue = rand() % 255;
        mGreen = rand() % 255;

//        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(pRefKF->GetPose());
//        auto T1 = T.inverse().matrix();
        auto T = pRefKF->GetPose().inv();
        if (s) {
            cntBoundaryUpdateSizes.push_back(pRefKF->mvBoundaryPoints[idx].size());
            Transformation(pRefKF->mvBoundaryPoints[idx], mvBoundaryPoints, T);
            AddObservation(pRefKF, idx);
        } else {
            cntBoundaryUpdateSizes.push_back(pRefKF->mvNotSeenBoundaryPoints[idx].size());
            Transformation(pRefKF->mvNotSeenBoundaryPoints[idx], mvBoundaryPoints, T);
            AddNotSeenObservation(pRefKF, idx);
        }

    }

    void MapPlane::AddObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;
    }

    void MapPlane::AddNotSeenObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mNotSeenObservations.count(pKF))
            return;
        mNotSeenObservations[pKF] = idx;
    }

    void MapPlane::AddVerObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mVerObservations.count(pKF))
            return;
        mVerObservations[pKF] = idx;
    }

    void MapPlane::AddParObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mParObservations.count(pKF))
            return;
        mParObservations[pKF] = idx;
    }

    void MapPlane::EraseObservation(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF)){
            mObservations.erase(pKF);
        }
    }

    void MapPlane::EraseParObservation(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mParObservations.count(pKF)){
            mParObservations.erase(pKF);
        }
    }

    void MapPlane::EraseVerObservation(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mVerObservations.count(pKF)){
            mVerObservations.erase(pKF);
        }
    }

    void MapPlane::EraseNotSeenObservation(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mNotSeenObservations.count(pKF)){
            mNotSeenObservations.erase(pKF);
        }
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetNotSeenObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mNotSeenObservations;
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetVerObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mVerObservations;
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetParObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mParObservations;
    }

    int MapPlane::GetIndexInKeyFrame(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    int MapPlane::GetNotSeenIndexInKeyFrame(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mNotSeenObservations.count(pKF))
            return mNotSeenObservations[pKF];
        else
            return -1;
    }

    void MapPlane::SetWorldPos(const cv::Mat &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
//        float s = cv::sum(Pos)[0];
//        if(s!=s) throw;
        Pos.copyTo(mWorldPos);
    }

    cv::Mat MapPlane::GetWorldPos(){
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    void MapPlane::UpdateBoundary(const ORB_SLAM2::Frame &pF, int id) {
        cntBoundaryUpdateSizes.push_back(pF.mvBoundaryPoints[id].size());
        Transformation(pF.mvBoundaryPoints[id], mvBoundaryPoints, pF.mTcw.inv());
//        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( pF.mTcw );
//        pcl::transformPointCloud( pF.mvBoundaryPoints[id], *mvBoundaryPoints, T.inverse().matrix());
    }

    bool MapPlane::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    void MapPlane::Replace(MapPlane *pMP) {
        if(pMP->mnId==this->mnId)
            return;

        std::map<KeyFrame*, int> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs=mObservations;
            mObservations.clear();
            mbBad=true;
            mpReplaced = pMP;
        }

        for(map<KeyFrame*,int>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
        {
            // Replace measurement in keyframe
            KeyFrame* pKF = mit->first;

            if(!pMP->IsInKeyFrame(pKF))
            {
                pKF->ReplaceMapPlaneMatch(mit->second, pMP);
                pMP->AddObservation(pKF,mit->second);
            }
            else
            {
                pKF->EraseMapPlaneMatch(mit->second);
            }
        }

        mpMap->EraseMapPlane(this);
    }

    bool MapPlane::isBad()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }

    KeyFrame* MapPlane::GetReferenceKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }
}