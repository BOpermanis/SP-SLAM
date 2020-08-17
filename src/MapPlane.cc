//
// Created by fishmarch on 19-5-24.
//


#include "MapPlane.h"

#include <mutex>
#include <time.h>



namespace ORB_SLAM2{

    typedef cv::Vec3f PointT;
    typedef std::vector<PointT> PointCloud;

    PointT crossProduct(PointT &v_A, PointT &v_B) {
        PointT c_P;
        c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
        c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
        c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
        return c_P;
    }

    cv::Vec2f to_plane_coords(PointT pt, cv::Mat3f &A1){
        auto a = cv::Mat(pt);
        cv::Mat b = A1 * a;
        return cv::Vec2f(b.at<float>(1), b.at<float>(2));
    }

    cv::Mat3f projector_matrix(cv::Mat &coef, int i0, int i1){
        PointT plane_norm(coef.at<float>(0), coef.at<float>(1), coef.at<float>(2));

        if(i0 == -1){
            i0 = 0;
            if (abs(plane_norm[i0]) < abs(plane_norm[1])) i0 = 1;
            if (abs(plane_norm[i0]) < abs(plane_norm[2])) i0 = 2;
            i1 = 0;
            if(i0 == 0) i1 = 1;
        }

        if(plane_norm[i0] > 0) plane_norm *= -1;

        PointT e1(0.0, 0.0, 0.0);

        if (plane_norm[i0] > 0){
            e1[i1] = plane_norm[i0];
            e1[i0] = -plane_norm[i1];
        }else{
            e1[i1] = -plane_norm[i0];
            e1[i0] = plane_norm[i1];
        }

        e1 /= cv::sum(e1)[0];
        PointT e2 = crossProduct(plane_norm, e1);
        e2 /= cv::sum(e2)[0];

        cv::Mat A = cv::Mat::zeros(cv::Size(3, 3),CV_32F);
        A.at<cv::Vec3f>(0) = plane_norm;
        A.at<cv::Vec3f>(1) = e1;
        A.at<cv::Vec3f>(2) = e2;

        return A.t().inv();;
    }


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

    MapPlane::MapPlane(const cv::Mat &Pos, ORB_SLAM2::KeyFrame *pRefKF, int idx, Map* pMap, std::vector<bool> is_image_border, bool s):
    mnBALocalForKF(0), mpMap(pMap), mbSeen(s), mpRefKF(pRefKF), mbBad(false) {
        Pos.copyTo(mWorldPos);
        mnId = nLastId++;
        if(mnId == 1)
            srand(time(0));

        mRed = rand() % 255;
        mBlue = rand() % 255;
        mGreen = rand() % 255;
        gridmap = grid_map::GridMap( { "layer" });
        gridmap.setGeometry(grid_map::Length(3.0, 3.0), 0.1, grid_map::Position(5.0, 5.0));
        gridmap["layer"].setConstant(0.0);
//        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(pRefKF->GetPose());
//        auto T1 = T.inverse().matrix();
        auto T = pRefKF->GetPose().inv();
        if (s) {
            mvIsImageBoundary.push_back(is_image_border);
            cntBoundaryUpdateSizes.push_back(pRefKF->mvBoundaryPoints[idx].size());
            Transformation(pRefKF->mvBoundaryPoints[idx], mvBoundaryPoints, T);
            AddObservation(pRefKF, idx);
        } else {
            mvIsImageBoundary.push_back(is_image_border);
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
        mvIsImageBoundary.push_back(pF.mvIsImageBoundary);
        Transformation(pF.mvBoundaryPoints[id], mvBoundaryPoints, pF.mTcw.inv());
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

    void MapPlane::polygonToGrid(){
        unique_lock<mutex> lock(mMutexGridMap);
        unique_lock<mutex> lock2(mpMap->mMutexGridmapping);
        auto coef = GetWorldPos();

        auto A1 = projector_matrix(coef, i0, i1);

        int j, cnt;
        int update_size = cntBoundaryUpdateSizes.size();
        for(int i=previous_update_size_index; i < update_size; i++){
            cnt = cntBoundaryUpdateSizes[i];
            grid_map::Polygon polygon;

            for(j=previous_cnt; j<cnt+previous_cnt; j++){
                auto a = cv::Mat(mvBoundaryPoints[j]);
                cv::Mat b = A1 * a + 5.0;
                polygon.addVertex(grid_map::Position(b.at<float>(1), b.at<float>(2)));
            }

            for (grid_map::PolygonIterator iterator(gridmap, polygon); !iterator.isPastEnd(); ++iterator)
                gridmap.at("layer", *iterator) += 1;

            previous_cnt = j;
            previous_update_size_index = update_size;
        }
//        mvBoundaryPoints.clear();
//        cntBoundaryUpdateSizes.clear();
    }
    cv::Mat MapPlane::GetGridMap(){
        unique_lock<mutex> lock(mMutexGridMap);
        const float minValue = 0.0;
        const float maxValue = 2.0;
        cv::Mat image;
        grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridmap, "layer", CV_16UC1, minValue, maxValue, image);
        return image;
    }

}