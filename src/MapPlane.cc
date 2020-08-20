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

        auto T = pRefKF->GetPose();
        mvViewPoints.emplace_back(T.at<float>(0, 3), T.at<float>(1, 3), T.at<float>(2, 3));
        auto T1 = T.inv();
        if (s) {
            mvIsImageBoundary.push_back(is_image_border);
            cntBoundaryUpdateSizes.push_back(pRefKF->mvBoundaryPoints[idx].size());
            Transformation(pRefKF->mvBoundaryPoints[idx], mvBoundaryPoints, T1);
            AddObservation(pRefKF, idx);
        } else {
            mvIsImageBoundary.push_back(is_image_border);
            cntBoundaryUpdateSizes.push_back(pRefKF->mvNotSeenBoundaryPoints[idx].size());
            Transformation(pRefKF->mvNotSeenBoundaryPoints[idx], mvBoundaryPoints, T1);
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
        // a*x + b*x + c*x + d = 0
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    void MapPlane::UpdateBoundary(const ORB_SLAM2::Frame &pF, int id) {
        unique_lock<mutex> lock(mMutexGridMap);
        mvViewPoints.emplace_back(pF.mTcw.at<float>(0, 3), pF.mTcw.at<float>(1, 3), pF.mTcw.at<float>(2, 3));
        cntBoundaryUpdateSizes.push_back(pF.mvBoundaryPoints[id].size());
        mvIsImageBoundary.push_back(pF.mvIsImageBoundary);
        Transformation(pF.mvBoundaryPoints[id], mvBoundaryPoints, pF.mTcw.inv());
//        projectMapPointsOnGridMap(pF);
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
    float center_coord = 1.3;
    float ratio_obstacle = 0.95;
    int to_gridmap_index(float x){
        return int(70*(x + center_coord));
    }

    bool is_obstacle(cv::Point2i &pt1, cv::Point2i &pt2, int x_view, int y_view){
        auto x1 = float(pt1.x - x_view);
        auto x2 = float(pt2.x - x_view);
        auto y1 = float(pt1.y - y_view);
        auto y2 = float(pt2.y - y_view);

        auto x3 = float((x1 + x2) / 2.0);
        auto y3 = float((y1 + y2) / 2.0);
        float norm3 = sqrt(x3 * x3 + y3 * y3);
        x3 /= norm3;
        y3 /= norm3;

        float scalar1 = abs(x1 * x3 + y1 * y3);
        float scalar2 = abs(x2 * x3 + y2 * y3);

//        return !(norm1 * ratio_obstacle > norm2 || norm2 * ratio_obstacle > norm1);

        return abs(scalar1 - scalar2) / (scalar1 + scalar2) < 0.01;
    }

    void MapPlane::polygonToGrid(KeyFrame *pF){
        unique_lock<mutex> lock(mMutexGridMap);
        unique_lock<mutex> lock2(mpMap->mMutexGridmapping);

        auto coef = GetWorldPos();
        auto A1 = projector_matrix(coef, i0, i1);

        int ix, iy;
        for(const auto pMp: pF->GetMapPoints())
            if(pMp!=NULL){
                auto mpt = pMp->GetWorldPos();
                auto a = cv::Mat(mpt);
                cv::Mat b = A1 * a;
                float height = b.at<float>(0);
                if (height < -0.1){
                    int x = to_gridmap_index(b.at<float>(1));
                    int y = to_gridmap_index(b.at<float>(2));
                    if (0<=y<gridmap.cols && 0<=x<gridmap.rows)
                        temp.at<float>(y, x) = temp.at<float>(y, x) * (1.0f - alpha) - alpha;
//                    for(ix=x-1; (ix<=x+1) ;ix++)
//                        for(iy=y-1; (iy<=y+1) ;iy++)
//                            if (0<=iy<gridmap.cols && 0<=ix<gridmap.rows)
//                                temp.at<float>(iy, ix) = temp.at<float>(iy, ix) * (1.0f - alpha) - alpha;
                }
            }

        int j, cnt, j2;
        int update_size = cntBoundaryUpdateSizes.size();
        for(int i=previous_update_size_index; i < update_size; i++){
            auto a_view = cv::Mat(mvViewPoints[i]);
            cv::Mat b_view = A1 * a_view;
//            int x_view = to_gridmap_index(b_view.at<float>(1));
//            int y_view = to_gridmap_index(b_view.at<float>(2));

            cnt = cntBoundaryUpdateSizes[i];

            std::vector<cv::Point> polygon;
            cv::Point pt_prev;
            j2 = 0;

            for(j=previous_cnt; j<cnt+previous_cnt; j++){
                auto a = cv::Mat(mvBoundaryPoints[j]);
                cv::Mat b = A1 * a;
                int x = to_gridmap_index(b.at<float>(1));
                int y = to_gridmap_index(b.at<float>(2));
                if (0<=x<gridmap.cols && 0<=y<gridmap.rows){
                    cv::Point pt(x, y);
                    polygon.push_back(pt);
                    pt_prev = pt;
                }
                j2 += 1;
            }
            std::vector<std::vector<cv::Point>> polygons;
            polygons.push_back(polygon);
            cv::drawContours(temp, polygons, -1, 1, -1);

            mask = temp != 0;
            temp = gridmap * (1-alpha) + temp * alpha;
            temp.copyTo(gridmap, mask);
            temp *= 0.0;
            previous_cnt = j;
            previous_update_size_index = update_size;
        }
//        mvBoundaryPoints.clear();
//        cntBoundaryUpdateSizes.clear();
    }

    void MapPlane::projectMapPointsOnGridMap(KeyFrame *pF){
        unique_lock<mutex> lock(mMutexGridMap);
        unique_lock<mutex> lock2(mpMap->mMutexGridmapping);
        auto coef = GetWorldPos();
        auto A1 = projector_matrix(coef, i0, i1);
        int ix, iy;
        for(const auto pMp: pF->GetMapPoints())
            if(pMp!=NULL){
                auto mpt = pMp->GetWorldPos();
                auto a = cv::Mat(mpt);
                cv::Mat b = A1 * a;
                float height = - b.at<float>(0);
                if (height > 0.2){
                    int x = to_gridmap_index(b.at<float>(1));
                    int y = to_gridmap_index(b.at<float>(2));
//                    if (0<=y<=gridmap.cols && 0<=x<=gridmap.rows)
//                        gridmap.at<float>(x, y) = gridmap.at<float>(x, y) * (1 - alpha) - alpha;
                    for(ix=x-1; (ix<=x+1) ;ix++)
                        for(iy=y-1; (iy<=y+1) ;iy++)
                            if (0<=iy<=gridmap.cols && 0<=ix<=gridmap.rows)
                                gridmap.at<float>(ix, iy) = gridmap.at<float>(ix, iy) * (1 - alpha) - alpha;
                }
            }
    }

    cv::Mat MapPlane::GetGridMap(){
        unique_lock<mutex> lock(mMutexGridMap);
        return gridmap.clone();
    }

}