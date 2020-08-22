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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "MapPlane.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
//#include <pcl/common/transforms.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
#include <random>

namespace ORB_SLAM2
{
//    typedef pcl::PointXYZRGB PointT;
//    typedef pcl::PointCloud<PointT> PointCloud;

//    typedef cv::Vec3f PointT;
//    typedef std::vector<PointT> PointCloud;

MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}



typedef std::vector<cv::Vec3f> Points;

//    for i in range(len(poly)-1):
//      a, b = poly[i], poly[i + 1]
//      v = (b-a) / n
//      sides.append(a)
//      for l in range(0, n):
//          a0 = a + l * v
//          v1 = (cent - a0) / n
//          sides.append(a0)
//          for l1 in range(1, n):
//              sides.append(a0 + l1 * v1)

Points make_grid(const Points &polygon){
    Points points;
    int n = 10;
    int num_nodes = polygon.size();
    cv::Vec3f cent(0.0f, 0.0f, 0.0f);
    for (auto &pt: polygon){
        cent[0] += pt[0];
        cent[1] += pt[1];
        cent[2] += pt[2];
    }
    cent /= (float) num_nodes;

    for(int i =0; i < num_nodes-1 ; i ++){
        cv::Vec3f a = polygon[i];
        cv::Vec3f v = (polygon[i+1] - a) / (float) n;
        points.push_back(a);
        for (float l=0; l < n; l++){
            cv::Vec3f a0 = a + l * v;
            cv::Vec3f v1 = (cent - a0) / (float) n;
            points.push_back(a0);
            for (float l1=0; l1 < n; l1++){
                points.push_back(a0 + l1 * v1);
            }
        }
    }
    return points;
}


    void Transformation1(const Points &cloud_in, Points &cloud_out, const cv::Mat4f &T){
        cv::Mat vec1 = cv::Mat::ones(4, 1, CV_32F);
        cv::Mat vec2;

        for (auto &pt: cloud_in){
            vec1.at<float>(0, 0) = pt[0];
            vec1.at<float>(1, 0) = pt[1];
            vec1.at<float>(2, 0) = pt[2];
            vec2 = T * vec1;
            float v = vec2.at<float>(3, 0);
            cloud_out.push_back(cv::Vec3f(vec2.at<float>(0, 0) / v, vec2.at<float>(1, 0) / v, vec2.at<float>(2, 0) / v));
        }
    }

void MapDrawer::DrawMapPlanes(bool bAssumed) {

    const vector<MapPlane*> &vpMPs = mpMap->GetAllMapPlanes();
    if(vpMPs.empty())
        return;
    glPointSize(mPointSize/2);
    glBegin(GL_POINTS);

    for(auto pMP : vpMPs){
        map<KeyFrame*, int> observations = pMP->GetObservations();
        float ir = pMP->mRed;
        float ig = pMP->mGreen;
        float ib = pMP->mBlue;
        float norm = sqrt(ir*ir + ig*ig + ib*ib);
        glColor3f(ir/norm, ig/norm, ib/norm);
        for(auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++){
            KeyFrame* frame = mit->first;
            int id = mit->second;
            if(!bAssumed && id >= frame->mnRealPlaneNum){
                continue;
            }

            Points points_global;
            Transformation1(make_grid(frame->mvBoundaryPoints[id]), points_global, frame->GetPose().inv());

            for(auto& p : points_global){
                if(bAssumed ){
                    glColor3f(1, 0, 0);
                }else{
                    glColor3f(ir/norm, ig/norm, ib/norm);
                }
                glVertex3f(p[0], p[1], p[2]);
            }
        }
    }
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();


    for(auto &line : mpMap->GetAllMapLines()){
        auto poz = line->GetWorldPos();
        glColor3f(0.0f,0.1f,0.0f);
        glBegin(GL_LINES);
        float v1 = poz.at<float>(0, 3);
        float v2 = poz.at<float>(1, 3);
        glVertex3f(poz.at<float>(0, 0) / v1,poz.at<float>(0, 1) / v1,poz.at<float>(0, 2) / v1);
        glVertex3f(poz.at<float>(1, 0) / v2,poz.at<float>(1, 1) / v2,poz.at<float>(1, 2) / v2);
//        cout << poz.at<float>(0, 3) << " " << poz.at<float>(1, 3) << endl;
        glEnd();
    }

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
