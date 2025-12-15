/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;
#ifndef ANDROID_ON_
#include <ros/console.h>
#include <ros/assert.h>
#endif
#include "parameters.h"
#include "../utility/tic_toc.h"
#include <set>
#include <map>
#include "featureTracker/feature_tracker.h"
namespace ORB_SLAM3
{
class MapPoint;
};

class FeaturePerFrame
{
  public:
    // FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    // {
    //     point.x() = _point(0);
    //     point.y() = _point(1);
    //     point.z() = _point(2);
    //     uv.x() = _point(3);
    //     uv.y() = _point(4);
    //     velocity.x() = _point(5); 
    //     velocity.y() = _point(6); 
    //     cur_td = td;
    //     is_stereo = false;
    // }
    // void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    // {
    //     pointRight.x() = _point(0);
    //     pointRight.y() = _point(1);
    //     pointRight.z() = _point(2);
    //     uvRight.x() = _point(3);
    //     uvRight.y() = _point(4);
    //     velocityRight.x() = _point(5); 
    //     velocityRight.y() = _point(6); 
    //     is_stereo = true;
    // }
    double cur_td;
    // Vector3d point, pointRight;
    // Vector2d uv, uvRight;
    // Vector2d velocity, velocityRight;
    // bool is_stereo;
    int main_cam = -1;                  //标记本帧以哪个相机为准
    Vector3d point[NUM_CAM];           //单帧多目观测，用数组组织
    Vector2d uv[NUM_CAM];
    Vector2d velocity[NUM_CAM];
    bool is_observed[NUM_CAM] = {false};//标记本帧观测到哪些相机了
    bool is_stereoX(){ int obsNum = 0; for(bool& obs: is_observed){if(obs) obsNum++; }; return obsNum >= 2; }

    FeaturePerFrame(int _main_cam, const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        assert(0 <= _main_cam && _main_cam < NUM_CAM);
        main_cam = _main_cam;
        point[main_cam].x() = _point(0);
        point[main_cam].y() = _point(1);
        point[main_cam].z() = _point(2);
        uv[main_cam].x() = _point(3);
        uv[main_cam].y() = _point(4);
        velocity[main_cam].x() = _point(5); 
        velocity[main_cam].y() = _point(6); 
        cur_td = td;
        for (int cid = 0; cid < NUM_CAM; cid++)
        {
            is_observed[cid] = false;
        }
        is_observed[main_cam] = true;
    }

    void otherObservation(int other_id, const Eigen::Matrix<double, 7, 1> &_point)
    {
        assert(0 <= other_id && other_id < NUM_CAM);      
        point[other_id].x() = _point(0);
        point[other_id].y() = _point(1);
        point[other_id].z() = _point(2);
        uv[other_id].x() = _point(3);
        uv[other_id].y() = _point(4);
        velocity[other_id].x() = _point(5); 
        velocity[other_id].y() = _point(6);
        is_observed[other_id] = true;
    }
};

class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;//fix,1,默认最新帧一定观测到了地图MP点.
                                              //fix,2,默认从起始帧往后都是连续观测到地图MP点.
    int used_num;
    double estimated_depth;//这里默认左目2d点对应的深度
                           //fix, 多目情况下要增加其它目深度.
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
                    // 滑窗优化后求解出的深度是负数,地图点MP的solve_flag会被置为2
    bool track_keep;        //标记是否被最新帧追踪到
    int flag_opti_frame_id; //标记正参与哪一帧的优化
    int create_frame_id;    //标记是哪个帧创建的
    FeaturePerId(int _feature_id, int _start_frame, int _create_frame_id = -1)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0), track_keep(true), flag_opti_frame_id(-1), create_frame_id(_create_frame_id)
    {
    }

    int endFrame();
};

class FeatureFuseInfo{
  public:
    const int feature_id1;
    const int feature_id2;
};

//orb相关地图点
class FeaturePerFrameOrb{
  public:

    int main_cam = -1;        //标记本帧以哪个相机为准
    Vector3d point;           //单帧多目观测，用数组组织
    Vector2d uv;

    // FeaturePerFrameOrb(int _main_cam, const Eigen::Matrix<double, 7, 1> &_point)
    // 需要check一下是否是这样的
    FeaturePerFrameOrb(int _main_cam, const cv::Point2f& _point, const cv::Point2f& _un_point)
    {
        assert(0 <= _main_cam && _main_cam < NUM_CAM);
        main_cam = _main_cam;
        point.x() = _un_point.x;
        point.y() = _un_point.y;
        point.z() = 1.0;
        uv.x() = _point.x;
        uv.y() = _point.y;
    }
};

// 自定义比较器：仅比较pair.first
struct CompareByFrameIdx {
    bool operator()(const std::pair<int, std::vector<FeaturePerFrameOrb>>& a, const std::pair<int, std::vector<FeaturePerFrameOrb>>& b) const {
        return a.first < b.first;
    }
};
class FeaturePerIdOrb
{
  public:
    const ORB_SLAM3::MapPoint* orbMPptr = nullptr;
    int start_frame;
    std::set<std::pair<int, std::vector<FeaturePerFrameOrb>>, CompareByFrameIdx> obs;//按照偏离帧idx自动排序
    // vector<FeaturePerFrame> feature_per_frame;//fix,1,默认最新帧一定观测到了地图MP点.
    //                                           //fix,2,默认从起始帧往后都是连续观测到地图MP点.
    // int used_num;
    // double estimated_depth;//这里默认左目2d点对应的深度
    //                        //fix, 多目情况下要增加其它目深度.
    // int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
    //                 // 滑窗优化后求解出的深度是负数,地图点MP的solve_flag会被置为2
    bool track_keep;        //标记是否被最新帧追踪到
    int flag_opti_frame_id; //标记正参与哪一帧的优化
    int create_frame_id;    //标记是哪个帧创建的
    FeaturePerIdOrb(ORB_SLAM3::MapPoint* _orbMPptr, int _start_frame, int _create_frame_id = -1)
        : orbMPptr(_orbMPptr), start_frame(_start_frame),
          // used_num(0), estimated_depth(-1.0), solve_flag(0), 
          track_keep(true), flag_opti_frame_id(-1), create_frame_id(_create_frame_id)
    {
    }

    void PrintObsSimple(){
      if(obs.empty()) return;
      std::cout << "FeaturePerIdOrb obsSimple, mnId=," << orbMPptr << ",start_frame=," << start_frame << ",start=," << obs.begin()->first << ",end=," << obs.rbegin()->first << ",obs=,";
      for (auto itObs = obs.begin(); itObs != obs.end();)
      {
        std::cout << itObs->first << ",";
        itObs++;
      }
      // std::cout << std::endl;
    }
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);
    void clearState();
    int getFeatureCount();
    // bool addFeatureCheckParallax(int cur_frame_id, int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    // bool addFeatureCheckParallax(int cur_frame_id, int frame_count, const std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &image, double td);
    bool addFeatureCheckParallax(int cur_frame_id, int frame_count, const FeatureTracker::TrackInfoComplex &image, double td);
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth();
    VectorXd getDepthVector();
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], bool triangleAll = false);
    void triangulate2(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], bool withScale = false);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    // void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBackShiftDepth(const std::vector<Eigen::Matrix3d>& marg_R, const std::vector<Eigen::Vector3d>& marg_P, const std::vector<Eigen::Matrix3d>& new_R, const std::vector<Eigen::Vector3d>& new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier(set<int> &outlierIndex);
    void removeOutlier(set<ORB_SLAM3::MapPoint*> &outlierIndex);
    list<FeaturePerId> feature;                                       //这个list结构需要优化,不然查找时太耗时了!
    list<std::pair<FeatureFuseInfo,FeaturePerId>> featureTryFuse;     //临时融合点
    std::map<ORB_SLAM3::MapPoint*, FeaturePerIdOrb> featureOrb;
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[2];
};

#endif