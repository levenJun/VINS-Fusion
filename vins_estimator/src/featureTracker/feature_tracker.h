/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
    FeatureTracker();
    // map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>  trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    // void setMask();
    void setMask(int cid);
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF(int cid);
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    std::vector<int> removeOutliers(set<int> &removePtsIds);//返回内点数
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack;                                     //用于显示的图(debug)
    struct TrackInfoMono{
        cv::Mat mask;                                        //多目copy:单目追踪的mask
        cv::Mat fisheye_mask;
        cv::Mat prev_img, cur_img;                           //多目copy:单目追踪的前后帧图像
        vector<cv::Point2f> n_pts;                           //多目copy:单目追踪后补的点
        vector<cv::Point2f> predict_pts;                     //多目copy:单目追踪预测点
        vector<cv::Point2f> predict_pts_debug;               //多目copy:单目追踪预测点
        vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;//多目copy:单目追踪前后帧点+到其它目双目匹配点
        vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;//多目copy:单目特征点去畸变后结果
        vector<cv::Point2f> pts_velocity, right_pts_velocity;//多目copy:单目特征的速度(z1平面，真实时间速度)
        vector<int> ids, ids_right;                          //多目copy:最新帧左目特征id,右目匹配到的特征id
        vector<int> track_cnt;                               //多目copy:单目追踪纯 点次数
        map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;              //多目copy:最新帧[id,un点]记录, 上一帧[id,un点]记录
        map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;  //多目copy:最新帧右目[id,un点]记录, 上一帧右目[id,un点]记录
        map<int, cv::Point2f> prevLeftPtsMap;                               //多目copy:上一帧[id,ori点]记录
    };
    TrackInfoMono vTrackInfoMono[NUM_CAM];
    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    std::atomic<int> n_id{0};                           //全局唯一自增加id
    bool hasPrediction[NUM_CAM];
};
