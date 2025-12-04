/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#ifndef ANDROID_ON_
#include <ros/ros.h>
#else
    #define ROS_WARN std::cout << std::endl << " ros warn: " << 
    #define ROS_DEBUG std::cout << std::endl << " ros debug: " << 
    #define ROS_INFO std::cout << std::endl << " ros info: " << 
    #define ROS_BREAK() exit(-1)
    #define ROS_ASSERT assert
#endif
#define SHOW_TRACK_NAME "track_img"

#include <vector>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>
#include <atomic>  // 需包含原子操作头文件

using namespace std;

// const double FOCAL_LENGTH = 460.0;
const double FOCAL_LENGTH = 230.0;
const int WINDOW_SIZE = 20;
// const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR
const int NUM_CAM = 2;//相机数
extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;
extern std::vector<cv::Mat> ImgMask;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string OUTPUT_FOLDER;
extern std::string IMU_TOPIC;
extern double TD;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern int ROW, COL;
extern int NUM_OF_CAM;
extern int STEREO;
extern int USE_IMU;
extern int MULTIPLE_THREAD;
// pts_gt for debug purpose;
extern map<int, Eigen::Vector3d> pts_gt;

extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int BLOCK_NUM;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;

void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

struct MetricStatistic{
    double timeStamp = -1;
    double timeImageAll = 0;//estimator.inputImage总耗时
    double timeTrackAll = 0;
    double timeLKLeftOnce = 0;
    double timeLKLeftTwice = 0;
    double timeGFTTLeft = 0;
    double timeGFTTLeftTestOnce = 0;
    double timeLKRightTwice = 0;
    double timeImuAll = 0;
    double timeImgAll = 0;
    double timeImgAddFeature = 0;
    double timeImgOptiAll = 0;
    double timeImgOptiWin = 0;
    double timeImgOptiCeres = 0;
    double timeImgSlideiWin = 0;

    int fNumLkPreAll = 0;      //前后帧追踪总点
    int fNumLkPreLeft = 0;     //前后帧追踪左目点
    int fNumLkStereo = 0;      //双目追踪点
    int fNumOptWinAll = 0;     //滑窗优化的所有MP点
    int fNumOptWinLeft = 0;    //滑窗优化的所有以左目为主的点
    int fNumOptWinInlier = 0;  //滑窗优化后内点 
    int fNumOptWinInlierRight = 0;  //滑窗优化后内点 

    void Clear();
};

extern MetricStatistic mMetricStatistic;

extern std::atomic<int> g_frameId;
int getGlobalFrameId(bool selfPlus);