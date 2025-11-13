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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "utility/UtilsKeybordManager.hpp"
#include <atomic>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>
#include "Stereo-Inertial3/DataReader.hpp"
#include "HelperDataSaver.hpp"

// #include <System.h>

ros::Publisher pub_camRawImg0, pub_camRawImg1;

int startIndex = 0;
int endIndex = 90000;
std::shared_ptr<DataReader> mpHeadDataReader = nullptr;
std::shared_ptr<MyHelpers::HelperDataSaver> mHelperDataSaver = nullptr;
Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

std::string EigenVector3dToStr(const Eigen::Vector3d& v3d){
    return std::to_string(v3d(0)) + "," + std::to_string(v3d(1)) + "," + std::to_string(v3d(2));
};

// extract images with same timestamp from two topics
void sync_process()
{
    std::cout << "try start sync_process." << std::endl;
    std::vector<std::vector<float>> cameraInfo = mpHeadDataReader->ReadCameraInfo();
    // if(!mpHeadDataReader || mpHeadDataReader->getImgSize(0) <= 0){
    //     cerr << "ERROR: Failed to load images or IMU " << endl;
    //     return ;
    // }
    mHelperDataSaver = std::shared_ptr<MyHelpers::HelperDataSaver>(new MyHelpers::HelperDataSaver());
    mHelperDataSaver->setRootDirectory("./output/p1/");
    mHelperDataSaver->startSaving();

    std::shared_ptr<LevenBF::Utils::UtilsKeybordManager> mKeybordManager = std::make_shared<LevenBF::Utils::UtilsKeybordManager>();
    mKeybordManager->start();

    cv::Mat imLeft, imRight;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    mpHeadDataReader->ResetStereoIdx();
    PosedStereoFrame data_cam0 =  mpHeadDataReader->ReadStereoImage();
    std::vector<ImuFrame> imusRaw;
    int dIdx = -1;
    while (!data_cam0.im.empty() && data_cam0.status != -1)
    {
        dIdx++;
        if(dIdx < startIndex){
            data_cam0 = mpHeadDataReader->ReadStereoImage();
            continue;
        }
        if(dIdx > endIndex){
            std::cout << "read offline done,endIndex=," << endIndex << ",dIdx=," << dIdx << std::endl;
            break;
        }

        // std::cout << "dIdx=" << dIdx << std::endl;
        if (data_cam0.stamp < 1)
        {
            usleep(1);
            continue;
        }
        if(data_cam0.status == -2){
            std::cout << "warn, data_cam0.status=," << data_cam0.status << std::endl;
            usleep(1);
            continue;                
        }
        std::cout << "dIdx=" << dIdx << ",endIndex=," << endIndex << std::endl;
        if(dIdx % 2 != 0){
            // continue;
        }
        imusRaw.clear();
        if(mpHeadDataReader->ReadImus(data_cam0.stamp, imusRaw)){
            std::cout << "imusRaw size=," << imusRaw.size() << std::endl;
            for (auto& imuOneRaw : imusRaw)
            {
                estimator.inputIMU(imuOneRaw.mStamp, imuOneRaw.vAcc.cast<double>(), imuOneRaw.vGyro.cast<double>());
            }
        }
        double timeElapsedms = -1;
        if(STEREO)
        {
            // cv::Mat image0, image1;
            // std_msgs::Header header;
            // double time = 0;
            // m_buf.lock();

            // Read image from file
            double tframe = data_cam0.stamp;
            imLeft = data_cam0.im[0];
            imRight = data_cam0.im[1];
            if(imLeft.empty() || imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  tframe << endl;
                continue;
            }        

            if(false){
                // clahe
                clahe->apply(imLeft,imLeft);
                clahe->apply(imRight,imRight);
            }else{
                //另外的方式作图像增强
                cv::Scalar meanLeft =cv::mean(imLeft);
                cv::Scalar meanRight =cv::mean(imRight);

                imLeft=(imLeft-meanLeft[0]*0.1);
                meanLeft=cv::mean(imLeft);
                float scaleLeft=128.0/meanLeft[0];
                imLeft=(imLeft)*scaleLeft;


                imRight=(imRight-meanRight[0]*0.1);
                meanRight=cv::mean(imRight);
                float scaleRight=128.0/meanRight[0];
                imRight=(imRight)*scaleRight;
            }

            {
                //将图片发布出去
                sensor_msgs::ImagePtr msgImgCam0 = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
                msgImgCam0->header.stamp.fromSec(tframe);
                // // msgImgCam0->header.frame_id = "cam0";
                // msgImgCam0->height = 480;
                // msgImgCam0->width = 640;
                // msgImgCam0->step = 640;

                // {
                //     cout << "print img_msg info:" << endl
                //             << "encoding:" << msgImgCam0->encoding << endl
                //             << "header:" << msgImgCam0->header << endl
                //             << "height:" << msgImgCam0->height << endl
                //             << "width:" << msgImgCam0->width << endl
                //             << "is_bigendian:" << msgImgCam0->is_bigendian << endl
                //             << "step:" << msgImgCam0->step << endl;
                // }

                pub_camRawImg0.publish(msgImgCam0);
            }
            auto start = std::chrono::system_clock::now();
            // m_buf.unlock();
            if(!imLeft.empty() && !imRight.empty())
                estimator.inputImage(tframe, imLeft, imRight);
            auto end = std::chrono::system_clock::now();
            timeElapsedms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }
        else
        {
            // Read image from file
            double tframe = data_cam0.stamp;
            imLeft = data_cam0.im[0];
            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  tframe << endl;
                continue;
            }
            if(false){
                // clahe
                clahe->apply(imLeft,imLeft);
            }else{
                //另外的方式作图像增强
                cv::Scalar meanLeft =cv::mean(imLeft);

                imLeft=(imLeft-meanLeft[0]*0.1);
                meanLeft=cv::mean(imLeft);
                float scaleLeft=128.0/meanLeft[0];
                imLeft=(imLeft)*scaleLeft;
            }

            auto start = std::chrono::system_clock::now();
            // m_buf.unlock();
            if(!imLeft.empty())
                estimator.inputImage(tframe, imLeft);
            auto end = std::chrono::system_clock::now();
            timeElapsedms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        }

        {
            bool bInited = false;
            bInited = estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR;
            int winMaxIdx = WINDOW_SIZE;
            if(!bInited){
                winMaxIdx = estimator.frame_count;
            }
            Eigen::Vector3d v3R = Eigen::Vector3d::Zero();
            Eigen::Vector3d v3t = Eigen::Vector3d::Zero();
            Eigen::Vector3d v3V = Eigen::Vector3d::Zero();
            Eigen::Vector3d v3Ba = Eigen::Vector3d::Zero();
            Eigen::Vector3d v3Bg = Eigen::Vector3d::Zero();
            
            std::vector<Eigen::Vector3d> v3Re0(NUM_OF_CAM);
            std::vector<Eigen::Vector3d> v3te0(NUM_OF_CAM);
            for (int cid = 0; cid < NUM_OF_CAM; cid++)
            {
                v3Re0[cid] = Eigen::Vector3d::Zero();
                v3te0[cid] = Eigen::Vector3d::Zero();
            }
            double td = 0;
            {
                Eigen::Quaterniond poseQ = Eigen::Quaterniond(estimator.Rs[winMaxIdx]);
                Sophus::SO3d poseSO3 = Sophus::SO3d(poseQ);
                v3R = poseSO3.log();
                v3t = estimator.Ps[winMaxIdx];
                v3V = estimator.Vs[winMaxIdx];
                v3Ba = estimator.Bas[winMaxIdx];
                v3Bg = estimator.Bgs[winMaxIdx];

                //计算配置文件的外參和实时外參的增量
                for (int cid = 0; cid < NUM_OF_CAM; cid++)
                {
                    Eigen::Quaterniond exRicOri = Eigen::Quaterniond(RIC[cid]);
                    Eigen::Vector3d exTicOri = TIC[cid];
                    Sophus::SE3d exSE3Ori = Sophus::SE3d(exRicOri, exTicOri);

                    Eigen::Quaterniond exRicEst = Eigen::Quaterniond(estimator.ric[cid]);
                    Eigen::Vector3d exTicEst = estimator.tic[cid];
                    Sophus::SE3d exSE3Est = Sophus::SE3d(exRicEst, exTicEst);

                    Sophus::SE3d exSE3Diff = exSE3Ori.inverse() * exSE3Est;

                    v3Re0[cid] = exSE3Diff.so3().log();
                    v3te0[cid] = exSE3Diff.translation();
                }
                td = estimator.td;                
            }
            std::cout << "TrackStereo done, timestamp=," << std::fixed << std::setprecision(6) << data_cam0.stamp
                        << ",mState=," << bInited
                        << ",costms=," << timeElapsedms*1.e-3
                        << ",v3R=," << Utility::EigenVector3dToStr(v3R) << "," << v3R.norm()
                        << ",v3t=," << Utility::EigenVector3dToStr(v3t) << "," << v3t.norm()
                        << ",v3V=," << Utility::EigenVector3dToStr(v3V) << "," << v3V.norm()
                        << ",v3Ba=," << Utility::EigenVector3dToStr(v3Ba) << "," << v3Ba.norm()
                        << ",v3Bg=," << Utility::EigenVector3dToStr(v3Bg) << "," << v3Bg.norm()
                        << ",td=," << td;

            for (int cid = 0; cid < NUM_OF_CAM; cid++)
            {
                std::cout << ",deltaExI=," << cid << ",v3Re0=," << Utility::EigenVector3dToStr(v3Re0[cid]) << "," << v3Re0[cid].norm()
                            << ",v3te0=," << Utility::EigenVector3dToStr(v3te0[cid]) << "," << v3te0[cid].norm();
            }
            std::cout << std::endl;
        }
        if(mHelperDataSaver){
            if(estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
                double timestamp = data_cam0.stamp;
                const Eigen::Vector3d poseT = estimator.Ps[WINDOW_SIZE];
                Eigen::Quaterniond poseQ;
                poseQ = Eigen::Quaterniond(estimator.Rs[WINDOW_SIZE]);

                mHelperDataSaver->savePoseData(timestamp, poseT, poseQ);
            }
        }

        data_cam0 = mpHeadDataReader->ReadStereoImage();        
        usleep(1000 * 30);
        // std::chrono::milliseconds dura(2);
        // std::this_thread::sleep_for(dura);

        if(mKeybordManager && mKeybordManager->isKeySpace()){
            int index = 0;
            while(mKeybordManager && !mKeybordManager->isKeySpace()){                
                usleep(1000 * 1500);
                if(index++ % 50 == 0){
                    std::cout << ("space pause!, index=" + std::to_string(index)) << std::endl;
                }
            }
        }
        
    }
    std::cout << "try stop sync_process. 1" << std::endl;    
    if(mHelperDataSaver) mHelperDataSaver->stopSaving();
    mHelperDataSaver = nullptr;    
    
    if(mKeybordManager){
        mKeybordManager->stop();
    }
    mKeybordManager = nullptr;

    for (int sid = 3; sid >= 1; sid--)
    {
        std::cout << "test finish.." << sid << std::endl;
        std::chrono::milliseconds dura(1000);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

//1)ros相关的初始化
//2)读取配置,并初始化系统
//3)注册回调监听
//4)读取离线数据
//5)单开一个线程来执行数据顺序输入并处理
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    if(argc < 3)
    {
        printf("please intput: rosrun vins vins_node [config file] [dataDir] [start index]\n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml ~/catkin_ws/data/xxx\n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

    string dataDir = string(argv[2]);
    cout << "dataDir = " << dataDir << endl;

    // int startIndex = 0;
    // int endIndex = 90000;
    if(argc >= 4){
        startIndex = std::atoi(argv[3]);
    }
    if(argc >= 5){
        endIndex = std::atoi(argv[4]);
    }
    cout << "startIndex = " << startIndex << ",endIndex = " << endIndex << endl;
    

    // mpHeadDataReader = std::make_shared<DATA_READER::HeadDataReader>(dataDir + "/head", 4, 1, startIndex, endIndex);
    mpHeadDataReader = std::make_shared<DataReader>(dataDir + "/");

    // int tot_images = mpHeadDataReader->getImgSize(0);
    // if((tot_images<=0))
    // {
    //     cerr << "ERROR: Failed to load images or IMU " << endl;
    //     return 1;
    // }

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    pub_camRawImg0 = n.advertise<sensor_msgs::Image>(IMAGE0_TOPIC, 10);
    pub_camRawImg1 = n.advertise<sensor_msgs::Image>(IMAGE1_TOPIC, 10);

    // ros::Subscriber sub_imu;
    // if(USE_IMU)
    // {
    //     sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    // }
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    // ros::Subscriber sub_img1;
    // if(STEREO)
    // {
    //     sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    // }
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
