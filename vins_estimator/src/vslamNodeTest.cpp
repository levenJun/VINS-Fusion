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

// #include <System.h>
#include "dataset_io/HeadDataReader.h"

ros::Publisher pub_camRawImg0, pub_camRawImg1;

std::shared_ptr<DATA_READER::HeadDataReader> mpHeadDataReader = nullptr;

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

// extract images with same timestamp from two topics
void sync_process()
{
    if(!mpHeadDataReader || mpHeadDataReader->getImgSize(0) <= 0){
        cerr << "ERROR: Failed to load images or IMU " << endl;
        return ;
    }

    std::shared_ptr<LevenBF::Utils::UtilsKeybordManager> mKeybordManager = std::make_shared<LevenBF::Utils::UtilsKeybordManager>();
    mKeybordManager->start();

    cv::Mat imLeft, imRight;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    while(mpHeadDataReader->isRun() || !mpHeadDataReader->isDataBufferEmpty())
    {
        DATA_READER::Data data_head = mpHeadDataReader->read();
        if (data_head.videoFrame.stamp < 1)
        {
            usleep(1);
            continue;
        }

        int imuSize = data_head.headImuPredict.imus.size();       
        for(int indexImu = 0; indexImu < imuSize; indexImu++){
            DATA_READER::IMUFrame& curImu = data_head.headImuPredict.imus[indexImu];
            estimator.inputIMU(curImu.stamp, curImu.acc, curImu.gro);
        }

        if(STEREO)
        {
            // cv::Mat image0, image1;
            // std_msgs::Header header;
            // double time = 0;
            // m_buf.lock();

            // Read image from file
            double tframe = data_head.videoFrame.stamp;
            imLeft = data_head.videoFrame.im[0];
            imRight = data_head.videoFrame.im[1];
            if(imLeft.empty() || imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  data_head.videoFrame.stamp << endl;
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

            // m_buf.unlock();
            if(!imLeft.empty() && !imRight.empty())
                estimator.inputImage(tframe, imLeft, imRight);
        }
        else
        {
            // Read image from file
            double tframe = data_head.videoFrame.stamp;
            imLeft = data_head.videoFrame.im[0];
            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  data_head.videoFrame.stamp << endl;
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

            // m_buf.unlock();
            if(!imLeft.empty())
                estimator.inputImage(tframe, imLeft);
        }
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

    if(mKeybordManager){
        mKeybordManager->stop();
    }
    mKeybordManager = nullptr;

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

    int startIndex = 0;
    int endIndex = 90000;
    if(argc >= 4){
        startIndex = std::atoi(argv[3]);
    }
    cout << "startIndex = " << startIndex << endl;

    mpHeadDataReader = std::make_shared<DATA_READER::HeadDataReader>(dataDir + "/head", 4, 1, startIndex, endIndex);
    
    int tot_images = mpHeadDataReader->getImgSize(0);
    if((tot_images<=0))
    {
        cerr << "ERROR: Failed to load images or IMU " << endl;
        return 1;
    }

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
