#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
// #include <ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
// #include "utility/visualization.h"
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

#ifdef ANDROID_ON_
#include <sched.h>
#include <unistd.h>
#include <sys/syscall.h>
// ==================== 绑定核心的辅助函数 ====================
// 输入: core_ids (例如 {7} 代表超大核，{4,5,6} 代表大核)
void BindToCores(const std::vector<int>& core_ids) {
    cpu_set_t mask;
    CPU_ZERO(&mask); // 清空掩码

    // 将指定的核心加入掩码
    for (int id : core_ids) {
        CPU_SET(id, &mask);
    }

    // 获取当前线程 ID (用于打印日志)
    pid_t tid = syscall(__NR_gettid);

    // 设置亲和性 (0 代表当前线程)
    if (sched_setaffinity(0, sizeof(mask), &mask) < 0) {
        std::cerr << "[Error] Failed to bind Thread " << tid << " to cores!" << std::endl;
    } else {
        std::cout << "[Success] Thread " << tid << " bound to cores: ";
        for(int id : core_ids) std::cout << id << " ";
        std::cout << "(XR2 Prime/Big cores are recommended)" << std::endl;
    }
}
#endif
// #include <System.h>

// ros::Publisher pub_camRawImg0, pub_camRawImg1;

int startIndex = 0;
int endIndex = 90000;
std::shared_ptr<DataReader> mpHeadDataReader = nullptr;
std::shared_ptr<ORB_SLAM3::HelperDataSaver> mHelperDataSaver = nullptr;
Estimator estimator;

std::mutex m_buf;

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
    mHelperDataSaver = std::shared_ptr<ORB_SLAM3::HelperDataSaver>(new ORB_SLAM3::HelperDataSaver());
    mHelperDataSaver->setRootDirectory("./output/p1/");
    mHelperDataSaver->startSaving();
    mHelperDataSaver->saveMaskImage();

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
        mMetricStatistic.Clear();
        mMetricStatistic.timeStamp = data_cam0.stamp;
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
            mMetricStatistic.timeImageAll = timeElapsedms*1.e-3;
            std::cout << "TrackStereo MetricStatistic, timeStamp=," << mMetricStatistic.timeStamp 
                        << ",timeImageAll=," << mMetricStatistic.timeImageAll
                        << ",fNumLkPreAll=," << mMetricStatistic.fNumLkPreAll << ",fNumLkPreLeft=," << mMetricStatistic.fNumLkPreLeft << ",fNumLkStereo=," << mMetricStatistic.fNumLkStereo 
                        << ",fNumOptWinAll=," << mMetricStatistic.fNumOptWinAll << ",fNumOptWinLeft=," << mMetricStatistic.fNumOptWinLeft << ",fNumOptWinInlier=," << mMetricStatistic.fNumOptWinInlier << ",fNumOptWinInlierRight=," << mMetricStatistic.fNumOptWinInlierRight 
                        << ",timeTrackAll=," << mMetricStatistic.timeTrackAll << ",timeLKLeftOnce=," << mMetricStatistic.timeLKLeftOnce << ",timeLKLeftTwice=," << mMetricStatistic.timeLKLeftTwice << ",timeGFTTLeft=," << mMetricStatistic.timeGFTTLeft << ",timeGFTTLeftTestOnce=," << mMetricStatistic.timeGFTTLeftTestOnce << ",timeLKRightTwice=," << mMetricStatistic.timeLKRightTwice
                        << ",timeImuAll=," << mMetricStatistic.timeImuAll
                        << ",timeImgAll=," << mMetricStatistic.timeImgAll << ",timeImgAddFeature=," << mMetricStatistic.timeImgAddFeature << ",timeImgOptiAll=," << mMetricStatistic.timeImgOptiAll << ",timeImgOptiWin=," << mMetricStatistic.timeImgOptiWin << ",timeImgOptiCeres=," << mMetricStatistic.timeImgOptiCeres << ",timeImgSlideiWin=," << mMetricStatistic.timeImgSlideiWin 
                        << std::endl;
            std::cout << "OpenCV threads:" << cv::getNumThreads() << std::endl;  // 输出：2
        }
        if(mHelperDataSaver){
            if(estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
                double timestamp = data_cam0.stamp;
                const Eigen::Vector3d poseT = estimator.Ps[WINDOW_SIZE];
                Eigen::Quaterniond poseQ;
                poseQ = Eigen::Quaterniond(estimator.Rs[WINDOW_SIZE]);

                mHelperDataSaver->savePoseData(timestamp, poseT, poseQ);
            }
            if(!estimator.featureTracker.imTrack.empty()){
                double timestamp = data_cam0.stamp;
                std::string trackName = "track_" + std::to_string(timestamp) + "_" + std::to_string(NUM_CAM) + "_" + std::to_string(estimator.solver_flag) + ".jpg";
                mHelperDataSaver->saveTrackImage(trackName, estimator.featureTracker.imTrack);
                // mHelperDataSaver->saveTrackImage(trackName, estimator.featureTracker.vTrackInfoMono[0].mask);
            }
        }

        data_cam0 = mpHeadDataReader->ReadStereoImage();        
        // usleep(1000 * 30);
        usleep(1000 * 3);
        // std::chrono::milliseconds dura(2);
        // std::this_thread::sleep_for(dura);

        // if(mKeybordManager && !mKeybordManager->isKeySpace()){
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


int main(int argc, char **argv)
{
    // ros::init(argc, argv, "vins_estimator");
    // ros::NodeHandle n("~");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
#ifdef ANDROID_ON_
    BindToCores({4,5});
#endif
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
    
    cv::setNumThreads(1);

    // mpHeadDataReader = std::make_shared<DATA_READER::HeadDataReader>(dataDir + "/head", 4, 1, startIndex, endIndex);
    mpHeadDataReader = std::make_shared<DataReader>(dataDir + "/");

    // int tot_images = mpHeadDataReader->getImgSize(0);
    // if((tot_images<=0))
    // {
    //     cerr << "ERROR: Failed to load images or IMU " << endl;
    //     return 1;
    // }

    sync_process();
    return 0;
}
