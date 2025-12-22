/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#ifndef ANDROID_ON_
#include "../utility/visualization.h"
#endif
#include "MapPoint.h"
Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
#ifdef VIEWER_ON_
    cv::namedWindow(SHOW_TRACK_NAME, cv::WINDOW_NORMAL);
#endif
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    while(!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    cur_frame_id = getGlobalFrameId(true);
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();//这里信息矩阵可以进一步修正:真实配置的焦距;考虑畸变模型后
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if(!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if(USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }
        
        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if(restart)
    {
        clearState();
        setParameter();
    }
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    // const Sophus::SE3d diffPose; 
    FeatureTracker::TrackInfoMonoOrb trackOrbPre[NUM_CAM];
    inputImage(t, _img, _img1, trackOrbPre, nullptr);
};

//trackOrbPre:上帧的orb点
//diffPose:orb和vins的对齐pose
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1, const FeatureTracker::TrackInfoMonoOrb (&trackOrbPre)[NUM_CAM], const std::shared_ptr<Sophus::SE3d> diffPose)
{
    inputImageCnt++;
    cur_frame_id = getGlobalFrameId(true);    
    std::cout << "----------imageCnt:" << inputImageCnt << ",cur_frame_id=," << cur_frame_id << "----------------" << std::endl;
    std::cout << "----------img_time:" << t << std::endl;
    if(featureBuf.size() >= 1){
        std::cout << "inputImage warn !!!!!!!!!!!!!!! before track, featureBuf.size=," << featureBuf.size() << std::endl;
    }
    std::pair<double, std::shared_ptr<FeatureTracker::TrackInfoComplex>> featureFrameMulti;//多目追踪结果
    featureFrameMulti.first = t;
    featureFrameMulti.second = std::shared_ptr<FeatureTracker::TrackInfoComplex>(new FeatureTracker::TrackInfoComplex());
    featureFrameMulti.second->diffPose = diffPose;
    // std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> featureFrameMulti;//多目追踪结果
    //featureFrame[id1][i].first是本帧的追踪到的特征所属相机cid:有0和1的双目id
    //featureFrame[id1][i].second是本帧的追踪到的特征 像素px等信息    
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc mTicTocMetric;
    TicToc featureTrackerTime;
    
    // const Sophus::SE3d diffPose; 
    // FeatureTracker::TrackInfoMonoOrb trackOrbPre[NUM_CAM];
    if(_img1.empty())
        featureFrameMulti.second->mOfs = featureTracker.trackImageMultiThread(t, _img, cv::Mat(), trackOrbPre);
    else
        featureFrameMulti.second->mOfs = featureTracker.trackImageMultiThread(t, _img, _img1, trackOrbPre);
    // featureFrame = featureFrameMulti[0];//暂时只取左目结果
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());
    // featureFrameMulti.second->mOrbs = featureTracker.vTrackInfoMonoOrb;
    for (int cid = 0; cid < NUM_CAM; cid++)
        featureFrameMulti.second->mOrbs[cid] = featureTracker.vTrackInfoMonoOrb[cid];
    
    mMetricStatistic.timeTrackAll = mTicTocMetric.tocMs();

    //计算最新帧track的orb点的平均baErr
    if(true && diffPose)//debug
    {

        std::pair<int, double> baErrByFrameLatest = {0,0.0};
        const int curFrame = frame_count - 1;        
        for (int cid = 0; cid < NUM_CAM; cid++){
            Eigen::Matrix4d posei;
            getPoseInWorldFrameOfCamera(curFrame, posei, cid);
            Sophus::SE3d poseSi = transPoseM4toSophus(posei);            
            const FeatureTracker::TrackInfoMonoOrb& curTrackInfoMonoOrb = trackOrbPre[cid];
            for (int pid = 0; pid < curTrackInfoMonoOrb.orbMPs.size(); pid++)
            {
                ORB_SLAM3::MapPoint* pOrbMP = curTrackInfoMonoOrb.orbMPs[pid];
                if(!pOrbMP || pOrbMP->isBad(false)){
                    continue;
                }
                const Eigen::Vector3d& orbPoseOri = pOrbMP->GetWorldPos().cast<double>();
                const Eigen::Vector3d& orbPose = (*diffPose) * orbPoseOri;


                Eigen::Vector3d pts_i(curTrackInfoMonoOrb.cur_un_pts[pid].x, curTrackInfoMonoOrb.cur_un_pts[pid].y, 1.0);

                Vector3d pts_i_cam = poseSi.inverse() * orbPose;
                Vector2d baErrZ1 = (pts_i_cam / pts_i_cam.z()).head<2>() - pts_i.head<2>();

                double rx = baErrZ1.x();
                double ry = baErrZ1.y();
                double err = sqrt(rx * rx + ry * ry);
                baErrByFrameLatest.first++;
                baErrByFrameLatest.second += err;
            }            
        }
        
        std::cout << "baErrByFrameLatest pOrbMP.id=," << curFrame << ",num=," << baErrByFrameLatest.first << ",ave_err=," << (baErrByFrameLatest.second/baErrByFrameLatest.first)*FOCAL_LENGTH << std::endl;

    }


    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
    #ifdef VIEWER_ON_
        cv::imshow(SHOW_TRACK_NAME,imgTrack);
        cv::waitKey(1.0);
    #endif
    #ifndef ANDROID_ON_
        pubTrackImage(imgTrack, t);
    #endif
    }
    // return;//看看纯track的性能消耗
    //把上面追踪到本帧的特征信息打上时间戳,缓存到队列featureBuf.
    //然后再从哦featureBuf依次取frame特征进行后续操作.
    if(MULTIPLE_THREAD)  
    {     
        // if(inputImageCnt % 2 == 0)
        if(inputImageCnt % 1 == 0)
        {
            mBuf.lock();
            // featureBuf.push(make_pair(t, featureFrame));
            // featureBuf.push(make_pair(t, featureFrameMulti));
            featureBuf.push(featureFrameMulti);
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        // featureBuf.push(make_pair(t, featureFrame));
        // featureBuf.push(make_pair(t, featureFrameMulti));
        featureBuf.push(featureFrameMulti);
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
    
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);//待 fix: 需要处理imu超前太多数据，会发生pose来回拉扯跳变.
    #ifndef ANDROID_ON_
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
    #endif
        mPropagate.unlock();
    }
}

//默认就只是添加左目,单目特征
void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    // std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> featureFrameMulti;
    // featureFrameMulti.emplace_back(featureFrame);
    std::pair<double, std::shared_ptr<FeatureTracker::TrackInfoComplex>> featureFrameMulti;//多目追踪结果
    featureFrameMulti.first = t;
    featureFrameMulti.second = std::shared_ptr<FeatureTracker::TrackInfoComplex>(new FeatureTracker::TrackInfoComplex());
    featureFrameMulti.second->mOfs.emplace_back(featureFrame);
    for (int cid = 1; cid < NUM_CAM; cid++)
    {
        featureFrameMulti.second->mOfs.emplace_back(map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>());
    }
    
    mBuf.lock();
    // featureBuf.push(make_pair(t, featureFrame));
    // featureBuf.push(make_pair(t, featureFrameMulti));
    featureBuf.push(featureFrameMulti);
    mBuf.unlock();

    if(!MULTIPLE_THREAD)
        processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

//多线程模式下，在Estimator::setParameter()函数中单独开1个现场执行本函数
void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        // pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        // pair<double, std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > feature;
        std::pair<double, std::shared_ptr<FeatureTracker::TrackInfoComplex>> feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if(!featureBuf.empty())
        {
            mBuf.lock();
            while (!featureBuf.empty())//只处理buf的最新一帧数据,因为对于外部orb而言,拿到的vins的状态的就是最新一帧的状态.
            {
                feature = featureBuf.front();
                curTime = feature.first + td;//估计的同步时间差作fix
                featureBuf.pop();
            }
            mBuf.unlock();
            cout << "processMeasurements, time=," << curTime << endl;
            while(1)
            {
                if ((!USE_IMU  || IMUAvailable(feature.first + td)))
                    break;
                else
                {
                    // printf("wait for imu ... \n");//打印数据太多
                    cout << "wait for imu ..." << endl;
                    // if (! MULTIPLE_THREAD)//不能直接返回,必须等到imu到达
                    //     return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            if(USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector);

            // featureBuf.pop();//在函数开头已经进行pop了
            mBuf.unlock();
            if(!featureBuf.empty()) cout << "processMeasurements, 2 featureBuf.size=,=," << featureBuf.size() << endl;
            TicToc mTicTocMetric;
            if(USE_IMU)
            {
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            mMetricStatistic.timeImuAll = mTicTocMetric.tocMs();
            mProcess.lock();
            mTicTocMetric.tic();
            processImage(*feature.second, feature.first);
            prevTime = curTime;
            mMetricStatistic.timeImgAll = mTicTocMetric.tocMs();
            printStatistics(*this, 0);
        #ifndef ANDROID_ON_
            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);

            pubOdometry(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
        #endif
            mProcess.unlock();
        }

        // if (! MULTIPLE_THREAD)
        if (! MULTIPLE_THREAD && featureBuf.empty())//必须把buf处理空了才结束本函数
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}


void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        // tmp_pre_integration 是上一帧末imu测量值开始，本帧累积imu积分结果
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

// void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
// void Estimator::processImage(const std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &image, const double header)
void Estimator::processImage(const FeatureTracker::TrackInfoComplex &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.mOfs[0].size());
    assert(image.mOfs.size() == NUM_CAM);
    TicToc mTicTocMetric;
    if (f_manager.addFeatureCheckParallax(cur_frame_id, frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }
    mMetricStatistic.timeImgAddFeature = mTicTocMetric.tocMs();
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    // ImageFrame imageframe(image, header);
    ImageFrame imageframe(image.mOfs[0], header);                   //fix:这里只用到单目追踪结果,且默认是左目?实际可能是多目，且存在纯右目? 
                                                            //这里就把原始的左目追踪结果给之,不会影响流程? 需要关注!!
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    // 尝试在线外参R标定
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            //取出最新帧和次新帧特征匹配，纯左目
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            //逐渐累积多帧的视觉和imu观察，逐渐求qic。到满窗才返回true
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))//满帧后才会估旋转
            {
                ROS_WARN("initial extrinsic rotation calib success");
                // ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                std::cout << "warn initial extrinsic rotation: " << endl << calib_ric << std::endl;
                ric[0] = calib_ric;
                RIC[0] = calib_ric;//这里直接把离线标定的外參都改啦? fix!
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    mTicTocMetric.tic();
    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;   
                }
                if(result)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if(STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);//fix:纯左目pnp,可以尝试多目pnp
            //此处应该执行全量BA,优化得到当前curF的pose
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);       //这里需要限制前后帧三角化
            //此处应该执行全量BA,同时优化得到当前curF的pose和特征点P坐标
            if (frame_count == WINDOW_SIZE)//只在满帧时才正式进行vi联合初始化
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];//此处已经是imu系的R了
                    frame_it->second.T = Ps[i];//此处已经是imu系的t了
                    i++;
                }
                
                // vi联合初始化:估计所有帧的速度.还有bg,重力g.
                // 然后所有状态对齐重力方向,并且所有地图点全部重新三角化.
                // 所有状态和所有地图点的深度值作为初始值,参与下面的vi的滑窗BA优化.
                bool result = visualInitialAlignIgnoreScale();

                if(result){
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");                    
                }else{
                    slideWindow();
                }
            }
        }

        // stereo only initilization
        if(STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if(frame_count == WINDOW_SIZE)
            {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;//初始化过程中，不满帧时，frame_count持续++
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else
    {
        TicToc t_solve;
        if(!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric, false);//在滑窗优化前，直接提前三角化了. fix, maybe在滑窗优化后再三角化更好?
        // f_manager.triangulate(frame_count, Ps, Rs, tic, ric, true);//在滑窗优化前，直接提前三角化了. fix, maybe在滑窗优化后再三角化更好?
        // f_manager.triangulate(frame_count, Ps, Rs, tic, ric);//在滑窗优化前，直接提前三角化了. fix, maybe在滑窗优化后再三角化更好?
        optimization();
        mMetricStatistic.timeImgOptiWin = mTicTocMetric.tocMs();
        //计算所有地图点MP的平均重投影误差,大于3个px就剔除
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);//从地图容器中剔除
        if (! MULTIPLE_THREAD)//在串行模式下,才会从光流追踪参考数据中剔除
        {
            std::vector<int> trackInlier = featureTracker.removeOutliers(removeIndex);//返回多目内点
            predictPtsInNextFrame();
            mMetricStatistic.fNumOptWinInlier = trackInlier[0];
            if(NUM_CAM > 1) mMetricStatistic.fNumOptWinInlierRight = trackInlier[1];
        }
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric, true);//在滑窗优化前，直接提前三角化了. fix, maybe在滑窗优化后再三角化更好?
        slideWindow();
        set<ORB_SLAM3::MapPoint*> removeIndexOrb;//剔除无效的orb点
        outliersRejection(removeIndexOrb);
        if(!removeIndexOrb.empty()){
            f_manager.removeOutlier(removeIndexOrb);//从地图容器中剔除
            if (! MULTIPLE_THREAD)//在串行模式下,才会从光流追踪参考数据中剔除
            {
                std::vector<int> trackInlierOrb = featureTracker.removeOutliers(removeIndexOrb);//返回多目内点
            }
        }
        f_manager.removeFailures();//剔除深度为负的地图MP点
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
        mMetricStatistic.timeImgSlideiWin = mTicTocMetric.tocMs() - mMetricStatistic.timeImgOptiWin;
    }  
    mMetricStatistic.timeImgOptiAll = mTicTocMetric.tocMs();
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            // Vector3d pts_j = it_per_frame.point;
            if(it_per_frame.is_observed[0] == false) continue;//只取左目点
            Vector3d pts_j = it_per_frame.point[0];
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose(); //右乘R
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);       //右乘t
        frame_it->second.R = R_pnp * RIC[0].transpose();//从c系转到i系描述
        frame_it->second.T = T_pnp;                     //从c系转到i系描述
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    std::cout << "g0     " << g.transpose() << std::endl;
    std::cout << "my R0  " << Utility::R2ypr(Rs[0]).transpose() << std::endl;
    // ROS_DEBUG_STREAM("g0     " << g.transpose());
    // ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    f_manager.clearDepth();
    // f_manager.triangulate(frame_count, Ps, Rs, tic, ric, true);
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    {
        std::cout << "acc_0=," << acc_0.transpose() << ", Ri*acc_0=," << (Rs[frame_count-1]*acc_0).transpose() << std::endl;
    }

    return true;
}

bool Estimator::visualInitialAlignIgnoreScale()
{
    solveGyroscopeBias(all_image_frame, Bgs);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    //此处应该初始化所有帧的速度和重力g
    Vector3d tmpG;
    VectorXd x;
    bool tmpRet = LinearAlignmentIgnorScale(all_image_frame, tmpG, x);

    {
        Matrix3d R0 = Utility::g2R(tmpG);
        std::cout << "acc_0=," << acc_0.transpose() << ", Ri*acc_0=," << (R0*Rs[frame_count-1]*acc_0).transpose() << std::endl;

        std::cout << "acc_0=," << acc_0.transpose() << std::endl;
        std::cout << "gyr_0=," << gyr_0.transpose() << std::endl;
        std::cout << "LinearAlignmentIgnorScale result:" << std::endl
                    << "tmpRet=," << tmpRet << std::endl
                    << "tmpG=," << tmpG.transpose() << std::endl
                    << "x=" << x.transpose() << std::endl;  

    }

    if(!tmpRet){
        ROS_DEBUG("solve g failed!");
        return false;        
    }

    g = tmpG;

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    // double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    // for (int i = frame_count; i >= 0; i--)
    //     Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);//这是重力对齐变换.
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;//消除yaw角对齐
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)//将所有帧的R,T,V进行重力对齐
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    std::cout << "g0     " << g.transpose() << std::endl;
    std::cout << "my R0  " << Utility::R2ypr(Rs[0]).transpose() << std::endl;
    // ROS_DEBUG_STREAM("g0     " << g.transpose());
    // ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    f_manager.clearDepth();
    // f_manager.triangulate(frame_count, Ps, Rs, tic, ric, true);//重新三角化所有地图点.
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);//重新三角化所有地图点.

    {
        std::cout << "acc_0=," << acc_0.transpose() << ", Ri*acc_0=," << (Rs[frame_count-1]*acc_0).transpose() << std::endl;
    }

    return true;

};

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        //计算z=1平面上的平均视差 (像素/f)
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

//把所有帧和地图点的实时状态转移到优化buf中。注意地图点的buf转移
void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    VectorXd dep = f_manager.getDepthVector();//注意，这里固定取用used_num>=4的地图点, fix：如果只是右目地图点,满足次数大于4但是前后帧三角化还不稳定;还有track_keep丢失但是满足有效性的点.
                                                //fix, 在滑窗中被选中进行优化的MP点，增加标记frameId

    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    //融合imu时，需要保持和滑窗起始帧yaw角对齐
    if(USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));//对齐变换
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
            
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if(USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    //注意特征点的深度恢复
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);//这里求解出的深度是负数,地图点MP的solve_flag会被置为2

    if(USE_IMU)
        td = para_Td[0][0];

}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        //return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    TicToc mTicTocCeres;
    //把所有帧和地图点的实时状态转移到优化buf中。注意地图点的buf转移
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    //先设置待估计状态量
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if(USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    //继续设置所有的相关观测
    // 边缘化约束
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    //预积分约束. fix:注意那些时差太长的预积分约束需要剔除
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)//这里dt太长，需要剔除这个预积分观测.
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    //视觉BA约束
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)//fix,这里直接以 used_num 为有效点标记.no 需要重新弄标记! 这里用标记来判断,且需要拿特征深度和状态深度比对.
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;//imu_i是参考帧序,imu_j是当前帧序.
        
        const int main_cam = it_per_id.feature_per_frame[0].main_cam;
        // Vector3d pts_i = it_per_id.feature_per_frame[0].point;//参考帧下的左目观测.fix,有可能是右目观测.
        Vector3d pts_i = it_per_id.feature_per_frame[0].point[main_cam];//参考帧下的左目观测.fix,有可能是右目观测.
        mMetricStatistic.fNumOptWinAll++;
        if(main_cam == 0) mMetricStatistic.fNumOptWinLeft++;
/*
        for (auto &it_per_frame : it_per_id.feature_per_frame)//依次遍历所有观测. fix,这里认为是连续帧不断的观测.
        {
            imu_j++;
            if (imu_i != imu_j)//构建单相机前后帧观测.  fix,这里只构建参考帧是左目的, 但是还有参考帧是右目的
            {
                Vector3d pts_j = it_per_frame.point;
                // velocity是图像特征在z1平面坐标的速度
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if(STEREO && it_per_frame.is_stereo)//本帧对MP是多目观测,构建左右双目和前后不同相机观测.
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)//构建前后不同相机观测.  fix,这里只考虑了前左后右,实际还有前右后左
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else//构建参考帧 单帧的双目观测.  fix,这里只构建了从左到右的观测, 实际还应该有从右到左的观测.
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
               
            }
            f_m_cnt++;
        }
*/

        for (auto &it_per_frame : it_per_id.feature_per_frame)//依次遍历所有观测. fix,这里认为是连续帧不断的观测.
        {
            imu_j++;

            //好几种情况
            if(imu_i == imu_j){//同帧
                for (int cid = 0; cid < NUM_CAM; cid++)
                {
                    if(!it_per_frame.is_observed[cid]){
                        continue;
                    }
                    //同目跳过(就是参考帧)
                    //异目,构建同帧双目约束
                    if(cid == main_cam){
                        continue;
                    }else{
                        Vector3d pts_j = it_per_frame.point[cid];
                        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity[main_cam], it_per_frame.velocity[cid],
                                                                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        problem.AddResidualBlock(f, loss_function, para_Ex_Pose[main_cam], para_Ex_Pose[cid], para_Feature[feature_index], para_Td[0]);
                    }
                }
            }else{//前后帧
                for (int cid = 0; cid < NUM_CAM; cid++)
                {
                    if(!it_per_frame.is_observed[cid]){
                        continue;
                    }
                    //同目,构建同目前后帧约束
                    //异目,构建异目前后帧约束
                    Vector3d pts_j = it_per_frame.point[cid];
                    if(cid == main_cam){
                        // velocity是图像特征在z1平面坐标的速度
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity[main_cam], it_per_frame.velocity[cid],
                                                                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[main_cam], para_Feature[feature_index], para_Td[0]);
                    }else{
                        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity[main_cam], it_per_frame.velocity[cid],
                                                                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[main_cam], para_Ex_Pose[cid], para_Feature[feature_index], para_Td[0]);
                    }
                }
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;//SPARSE_SCHUR或DENSE_SCHUR就是启用了舒尔补加速了
    // options.linear_solver_type = ceres::SPARSE_SCHUR;//SPARSE_SCHUR或DENSE_SCHUR就是启用了舒尔补加速了
    // options.num_threads = 2;
    // options.num_threads = 8;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    mTicTocCeres.tic();
    ceres::Solve(options, &problem, &summary);
    mMetricStatistic.timeImgOptiCeres = mTicTocCeres.tocMs();
    // cout << "ceres summary: " <<  summary.BriefReport() << endl;
    cout << "ceres summary: " <<  summary.FullReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    //printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    //printf("frame_count: %d \n", frame_count);

    if(frame_count < WINDOW_SIZE)
        return;
    
    TicToc t_whole_marginalization;
    // 边缘化最老帧 or 边缘化次新帧
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                //对照上面的优化前约束逐个fix 视觉观测.
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)//要求起始参考帧是最老帧，才会进行下面的边缘化操作
                    continue;

                const int main_cam = it_per_id.feature_per_frame[0].main_cam;
                // Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point[main_cam];
/*
                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }

*/
                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;

                    //好几种情况
                    if(imu_i == imu_j){//同帧
                        for (int cid = 0; cid < NUM_CAM; cid++)
                        {
                            if(!it_per_frame.is_observed[cid]){
                                continue;
                            }
                            //同目跳过(就是参考帧)
                            //异目,构建同帧双目约束
                            if(cid == main_cam){
                                continue;
                            }else{
                                Vector3d pts_j = it_per_frame.point[cid];
                                ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity[main_cam], it_per_frame.velocity[cid],
                                                                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                            vector<double *>{para_Ex_Pose[main_cam], para_Ex_Pose[cid], para_Feature[feature_index], para_Td[0]},
                                                                                            vector<int>{2});
                                marginalization_info->addResidualBlockInfo(residual_block_info);

                            }
                        }
                    }else{//前后帧
                        for (int cid = 0; cid < NUM_CAM; cid++)
                        {
                            if(!it_per_frame.is_observed[cid]){
                                continue;
                            }
                            //同目,构建同目前后帧约束
                            //异目,构建异目前后帧约束
                            Vector3d pts_j = it_per_frame.point[cid];                    
                            if(cid == main_cam){
                                // velocity是图像特征在z1平面坐标的速度
                                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity[main_cam], it_per_frame.velocity[cid],
                                                                                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);

                                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                                vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[main_cam], para_Feature[feature_index], para_Td[0]},
                                                                                                vector<int>{0, 3});
                                marginalization_info->addResidualBlockInfo(residual_block_info);

                            }else{
                                ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity[main_cam], it_per_frame.velocity[cid],
                                                                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                            vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[main_cam], para_Ex_Pose[cid], para_Feature[feature_index], para_Td[0]},
                                                                                            vector<int>{0, 4});
                                marginalization_info->addResidualBlockInfo(residual_block_info);                                
                            }
                        }
                    }

                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else//边缘化次新帧:只考虑了上次边缘化约束,其余约束1个都没考虑.
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

//1,把实际状态进行滑窗修改:R,P,V,Ba,Bg, 另外还有预积分测量,imu的原始dt,acc,gyro
//2,把特征MP的起始参考帧和参考深度作滑窗修改
void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

//滑窗后，将特征点的起始参考帧和参考深度值作修改!
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)//滑窗优化阶段
    {
/*
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        //切换参考帧和深度值
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);//针对0帧起始和非0帧起始分别处理
*/
        std::vector<Eigen::Matrix3d> marg_R; 
        std::vector<Eigen::Vector3d> marg_P;
        std::vector<Eigen::Matrix3d> new_R;
        std::vector<Eigen::Vector3d> new_P;
        marg_R.resize(NUM_CAM);
        marg_P.resize(NUM_CAM);
        new_R.resize(NUM_CAM);
        new_P.resize(NUM_CAM);
        for (int cid = 0; cid < NUM_CAM; cid++)
        {
            marg_R[cid] = back_R0 * ric[cid];
            new_R[cid] = Rs[0] * ric[cid];
            marg_P[cid] = back_P0 + back_R0 * tic[cid];
            new_P[cid] = Ps[0] + Rs[0] * tic[cid];
        }
        //切换参考帧和深度值
        f_manager.removeBackShiftDepth(marg_R, marg_P, new_R, new_P);//针对0帧起始和非0帧起始分别处理        
    }
    else//初始化阶段
        f_manager.removeBack();//针对0帧起始和非0帧起始分别处理，特别地0帧起始特征直接整个删除
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrameOfCamera(Eigen::Matrix4d &T, int cid){
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count] * ric[cid];
    T.block<3, 1>(0, 3) = Rs[frame_count] * tic[cid] + Ps[frame_count];
};

void Estimator::getPoseInWorldFrameOfCamera(int index, Eigen::Matrix4d &T, int cid){
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index] * ric[cid];
    T.block<3, 1>(0, 3) = Rs[index] * tic[cid] + Ps[index];
};

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

Sophus::SE3d Estimator::transPoseM4toSophus(Eigen::Matrix4d &T){
    Eigen::Quaterniond poseQ = Eigen::Quaterniond(T.block<3,3>(0,0));
    return Sophus::SE3d(poseQ,  T.block<3,1>(0,3));
};

void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);//恒速模型预测
    // map<int, Eigen::Vector3d> predictPts;
    map<int, Eigen::Vector3d> predictPts[NUM_CAM];

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                const int main_cam = it_per_id.feature_per_frame[0].main_cam;

                double depth = it_per_id.estimated_depth;
                // Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_j = ric[main_cam] * (depth * it_per_id.feature_per_frame[0].point[main_cam]) + tic[main_cam];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                for (int cid = 0; cid < NUM_CAM; cid++)
                {
                    Vector3d pts_cam = ric[cid].transpose() * (pts_local - tic[cid]);
                    int ptsIndex = it_per_id.feature_id;
                    predictPts[cid][ptsIndex] = pts_cam;//特征点在左目的3d坐标
                }
            }
        }
    }
    for (int cid = 0; cid < NUM_CAM; cid++)
    {
        if(predictPts[cid].size() > 0){
            std::cout <<  "predictPtsInNextFrame, cid=," << cid << ",num=," << predictPts[cid].size() << std::endl;
            featureTracker.setPrediction(cid, predictPts[cid]);//暂时只设置左目预测点
        }
    }
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

//计算所有地图点MP的平均重投影误差,大于3个px就剔除
void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        const int main_cam = it_per_id.feature_per_frame[0].main_cam;
        // Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point[main_cam];
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            //fix  需要对照滑窗 BA 约束的几点进行修改
            imu_j++;
/*
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(STEREO && it_per_frame.is_stereo)
            {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {            
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
*/
            //好几种情况
            if(imu_i == imu_j){//同帧
                for (int cid = 0; cid < NUM_CAM; cid++)
                {
                    if(!it_per_frame.is_observed[cid]){
                        continue;
                    }
                    //同目跳过(就是参考帧)
                    //异目,构建同帧双目约束
                    if(cid == main_cam){
                        continue;
                    }else{
                        Vector3d pts_j = it_per_frame.point[cid];
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[main_cam], tic[main_cam], 
                                                            Rs[imu_j], Ps[imu_j], ric[cid], tic[cid],
                                                            depth, pts_i, pts_j);
                        err += tmp_error;
                        errCnt++;
                        //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);                        
                    }
                }
            }else{//前后帧
                for (int cid = 0; cid < NUM_CAM; cid++)
                {
                    if(!it_per_frame.is_observed[cid]){
                        continue;
                    }
                    //同目,构建同目前后帧约束
                    //异目,构建异目前后帧约束
                    Vector3d pts_j = it_per_frame.point[cid];
                    if(cid == main_cam){
                        Vector3d pts_j = it_per_frame.point[cid];
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[main_cam], tic[main_cam], 
                                                            Rs[imu_j], Ps[imu_j], ric[cid], tic[cid],
                                                            depth, pts_i, pts_j);
                        err += tmp_error;
                        errCnt++;
                        //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                    }else{
                        Vector3d pts_j = it_per_frame.point[cid];
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[main_cam], tic[main_cam], 
                                                            Rs[imu_j], Ps[imu_j], ric[cid], tic[cid],
                                                            depth, pts_i, pts_j);
                        err += tmp_error;
                        errCnt++;
                        //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                    }
                }
            }
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)//平均重投影误差大于3就认为是剔除点
            removeIndex.insert(it_per_id.feature_id);

    }
}

void Estimator::outliersRejection(set<ORB_SLAM3::MapPoint*> &removeIndexOrb){
    //删除bad点
    //删除观测总数为0的点
    //删除重投影误差大的点
    int remove1 = 0,remove2 = 0,remove3 = 0,remove4 = 0;
    removeIndexOrb.clear();
    for (auto &it_per_id : f_manager.featureOrb){
        if(!it_per_id.first){
            removeIndexOrb.insert(it_per_id.first);
            remove1++;
            continue;
        }
        if(it_per_id.first->isBad(false)){
            removeIndexOrb.insert(it_per_id.first);
            remove2++;
            continue;
        }
        if(it_per_id.second.obs.empty()){
            removeIndexOrb.insert(it_per_id.first);
            remove3++;
            continue;
        }
        const int endIdx = it_per_id.second.start_frame + it_per_id.second.obs.rbegin()->first;
        if(endIdx < 0){
            removeIndexOrb.insert(it_per_id.first);
            remove4++;
            continue;
        }
        if(endIdx > WINDOW_SIZE){
            std::cout << "outliersRejection err 1, endIdx=," << endIdx << std::endl;
        }

    }
    std::cout << "Estimator removeIndexOrb.size=," << removeIndexOrb.size() << ",remove1=," << remove1 << ",remove2=," 
                << remove2 << ",remove3=," << remove3 << ",remove4=," << remove4 << std::endl;
};

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

// std::string EigenVector3dToStr(const Eigen::Vector3d& v3d){
//     return std::to_string(v3d(0)) + "," + std::to_string(v3d(1)) + "," + std::to_string(v3d(2));
// };

void Estimator::updateLatestStates()
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }

    cout << "latest_time=," << latest_time << ",latest_P=," << Utility::EigenVector3dToStr(latest_P)
            << ",latest_Q=," << latest_Q.coeffs()(0) << "," << latest_Q.coeffs()(1) << "," << latest_Q.coeffs()(2) << "," << latest_Q.coeffs()(3)
            << ",latest_V=," << Utility::EigenVector3dToStr(latest_V)
            << ",latest_Ba=," << Utility::EigenVector3dToStr(latest_Ba)
            << ",latest_Bg=," << Utility::EigenVector3dToStr(latest_Bg)
            << endl;

    mPropagate.unlock();
}

static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);
void Estimator::printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    std::cout << "ros debug " << "position: " << estimator.Ps[WINDOW_SIZE].transpose() << std::endl;
    std::cout << "ros debug " << "orientation: " << estimator.Vs[WINDOW_SIZE].transpose() << std::endl;
    if (ESTIMATE_EXTRINSIC)
    {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            //ROS_DEBUG("calibration result for camera %d", i);
            std::cout << "ros debug " << "extirnsic tic: " << estimator.tic[i].transpose() << std::endl;
            std::cout << "ros debug " << "extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose() << std::endl;

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
            eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            if(i == 0)
                fs << "body_T_cam0" << cv_T ;
            else
                fs << "body_T_cam1" << cv_T ;
        }
        fs.release();
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    std::cout << "ros debug " << ("vo solver costs: " + std::to_string(t) + " ms") << std::endl;
    std::cout << "ros debug " << ("average of time " + std::to_string(sum_of_time / sum_of_calculation) + " ms") << std::endl;

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    std::cout << "ros debug " << ("sum of path " + std::to_string(sum_of_path)) << std::endl;
    if (ESTIMATE_TD)
        std::cout << "ros info " << ("td " + std::to_string(estimator.td)) << std::endl;
}