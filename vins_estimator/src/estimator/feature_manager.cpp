/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}

//本帧最新特征刷新地图点列表feature（老点累加观测，新点创建新MP）
//用追踪强弱和平移视差来判断是否要KF:MARGIN_OLD
// bool FeatureManager::addFeatureCheckParallax(int cur_frame_id, int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
bool FeatureManager::addFeatureCheckParallax(int cur_frame_id, int frame_count, const std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;
    //image[fid][i].first是本帧的追踪到的特征所属相机cid:有0和1的双目id
    //image[fid][i].second是本帧的追踪到的特征 像素px等信息

    //以特征MP为核心构建所有帧的观测.
    //feature是滑窗地图所有MP点.  feature[i]是单个特征, feature[i].feature_per_frame 是FeaturePerFrame列表，记录所有帧对本特征的观测信息.
    // int curCamId = 0;   //主相机id
    for (int curCamId = 0; curCamId < image.size(); curCamId++)
    for (auto &id_pts : image[curCamId])
    // for (auto &id_pts : image)
    {
        // FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        // assert(id_pts.second[0].first == 0);
        assert(id_pts.second[0].first == curCamId);                         //要求单目观测,首个观测必须是主相机观测
        FeaturePerFrame f_per_fra(curCamId, id_pts.second[0].second, td);
        if(id_pts.second.size() >= 2)//数目为2即为双目，需要添加右目观测
        {
            for (int cdx = 1; cdx < id_pts.second.size(); cdx++)
            {
                assert(id_pts.second[cdx].first != curCamId);
                f_per_fra.otherObservation(id_pts.second[cdx].first, id_pts.second[cdx].second);
            }
            
            // f_per_fra.rightObservation(id_pts.second[1].second);
            // assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count, cur_frame_id));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    // if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
    if (frame_count < 2 || last_track_num < 20*1.5 || long_track_num < 40*1.5 || new_feature_num > 0.5 * last_track_num)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            double ans = compensatedParallax2(it_per_id, frame_count);
            if(ans == 0) continue;
            parallax_sum += ans;
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            if(it.feature_per_frame[idx_l].is_observed[0] == false || 
                it.feature_per_frame[idx_r].is_observed[0] == false)//保证左目都有对应观测!
            {
                continue;
            }
            a = it.feature_per_frame[idx_l].point[0];

            b = it.feature_per_frame[idx_r].point[0];
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}
// 用PnP方法直接估最新帧pose，且pose是描述imu的
// 基于opencv的接口:cv::solvePnP
// fix：地图MP点，可能存在track_keep=false的点，需要识别并不采纳
void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{

    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;//目前只是纯左目特征的pnp
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {
            if (it_per_id.estimated_depth > 0)
            {
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    // Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];//fix:默认就是左目点*左目深度
                    if(it_per_id.feature_per_frame[index].is_observed[0] == false) continue;//要求左目点必须有观测
                    int mainCam = it_per_id.feature_per_frame[0].main_cam;
                    Vector3d ptsInCam = ric[mainCam] * (it_per_id.feature_per_frame[0].point[mainCam] * it_per_id.estimated_depth) + tic[mainCam];//fix:默认就是左目点*左目深度
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    //如下是3d点和左目观测
                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point[0].x(), it_per_id.feature_per_frame[index].point[0].y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[0];
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose();                       //已经转到imu系描述了
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;      //已经转到imu系描述了

            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}

//triangleAll:false-只作双目三角化;true-按原始流程作三角化
void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], bool triangleAll)
{
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;
        const int mainCam = it_per_id.feature_per_frame[0].main_cam;
        // if(STEREO && it_per_id.feature_per_frame[0].is_stereo)//双目三角化,算是靠谱。fix:这里要求只能是初始参考帧双目,中间的双目被忽视了
        if(STEREO && it_per_id.feature_per_frame[0].is_stereoX())//双目三角化,算是靠谱。fix:这里要求只能是初始参考帧双目,中间的双目被忽视了
        {
            //找第一个有效从camera
            int slaveCam = -1;
            for (int cid = 0; cid < NUM_CAM; cid++)
            {
                if(cid == mainCam) continue;
                if(it_per_id.feature_per_frame[0].is_observed[cid]){
                    slaveCam = cid;
                    break;
                }
            }
            if(slaveCam == -1 || slaveCam == mainCam){
                std::cout << "triangulate warn 1, mainCam=," << mainCam << ",slaveCam=," << slaveCam << std::endl;
                continue;
            }
            
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[mainCam];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[mainCam];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;
            // Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            // Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[slaveCam];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[slaveCam];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            // point0 = it_per_id.feature_per_frame[0].point.head(2);
            // point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            point0 = it_per_id.feature_per_frame[0].point[mainCam].head(2);
            point1 = it_per_id.feature_per_frame[0].point[slaveCam].head(2);
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        else if(!triangleAll){
            continue;
        }
        else if(it_per_id.feature_per_frame.size() > 1)//前后多帧三角化. fix,1这里只用了左目特征;2这里没有检查基线长度;3这里只用了前后2帧
        {
            if(it_per_id.feature_per_frame[0].is_observed[mainCam] == false){
                std::cout << "triangulate warn 2, mainCam=," << mainCam << std::endl;
                continue;
            }
            if(it_per_id.feature_per_frame[1].is_observed[mainCam] == false){
                continue;
            }
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[mainCam];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[mainCam];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            imu_i++;
            Eigen::Matrix<double, 3, 4> rightPose;
            // Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            // Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[mainCam];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[mainCam];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            // point0 = it_per_id.feature_per_frame[0].point.head(2);
            // point1 = it_per_id.feature_per_frame[1].point.head(2);
            point0 = it_per_id.feature_per_frame[0].point[mainCam].head(2);
            point1 = it_per_id.feature_per_frame[1].point[mainCam].head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[mainCam];//以主相机pose为基准
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[mainCam];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            int slaveCam = mainCam;
            if(it_per_frame.is_observed[slaveCam] == false){
                for (int cid = 0; cid < NUM_CAM; cid++)
                {
                    if(cid == mainCam) continue;
                    if(it_per_frame.is_observed[cid]){
                        slaveCam = cid;
                        break;
                    }
                }
            }
            if(it_per_frame.is_observed[slaveCam] == false){
                std::cout << "triangulate warn 3, mainCam=," << mainCam << std::endl;
                continue;//没找到
            }

            // Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            // Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[slaveCam];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[slaveCam];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            // Eigen::Vector3d f = it_per_frame.point.normalized();
            Eigen::Vector3d f = it_per_frame.point[slaveCam].normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

void FeatureManager::triangulate2(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], bool withScale)
{
    int triNumStereo = 0, triNumTwo = 0, triNumMulti = 0, triNumMultiFail0 = 0, triNumMultiFail1 = 0, triNumMultiFail2 = 0;
    // 定义一个固定的像素误差阈值 (基于 2.5 sigma * 2 像素噪声)
    const double MAX_REPROJ_ERROR_PIXELS = 5.0; 
    // 定义最小视差角 (cos(2.5 度) ~ 0.999)
    const double MIN_PARALLAX_COS_MOTION = 0.999;
    // 定义立体匹配的最小视差角 (cos(0.4 度) ~ 0.99998), 允许点更远
    // const double MIN_PARALLAX_COS_STEREO = 0.99998;
    const double MIN_PARALLAX_COS_STEREO = 0.9998;
    // const double MIN_PARALLAX_COS_STEREO = 0.99992;
    double minDepth = withScale? 0.1 : 0;
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;
        const int mainCam = it_per_id.feature_per_frame[0].main_cam;
        // if(STEREO && it_per_id.feature_per_frame[0].is_stereo)//双目三角化,算是靠谱。fix:这里要求只能是初始参考帧双目,中间的双目被忽视了
        if(STEREO && it_per_id.feature_per_frame[0].is_stereoX())//双目三角化,算是靠谱。fix:这里要求只能是初始参考帧双目,中间的双目被忽视了
        {
            //找第一个有效从camera
            int slaveCam = -1;
            for (int cid = 0; cid < NUM_CAM; cid++)
            {
                if(cid == mainCam) continue;
                if(it_per_id.feature_per_frame[0].is_observed[cid]){
                    slaveCam = cid;
                    break;
                }
            }
            if(slaveCam == -1 || slaveCam == mainCam){
                std::cout << "triangulate warn 1, mainCam=," << mainCam << ",slaveCam=," << slaveCam << std::endl;
                continue;
            }
            
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[mainCam];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[mainCam];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;
            // Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            // Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[slaveCam];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[slaveCam];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            // point0 = it_per_id.feature_per_frame[0].point.head(2);
            // point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            point0 = it_per_id.feature_per_frame[0].point[mainCam].head(2);
            point1 = it_per_id.feature_per_frame[0].point[slaveCam].head(2);
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

            Eigen::Vector3d ray_left_w = R0 * Eigen::Vector3d(point0.x(), point0.y(), 1.0);
            Eigen::Vector3d ray_right_w = R1 * Eigen::Vector3d(point1.x(), point1.y(), 1.0);
            double cos_parallax_stereo = ray_left_w.dot(ray_right_w) / (ray_left_w.norm() * ray_right_w.norm());
            
            double depth = -1;
            // 只有视差足够大时才尝试
            if (cos_parallax_stereo < MIN_PARALLAX_COS_STEREO)
            {
                Eigen::Vector3d point3d_w; // 3D点 in World frame
                triangulatePoint(leftPose, rightPose, point0, point1, point3d_w);

                // 转换到左相机坐标系
                Eigen::Vector3d point_in_cam_left = leftPose.leftCols<3>() * point3d_w + leftPose.rightCols<1>();
                // 转换到右相机坐标系
                Eigen::Vector3d point_in_cam_right = rightPose.leftCols<3>() * point3d_w + rightPose.rightCols<1>();

                // --- 1b. 正深度检查 (Positive Depth Check) ---
                if (point_in_cam_left.z() > minDepth && point_in_cam_right.z() > minDepth)
                {
                    // --- 1c. 重投影误差检查 (Reprojection Error Check) ---
                    Eigen::Vector2d proj_left(point_in_cam_left.x() / point_in_cam_left.z(), point_in_cam_left.y() / point_in_cam_left.z());
                    Eigen::Vector2d proj_right(point_in_cam_right.x() / point_in_cam_right.z(), point_in_cam_right.y() / point_in_cam_right.z());

                    double err_left_px = (proj_left - point0).norm() * FOCAL_LENGTH;
                    double err_right_px = (proj_right - point1).norm() * FOCAL_LENGTH;

                    if (err_left_px < MAX_REPROJ_ERROR_PIXELS && err_right_px < MAX_REPROJ_ERROR_PIXELS)
                    {
                        // 所有检查通过
                        depth = point_in_cam_left.z();
                        // it_per_id.estimated_depth = point_in_cam_left.z();                        
                        // continue; // 三角化成功, 跳过后续SVD
                    }
                }
            }
            /*
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            */
            if (depth > minDepth)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
           triNumStereo++;
            continue;
        }
        else if(false && it_per_id.feature_per_frame.size() > 1)//前后多帧三角化. fix,1这里只用了左目特征;2这里没有检查基线长度;3这里只用了前后2帧
        // else if(it_per_id.feature_per_frame.size() > 1)//前后多帧三角化. fix,1这里只用了左目特征;2这里没有检查基线长度;3这里只用了前后2帧
        {
            if(it_per_id.feature_per_frame[0].is_observed[mainCam] == false){
                std::cout << "triangulate warn 2, mainCam=," << mainCam << std::endl;
                continue;
            }
            if(it_per_id.feature_per_frame[1].is_observed[mainCam] == false){
                continue;
            }
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[mainCam];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[mainCam];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            imu_i++;
            Eigen::Matrix<double, 3, 4> rightPose;
            // Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            // Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[mainCam];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[mainCam];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            // point0 = it_per_id.feature_per_frame[0].point.head(2);
            // point1 = it_per_id.feature_per_frame[1].point.head(2);
            point0 = it_per_id.feature_per_frame[0].point[mainCam].head(2);
            point1 = it_per_id.feature_per_frame[1].point[mainCam].head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > minDepth)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            triNumTwo++;
            continue;
        }
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        triNumMulti++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        // Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        // Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[mainCam];//以主相机pose为基准
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[mainCam];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        double motionLine = 0;//位移总长度
        for (auto &it_per_frame : it_per_id.feature_per_frame)//fix,这里对单帧只考虑了主相机观测,其余相机观测也可以考虑进来
        {
            imu_j++;
            int slaveCam = mainCam;
            if(it_per_frame.is_observed[slaveCam] == false){
                for (int cid = 0; cid < NUM_CAM; cid++)
                {
                    if(cid == mainCam) continue;
                    if(it_per_frame.is_observed[cid]){
                        slaveCam = cid;
                        break;
                    }
                }
            }
            if(it_per_frame.is_observed[slaveCam] == false){
                std::cout << "triangulate warn 3, mainCam=," << mainCam << std::endl;
                continue;//没找到
            }

            // Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            // Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[slaveCam];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[slaveCam];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            // Eigen::Vector3d f = it_per_frame.point.normalized();
            Eigen::Vector3d f = it_per_frame.point[slaveCam].normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            motionLine += t.norm();
            if (imu_i == imu_j)
                continue;
        }
        if(motionLine < 0.15){//位移不够，暂留不处理
            triNumMultiFail0++;
            continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double depth = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = depth;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
            triNumMultiFail1++;
            continue;
        }

        //进一步校验重投影误差
        Eigen::Vector3d P_in_cam_i( (svd_V[0] / svd_V[3]), (svd_V[1] / svd_V[3]), depth );

        // --- 3b. SVD 重投影误差检查 (SVD Reprojection Error Check) ---
        bool reproj_ok = true;
        imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            int slaveCam = mainCam;
            if(it_per_frame.is_observed[slaveCam] == false){
                for (int cid = 0; cid < NUM_CAM; cid++)
                {
                    if(cid == mainCam) continue;
                    if(it_per_frame.is_observed[cid]){
                        slaveCam = cid;
                        break;
                    }
                }
            }
            if(it_per_frame.is_observed[slaveCam] == false){
                std::cout << "triangulate warn 4, mainCam=," << mainCam << std::endl;
                continue;//没找到
            }

            // Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            // Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[slaveCam];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[slaveCam];
            Eigen::Vector3d t_ci_cj = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R_ci_cj = R0.transpose() * R1;

            // // 重新计算 T_ci_cj
            // Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            // Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            // Eigen::Vector3d t_ci_cj = R0.transpose() * (t1 - t0);
            // Eigen::Matrix3d R_ci_cj = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R_ci_cj.transpose();
            P.rightCols<1>() = -R_ci_cj.transpose() * t_ci_cj;

            // 将点 P_in_cam_i 变换到 P_in_cam_j
            Eigen::Vector3d P_in_cam_j = P.leftCols<3>() * P_in_cam_i + P.rightCols<1>();

            // 检查所有帧的正深度
            if (P_in_cam_j.z() <= 0)
            {
                reproj_ok = false;
                break;
            }

            Eigen::Vector2d proj_j(P_in_cam_j.x() / P_in_cam_j.z(), P_in_cam_j.y() / P_in_cam_j.z());
            Eigen::Vector2d obs_j = it_per_frame.point[slaveCam].head<2>();
            double pixel_err = (proj_j - obs_j).norm() * FOCAL_LENGTH;
            
            if (pixel_err > MAX_REPROJ_ERROR_PIXELS)
            {
                reproj_ok = false;
                break;
            }
        }

        if (reproj_ok)
        {
            // 所有检查通过
            it_per_id.estimated_depth = depth;
            //if(svd_method >= 0.1)
            //{
            //    std::cout<<"using this SVD triangulate!"<<std::endl;
            //}
        }
        else
        {
            triNumMultiFail2++;
            // SVD 失败 (深度为负或重投影误差大)
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }

    std::cout << "try triangulate, frameCnt=," << frameCnt << ",triNumStereo=," << triNumStereo << ",triNumTwo=," << triNumTwo << ",triNumMulti=," << triNumMulti 
                << ",triNumMultiFail0=," << triNumMultiFail0 << ",triNumMultiFail1=," << triNumMultiFail1 << ",triNumMultiFail2=," << triNumMultiFail2 << std::endl;
}

void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

//切换参考帧和深度值
//fix:注意起始0帧情况下,需要删除起始帧观测，并且需要转移参考帧和深度! 左右目都考虑
// void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
void FeatureManager::removeBackShiftDepth(const std::vector<Eigen::Matrix3d>& marg_R, const std::vector<Eigen::Vector3d>& marg_P, const std::vector<Eigen::Matrix3d>& new_R, const std::vector<Eigen::Vector3d>& new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        //起始非0帧,只是修改参考帧id. fix,注意右目参考帧情况
        if (it->start_frame != 0)
            it->start_frame--;
        else//起始0帧，删除起始帧观测,并把深度转移到后一帧.  fix,注意右目参考帧，深度转移需要在右目上进行:1)后一帧右目有观测就转移到右目.2)后一帧右目没有观测,左目必有观测,就把参考帧切成左目!
        {
            const int main_cam = it->feature_per_frame[0].main_cam;
            // Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point[main_cam];
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                const int main_cam2 = it->feature_per_frame[0].main_cam;//切换新的参考目

                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                // Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                // Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                Eigen::Vector3d w_pts_i = marg_R[main_cam] * pts_i + marg_P[main_cam];
                Eigen::Vector3d pts_j = new_R[main_cam2].transpose() * (w_pts_i - new_P[main_cam2]);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

//针对初始化阶段,切换地图点MP的参考帧和深度
//fix：为什么没有切换深度值?
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}
//算是ok无需fix了?
void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)//因为是保留最新帧，所以最新帧的观测保留，而最新帧序号会--, 只需要参考帧序号--即可，深度值不变.
        {
            it->start_frame--;
        }
        else//起始帧早于或者等于次新帧.
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)//起始帧早于次新帧，并且结束帧也早于次新帧,那么不用管
                continue;
            //起始帧早于次新帧，并且结束帧包含次新帧,那么直接把次新帧观测删除:注意,起始帧刚好是次新帧，深度值要切换不?
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
            }else{
                if(it->start_frame == frame_count - 1){//如果起始帧就是次新帧，那么把深度清空!
                    if(false){
                        it->estimated_depth = -1;
                    }
                }
            }
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    if(frame_i.main_cam != frame_j.main_cam) return ans;//要求是同一个主相机，才能计算
    // Vector3d p_j = frame_j.point;
    Vector3d p_j = frame_j.point[frame_j.main_cam];

    double u_j = p_j(0);
    double v_j = p_j(1);

    // Vector3d p_i = frame_i.point;
    Vector3d p_i = frame_i.point[frame_i.main_cam];
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}