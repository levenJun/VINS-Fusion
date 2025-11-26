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

#include "feature_tracker.h"
#include <opencv2/imgproc/types_c.h>

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    // hasPrediction = false;
    for (int cid = 0; cid < NUM_CAM; cid++)
    {
        hasPrediction[cid] = false;
    }
}

// void FeatureTracker::setMask()
void FeatureTracker::setMask(int cid)
{
    vTrackInfoMono[cid].mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < vTrackInfoMono[cid].cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(vTrackInfoMono[cid].track_cnt[i], make_pair(vTrackInfoMono[cid].cur_pts[i], vTrackInfoMono[cid].ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    vTrackInfoMono[cid].cur_pts.clear();
    vTrackInfoMono[cid].ids.clear();
    vTrackInfoMono[cid].track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (vTrackInfoMono[cid].mask.at<uchar>(it.second.first) == 255)
        {
            vTrackInfoMono[cid].cur_pts.push_back(it.second.first);
            vTrackInfoMono[cid].ids.push_back(it.second.second);
            vTrackInfoMono[cid].track_cnt.push_back(it.first);
            cv::circle(vTrackInfoMono[cid].mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

bool FeatureTracker::splitBlockGoodFeaturesToTrack(const cv::Mat& cur_img, const int num_curpts, std::vector<cv::Point2f>& new_pts, const int num_max, const int min_dist, cv::Mat& mask){
    new_pts.clear();
    int need_cnt = num_max - num_curpts;  // 还需提取的特征点数量
    if (need_cnt <= 0) return true;
    assert(BLOCK_NUM >= 1 && (BLOCK_NUM % 2 == 1));

    int img_h = cur_img.rows;
    int img_w = cur_img.cols;
    int block_w = img_w / BLOCK_NUM;          // 单块宽度（基础值）
    int block_h = img_h / BLOCK_NUM;          // 单块高度（基础值）
 
    // 计算每个块的基础提点数量 + 余数分配
    int per_block_base = need_cnt / (BLOCK_NUM * BLOCK_NUM);
    int remainder = need_cnt % (BLOCK_NUM * BLOCK_NUM);
 
    const int blockHalf = BLOCK_NUM / 2;
    for (int cid = 0; cid <= blockHalf; cid++)
    for (int cflag = -1; cflag <= 1; cflag+=2)
    {
        if(cid == 0 && cflag != -1) continue;
        int i = blockHalf + cid * cflag;            //1,0,2
        for (int rid = 0; rid <= blockHalf; rid++)
        for (int rflag = -1; rflag <= 1; rflag+=2)
        {
            if(rid == 0 && rflag != -1) continue;
            int j = blockHalf + rid * rflag;        //1,0,2

            // std::cout << "splitBlockGoodFeaturesToTrack try, i=," << i << ",j=," << j << std::endl;

            // 1. 计算当前块的ROI（处理边界：最后一行/列包含剩余像素）
            int x_start = j * block_w;
            int y_start = i * block_h;
            int cur_block_w = (j == BLOCK_NUM-1) ? (img_w - x_start) : block_w;
            int cur_block_h = (i == BLOCK_NUM-1) ? (img_h - y_start) : block_h;
            cv::Rect block_roi(x_start, y_start, cur_block_w, cur_block_h);
            
            // 跳过无效ROI（如空块）
            if (cur_block_w <=0 || cur_block_h <=0) continue;
 
            // 2. 计算当前块的目标提点数量（余数优先分配给前N个块）
            int block_target = per_block_base;
            if(remainder > 0){
                if(remainder >= 2){
                    block_target += 2;
                    remainder -= 2;
                }else{
                    block_target++;
                    remainder--;
                }
            }
            if (block_target <=0 ) continue;
 
            cv::Mat block_img = cur_img(block_roi);  // 取当前块的图像ROI            
            // 3. 提取当前块的mask子区域（避免与已有点重复）
            cv::Mat sub_mask = mask(block_roi);
 
            // 4. 块内独立提取特征点
            std::vector<cv::Point2f> block_pts;
            cv::goodFeaturesToTrack(
                block_img, block_pts, block_target, 
                0.01, min_dist, sub_mask  // 保持原算法的质量阈值和距离约束
            );
            // 5. 转换块内点坐标到全图坐标（关键！）
            for (auto& pt : block_pts) {
                pt.x += x_start;  // 块内x → 全图x
                pt.y += y_start;  // 块内y → 全图y
            }

            new_pts.insert(new_pts.end(), block_pts.begin(), block_pts.end());
        }
    }

    std::cout << "splitBlockGoodFeaturesToTrack, need_cnt=," << need_cnt << ",new_pts.size=," << new_pts.size() << ",diffsize=," << (need_cnt - new_pts.size()) << std::endl;
    return true;
};

// map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc mTicTocMetric;
    TicToc mTicTocLKLeftOnce;
    TicToc mTicTocLKLeftTwice;
    TicToc mTicTocGFTTLeft;
    TicToc mTicTocLKRightTwice;

    TicToc t_r;
    cur_time = _cur_time;

    //所有目,单独追踪,尝试补点,只是记录补点px，不正式补点
    //所有目,尝试双目匹配
    //开始汇总补点
    //开始去畸变
    //开始计算速度之类的操作
    //cur到pre的转移操作
    //准备返回数据
    for (int cid = 0; cid < NUM_CAM; cid++)
    {
        //所有目,单独追踪,尝试补点,只是记录补点px，不正式补点
        vTrackInfoMono[cid].cur_img = cid == 0? _img : _img1;
        row = vTrackInfoMono[cid].cur_img.rows;
        col = vTrackInfoMono[cid].cur_img.cols;
        /*    
        cv::Mat rightImg = cid == 0? _img1 : cv::Mat();
        {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe->apply(cur_img, cur_img);
            if(!rightImg.empty())
                clahe->apply(rightImg, rightImg);
        }
        */
        vTrackInfoMono[cid].cur_pts.clear();

        mTicTocLKLeftOnce.tic();
        mTicTocLKLeftTwice.tic();
        if (vTrackInfoMono[cid].prev_pts.size() > 0)
        {
            TicToc t_o;
            vector<uchar> status;
            vector<float> err;
            if(hasPrediction[cid])
            {
                vTrackInfoMono[cid].cur_pts = vTrackInfoMono[cid].predict_pts;
                cv::calcOpticalFlowPyrLK(vTrackInfoMono[cid].prev_img, vTrackInfoMono[cid].cur_img, vTrackInfoMono[cid].prev_pts, vTrackInfoMono[cid].cur_pts, status, err, cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                
                int succ_num = 0;
                for (size_t i = 0; i < status.size(); i++)
                {
                    if (status[i])
                        succ_num++;
                }
                if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(vTrackInfoMono[cid].prev_img, vTrackInfoMono[cid].cur_img, vTrackInfoMono[cid].prev_pts, vTrackInfoMono[cid].cur_pts, status, err, cv::Size(21, 21), 3);
            }
            else
                cv::calcOpticalFlowPyrLK(vTrackInfoMono[cid].prev_img, vTrackInfoMono[cid].cur_img, vTrackInfoMono[cid].prev_pts, vTrackInfoMono[cid].cur_pts, status, err, cv::Size(21, 21), 3);

            mMetricStatistic.timeLKLeftOnce += mTicTocLKLeftOnce.tocMs();
            // reverse check
            if(FLOW_BACK)
            {
                vector<uchar> reverse_status;
                vector<cv::Point2f> reverse_pts = vTrackInfoMono[cid].prev_pts;
                cv::calcOpticalFlowPyrLK(vTrackInfoMono[cid].cur_img, vTrackInfoMono[cid].prev_img, vTrackInfoMono[cid].cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && reverse_status[i] && distance(vTrackInfoMono[cid].prev_pts[i], reverse_pts[i]) <= 0.5)
                    {
                        status[i] = 1;
                    }
                    else
                        status[i] = 0;
                }
            }
            mMetricStatistic.timeLKLeftTwice += mTicTocLKLeftTwice.tocMs();
            
            for (int i = 0; i < int(vTrackInfoMono[cid].cur_pts.size()); i++)
                if (status[i] && !inBorder(vTrackInfoMono[cid].cur_pts[i]))
                    status[i] = 0;
            reduceVector(vTrackInfoMono[cid].prev_pts, status);
            reduceVector(vTrackInfoMono[cid].cur_pts, status);
            reduceVector(vTrackInfoMono[cid].ids, status);
            reduceVector(vTrackInfoMono[cid].track_cnt, status);
            ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
            //printf("track cnt %d\n", (int)ids.size());
        }

        for (auto &n : vTrackInfoMono[cid].track_cnt)
            n++;

        if(cid == 0){
            mMetricStatistic.fNumLkPreLeft = vTrackInfoMono[cid].cur_pts.size();
            mMetricStatistic.fNumLkPreAll = 0;
        }
        mMetricStatistic.fNumLkPreAll += vTrackInfoMono[cid].cur_pts.size();

        mTicTocGFTTLeft.tic();
        if (1)
        {
            //rejectWithF();
            ROS_DEBUG("set mask begins");
            TicToc t_m;
            setMask(cid);   //fix,这里也会有临近点剔除策略，这里不要做临近点剔除
            ROS_DEBUG("set mask costs %fms", t_m.toc());

            ROS_DEBUG("detect feature begins");
            TicToc t_t;
            int n_max_cnt = MAX_CNT - static_cast<int>(vTrackInfoMono[cid].cur_pts.size());
            // if (n_max_cnt > 0)
            if (n_max_cnt > 0 && n_max_cnt > MAX_CNT*0.1)
            {
                if(vTrackInfoMono[cid].mask.empty())
                    cout << "mask is empty " << endl;
                if (vTrackInfoMono[cid].mask.type() != CV_8UC1)
                    cout << "mask type wrong " << endl;
                // cv::goodFeaturesToTrack(vTrackInfoMono[cid].cur_img, vTrackInfoMono[cid].n_pts, MAX_CNT - vTrackInfoMono[cid].cur_pts.size(), 0.01, MIN_DIST, vTrackInfoMono[cid].mask);
                if(!splitBlockGoodFeaturesToTrack(vTrackInfoMono[cid].cur_img, vTrackInfoMono[cid].cur_pts.size(), vTrackInfoMono[cid].n_pts, MAX_CNT, MIN_DIST, vTrackInfoMono[cid].mask)){
                    vTrackInfoMono[cid].n_pts.clear();
                }
            }
            else
                vTrackInfoMono[cid].n_pts.clear();


            if(false)//测试GFTT提取耗时
            {
                TicToc mTicTocTest;
                vector<cv::Point2f> n_ptsTest;
                cv::Mat maskTest = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));
                int MIN_DIST_Test = 1;
                cv::goodFeaturesToTrack(vTrackInfoMono[cid].cur_img, n_ptsTest, MAX_CNT, 0.01, MIN_DIST_Test, maskTest);
                mMetricStatistic.timeGFTTLeftTestOnce = mTicTocTest.tocMs();
            }

            ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

            //printf("feature cnt after add %d\n", (int)ids.size());
        }
        mMetricStatistic.timeGFTTLeft += mTicTocGFTTLeft.tocMs();
    }

    for (int cid = 0; cid < NUM_CAM; cid++){
        if(cid != 0) continue;//就左目直接补点,其它目延迟补点
        for (auto &p : vTrackInfoMono[cid].n_pts)
        {
            vTrackInfoMono[cid].cur_pts.push_back(p);
            vTrackInfoMono[cid].ids.push_back(n_id++);
            vTrackInfoMono[cid].track_cnt.push_back(1);
        }
    }

    mTicTocLKRightTwice.tic();
    //所有目,尝试双目匹配
    for (int cid = 0; cid < NUM_CAM; cid++){
        if(!stereo_cam) continue;
        if(cid != 0) continue;      //目前只对左目作双目匹配
        cv::Mat rightImg = cid == 0? vTrackInfoMono[1].cur_img : vTrackInfoMono[0].cur_img;

        if(!rightImg.empty() && stereo_cam)
        {
            vTrackInfoMono[cid].ids_right.clear();
            vTrackInfoMono[cid].cur_right_pts.clear();
            vTrackInfoMono[cid].cur_un_right_pts.clear();
            vTrackInfoMono[cid].right_pts_velocity.clear();
            vTrackInfoMono[cid].cur_un_right_pts_map.clear();
            if(!vTrackInfoMono[cid].cur_pts.empty())
            {
                //printf("stereo image; track feature on right image\n");
                vector<cv::Point2f> reverseLeftPts;
                vector<uchar> status, statusRightLeft;
                vector<float> err;
                // cur left ---- cur right
                cv::calcOpticalFlowPyrLK(vTrackInfoMono[cid].cur_img, rightImg, vTrackInfoMono[cid].cur_pts, vTrackInfoMono[cid].cur_right_pts, status, err, cv::Size(21, 21), 3);
                // reverse check cur right ---- cur left
                if(FLOW_BACK)
                {
                    cv::calcOpticalFlowPyrLK(rightImg, vTrackInfoMono[cid].cur_img, vTrackInfoMono[cid].cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                    for(size_t i = 0; i < status.size(); i++)
                    {
                        if(status[i] && statusRightLeft[i] && inBorder(vTrackInfoMono[cid].cur_right_pts[i]) && distance(vTrackInfoMono[cid].cur_pts[i], reverseLeftPts[i]) <= 0.5)
                            status[i] = 1;
                        else
                            status[i] = 0;
                    }
                }

                vTrackInfoMono[cid].ids_right = vTrackInfoMono[cid].ids;
                reduceVector(vTrackInfoMono[cid].cur_right_pts, status);
                reduceVector(vTrackInfoMono[cid].ids_right, status);

                // mMetricStatistic.fNumLkStereo = vTrackInfoMono[cid].cur_right_pts.size();
                // only keep left-right pts
                /*
                reduceVector(cur_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(cur_un_pts, status);
                reduceVector(pts_velocity, status);
                */
                // cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
                // right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
            }
            // prev_un_right_pts_map = cur_un_right_pts_map;
        }
    }
    mMetricStatistic.timeLKRightTwice = mTicTocLKRightTwice.tocMs();

    mMetricStatistic.fNumLkStereo = vTrackInfoMono[0].cur_right_pts.size();
    //开始汇总补点
    for (int cid = 0; cid < NUM_CAM; cid++){
        if(cid == 0) continue;//就左目直接补点,其它目延迟补点

        //fix:新提点+双目匹配补点
        for (auto &p : vTrackInfoMono[cid].n_pts)
        {
            vTrackInfoMono[cid].cur_pts.push_back(p);
            vTrackInfoMono[cid].ids.push_back(n_id++);
            vTrackInfoMono[cid].track_cnt.push_back(1);
        }
    }

    //开始去畸变
    //开始计算速度之类的操作    
    for (int cid = 0; cid < NUM_CAM; cid++){
        vTrackInfoMono[cid].cur_un_pts = undistortedPts(vTrackInfoMono[cid].cur_pts, m_camera[cid]);
        vTrackInfoMono[cid].pts_velocity = ptsVelocity(vTrackInfoMono[cid].ids, vTrackInfoMono[cid].cur_un_pts, vTrackInfoMono[cid].cur_un_pts_map, vTrackInfoMono[cid].prev_un_pts_map);
        if(cid != 0) continue;//目前只有左目会双目匹配
        vTrackInfoMono[cid].cur_un_right_pts = undistortedPts(vTrackInfoMono[cid].cur_right_pts, m_camera[cid == 0? 1:0]);
        vTrackInfoMono[cid].right_pts_velocity = ptsVelocity(vTrackInfoMono[cid].ids_right, vTrackInfoMono[cid].cur_un_right_pts, vTrackInfoMono[cid].cur_un_right_pts_map, vTrackInfoMono[cid].prev_un_right_pts_map);
    }

    mMetricStatistic.timeTrackAll = mTicTocMetric.tocMs();

    if(SHOW_TRACK){
        cv::Mat rightImg = stereo_cam? vTrackInfoMono[1].cur_img : cv::Mat();
        // drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
        drawTrack(vTrackInfoMono[0].cur_img, rightImg, vTrackInfoMono[0].ids, vTrackInfoMono[0].cur_pts, vTrackInfoMono[0].cur_right_pts, vTrackInfoMono[0].prevLeftPtsMap);

        if(NUM_CAM > 1){
            for (int cid = 1; cid < NUM_CAM; cid++)
            {
                cv::Mat imTrackMono;
                drawTrackMono(cid, vTrackInfoMono[cid].cur_img, vTrackInfoMono[cid].ids, vTrackInfoMono[cid].cur_pts, vTrackInfoMono[cid].prevLeftPtsMap, imTrackMono);
                cv::hconcat(imTrack, imTrackMono, imTrack);
            }
        }
    }

    //cur到pre的转移操作
    //准备返回数据
    std::vector<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> trackedResult;
    trackedResult.resize(NUM_CAM);

    prev_time = cur_time;
    for (int cid = 0; cid < NUM_CAM; cid++)
    {
        vTrackInfoMono[cid].prev_img = vTrackInfoMono[cid].cur_img;
        vTrackInfoMono[cid].prev_pts = vTrackInfoMono[cid].cur_pts;
        vTrackInfoMono[cid].prev_un_pts = vTrackInfoMono[cid].cur_un_pts;
        vTrackInfoMono[cid].prev_un_pts_map = vTrackInfoMono[cid].cur_un_pts_map;

        hasPrediction[cid] = false;

        vTrackInfoMono[cid].prevLeftPtsMap.clear();
        for(size_t i = 0; i < vTrackInfoMono[cid].cur_pts.size(); i++)
            vTrackInfoMono[cid].prevLeftPtsMap[vTrackInfoMono[cid].ids[i]] = vTrackInfoMono[cid].cur_pts[i];
        
        //当前帧特征追踪匹配结果[fid][cid](特征信息)
        //featureFrame[id1][i].first是本帧的追踪到的特征所属相机cid:有0和1的双目id
        //featureFrame[id1][i].second是本帧的追踪到的特征 像素px等信息        
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& featureFrame = trackedResult[cid];
        featureFrame.clear();
        for (size_t i = 0; i < vTrackInfoMono[cid].ids.size(); i++)
        {
            int feature_id = vTrackInfoMono[cid].ids[i];
            double x, y ,z;
            x = vTrackInfoMono[cid].cur_un_pts[i].x;
            y = vTrackInfoMono[cid].cur_un_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = vTrackInfoMono[cid].cur_pts[i].x;
            p_v = vTrackInfoMono[cid].cur_pts[i].y;
            int camera_id = cid;
            double velocity_x, velocity_y;
            velocity_x = vTrackInfoMono[cid].pts_velocity[i].x;
            velocity_y = vTrackInfoMono[cid].pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }

        if(cid != 0) continue;//只有左目有双目匹配
        int rightCid = cid == 0? 1 : 0;
        if (!vTrackInfoMono[rightCid].cur_img.empty() && stereo_cam)
        {
            for (size_t i = 0; i < vTrackInfoMono[cid].ids_right.size(); i++)
            {
                int feature_id = vTrackInfoMono[cid].ids_right[i];
                double x, y ,z;
                x = vTrackInfoMono[cid].cur_un_right_pts[i].x;
                y = vTrackInfoMono[cid].cur_un_right_pts[i].y;
                z = 1;
                double p_u, p_v;
                p_u = vTrackInfoMono[cid].cur_right_pts[i].x;
                p_v = vTrackInfoMono[cid].cur_right_pts[i].y;
                int camera_id = rightCid;
                double velocity_x, velocity_y;
                velocity_x = vTrackInfoMono[cid].right_pts_velocity[i].x;
                velocity_y = vTrackInfoMono[cid].right_pts_velocity[i].y;

                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
        }

    }
    //printf("feature track whole time %f\n", t_r.toc());
    return trackedResult;
}

void FeatureTracker::rejectWithF(int cid)
{
    if (vTrackInfoMono[cid].cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(vTrackInfoMono[cid].cur_pts.size()), un_prev_pts(vTrackInfoMono[cid].prev_pts.size());
        for (unsigned int i = 0; i < vTrackInfoMono[cid].cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[cid]->liftProjective(Eigen::Vector2d(vTrackInfoMono[cid].cur_pts[i].x, vTrackInfoMono[cid].cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[cid]->liftProjective(Eigen::Vector2d(vTrackInfoMono[cid].prev_pts[i].x, vTrackInfoMono[cid].prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = vTrackInfoMono[cid].cur_pts.size();
        reduceVector(vTrackInfoMono[cid].prev_pts, status);
        reduceVector(vTrackInfoMono[cid].cur_pts, status);
        reduceVector(vTrackInfoMono[cid].cur_un_pts, status);
        reduceVector(vTrackInfoMono[cid].ids, status);
        reduceVector(vTrackInfoMono[cid].track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, vTrackInfoMono[cid].cur_pts.size(), 1.0 * vTrackInfoMono[cid].cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    // if (calib_file.size() == 2)
    if (calib_file.size() >= 2)    
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
{
    int cid = 0;
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[cid]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = vTrackInfoMono[cid].cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        // for (unsigned int i = 0; i < cur_pts.size(); i++)//fix:这里是bug or not?
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    int cid = 0;
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * vTrackInfoMono[cid].track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);//curLeftPts:左目追踪超过20帧的涂红，不到20帧的越多越接近红，越少越接近蓝
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)//curRightPts:右目特征，直接涂绿色
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);//curLeftPts:左目还绘制前后帧同一个特征的连线，作为绿色线
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

void FeatureTracker::drawTrackMono(const int cid, const cv::Mat &imLeft, 
                                    vector<int> &curLeftIds, vector<cv::Point2f> &curLeftPts, map<int, cv::Point2f> &prevLeftPtsMap,
                                cv::Mat &imOut)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    imOut = imLeft.clone();
        
    cv::cvtColor(imOut, imOut, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * vTrackInfoMono[cid].track_cnt[j] / 20);
        cv::circle(imOut, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);//curLeftPts:左目追踪超过20帧的涂红，不到20帧的越多越接近红，越少越接近蓝
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imOut, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);//curLeftPts:左目还绘制前后帧同一个特征的连线，作为绿色线
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

void FeatureTracker::setPrediction(int cid, map<int, Eigen::Vector3d> &predictPts)
{
    // hasPrediction = true;
    // int cid = 0;
    hasPrediction[cid] = true;
    vTrackInfoMono[cid].predict_pts.clear();
    vTrackInfoMono[cid].predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < vTrackInfoMono[cid].ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = vTrackInfoMono[cid].ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[cid]->spaceToPlane(itPredict->second, tmp_uv);
            vTrackInfoMono[cid].predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            vTrackInfoMono[cid].predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            vTrackInfoMono[cid].predict_pts.push_back(vTrackInfoMono[cid].prev_pts[i]);
    }
}

//默认是全局id,不区分是哪个相机的,所以要遍历所有相机
std::vector<int> FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::vector<int> inliers;
    inliers.resize(NUM_CAM);
    for (int cid = 0; cid < NUM_CAM; cid++)
    {
        std::set<int>::iterator itSet;
        vector<uchar> status;
        for (size_t i = 0; i < vTrackInfoMono[cid].ids.size(); i++)
        {
            itSet = removePtsIds.find(vTrackInfoMono[cid].ids[i]);
            if(itSet != removePtsIds.end())
                status.push_back(0);
            else
                status.push_back(1);
        }

        reduceVector(vTrackInfoMono[cid].prev_pts, status);
        reduceVector(vTrackInfoMono[cid].ids, status);
        reduceVector(vTrackInfoMono[cid].track_cnt, status);
        inliers[cid] = vTrackInfoMono[cid].prev_pts.size();
    }
    return inliers;
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}