#include "HelperOpencv.h"
#include <random>

namespace ORB_SLAM3{


bool HelperOpencv::DrawKeypoints(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts, cv::Mat& output, const int oriSize){
    // 创建一个输出图像，用于绘制匹配结果
    if(img.empty())
    {
        return false;
    } 
    // cv::Mat output;
    if(img.channels() < 3){
        cvtColor(img, output, cv::COLOR_GRAY2BGR);
    }else{
        output = img.clone();
    }
    int oriIdxMax = kpts.size() - 1;
    if(oriSize >= 0){
        oriIdxMax = oriSize - 1;
    }
    for (int idx = 0, kptSize = kpts.size(); idx < kptSize; idx++){
        if(idx <= oriIdxMax){
            cv::circle(output, kpts[idx].pt, 2, cv::Scalar(0, 255, 0),-1);
        }else{
            cv::circle(output, kpts[idx].pt, 2, cv::Scalar(0, 0, 255),-1);
        }        
    }
    return true;
};


bool HelperOpencv::DrawMatches(std::string show_name,const cv::Mat& img1, const cv::Mat& img2, 
                        std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, 
                        const std::vector<cv::DMatch>& matches,
                        cv::Mat& output, bool showIdx)
{

    // 创建一个输出图像，用于绘制匹配结果
    if(img1.empty()||img2.empty())
    {
        return false;
    }

    int kptSize = matches.size();
    std::vector<cv::KeyPoint> kptX1, kptX2;
    std::vector<cv::DMatch> matchesX;
    kptX1.reserve(kptSize);
    kptX2.reserve(kptSize);
    matchesX.reserve(kptSize);
    for (int idx = 0; idx < kptSize; idx++)
    {
        int pid1 = matches[idx].queryIdx;
        int pid2 = matches[idx].trainIdx;
        cv::DMatch cMatch = matches[idx];
        cMatch.queryIdx = idx;
        cMatch.trainIdx = idx;

        kptX1.push_back(keypoints1[pid1]);
        kptX2.push_back(keypoints2[pid2]);
        matchesX.push_back(cMatch);
    }
    cv::drawMatches(img1, kptX1, img2, kptX2, matchesX, output);

    if(showIdx){
        int kptSize = matches.size();
        for (int idx = 0; idx < kptSize; idx++)
        {
            int fontFace = cv::FONT_HERSHEY_SIMPLEX; // 字体类型
            double fontScale = 0.5; // 字体大小
            int thickness = 1; // 文字线条粗细
            std::string msg = std::to_string(idx);


            int pid = matches[idx].queryIdx;

            cv::Point2f textPt = keypoints1[pid].pt;
            textPt.x += 5;
            cv::putText(output, msg, keypoints1[pid].pt, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
        }
    }

    // std::string savePath = sSaveDir + "/" + show_name + ".jpg";
    // cv::imwrite(savePath, output);

    return true;
}

void HelperOpencv::SaveMatches(std::string show_name,const cv::Mat& img1, const cv::Mat& img2, 
                    std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, 
                    const std::vector<cv::DMatch>& matches,
                    const std::string sSaveDir, bool showIdx)
{
    cv::Mat drawout;
    if(!DrawMatches(show_name, img1, img2, keypoints1, keypoints2, matches, drawout, showIdx)){
        return;
    }
    std::string savePath = sSaveDir + "/" + show_name + ".jpg";
    cv::imwrite(savePath, drawout);
};



bool HelperOpencv::DrawMatchesFline(std::string show_name,const cv::Mat& img1, const cv::Mat& img2, 
                        std::vector<cv::Point2f> keypoints1, std::vector<cv::Point2f> keypoints2, 
                        const Eigen::Matrix3f& Fmatrix,
                        cv::Mat& output,
                        const std::pair<bool, cv::Point2f> e1,//图1上的极点:{有效极点?, 极点齐次坐标}
                        bool printLog)
{

    // 创建一个输出图像，用于绘制匹配结果
    if(img1.empty()||img2.empty())
    {
        return false;
    }
    if(keypoints1.size() != keypoints2.size()){
        return false;
    }
    // cv::Mat output;
    if(img1.channels() < 3){
        cvtColor(img1, output, cv::COLOR_GRAY2BGR);
    }else{
        output = img1.clone();
    }

    // 定义一个随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    int kptSize = keypoints1.size();
    for (int idx = 0; idx < kptSize; idx++)
    {

        cv::Scalar randomColor(dis(gen), dis(gen), dis(gen));

        // cv::circle(output, keypoints1[idx], 2, cv::Scalar(255, 0, 0),-1);
        cv::circle(output, keypoints1[idx], 2, randomColor,-1);
        
        {//画一个框:10个像素边长
            const int r = 5;
            cv::Point2f pt1,pt2;
            pt1.x=keypoints1[idx].x-r;
            pt1.y=keypoints1[idx].y-r;
            pt2.x=keypoints1[idx].x+r;
            pt2.y=keypoints1[idx].y+r;
            cv::rectangle(output,pt1,pt2, cv::Scalar(255,0,0));//color:蓝色框
        }
        int fontFace = cv::FONT_HERSHEY_SIMPLEX; // 字体类型
        double fontScale = 0.5; // 字体大小
        int thickness = 2; // 文字线条粗细
        std::string msg = std::to_string(idx);
        cv::Point2f textPt = keypoints1[idx];
        textPt.x += 5;
        cv::putText(output, msg, keypoints1[idx], fontFace, fontScale, randomColor, thickness);


        Eigen::Vector3f fline = Fmatrix * Eigen::Vector3f(keypoints2[idx].x, keypoints2[idx].y, 1.0);
        cv::Point2f pt1(-fline(2)/fline(0), 0); // 极线与图像上边界的交点
        //fix:这里之前算错了
        cv::Point2f pt2( -(fline(1)*img1.rows + fline(2))/fline(0) ,img1.rows);// 极线与图像下边界的交点
        double ratio = (float)idx / kptSize;
        cv::Point2f ptMid((1-ratio)*pt1.x + ratio*pt2.x, (1-ratio)*pt1.y + ratio*pt2.y);
        // cv::Point2f ptMid(0.5*(pt1.x + pt2.x), 0.5*(pt1.y + pt2.y));

        //把点到极线距离打出来
        if(printLog)
        {
            if(fline.head<2>().norm() > 0){
                double errnorm = (fline(0) * keypoints1[idx].x + fline(1) * keypoints1[idx].y + fline(2))/fline.head<2>().norm();
                std::cout << show_name <<  ",badMatch,idx=," << idx 
                            << ",keypoints1=," << keypoints1[idx].x << "," << keypoints1[idx].y 
                            << ",keypoints2=," << keypoints2[idx].x << "," << keypoints2[idx].y 
                            << ",errnorm=," << errnorm << std::endl;
            }
            std::cout << "fline=," << fline.transpose() << ",pt1.x=," << pt1.x << ",pt2.x=," << pt2.x << std::endl;
            std::cout << "fline_norm=," << (fline/fline.head<2>().norm()).transpose() << std::endl;
        }

        // cv::line(output, pt1, pt2, cv::Scalar(0, 255, 255));// 极线颜色，这里使用黄色
        cv::line(output, pt1, pt2, randomColor);// 极线颜色，这里使用黄色
        msg = "L-" + std::to_string(idx);
        cv::Point2f textPtLine = ptMid;
        textPtLine.x += 3;
        cv::putText(output, msg, textPtLine, fontFace, fontScale, randomColor, thickness);

        if(e1.first)
        {
            //用极点e1绘制极线
            cv::line(output, keypoints1[idx], e1.second, cv::Scalar(0,0,0));// 极线颜色，这里使用黑色
        }
    }
    // std::string savePath = sSaveDir + "/" + show_name + ".jpg";
    // cv::imwrite(savePath, output);
    return true;
}

void HelperOpencv::SaveMatchesFline(std::string show_name,const cv::Mat& img1, const cv::Mat& img2, 
                        std::vector<cv::Point2f> keypoints1, std::vector<cv::Point2f> keypoints2, 
                        const Eigen::Matrix3f& Fmatrix,
                        const std::string sSaveDir,
                        const std::pair<bool, cv::Point2f> e1,//图1上的极点:{有效极点?, 极点齐次坐标}
                        bool printLog)
{
    cv::Mat drawout;
    if(!DrawMatchesFline(show_name, img1, img2, keypoints1, keypoints2, Fmatrix, drawout, e1, printLog)){
        return;
    }
    std::string savePath = sSaveDir + "/" + show_name + ".jpg";
    cv::imwrite(savePath, drawout);
}

bool HelperOpencv::SearchNeiboursByGrid(const std::vector<cv::KeyPoint>& oriKpts, const std::vector<cv::Point2f>& searchKpts, 
                                    std::vector<int>& result,
                                    const double mcGridSize, const double distMax)
{
    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> mGridMap;

    //构造ori网格
    for (int i = 0, pSize = oriKpts.size(); i < pSize; ++i) {
	    const auto& pt = oriKpts[i].pt;
        GridKey key(
            static_cast<int>(std::floor(pt.x / mcGridSize)),  
            static_cast<int>(std::floor(pt.y / mcGridSize))  
        );
        mGridMap[key].push_back(i); // 存储特征点索引
    }

    //执行搜索
    result.clear();
    int searchSize = searchKpts.size();
    if(searchSize <= 0){
        return true;
    }
    result.resize(searchSize);
    std::fill(result.begin(), result.end(), -1);

    double distMax2 = distMax*distMax;
    for (int pid = 0; pid < searchSize; pid++)
    {
        const cv::Point2f& pt = searchKpts[pid];
 
        // 计算当前点所在网格及相邻网格（3x3邻域）  
        int grid_i = static_cast<int>(std::floor(pt.x / mcGridSize));  
        int grid_j = static_cast<int>(std::floor(pt.y / mcGridSize));  
 
        double minDis = -1;
        double minIdx = -1;
        for (int di = -1; di <= 1; ++di) {//周围邻域3*3个格子
            for (int dj = -1; dj <= 1; ++dj) {
                GridKey query_key(grid_i + di, grid_j + dj);
                auto it = mGridMap.find(query_key);
                if (it == mGridMap.end()) continue;  
 
                // 检查网格内所有点，判断坐标差是否小于0.5  
                for (int kpt_idx : it->second) {  
                    const auto& kpt = oriKpts[kpt_idx].pt;

                    double dx = (kpt.x - pt.x);  
                    double dy = (kpt.y - pt.y);
                    double dxy2 = dx*dx + dy*dy;

                    if(dxy2 > distMax2) continue;
                    
                    if(minDis < 0 || dxy2 < minDis){
                        minDis = dxy2;
                        minIdx = kpt_idx;
                    }
                }  
            }  
        }

        if(minIdx >= 0 && minDis >= 0){//找到合适目标点了
            result[pid] = minIdx;
        }
    }

    return true;
};

}