#pragma once
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "Eigen/Core"
#include <vector>  
#include <unordered_map>  
#include <opencv2/features2d.hpp> 
#include <mutex>
namespace MyHelpers{

class HelperOpencv
{
private:
    /* data */
public:
    HelperOpencv(/* args */){};
    ~HelperOpencv(){};

public:

    static bool DrawKeypoints(const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts, cv::Mat& output, const int oriSize = -1);

    static bool DrawMatches(std::string show_name,const cv::Mat& img1, const cv::Mat& img2, 
                        std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, 
                        const std::vector<cv::DMatch>& matches,
                        cv::Mat& output, bool showIdx = false);

    static bool DrawMatchesFline(std::string show_name,const cv::Mat& img1, const cv::Mat& img2, 
                            std::vector<cv::Point2f> keypoints1, std::vector<cv::Point2f> keypoints2, 
                            const Eigen::Matrix3f& Fmatrix,
                            cv::Mat& output,
                            const std::pair<bool, cv::Point2f> e1 = {false, cv::Point2f()},//图1上的极点:{有效极点?, 极点齐次坐标}
                            bool printLog = false);

    static void SaveMatches(std::string show_name,const cv::Mat& img1, const cv::Mat& img2, 
                        std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, 
                        const std::vector<cv::DMatch>& matches,
                        const std::string sSaveDir, bool showIdx = false);

    static void SaveMatchesFline(std::string show_name,const cv::Mat& img1, const cv::Mat& img2, 
                            std::vector<cv::Point2f> keypoints1, std::vector<cv::Point2f> keypoints2, 
                            const Eigen::Matrix3f& Fmatrix,
                            const std::string sSaveDir,
                            const std::pair<bool, cv::Point2f> e1 = {false, cv::Point2f()},//图1上的极点:{有效极点?, 极点齐次坐标}
                            bool printLog = false);

    //绘制三角面片
    static bool DrawDelaunay(cv::Mat& img, const std::vector<cv::Vec6f>& triangleList, cv::Scalar delaunayColor) {
        std::vector<cv::Point> pt(3);
        cv::Size size = img.size();
        cv::Rect rect(0, 0, size.width, size.height);

        for (int i = 0; i < triangleList.size(); i++) {
            cv::Vec6f t = triangleList[i];
            pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
            pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
            pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));

            // Draw rectangles completely inside the image.
            if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])) {
                line(img, pt[0], pt[1], delaunayColor, 1, cv::LINE_AA, 0);
                line(img, pt[1], pt[2], delaunayColor, 1, cv::LINE_AA, 0);
                line(img, pt[2], pt[0], delaunayColor, 1, cv::LINE_AA, 0);
            }
        }
        return true;
    }

    //绘制三角面片
    //img:被绘制图像
    //keys:图像中的特征点
    //triangleListByIdx:三角面片, triangleListByIdx[i]是第i个面片,triangleListByIdx[i][j]是第i个面片中的第j个顶点，单个面片顶点数固定为3
    static bool DrawDelaunay(cv::Mat& img, const std::vector<std::vector<int>>& triangleListByIdx, const std::vector<cv::KeyPoint>&  keys, cv::Scalar delaunayColor) {
        const int keysNum = keys.size();
        if(keysNum <= 0){
            std::cout << "DrawDelaunay warn 0! keysNume=," << keysNum << std::endl;
            return false;
        }
        std::vector<cv::Point> pt(3);
        cv::Size size = img.size();
        cv::Rect rect(0, 0, size.width, size.height);        
        int triId0 = -1;
        int triId1 = -1;
        int triId2 = -1;
        for (int i = 0; i < triangleListByIdx.size(); i++) {
            std::vector<int> triangleOne = triangleListByIdx[i];
            if(triangleOne.size() != 3){
                std::cout << "DrawDelaunay warn 1! triangleOne.size=" << triangleOne.size() << std::endl;
                return false;
            }
            for (int jdx = 0; jdx < 3; jdx++)
            {
                if(triangleOne[jdx] < 0 || triangleOne[jdx] >= keysNum){
                    std::cout << "DrawDelaunay warn 2! triangleOne[jdx] is invalid, i=," << i << ",jdx=," << jdx << ",triangleOne[jdx]=," << triangleOne[jdx] << std::endl;
                    return false;
                }
            }
            pt[0] = cv::Point(cvRound(keys[triangleOne[0]].pt.x), cvRound(keys[triangleOne[0]].pt.y));
            pt[1] = cv::Point(cvRound(keys[triangleOne[1]].pt.x), cvRound(keys[triangleOne[1]].pt.y));
            pt[2] = cv::Point(cvRound(keys[triangleOne[2]].pt.x), cvRound(keys[triangleOne[2]].pt.y));

            // Draw rectangles completely inside the image.
            if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])) {
                cv::line(img, pt[0], pt[1], delaunayColor, 1, cv::LINE_AA);
                cv::line(img, pt[1], pt[2], delaunayColor, 1, cv::LINE_AA);
                cv::line(img, pt[2], pt[0], delaunayColor, 1, cv::LINE_AA);
                cv::circle(img, pt[0], 2, cv::Scalar(0, 255, 0));
                cv::circle(img, pt[1], 2, cv::Scalar(0, 255, 0));
                cv::circle(img, pt[2], 2, cv::Scalar(0, 255, 0));
            }else{
                std::cout << "DrawDelaunay warn 3! triangleOne px is invalid, i=," << i << ",pt[0]=," << pt[0].x << "," << pt[0].y 
                            << ",pt[1]=," << pt[1].x << "," << pt[1].y 
                            << ",pt[2]=," << pt[2].x << "," << pt[2].y 
                            << std::endl;
            }
        }
        return true;
    }


    // 网格键类型（网格坐标）及哈希函数  
    using GridKey = std::pair<int, int>;  
    struct GridKeyHash {
        size_t operator()(const GridKey& k) const {  
            return (static_cast<size_t>(k.first) << 32) | k.second; // 组合哈希  
        }
    };

    //按输入oriKpts的坐标构建2d网格
    //searchKpts作为目标，去网格搜索距离distMax以内且最近的网格点
    //result和searchKpts大小一致，result[i]是第i个点搜索到的结果,如果result[i]=-1表示没有搜索近距离点
    //mcGridSize:构建网格的大小
    //distMax:邻居点必须小于这个阈值
    static bool SearchNeiboursByGrid(const std::vector<cv::KeyPoint>& oriKpts, const std::vector<cv::Point2f>& searchKpts, 
                                        std::vector<int>& result, 
                                        const double mcGridSize = 1.0, const double distMax = 0.5);
};

}