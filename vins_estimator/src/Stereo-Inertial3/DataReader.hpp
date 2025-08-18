#ifndef DATA_READER_HPP
#define DATA_READER_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <dirent.h>  // Linux目录遍历
#include <cstring>   // 字符串操作

// IMU数据结构（格式：time,gx,gy,gz,ax,ay,az）
struct ImuFrame {
    double mStamp;               // 时间戳 (秒)
    Eigen::Vector3f vGyro;       // 角速度 (x, y, z) [rad/s]
    Eigen::Vector3f vAcc;        // 加速度 (x, y, z) [m/s²]
};

// 带位姿的双目图像帧结构
struct PosedStereoFrame {
    int status{0};              // 0:成功, -1:无数据, -2:图像读取失败
    double stamp{0.0};          // 时间戳 (秒)
    std::vector<cv::Mat> im;    // 图像数据 (0-left, 1-right)
    std::shared_ptr<Sophus::SE3f> pose = nullptr;// 位姿数据 (无效时为默认空)
};

class DataReader {
private:
    // 内部数据结构：存储图像路径与时间戳
    struct StereoImageData {
        double stamp;           // 图像时间戳（从文件名提取）
        std::string leftPath;   // 左图路径 (cam0)
        std::string rightPath;  // 右图路径 (cam1)
    };

    struct PoseData {
        double stamp;           // 位姿时间戳
        Sophus::SE3f pose;      // 位姿数据 (SE3)
    };

    // 成员变量：m前缀+驼峰命名
    std::vector<std::vector<float>> mCameraInfo; // 双目相机内外参
    std::vector<StereoImageData> mStereoImages;  // 双目图像缓存（按时间戳排序）
    std::vector<ImuFrame> mImuFrames;            // IMU数据缓存（原始顺序，已排好序）
    std::vector<PoseData> mPoseFrames;           // 位姿数据缓存（原始顺序，已排好序）
    size_t mStereoIdx = 0;                       // 当前 stereo 读取索引
    size_t mImuIdx = 0;                          // 当前 IMU 读取索引
    const std::string mDataRoot;                 // 数据根目录路径
    bool mbLoadOk = false;                       // 标记是否正常加载数据
    bool mbPoseLoadOk = false;                   // 单独标记是否正常加载Pose文件
public:
    /** 构造函数：输入数据根目录，预加载所有数据 **/
    explicit DataReader(const std::string& dataFolder) : mDataRoot(dataFolder) {
        if(!LoadStereoImages()) return;                     // 加载双目图像路径（cam0/cam1）
        if(!LoadImuData()) return;                          // 加载IMU数据（CSV）

        if(!LoadPoseData()){
            mbPoseLoadOk = false;
            std::cout << "DataReader [WRAN], Load pose fail!" << std::endl;
        }else{
            mbPoseLoadOk = true;
        }
        
        // return;                         // 加载位姿数据（TXT，空格分隔）
        if(!LoadCameraInfo()) return;
        mbLoadOk = true;
    }

    bool IsLoadOk(){
        return mbLoadOk;
    }

    std::vector<std::vector<float>> ReadCameraInfo() {
        return mCameraInfo;
    };

    /** 读取下一帧带位姿的双目图像 **/
    PosedStereoFrame ReadStereoImage() {
        PosedStereoFrame frame;

        // 检查是否有未读取的图像
        if (mStereoIdx >= mStereoImages.size()) {
            frame.status = -1; // 无更多数据
            return frame;
        }

        // 获取当前图像数据
        const auto& imgData = mStereoImages[mStereoIdx];
        frame.stamp = imgData.stamp;
        frame.im.resize(2); // 0-left (cam0), 1-right (cam1)

        frame.im[0] = cv::imread(imgData.leftPath, cv::IMREAD_GRAYSCALE);
        frame.im[1] = cv::imread(imgData.rightPath, cv::IMREAD_GRAYSCALE);
        // 读取图像（彩色图）        
        // frame.im[0] = cv::imread(imgData.leftPath, cv::IMREAD_COLOR);
        // frame.im[1] = cv::imread(imgData.rightPath, cv::IMREAD_COLOR);

        // 检查图像是否读取成功
        if (frame.im[0].empty() || frame.im[1].empty()) {
            frame.status = -2; // 图像读取失败
            mStereoIdx++;
            return frame;
        }

        // 查找匹配的位姿（时间戳误差 < 1e-6秒，最近邻匹配）
        frame.pose = nullptr;
        const double timeEps1 = 1.e-3;
        const double timeEps2 = 2.e-3;
        if (mbPoseLoadOk && !mPoseFrames.empty()) {
            // 二分查找最接近的位姿时间戳
            auto it = std::lower_bound(mPoseFrames.begin(), mPoseFrames.end(), imgData.stamp,
                [timeEps1](const PoseData& p, double stamp) { return p.stamp < stamp - timeEps1; });
            if (it != mPoseFrames.end() && std::abs(it->stamp - imgData.stamp) < timeEps2) {
                // 找到有效位姿
                frame.pose = std::make_shared<Sophus::SE3f>();
                *frame.pose = it->pose;
            }
        }

        frame.status = 0; // 成功
        mStereoIdx++;     // 更新索引
        return frame;
    }


    /** 读取指定时间戳前的IMU数据（从上一次读取位置开始）**/
    bool ReadImus(double stamp, std::vector<ImuFrame>& imus) {
        imus.clear();
        if (mImuIdx >= mImuFrames.size()) return false;
        // std::cout << "ReadImus 1. mImuIdx=," << mImuIdx << ",stamp=," << stamp << ",mImuFrames.size=," << mImuFrames.size() 
        //             << ",front.time=," << mImuFrames.front().mStamp << ",back.time=," << mImuFrames.back().mStamp
        //             << ",mImuIdx.time=," << mImuFrames[mImuIdx].mStamp
        //             << std::endl;
        // 收集 [mImuIdx, ...] 中时间戳 <= stamp 的所有IMU数据
        size_t i = mImuIdx;
        while (i < mImuFrames.size() && mImuFrames[i].mStamp <= stamp) {
            imus.push_back(mImuFrames[i]);
            i++;
        }

        if (imus.empty()) return false;
        // std::cout << "ReadImus 2." << std::endl;
        mImuIdx = i; // 更新IMU索引到下一个未读取位置
        return true;
    }


    /** 重置stereo和IMU的读取索引（从头开始读取）**/
    void ResetStereoIdx() {
        mStereoIdx = 0;
        mImuIdx = 0;
    }


private:

    /** 辅助函数：获取目录下所有指定扩展名的文件路径 **/
    bool GetFilesInDir(const std::string& dirPath, const std::vector<std::string>& exts, 
                       std::vector<std::string>& filePaths) {
        DIR* dir = opendir(dirPath.c_str());
        if (!dir) {
            std::cerr << "[ERROR] 无法打开目录: " << dirPath << std::endl;
            return false;
        }

        dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type != DT_REG) continue; // 跳过非文件（目录、链接等）

            std::string fileName = entry->d_name;
            // 检查文件扩展名是否符合要求
            for (const auto& ext : exts) {
                if (fileName.size() >= ext.size() && 
                    fileName.substr(fileName.size() - ext.size()) == ext) {
                    filePaths.push_back(dirPath + "/" + fileName);
                    break;
                }
            }
        }
        closedir(dir);

        // 按文件名排序（假设文件名按时间戳递增命名）
        std::sort(filePaths.begin(), filePaths.end());
        return true;
    }


    /** 从文件名提取时间戳（假设格式："timestamp.ext"，如"1620000000.123.jpg"）**/
    double ExtractStampFromFilename(const std::string& fileName) {
        size_t dotPos = fileName.find_last_of('.');
        if (dotPos == std::string::npos) {
            std::cerr << "[WARN] 文件名无扩展名，无法提取时间戳: " << fileName << std::endl;
            return -1.0;
        }
        std::string stampStr = fileName.substr(0, dotPos); // 截取时间戳部分
        try {
            return std::stod(stampStr)*1.e-9; // 转换为时间戳（double）
        } catch (const std::invalid_argument& e) {
            std::cerr << "[WARN] 文件名格式错误，无法提取时间戳: " << fileName << std::endl;
            return -1.0; // 无效时间戳
        }
    }


    /** 读取cam0和cam1文件夹中的双目图像路径及时间戳 **/
    bool LoadStereoImages() {
        mStereoImages.clear();
        std::string cam0Dir = mDataRoot + "/cam0";
        std::string cam1Dir = mDataRoot + "/cam1";
        // std::vector<std::string> exts = {".jpg", ".png", ".jpeg"}; // 支持的图像扩展名
        std::vector<std::string> exts = {".png"}; // 支持的图像扩展名

        // 获取cam0和cam1中的图像文件列表,已按名字排序
        std::vector<std::string> cam0Files, cam1Files;
        bool cam0Ok = GetFilesInDir(cam0Dir, exts, cam0Files);
        bool cam1Ok = GetFilesInDir(cam1Dir, exts, cam1Files);

        if (!cam0Ok || !cam1Ok) return false; // 目录打开失败
        if (cam0Files.empty() || cam1Files.empty()) {
            std::cerr << "[WARN] cam0或cam1文件夹中无图像文件" << std::endl;
            return false;
        }
        if (cam0Files.size() != cam1Files.size()) {
            std::cerr << "[ERROR] cam0与cam1图像数量不匹配: " 
                      << cam0Files.size() << " vs " << cam1Files.size() << std::endl;
            return false;
        }

        // 提取时间戳并存储图像路径
        for (size_t i = 0; i < cam0Files.size(); ++i) {
            // 从文件名提取时间戳（文件名格式：path/to/timestamp.ext）
            std::string cam0FileName = cam0Files[i].substr(cam0Files[i].find_last_of('/') + 1);
            std::string cam1FileName = cam1Files[i].substr(cam1Files[i].find_last_of('/') + 1);
            double stamp0 = ExtractStampFromFilename(cam0FileName);//文件名是ns
            double stamp1 = ExtractStampFromFilename(cam1FileName);

            if (stamp0 < 0.0 || stamp1 < 0.0) continue; // 跳过无效时间戳
            if (std::abs(stamp0 - stamp1) > 1e-6) { // 检查左右目时间戳是否一致
                std::cerr << "[WARN] 左右目图像时间戳不匹配: " 
                          << stamp0 << " vs " << stamp1 << std::endl;
                continue;
            }

            mStereoImages.push_back({
                .stamp = stamp0,
                .leftPath = cam0Files[i],
                .rightPath = cam1Files[i]
            });
        }

        // // 按时间戳排序（确保时序严格正确）:不需要
        // std::sort(mStereoImages.begin(), mStereoImages.end(),
        //     [](const StereoImageData& a, const StereoImageData& b) {
        //         return a.stamp < b.stamp;
        //     });
        std::cout << "DataReader, Get mStereoImages size=," << mStereoImages.size() << std::endl;
        return true;
    }


    /** 读取IMU数据（CSV格式：time,gx,gy,gz,ax,ay,az，已按时间排序）**/
    bool LoadImuData() {
        const std::string imuPath = mDataRoot + "/imu0.csv";
        mImuFrames.clear();
        mImuFrames.reserve(1e3*60*10);//10分钟
        std::ifstream file(imuPath);
        if (!file.is_open()) {
            std::cerr << "[WARN] IMU文件打开失败: " << imuPath << std::endl;
            return false;
        }

        std::string line;
        std::getline(file, line); // 跳过表头（假设表头为：time,gx,gy,gz,ax,ay,az）

        std::vector<std::string> tokens;
        tokens.reserve(7);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            ImuFrame imu;
            std::string token;

            // 分割逗号，提取所有字段
            tokens.clear();
            while (std::getline(ss, token, ',')) {
                // 可选：去除字段前后的空格（如数据中存在空格分隔）
                // token.erase(0, token.find_first_not_of(" \t"));
                // token.erase(token.find_last_not_of(" \t") + 1);
                tokens.push_back(token);
            }
            if(tokens.size() > 0 &&  tokens[0] == "ts") continue;
            if(tokens.size() != 7){
                std::cout << "LoadImuData, warn, tokens.size=," << tokens.size() << std::endl;
                break;
            }

            // 解析CSV行：time,gx,gy,gz,ax,ay,az
            imu.mStamp = std::stod(tokens[0])*1.e-9;       // 时间戳
            imu.vGyro(0) = std::stof(tokens[1]);    // gx
            imu.vGyro(1) = std::stof(tokens[2]);    // gy
            imu.vGyro(2) = std::stof(tokens[3]);    // gz
            imu.vAcc(0) = std::stof(tokens[4]);     // ax
            imu.vAcc(1) = std::stof(tokens[5]);     // ay
            imu.vAcc(2) = std::stof(tokens[6]);     // az

            mImuFrames.push_back(imu);
        }
        std::cout << "DataReader, Get mImuFrames size=," << mImuFrames.size() << std::endl;
        return true;
    }


    /** 读取位姿数据（TXT格式：空格分隔，已按时间排序）**/
    bool LoadPoseData() {
        const std::string posePath = mDataRoot + "/pose.csv";
        mPoseFrames.clear();
        std::ifstream file(posePath);
        if (!file.is_open()) {
            std::cerr << "[WARN] 位姿文件打开失败（将返回无效位姿）: " << posePath << std::endl;
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            // 跳过空行或注释行
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            PoseData poseData;
            double tx, ty, tz, qx, qy, qz, qw;

            // 解析空格分隔的字段：stamp tx ty tz qx qy qz qw
            ss >> poseData.stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

            // 构造SE3位姿（四元数顺序：w为最后一位，需归一化）
            Eigen::Quaternionf q(qw, qx, qy, qz); // (w, x, y, z)
            q.normalize();
            poseData.pose = Sophus::SE3f(q, Eigen::Vector3f(tx, ty, tz));

            mPoseFrames.push_back(poseData);
        }
        mPoseFrames.pop_back();//最后一行可能无效
        std::cout << "DataReader, Get mPoseFrames size=," << mPoseFrames.size() << std::endl;
        return true;
    }

    bool LoadCameraInfo(){
        const std::string cameraInfoPath = mDataRoot + "/camera_info.csv";
        mCameraInfo.clear();
        std::ifstream file(cameraInfoPath);
        if (!file.is_open()) {
            std::cerr << "[WARN] 相机参数文件打开失败（将返回无效位姿）: " << cameraInfoPath << std::endl;
            return false;
        }
        std::string line;
        while (std::getline(file, line)){
            // 跳过空行或注释行
            if (line.empty() || line[0] == '#') continue;
            if (line[0] == 'c') continue;

            std::vector<float> res;

            std::stringstream ss(line);
            std::string token;
            for (int i = 0; i < 18; i++)
            {
                std::getline(ss, token, ',');
                res.push_back(stod(token));
            }
            mCameraInfo.push_back(res);
        }
        std::cout << "DataReader, Get mCameraInfo size=," << mCameraInfo.size() << std::endl;
        return true;
    }

};

#endif // DATA_READER_HPP