#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <chrono>   // 新增：时间相关头文件
#include <ctime>    // 新增：时间格式化
#include <iomanip>  // 新增：时间格式化
#include <sstream>  // 新增：字符串流
#include "HelperFile.hpp"
#include "HelperTime.h"
// namespace fs = std::filesystem;
namespace MyHelpers{
class HelperDataSaver {
private:
    std::string root_dir_;          // 根目录（包含时间子目录）
    bool is_saving_ = false;        // 保存状态标志
    std::ofstream imu_file_;        // IMU文件流
    std::ofstream pose_file_;       // Pose文件流
    std::ofstream posekey_file_;    // Pose文件流
    std::mutex imu_mutex_;          // IMU写入互斥锁
    std::mutex pose_mutex_;         // Pose写入互斥锁
    std::mutex posekey_mutex_;      // Pose写入互斥锁
    std::mutex image_mutex_;        // 图像写入互斥锁

    // 创建目录（若不存在）
    bool createDirectory(const std::string& path) {
        if(HelperFile::DirectoryExists(path)){
            return true;
        }
        if(HelperFile::CreateDir(path) != 0){
            return false;
        }
        return true;
    }

    // 新增：生成当前时间字符串（格式：YYYYMMDDHHMMSS）
    std::string getCurrentTimeString() {
        return HelperTime::GetCurrentTimeStr();
    }

public:
    HelperDataSaver() = default;
    ~HelperDataSaver() { stopSaving(); }

    // 修订：设置根目录，并自动创建"root/YYYYMMDDHHMMSS"子目录作为实际根目录
    void setRootDirectory(const std::string& root) {        
        std::string time_str = getCurrentTimeString();  // 获取当前时间字符串
        if (time_str.empty()) {
            std::cerr << "错误：时间字符串生成失败，无法创建子目录" << std::endl;
            root_dir_.clear();
            return;
        }

        // 拼接路径：root + "/" + 时间字符串（例如："data/20250711153045"）
        root_dir_ = root + "/" + time_str;
        std::cout << "HelperDataSaver try setRootDirectory, root_dir_=," << root_dir_ << "." << std::endl;
        // 创建包含时间子目录的根目录
        if (!createDirectory(root_dir_)) {
            std::cerr << "错误：根目录创建失败: " << root_dir_ << std::endl;
            root_dir_.clear();  // 标记路径无效
        }
        else {
            std::cout << "根目录已设置: " << root_dir_ << std::endl;
        }
    }

    // 开启数据保存（创建cam0、cam1子目录和文件）
    bool startSaving() {
        if (is_saving_) {
            std::cerr << "错误：已处于保存状态" << std::endl;
            return false;
        }
        if (root_dir_.empty()) {
            std::cerr << "错误：未设置有效根目录（请先调用setRootDirectory）" << std::endl;
            return false;
        }

        // 创建相机子目录（cam0、cam1）
        if (!createDirectory(root_dir_ + "/cam0") || !createDirectory(root_dir_ + "/cam1")) {
            return false;
        }

        // 打开IMU文件（覆盖模式）
        imu_file_.open(root_dir_ + "/imu0.csv", std::ios::out | std::ios::trunc);
        if (!imu_file_.is_open()) {
            std::cerr << "IMU文件打开失败: " << root_dir_ << "/imu0.csv" << std::endl;
            return false;
        }

        // 打开Pose文件（覆盖模式）
        pose_file_.open(root_dir_ + "/pose.csv", std::ios::out | std::ios::trunc);
        if (!pose_file_.is_open()) {
            std::cerr << "Pose文件打开失败: " << root_dir_ << "/pose.csv" << std::endl;
            imu_file_.close();
            return false;
        }

        // 打开KeyPose文件（覆盖模式）
        posekey_file_.open(root_dir_ + "/pose_key.csv", std::ios::out | std::ios::trunc);
        if (!posekey_file_.is_open()) {
            std::cerr << "Pose文件打开失败: " << root_dir_ << "/pose_key.csv" << std::endl;
            imu_file_.close();
            pose_file_.close();
            return false;
        }        

        is_saving_ = true;
        std::cout << "数据保存已启动，保存路径: " << root_dir_ << std::endl;
        return true;
    }

    // 停止数据保存（关闭文件流）
    void stopSaving() {
        if (!is_saving_) return;
        imu_file_.close();
        pose_file_.close();
        posekey_file_.close();
        is_saving_ = false;
        std::cout << "数据保存已停止，文件已保存至: " << root_dir_ << std::endl;
    }

    // 保存双目图像（cam0: left/left1, cam1: right/right1）
    void saveStereoImage(int cam_id, const cv::Mat& image, double timestamp) {
        if (!is_saving_) return;
        if (cam_id != 0 && cam_id != 1) {
            std::cerr << "无效相机ID: " << cam_id << "（仅支持0或1）" << std::endl;
            return;
        }
        if (image.empty()) {
            std::cerr << "警告：空图像，跳过保存" << std::endl;
            return;
        }
        long long timestamp_ns = timestamp * 1e9;
        std::lock_guard<std::mutex> lock(image_mutex_);
        std::string cam_dir = (cam_id == 0) ? "cam0" : "cam1";
        std::string filename = std::to_string(timestamp_ns) + ".png";  // 时间戳命名
        std::string path = root_dir_ + "/" + cam_dir + "/" + filename;

        if (!cv::imwrite(path, image)) {
            std::cerr << "图像保存失败: " << path << std::endl;
        }
    }

    // 保存IMU数据（格式：timestamp, ax, ay, az, gx, gy, gz）
    void saveImuData(double timestamp, const std::vector<float>& imu_data) {
        if (!is_saving_ || imu_data.size() != 6) return;
        long long timestamp_ns = timestamp * 1e9;
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_file_ << std::fixed << std::setprecision(6) << timestamp_ns << ", "
        //imu_file_ << std::fixed << static_cast<long long>(timestamp) << ", "
            << imu_data[3] << ", " << imu_data[4] << ", " << imu_data[5] << ", "
            << imu_data[0] << ", " << imu_data[1] << ", " << imu_data[2] 
             << "\n";
    }

    // 保存Pose数据（格式：timestamp, qw, qx, qy, qz, tx, ty, tz）
    //TUM 轨迹格式（时间戳 tx ty tz qx qy qz qw）
    void savePoseData(double timestamp, const float* pose_data) {
        if (pose_data == nullptr) return;

        // 从pose_data提取旋转矩阵（3x3）和位置（tx, ty, tz）
        Eigen::Matrix3d R;
        R << pose_data[0], pose_data[4], pose_data[8],
            pose_data[1], pose_data[5], pose_data[9],
            pose_data[2], pose_data[6], pose_data[10];
        Eigen::Quaterniond poseR(R);  // 旋转矩阵转四元数（确保单位化）
        poseR.normalize();
        Eigen::Vector3d poseT;
        poseT << pose_data[12], pose_data[13], pose_data[14];

        savePoseData(timestamp, poseT, poseR);
    }

    void savePoseData(double timestamp, const Eigen::Vector3d& poseT, const Eigen::Quaterniond& poseR) {
        if (!is_saving_) return;

        double tx = poseT(0), ty = poseT(1), tz = poseT(2);

        std::lock_guard<std::mutex> lock(pose_mutex_);
        //pose_file_ << std::fixed << static_cast<long long>(timestamp) << ", "
        //    << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ", "
        //    << tx << ", " << ty << ", " << tz << "\n";

        pose_file_ << std::fixed << std::setprecision(6)  // 时间戳保留6位小数（纳秒级精度）
            << timestamp << " "                // 时间戳（秒）
            << tx << " " << ty << " " << tz << " "  // 位移（米）
            << poseR.x() << " " << poseR.y() << " " << poseR.z() << " " << poseR.w()  // 四元数（qx,qy,qz,qw）
            << "\n";
    }

    void savePoseKeyData(double timestamp, const Eigen::Vector3d& poseT, const Eigen::Quaterniond& poseR) {
        if (!is_saving_) return;

        double tx = poseT(0), ty = poseT(1), tz = poseT(2);

        std::lock_guard<std::mutex> lock(posekey_mutex_);

        posekey_file_ << std::fixed << std::setprecision(6)  // 时间戳保留6位小数（纳秒级精度）
            << timestamp << " "                // 时间戳（秒）
            << tx << " " << ty << " " << tz << " "  // 位移（米）
            << poseR.x() << " " << poseR.y() << " " << poseR.z() << " " << poseR.w()  // 四元数（qx,qy,qz,qw）
            << "\n";
    }

    void clearPoseKeyData(){
        // 假设文件已打开并写入内容
        posekey_file_.close(); // 关闭文件
        // 重新打开（自动清空内容）
        posekey_file_.open(root_dir_ + "/pose_key.csv", std::ios::out | std::ios::trunc);        
        
    }

};
};