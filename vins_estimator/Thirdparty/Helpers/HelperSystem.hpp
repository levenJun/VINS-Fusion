#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sstream>
namespace ORB_SLAM3{

class HelperSystem
{
private:
    /* data */
public:
    HelperSystem(/* args */){};
    ~HelperSystem(){};

public:

    // 获取进程的总CPU时间
    static long GetTotalCpuTime() {
        std::ifstream file("/proc/stat");
        if(!file.is_open()){
            std::cerr << "Failed to open " << "/proc/stat" << std::endl;
            return -1;
        }
        std::string line;
        std::getline(file, line);
        std::istringstream iss(line);

        std::string cpu;
        long user, nice, system, idle, iowait, irq, softirq, steal;
        iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

        return user + nice + system + idle + iowait + irq + softirq + steal;
    }

    // 获取当前进程的CPU时间
    static bool GetProcessCpuTime(pid_t pid, long &utime, long &stime, double& rssKb) {
        if(pid < 0){ pid = getpid(); }
        if(pid < 0){ return false; }
        std::string statFilePath = "/proc/" + std::to_string(pid) + "/stat";
        std::ifstream statFile(statFilePath);

        if (!statFile.is_open()) {
            std::cerr << "Failed to open " << statFilePath << std::endl;
            return false;
        }

        std::string line;
        std::getline(statFile, line);
        statFile.close();

        std::istringstream iss(line);
        std::string token;
        int fieldIndex = 1;
        while (iss >> token) {
            if (fieldIndex == 14) {
                utime = std::stol(token); // 用户态时间
            } else if (fieldIndex == 15) {
                stime = std::stol(token); // 内核态时间
                // break;
            }else if(fieldIndex == 24){
                rssKb = std::stol(token);// 第24字段为rss
                break;
            }
            ++fieldIndex;
        }
        if(fieldIndex < 24){
            return false;
        }
        static long pageSize = sysconf(_SC_PAGESIZE);
        static double pageSizeKb = pageSize / 1024.0;
        rssKb *= pageSizeKb;
        return true;
    }

    // 获取当前进程的内存使用情况
    static long GetMemoryUsage(pid_t pid) {
        if(pid < 0){ pid = getpid(); }
        if(pid < 0){ return -1; }
        std::string statusFilePath = "/proc/" + std::to_string(pid) + "/status";
        std::ifstream statusFile(statusFilePath);

        if (!statusFile.is_open()) {
            std::cerr << "Failed to open " << statusFilePath << std::endl;
            return -1;
        }

        std::string line;
        while (std::getline(statusFile, line)) {
            if (line.find("VmRSS:") == 0) {
                std::istringstream iss(line);
                std::string key, value, unit;
                iss >> key >> value >> unit;
                return std::stol(value); // 返回内存使用量（单位：KB）
            }
        }
        return -1;
    }

    static bool PrintSystemStatus(){
        
        static long prevTotalCpuTime = -1;
        static long prevUtime = -1;
        static long prevStime = -1;

        pid_t pid = getpid();
        long curTotalCpuTime, curUtime, curStime;
        double memoryUsage;
        curTotalCpuTime = GetTotalCpuTime();
        if(!GetProcessCpuTime(pid, curUtime, curStime, memoryUsage)){
            curUtime = -1;
            curStime = -1;
            memoryUsage = -1;
        };
        if(curTotalCpuTime < 0 || curUtime < 0 || curStime < 0){
            std::cout << "PrintSystemStatus fail 1: cpu" << std::endl;
            return false;
        }

        if(prevTotalCpuTime < 0 || prevUtime < 0 || prevStime < 0){
            prevTotalCpuTime = curTotalCpuTime;
            prevUtime = curUtime;
            prevStime = curStime;
        }

        long processCpuTime = (curUtime - prevUtime) + (curStime - prevStime);
        long totalCpuDelta = curTotalCpuTime - prevTotalCpuTime;
        if(totalCpuDelta == 0) totalCpuDelta = 1.0;
        double cpuUsage = 100.0 * processCpuTime / totalCpuDelta;

        prevTotalCpuTime = curTotalCpuTime;
        prevUtime = curUtime;
        prevStime = curStime;

        // long memoryUsage = GetMemoryUsage(pid);
        if(memoryUsage < 0){
            std::cout << "PrintSystemStatus fail 2: memory" << std::endl;
            return false;
        }
        std::cout <<  "PrintSystemStatus cpuUsage=," << cpuUsage << ",memoryUsage=," << (memoryUsage/1024.0) << std::endl;
        return true;
    }

};

};