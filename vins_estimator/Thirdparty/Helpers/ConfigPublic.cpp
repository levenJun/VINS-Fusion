#include "ConfigPublic.h"
#include <sstream>

std::pair<long, std::string> ConfigPublic::gStartupTime = {-1, ""};  //  在程序开启后的启动时间:{毫秒,年月日时分秒字符串}
std::pair<bool, std::string> ConfigPublic::gOutPutDirMatchStereo = {true, ""};//  输出数据的dir:{是否输出?,对应的dir}
std::pair<bool, std::string> ConfigPublic::gOutPutDirMatchTrack = {true, ""};//  输出数据的dir:{是否输出?,对应的dir}

//启动时的一些初始化操作
void ConfigPublic::ConfigPublicStartuper::OnConfigPublicStartup(){

    //获取统一的启动时间
    auto now = std::chrono::system_clock::now();
    auto nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm *tmNow = std::localtime(&nowTime);
    std::ostringstream oss;
    oss << std::put_time(tmNow, "%Y-%m-%d_%H-%M-%S");

    ConfigPublic::gStartupTime.first = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    ConfigPublic::gStartupTime.second = oss.str();
    if(ConfigPublic::gOutPutDirMatchStereo.first){
        ConfigPublic::gOutPutDirMatchStereo.second = ConfigPublic::gOutPutDirBase + "/matchout/" + ConfigPublic::gStartupTime.second + "/stereo/";
    }
    if(ConfigPublic::gOutPutDirMatchTrack.first){
        ConfigPublic::gOutPutDirMatchTrack.second = ConfigPublic::gOutPutDirBase + "/matchout/" + ConfigPublic::gStartupTime.second + "/track/";
    }
    std::cout << "OnConfigPublicStartup gStartupTime=," << ConfigPublic::gStartupTime.first << "," << ConfigPublic::gStartupTime.second << std::endl;
    std::cout << "OnConfigPublicStartup gOutPutDirMatchStereo=," << ConfigPublic::gOutPutDirMatchStereo.first << "," << ConfigPublic::gOutPutDirMatchStereo.second << ",inst=," << &ConfigPublic::gOutPutDirMatchStereo << std::endl;

};

ConfigPublic::ConfigPublicStartuper gConfigPublicStartuper;