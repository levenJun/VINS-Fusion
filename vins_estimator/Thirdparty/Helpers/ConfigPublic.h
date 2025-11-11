/*** 
 * @Author: leven
 * @LastEditors: leven
 * @Description: 一些公共配置
 */
#pragma once

#include <string>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <vector>
using namespace std;

namespace ConfigPublic
{

    static const std::string gOutPutDirBase = "./output/";

    // https://blog.csdn.net/qq_30541261/article/details/71440001
    static const std::vector<std::vector<float>> gColorList =   //下面的颜色只能在后面叠加,不能删除和中间插入
    {
        {0.0,0.0,0.0},  //黑色:
        {1.0,0.0,0.0},  //红色
        {1.0,0.55,0.0}, //深橙色            RGB(255,140,0)
        {1.0,0.85,0.0}, //金色              RGB(255,215,0)
        {0.75,0.05,0.92},//马鞍棕偏紫        RGB(192,14,235)
        {0.0,0.75,1.0}, //深天蓝            RGB(0,191,255)
        {0.2,0.8,0.2},   //柠檬绿           RGB(50,205,50)

        {0.5,0.0,0.5},  //紫色
        {1.0,0.65,0.0}, //橙色              RGB(255,165,0)
        {1.0,1.0,0.0}   //黄色
    };

    extern std::pair<long, std::string> gStartupTime ;  // = {-1, ""} 在程序开启后的启动时间:{毫秒,年月日时分秒字符串}
    extern std::pair<bool, std::string> gOutPutDirMatchStereo ;// = {true, ""} 输出数据的dir:{是否输出?,对应的dir}
    extern std::pair<bool, std::string> gOutPutDirMatchTrack ;// = {true, ""} 输出数据的dir:{是否输出?,对应的dir}



    /*** 
     * @description: 开机时,执行某些初始化操作
     */
    class ConfigPublicStartuper
    {
    public:
        ConfigPublicStartuper(){
            OnConfigPublicStartup();
        };
        virtual ~ConfigPublicStartuper(){};
    protected:
        void OnConfigPublicStartup();//具体的初始化执行逻辑
    };

    extern ConfigPublicStartuper gConfigPublicStartuper;
};