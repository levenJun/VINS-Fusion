#include "UtilsDumpData.h"
#include "UtilsTime.h"      //获取当前时间字符串需要
#include "UtilsFile.h"      //创建dir需要
#include <fstream>
#include <sstream>
#include <iostream>
#include <mutex>
#include "UtilsLog.h"

namespace LevenBF{
namespace Utils{
namespace UtilsDumpData{

std::recursive_mutex gDumpMutex;
bool gDumpDataOn = false;
string gdumpfileDir;
string gLgfilePre;
string gStartupTimeStr;

string gNamePoseImgUpdateEskf = "PoseImgUpdateEskf";
string gNamePoseImgUpdateMsckf = "PoseImgUpdateMsckf";

string gNamePoseImuPredictEskf = "PoseImuPredictEskf";
string gNamePoseImuPredictMsckf = "PoseImuPredictMsckf";

bool InitUtilsDumpData(string dumpfileDir, string dumpfilePre)
{
    if(UtilsFile::CreateDir(dumpfileDir) != 0){
        return false;
    }
    gdumpfileDir = dumpfileDir;
    gLgfilePre = dumpfilePre;

    MYLOG_FI("InitUtilsDumpData init success");
    return true;
};

bool StopUtilsDumpData(){
    std::lock_guard<std::recursive_mutex> lck(gDumpMutex);
    MYLOG_FW("StopUtilsDumpData");
    gDumpDataOn = false;
    return true;
};
bool StartUtilsDumpData(){
    std::lock_guard<std::recursive_mutex> lck(gDumpMutex);
    if(true){
        ForeSwitchDumpFile();
    }
    gDumpDataOn = true;
    MYLOG_FW("StartUtilsDumpData");
    return true;    
};

void ForeSwitchDumpFile()
{
    std::lock_guard<std::recursive_mutex> lck(gDumpMutex);
    string timeYmdhms, timeYmd;
    LevenBF::Utils::UtilsTime::GetCurrentTimeStr3(timeYmdhms, timeYmd);
    gStartupTimeStr = timeYmdhms;
};

void DumpPoseImgUpdate(const PoseWithTime& eskfPose,  const PoseWithTime& msckfPose, string handName)
{
    std::lock_guard<std::recursive_mutex> lck(gDumpMutex);
    if(!gDumpDataOn){
        return;
    }

    if(eskfPose.valid)
    {
	    stringstream ss;   
        ss << LevenBF::Utils::UtilsDumpData::gdumpfileDir 
            << LevenBF::Utils::UtilsDumpData::gLgfilePre << gStartupTimeStr << "_" << handName << "_" << gNamePoseImgUpdateEskf << ".txt";        
        string fileName = ss.str();

        ss.str("");
        ss << std::fixed << eskfPose.time
            << " " << eskfPose.tx << " " << eskfPose.ty << " " << eskfPose.tz
            << " " << eskfPose.qx << " " << eskfPose.qy << " " << eskfPose.qz << " " << eskfPose.qw;
        // string msg = ss.str();

        LevenBF::Utils::UtilsDumpData::Dump2File(fileName, ss.str());
        ss.str("");
    }    

    if(msckfPose.valid)
    {
	    stringstream ss;   
        ss << LevenBF::Utils::UtilsDumpData::gdumpfileDir 
            << LevenBF::Utils::UtilsDumpData::gLgfilePre << gStartupTimeStr << "_" << handName << "_" << gNamePoseImgUpdateMsckf << ".txt";        
        string fileName = ss.str();

        ss.str("");
        ss << std::fixed << msckfPose.time
            << " " << msckfPose.tx << " " << msckfPose.ty << " " << msckfPose.tz
            << " " << msckfPose.qx << " " << msckfPose.qy << " " << msckfPose.qz << " " << msckfPose.qw;
        // string msg = ss.str();

        LevenBF::Utils::UtilsDumpData::Dump2File(fileName, ss.str());
        ss.str("");
    }    

};

void DumpPoseImuPredict(const PoseWithTime& eskfPose,  const PoseWithTime& msckfPose, string handName)
{
    std::lock_guard<std::recursive_mutex> lck(gDumpMutex);
    if(!gDumpDataOn){
        return;
    }

    if(eskfPose.valid)
    {
	    stringstream ss;   
        ss << LevenBF::Utils::UtilsDumpData::gdumpfileDir 
            << LevenBF::Utils::UtilsDumpData::gLgfilePre << gStartupTimeStr << "_" << handName << "_" << gNamePoseImuPredictEskf << ".txt";        
        string fileName = ss.str();

        ss.str("");
        ss << std::fixed << eskfPose.time
            << " " << eskfPose.tx << " " << eskfPose.ty << " " << eskfPose.tz
            << " " << eskfPose.qx << " " << eskfPose.qy << " " << eskfPose.qz << " " << eskfPose.qw;
        // string msg = ss.str();

        LevenBF::Utils::UtilsDumpData::Dump2File(fileName, ss.str());
        ss.str("");
    }    

    if(msckfPose.valid)
    {
	    stringstream ss;   
        ss << LevenBF::Utils::UtilsDumpData::gdumpfileDir 
            << LevenBF::Utils::UtilsDumpData::gLgfilePre << gStartupTimeStr << "_" << handName << "_" << gNamePoseImuPredictMsckf << ".txt";        
        string fileName = ss.str();

        ss.str("");
        ss << std::fixed << msckfPose.time
            << " " << msckfPose.tx << " " << msckfPose.ty << " " << msckfPose.tz
            << " " << msckfPose.qx << " " << msckfPose.qy << " " << msckfPose.qz << " " << msckfPose.qw;
        // string msg = ss.str();

        LevenBF::Utils::UtilsDumpData::Dump2File(fileName, ss.str());
        ss.str("");
    }    

};

void Dump2File(const string& dumpfilename, const string& msg)
{
    ofstream fout;
    fout.open(dumpfilename, ios_base::app);
    fout << msg << std::endl;
    fout << flush; 
    fout.close(); 	
};

}}}
