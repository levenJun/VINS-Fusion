#include "UtilsLog.h"
#include "UtilsTime.h"      //获取当前时间字符串需要
#include "UtilsFile.h"      //创建dir需要
#include <fstream>
#include <sstream>
#include <iostream>
#include <mutex>

bool gLogPureOn = false;
void m_log_v(string msg, string filename, string func , int line, bool log2file)//白色
{
    string head = "";
    string tail = "";
    string info;
    if(gLogPureOn){
        info = "log_v : " + msg;
    }else{
        info = "log_v " + filename + " " + func + " " + std::to_string(line) + ": " + msg;
    }
    m_log_all(info, head, tail, true, log2file, LevenBF::Utils::LogLevel::VERBOSE);
};

void m_log_d(string msg, string filename, string func , int line, bool log2file)//蓝色
{
    string head = "\033[1;34m";
    string tail = "\033[0m";
    string info;    
    if(gLogPureOn){
        info = "log_d : " + msg;
    }else{
        info = "log_d " + filename + " " + func + " " + std::to_string(line) + ": " + msg;
    }
    m_log_all(info, head, tail, true, log2file, LevenBF::Utils::LogLevel::DEBUG);
};

void m_log_i(string msg, string filename, string func , int line, bool log2file)//白色
{
    string head = "";
    string tail = "";
    string info;    
    if(gLogPureOn){
        info = "log_i : " + msg;
    }else{
        info = "log_i " + filename + " " + func + " " + std::to_string(line) + ": " + msg;
    }
    m_log_all(info, head, tail, true, log2file, LevenBF::Utils::LogLevel::INFO);
};

void m_log_w(string msg, string filename, string func , int line, bool log2file)//黄色
{
    string head = "\033[1;33m";
    string tail = "\033[0m";
    string info;    
    if(gLogPureOn){
        info = "log_w : " + msg;
    }else{
        info = "log_w " + filename + " " + func + " " + std::to_string(line) + ": " + msg;
    }
    m_log_all(info, head, tail, true, log2file, LevenBF::Utils::LogLevel::WARN);
};

void m_log_e(string msg, string filename, string func , int line, bool log2file)//红色
{
    string head = "\033[1;31m";
    string tail = "\033[0m";
    string info;    
    if(gLogPureOn){
        info = "log_e : " + msg;
    }else{
        info = "log_e " + filename + " " + func + " " + std::to_string(line) + ": " + msg;
    }
    m_log_all(info, head, tail, true, log2file, LevenBF::Utils::LogLevel::ERROR);
};

void m_log_all(const string& info, const string& head, const string& tail, bool time_on, bool log2file, LevenBF::Utils::LogLevel logLevel)
{   
    string timeYmdhms, timeYmd;
    LevenBF::Utils::UtilsTime::GetCurrentTimeStr3(timeYmdhms, timeYmd);
    LevenBF::Utils::UtilsLog::LogOut(timeYmdhms, timeYmd, info, head, tail, time_on, log2file, logLevel);
};


namespace LevenBF{
namespace Utils{
namespace UtilsLog{

std::recursive_mutex gLogMutex;
bool gLogOn = false;
bool gLogShieldFileOn = false;
string gLogfileDir;
string gLgfilePre;
string gStartupTimeStr;
LogLevel gLogLevelConsole = LogLevel::VERBOSE;
LogLevel gLogLevelFile = LogLevel::VERBOSE;

bool InitUtilsLog(string logfileDir, string logfilePre)
{
    if(UtilsFile::CreateDir(logfileDir) != 0){
        return false;
    }
    gLogfileDir = logfileDir;
    gLgfilePre = logfilePre;

    ForeSwitchLogFile();
    SetForePureMode(true);
    SetForeShieldFileMode(false);
    StartUtilsLog();

    MYLOG_FV("log model init success");
    MYLOG_FD("log model init success");        
    MYLOG_FI("log model init success");
    MYLOG_FW("log model init success");
    MYLOG_FE("log model init success");
    return true;
};

bool StopUtilsLog(){
    std::lock_guard<std::recursive_mutex> lck(LevenBF::Utils::UtilsLog::gLogMutex);
    MYLOG_FW("StopUtilsLog");
    gLogOn = false;
    return true;
};
bool StartUtilsLog(){
    std::lock_guard<std::recursive_mutex> lck(LevenBF::Utils::UtilsLog::gLogMutex);
    if(true){
        ForeSwitchLogFile();
    }
    gLogOn = true;
    MYLOG_FW("StartUtilsLog");
    return true;    
};

void SetForePureMode(bool fore)
{
    gLogPureOn = fore;
}; 

void SetForeShieldFileMode(bool fore)
{
    std::lock_guard<std::recursive_mutex> lck(LevenBF::Utils::UtilsLog::gLogMutex);    
    gLogShieldFileOn = fore;
};

void ForeSwitchLogFile()
{
    std::lock_guard<std::recursive_mutex> lck(LevenBF::Utils::UtilsLog::gLogMutex);
    string timeYmdhms, timeYmd;
    LevenBF::Utils::UtilsTime::GetCurrentTimeStr3(timeYmdhms, timeYmd);
    gStartupTimeStr = timeYmdhms;
};

void SetLogLevelConsole(LogLevel logLevel)
{
    std::lock_guard<std::recursive_mutex> lck(LevenBF::Utils::UtilsLog::gLogMutex);
    gLogLevelConsole = logLevel;
};

void SetLogLevelFile(LogLevel logLevel)
{
    std::lock_guard<std::recursive_mutex> lck(LevenBF::Utils::UtilsLog::gLogMutex);
    gLogLevelFile = logLevel;
};

void LogOut(const string& timeYmdhms, const string& timeYmd, 
                const string& info, const string& head, const string& tail, 
                bool time_on, bool log2file,
                LogLevel logLevel)
{  

    std::lock_guard<std::recursive_mutex> lck(LevenBF::Utils::UtilsLog::gLogMutex);
    if(!gLogOn){
        return;
    }
    if(logLevel >= gLogLevelConsole){
        if(time_on){
            std::cout << timeYmdhms << " " << head << info << tail << std::endl;
        }else{
            std::cout << head << info << tail << std::endl;        
        }
    }

    if(log2file && !gLogShieldFileOn && logLevel >= gLogLevelFile){
	    stringstream ss;   
        // ss << LevenBF::Utils::UtilsLog::gLogfileDir << LevenBF::Utils::UtilsLog::gLgfilePre << timeYmd << ".log";
        ss << LevenBF::Utils::UtilsLog::gLogfileDir 
            << LevenBF::Utils::UtilsLog::gLgfilePre << gStartupTimeStr << "_" << timeYmd << ".log";        
        string msg;
        if(time_on){
            msg.append(timeYmdhms);
            msg.append(" ");
            msg.append(info);
        }else{
            msg.append(info);
        }
        LevenBF::Utils::UtilsLog::Log2File(ss.str(), msg);
        ss.str("");
    }
};

void Log2File(const string& logfilename, const string& msg)
{
    ofstream fout;
    fout.open(logfilename, ios_base::app);
    fout << msg << std::endl;
    fout << flush; 
    fout.close(); 	
};

}}}
