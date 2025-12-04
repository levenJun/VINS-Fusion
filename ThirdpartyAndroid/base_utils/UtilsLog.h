/*** 
 * @Author: your name
 * @LastEditors: leven
 * @Description: 日志模块。分级分颜色，可只输出到屏幕，可同时输出到屏幕+输出到日志文件
 * @可以输入预定的版权声明、个性签名、空行等
 */
#pragma once
#include <string.h>
#include <iostream>
#include <sstream>

using namespace std;

namespace LevenBF{
namespace Utils{
enum LogLevel{    
    VERBOSE = 0,
    DEBUG,
    INFO,
    WARN,
    ERROR
};
}
}

extern void m_log_v(string msg, string filename = "", string func = "", int line = 0, bool log2file = false);
extern void m_log_d(string msg, string filename = "", string func = "", int line = 0, bool log2file = false);
extern void m_log_i(string msg, string filename = "", string func = "", int line = 0, bool log2file = false);
extern void m_log_w(string msg, string filename = "", string func = "", int line = 0, bool log2file = false);
extern void m_log_e(string msg, string filename = "", string func = "", int line = 0, bool log2file = false);
extern void m_log_all(const string& info, const string& head, const string& tail, bool time_on = true, bool log2file = false, LevenBF::Utils::LogLevel logLevel = LevenBF::Utils::LogLevel::VERBOSE);

#define MYLOG_V(msg) m_log_v((msg), "", __FUNCTION__, __LINE__);
#define MYLOG_D(msg) m_log_d((msg), "", __FUNCTION__, __LINE__);
#define MYLOG_I(msg) m_log_i((msg), "", __FUNCTION__, __LINE__);
#define MYLOG_W(msg) m_log_w((msg), "", __FUNCTION__, __LINE__);
#define MYLOG_E(msg) m_log_e((msg), "", __FUNCTION__, __LINE__);

#define MYLOG_FV(msg) m_log_v((msg), "", __FUNCTION__, __LINE__, true);
#define MYLOG_FD(msg) m_log_d((msg), "", __FUNCTION__, __LINE__, true);
#define MYLOG_FI(msg) m_log_i((msg), "", __FUNCTION__, __LINE__, true);
#define MYLOG_FW(msg) m_log_w((msg), "", __FUNCTION__, __LINE__, true);
#define MYLOG_FE(msg) m_log_e((msg), "", __FUNCTION__, __LINE__, true);

namespace LevenBF{
namespace Utils{
namespace UtilsLog{

bool InitUtilsLog(string logfileDir, string logfilePre);
bool StopUtilsLog();                             //暂停log功能
bool StartUtilsLog();                            //恢复log功能
void SetForePureMode(bool fore);                 //是否强制纯净输出(不输出函数名和行数)
void SetForeShieldFileMode(bool fore);           //是否强制屏蔽文件输出
void ForeSwitchLogFile();                        //强制切换新输出文件
void SetLogLevelConsole(LogLevel logLevel);      //设置输出到控制台最低Log等级
void SetLogLevelFile(LogLevel logLevel);         //设置输出到文件的最低Log等级
void LogOut(const string& timeYmdhms, const string& timeYmd, const string& info, const string& head, const string& tail, bool time_on, bool log2file, LogLevel logLevel);
void Log2File(const string& logfilename, const string& msg);

}
}   
}