#pragma once

#include <chrono>
#include <time.h>
// #include <sys/time.h> 
#include <string>

using namespace std;

namespace LevenBF{
namespace Utils{

class UtilsTime
{

public:
    UtilsTime(/* args */){};
    ~UtilsTime(){};

public:

    static double GetCurrentTimeSecs(){
        return GetCurrentTimeMills() / 1000.0;
    }

	// static inline long GetCurrentTimeMills()
	// {
	// 	struct timeval t_current;
	// 	//get start time 
	// 	gettimeofday(&t_current, NULL); 
	// 	long result = ((long)t_current.tv_sec) * 1000 + (long)t_current.tv_usec / 1000; 
	// 	return result;
	// }

    /*** 
     * @description: 获取当前时间，对应的mills
     * @return {*}
     */    
	static inline long GetCurrentTimeMills()
	{
        auto timeNow = std::chrono::system_clock::now();
        auto tMills = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow.time_since_epoch());
		return tMills.count();
	}  

    /*** 
     * @description: 获取当前时间[年月日-时分秒]，以字符串形式输出
     * @return {*}
     */
    static std::string GetCurrentTimeStr1()
    {
        auto timeNow = std::chrono::system_clock::now();
        auto tMills = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow.time_since_epoch());
        auto tt = std::chrono::system_clock::to_time_t(timeNow);
        struct tm* ptm = localtime(&tt);
        char date[65] = {0};
        sprintf(date,
                "%d%02d%02d-%02d%02d%02d.%03d",
                (int)ptm->tm_year + 1900,
                (int)ptm->tm_mon + 1,
                (int)ptm->tm_mday,
                (int)ptm->tm_hour,
                (int)ptm->tm_min,
                // (int)ptm->tm_sec + (tMills.count() % 1000) * 1e-3   
                (int)ptm->tm_sec,
                (int)(tMills.count() % 1000)                                
                );

        return std::string(date);
    }    

    /*** 
     * @description: 获取当前时间[年月日]，以字符串形式输出
     * @return {*}
     */    
    static std::string GetCurrentTimeStr2()
    {
        auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm* ptm = localtime(&tt);
        char date[60] = {0};
        sprintf(date,
                "%d%02d%02d",
                (int)ptm->tm_year + 1900,
                (int)ptm->tm_mon + 1,
                (int)ptm->tm_mday);

        return std::string(date);
    }

    /*** 
     * @description: 获取当前时间，以字符串形式返回
     * @param {string&} timeYmdhms  [年月日-时分秒] 精确到毫秒
     * @param {string&} timeYmd     [年月日]
     * @return {*}
     */
    static void GetCurrentTimeStr3(string& timeYmdhms, string& timeYmd)
    {
        auto timeNow = std::chrono::system_clock::now();
        auto tMills = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow.time_since_epoch());
        auto tt = std::chrono::system_clock::to_time_t(timeNow);
        struct tm* ptm = localtime(&tt);        

        char date[65] = {0};
        sprintf(date,
                "%d%02d%02d-%02d%02d%02d.%03d",
                (int)ptm->tm_year + 1900,
                (int)ptm->tm_mon + 1,
                (int)ptm->tm_mday,
                (int)ptm->tm_hour,
                (int)ptm->tm_min,
                // (int)ptm->tm_sec + (tMills.count() % 1000) * 1e-3   
                (int)ptm->tm_sec,
                (int)(tMills.count() % 1000)                                
                ); 
        timeYmdhms = std::string(date);

        memset(date, 0, sizeof(date));
        sprintf(date,
                "%d%02d%02d",
                (int)ptm->tm_year + 1900,
                (int)ptm->tm_mon + 1,
                (int)ptm->tm_mday);
        timeYmd = std::string(date);
    }  

};

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


}    
}
