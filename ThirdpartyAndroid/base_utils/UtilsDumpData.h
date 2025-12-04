#pragma once
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

namespace LevenBF{
namespace Utils{

namespace UtilsDumpData{

struct PoseWithTime
{
    double time;
    double tx;
    double ty;
    double tz;
    double qx;
    double qy;
    double qz;
    double qw;

    bool valid = true;
};


bool InitUtilsDumpData(string dumpfileDir, string dumpfilePre);
bool StopUtilsDumpData();                             //暂停
bool StartUtilsDumpData();                            //恢复
void ForeSwitchDumpFile(); 

void DumpPoseImgUpdate(const PoseWithTime& eskfPose,  const PoseWithTime& msckfPose, string handName = "left");
void DumpPoseImuPredict(const PoseWithTime& eskfPose,  const PoseWithTime& msckfPose, string handName = "left");

void Dump2File(const string& dumpfilename, const string& msg);

}
}

}