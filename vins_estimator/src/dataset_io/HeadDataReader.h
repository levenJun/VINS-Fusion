#include "DataReaderBase.h"
#include <thread>
#include <mutex>
namespace DATA_READER
{
    class HeadDataReader : public DataReaderBase
{
private:
    /* data */
    std::string m_session_data_path;
    std::string mConfigFile;
    int mCameraNum = 4;
    std::vector<std::vector<std::string>> mAllImageName;
    std::vector<double> mImageTimeStamps;
    std::vector<IMUFrame> mAllImuData;

    std::string proPath = "/cam";
    std::thread *m_thread_read = nullptr;
    std::thread *m_thread_read_imu_pre = nullptr;
    static void ThreadDataRead(HeadDataReader *reader);
    static void ThreadIMUReadPre(HeadDataReader *reader);
    std::mutex m_mt_img;
    std::mutex m_mt_imu;
    bool run = true;
    int mStart=0;
    int mEnd=100000;

private:
    bool readAllImageName(std::string filePath, std::string proPath, int cameraNum, std::vector<std::vector<std::string>> &allImageName, std::vector<double> &imageTimeStamps);
    bool readAllImuDataCSV(std::string fileName, std::vector<IMUFrame> &allImuData);

public:
    HeadDataReader(std::string session_data_path, int cam_num = 4, int imu_num = 1 ,int start=0,int end=100000/* args */);
    ~HeadDataReader();

    Data read();
    Data readImuPre();
    int getImgSize(int cam = 0){ return mAllImageName[cam].size(); };
    int getCamNum() { return mCameraNum; }
    std::string getConfigFile(){return mConfigFile;}
    void close();
    bool isRun() { return run; }
    bool isDataBufferEmpty();
};


}
