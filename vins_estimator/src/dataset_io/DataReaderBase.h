#ifndef DATA_READER_BASE_H
#define DATA_READER_BASE_H
#include <opencv2/highgui.hpp>
#include <Eigen/Eigen>
#include <queue>

namespace DATA_READER
{
    struct IMUFrame
    {
        double stamp;
        Eigen::Vector3d acc;
        Eigen::Vector3d gro;
    };
    struct HandleIMU
    {
        std::vector<IMUFrame> left;
        std::vector<IMUFrame> right;
    };

    struct HeadIMU
    {
        std::vector<IMUFrame> imus;
    };

    struct VideoFrame
    {
        double stamp{0.0};
        std::vector<cv::Mat> im; // 0-left 1-right 2 top left 3 top right
    };
    struct Data
    {
        /* data */
        VideoFrame videoFrame;
        HandleIMU handImu;
        HeadIMU headImu;
        HeadIMU headImuPredict;
    };

    class DataReaderBase
    {
    public:
        /* data */
        std::queue<Data> m_data_buffer;
        std::queue<Data> m_data_buffer_imu_pre;

    public:
        virtual Data read() = 0;
        virtual Data readImuPre(){return Data();};
        virtual void close() = 0;
        void print(Data data) { printf("time=%f,images=%zu,acc=%zu,gro=%zu", data.videoFrame.stamp, data.videoFrame.im.size(), data.headImu.imus.size(), data.headImu.imus.size()); };
    };
}

#endif