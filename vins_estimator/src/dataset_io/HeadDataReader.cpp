#include "HeadDataReader.h"
#include <iostream>
#include <fstream>
#include "tools.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>
#include <math.h>
namespace DATA_READER
{

double readTimeStamps(std::string a)
{
    std::string::size_type nPos1 = std::string::npos;
    nPos1 = a.find_last_of("/");
    std::string::size_type nPos2 = std::string::npos;
    nPos2 = a.find_last_of(".");
    std::string aSub = a.substr(nPos1 + 1, nPos2 - nPos1 - 1);
    return std::stod(aSub);
}

std::vector<std::vector<std::string>> unifyAllFileName(std::vector<std::vector<std::string>> allImageName)
{
    std::vector<std::vector<std::string>> res;
    int camNum = allImageName.size();
    if(allImageName.empty() || allImageName[0].empty())
    {
        return res;
    }
    std::vector<int> startIndex;
    std::vector<int> endIndex;
    double minTimeStamps = readTimeStamps(allImageName[0][0]);
    
    for(int i=0;i<camNum;i++)
    {
        minTimeStamps = std::max(minTimeStamps, readTimeStamps(allImageName[i][0]));
    }

    for(int i=0;i<camNum;i++)
    {
        for(int j=0;j<allImageName[i].size();j++)
        {
            if(std::abs(readTimeStamps(allImageName[i][j]) - minTimeStamps)<0.1)
            {
                startIndex.push_back(j);
                break;
            }
        }
    }
    if(startIndex.size()!=camNum)
    {
        return res;
    }
    
    int minEndIndex = allImageName[0].size() - startIndex[0];
    for(int i=0;i<camNum;i++)
    {
        minEndIndex = std::min(minEndIndex, (int)(allImageName[i].size() - startIndex[i]));
    }

    for(int i=0;i<camNum;i++)
    {
        endIndex.push_back(minEndIndex + startIndex[i]);
    }

    res.resize(camNum);
    for(int i=0;i<camNum;i++)
    {
        for(int j=startIndex[i];j<endIndex[i];j++)
        {
            res[i].push_back(allImageName[i][j]);
        }
    }
    for(int i=0;i<camNum;i++)
    {
        std::cout << "finaly imageFileNumber in camera" << i << "is:" << endIndex[i] - startIndex[i] << std::endl;
    }
    return res;
}

bool HeadDataReader::readAllImageName(std::string filePath, std::string proPath, int cameraNum, std::vector<std::vector<std::string>> &allImageName, std::vector<double> &imageTimeStamps)
{
    allImageName.clear();
    imageTimeStamps.clear();
	for (int i = 0; i < cameraNum; i++)
	{
		std::string oneCameraPath = filePath + proPath + std::to_string(i) + "/";

		std::vector<std::string> oneCameraFileNames;
		getImageFilesAllPath(oneCameraPath, oneCameraFileNames);

		// need sort
		sortFileName(oneCameraFileNames);
        std::cout << "imageFileNumber in camera" << i << "is:" << oneCameraFileNames.size() << std::endl;
        allImageName.push_back(oneCameraFileNames);
    }

    allImageName = unifyAllFileName(allImageName);
    if(allImageName.size()!=cameraNum)
        return false;
    auto oneCameraFileNames = allImageName[0];
    imageTimeStamps.reserve(5000);
    for (int j = 0; j < oneCameraFileNames.size(); j++)
    {
        std::string a = oneCameraFileNames[j];
        double t = readTimeStamps(a) / 1e9;
        imageTimeStamps.push_back(t);
    }
	return !allImageName.empty();
}

bool HeadDataReader::readAllImuDataCSV(std::string fileName, std::vector<IMUFrame> &allImuData)
{
     allImuData.clear();
	std::ifstream fImu;
	fImu.open(fileName.c_str());
	allImuData.reserve(5000);
	if (!fImu.is_open())
	{
		std::cout << "get allImuData file error, filePath is:" << fileName << std::endl;
		return false;
	}

	while (!fImu.eof())
	{
		std::string s;
		getline(fImu, s);
		if (s.empty())
		{
            continue;
		}
		if (s[0] == '#')
			continue;

		if (!s.empty())
		{
			IMUFrame oneImuData;
			std::vector<std::string> res;

			std::string result;

			std::stringstream input(s);

            std::string oneWord;
			for(int i=0;i<7;i++)
            {
                getline(input, oneWord, ',');
                res.push_back(oneWord);
            }
			oneImuData.stamp = stod(res[0]) / 1e9;
			oneImuData.gro(0) = stof(res[1]);
			oneImuData.gro(1) = stof(res[2]);
			oneImuData.gro(2) = stof(res[3]);
			oneImuData.acc(0) = stof(res[4]);
			oneImuData.acc(1) = stof(res[5]);
			oneImuData.acc(2) = stof(res[6]);

			allImuData.push_back(oneImuData);
		}
	};
	std::cout << "get allImuData, total size is:" << allImuData.size() << std::endl;
	return true;
}

HeadDataReader::HeadDataReader(std::string session_data_path, int cam_num , int imu_num,int start,int end )
{
    readAllImageName(session_data_path, "/cam", cam_num, mAllImageName, mImageTimeStamps);
    readAllImuDataCSV(session_data_path + "/imu0.csv", mAllImuData);
	mConfigFile=session_data_path+"/xr2.yaml";
	mStart=start;
	mEnd=end;
    m_thread_read = new std::thread(ThreadDataRead, this);
    m_thread_read_imu_pre = new std::thread(ThreadIMUReadPre, this);
}
HeadDataReader::~HeadDataReader()
{
    run=false;
    if (m_thread_read)
    {
        m_thread_read->join();
        delete m_thread_read;
    }
    if (m_thread_read_imu_pre)
    {
        m_thread_read_imu_pre->join();
        delete m_thread_read_imu_pre;
    }
}

void HeadDataReader::ThreadIMUReadPre(HeadDataReader *reader)
{
    int i=reader->mStart;
	 std::cout<<"HeadDataReader reader->mStart="<<reader->mStart<<std::endl;
    int imuIndex_left=0;

    for (size_t j = imuIndex_left + 1; j < reader->mAllImuData.size(); j++)
    {
        if (reader->mAllImuData[j].stamp + 0.1 < reader->mImageTimeStamps[i])
        {
            continue;
        }
        else
        {
             imuIndex_left = j;
             break;
        }
    }
    while (reader->run)
    {
        while (reader->run && (reader->m_data_buffer_imu_pre.size() > 100 || (reader->mAllImuData[imuIndex_left].stamp > reader->m_data_buffer.front().videoFrame.stamp+0.07)))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        HeadIMU imuData;
        for(size_t j = imuIndex_left + 1; j<reader->mAllImuData.size(); j++)
        {
            if(imuData.imus.size()<10)
            {
                IMUFrame oneIumFrame;
                oneIumFrame.stamp = reader->mAllImuData[j].stamp;
                oneIumFrame.gro = reader->mAllImuData[j].gro;
                oneIumFrame.acc=reader->mAllImuData[j].acc;
                imuData.imus.push_back(oneIumFrame);
            }
            else
            {
                imuIndex_left = j;
                break;
            }
        }

      
        Data data;
        data.headImu=imuData;
        std::lock_guard<std::mutex> lk(reader->m_mt_imu);

        reader->m_data_buffer_imu_pre.push(data);

    }
    reader->run = false;

}
void HeadDataReader::ThreadDataRead(HeadDataReader *reader)
{
    int i=reader->mStart;
	 std::cout<<"HeadDataReader reader->mStart="<<reader->mStart<<std::endl;
    int imuIndex=0;
    int imuIndex_predict=0;
    size_t num=std::min(std::min(reader->mAllImageName[0].size(),reader->mAllImageName[1].size()),std::min(reader->mAllImageName[2].size(),reader->mAllImageName[3].size()));
    while (reader->run&&i<num-10&&i<reader->mEnd)
    {
        while (reader->m_data_buffer.size() > 5 && reader->run)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
		i++;
        
        VideoFrame videoFrame;
        videoFrame.im.resize(reader->mCameraNum);
        videoFrame.stamp = reader->mImageTimeStamps[i];
        
        for(size_t j=0;j<reader->mCameraNum;j++)
        {
            // std::cout<<"read "<<reader->mAllImageName[j][i]<<std::endl;
            cv::Mat oneCameraImage = cv::imread(reader->mAllImageName[j][i], CV_8UC1);
            videoFrame.im[j] = oneCameraImage;
            if(videoFrame.im[j].empty())
           {
            reader->run = false;
           return;
           }
        }
        

        HeadIMU imuData;
        HeadIMU headImuPredictData;
        for(size_t j = imuIndex + 1; j<reader->mAllImuData.size(); j++)
        {
			if(reader->mAllImuData[j].stamp +0.1< reader->mImageTimeStamps[i])
			{
				continue;
			}
            if(reader->mAllImuData[j].stamp < reader->mImageTimeStamps[i])
            {
                IMUFrame oneIumFrame;
                oneIumFrame.stamp = reader->mAllImuData[j].stamp;
                oneIumFrame.gro = reader->mAllImuData[j].gro;
                oneIumFrame.acc=reader->mAllImuData[j].acc;
              
                imuData.imus.push_back(oneIumFrame);
            }
            else
            {
                imuIndex = j;
                break;
            }
        }

        for(size_t j = imuIndex_predict + 1; j<reader->mAllImuData.size(); j++)
        {
			if(reader->mAllImuData[j].stamp +0.1< reader->mImageTimeStamps[i])
			{
				continue;
			}
            if(reader->mAllImuData[j].stamp < reader->mImageTimeStamps[i]+0.07)
            {
                IMUFrame oneIumFrame;
                oneIumFrame.stamp = reader->mAllImuData[j].stamp;
                oneIumFrame.gro = reader->mAllImuData[j].gro;
                oneIumFrame.acc=reader->mAllImuData[j].acc;
              
                headImuPredictData.imus.push_back(oneIumFrame);
            }
            else
            {
                imuIndex_predict = j;
                break;
            }
        }
    


      
        Data data;
        data.videoFrame=videoFrame;
        data.headImu=imuData;
        data.headImuPredict=headImuPredictData;
        std::lock_guard<std::mutex> lk(reader->m_mt_img);

        reader->m_data_buffer.push(data);

    }
    reader->run = false;
    


}

Data HeadDataReader::readImuPre()
{
    static int num=0;
    std::lock_guard<std::mutex> lk(m_mt_imu);
    if (m_data_buffer_imu_pre.empty())
    {
        return Data();
    }
    num++;
    auto data = m_data_buffer_imu_pre.front();
    
    m_data_buffer_imu_pre.pop();
    return data;

}


Data HeadDataReader::read()
{
    static int num=0;
    std::lock_guard<std::mutex> lk(m_mt_img);
    if (m_data_buffer.empty())
    {
        return Data();
    }
    num++;
    auto data = m_data_buffer.front();
    
    m_data_buffer.pop();
    /*
    static std::vector<cv::Mat> imgs;
    static std::vector<IMUFrame> imu;
    if(num==300)
    {
        static cv::Mat im0=data.videoFrame.im[0].clone();
        static cv::Mat im1=data.videoFrame.im[1].clone();
        static cv::Mat im2=data.videoFrame.im[2].clone();
        static cv::Mat im3=data.videoFrame.im[3].clone();
       
        imgs.push_back(im0);
        imgs.push_back(im1);
        imgs.push_back(im2);
        imgs.push_back(im3);
        imu=data.headImu.imus;

    }
    if(num>=300)
    {
        data.videoFrame.im=imgs;
        for(size_t i=0;i<data.headImu.imus.size();i++)
        {
            IMUFrame &p=data.headImu.imus[i];
            p.acc=imu[i].acc;
            p.gro=imu[i].gro;
        }

    }
    */
    return data;
}

void HeadDataReader::close()
{
    run = false;
}

bool HeadDataReader::isDataBufferEmpty(){
    std::lock_guard<std::mutex> lk(m_mt_img);
    return m_data_buffer.empty();
}
}