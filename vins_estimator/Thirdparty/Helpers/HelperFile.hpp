#pragma once
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h> // Windows下使用
#define mkdir(path, mode) _mkdir(path)
#else
#include <unistd.h> // Unix/Linux下使用
#endif
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <fstream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <sstream>
using namespace std;
namespace ORB_SLAM3{

class HelperFile
{

public:

    static bool DirectoryExists(const std::string& path) {
        struct stat info;
        if (stat(path.c_str(), &info) != 0) {
            return false; // 目录不存在
        } else if (info.st_mode & S_IFDIR) {
            return true; // 是目录
        }
        return false; // 不是目录
    }

    static bool CreateDirectory(const std::string& path) {
        if (DirectoryExists(path)) {
            std::cout << "Directory already exists: " << path << std::endl;
            return true;
        }

        int result = mkdir(path.c_str(), 0755); // 在Unix/Linux上，0755是权限设置
        if (result == 0) {
            std::cout << "Directory created: " << path << std::endl;
            return true;
        } else {
            std::cerr << "Failed to create directory: " << path << std::endl;
            return false;
        }
    }

	/*** 
	 * @description: 创建文件夹，如果待创建的文件夹父级目录不存在，会首先创建父级目录
	 * 					（特别注意，可能会因为权限问题导致创建失败，如是，请手动修改对应文件夹权限）
	 * @param {string} fullpath_str	待创建文件夹完整路径
	 * @param {char} splitKey		文件夹路径的分割字符
	 * @param {int} permit			文件夹创建后的权限，最好就设置为这个权限
	 * @return {*}	0，成功；其他，失败
	 */	
	static int CreateDir(std::string fullpath_str, char splitKey = '/', int permit = 0777){
        if (DirectoryExists(fullpath_str)) {
            std::cout << "Directory already exists: " << fullpath_str << std::endl;
            return 0;
        }
		
		// shared_ptr<vector<std::string>> fullpath = UtilsString::Split(fullpath_str, splitKey);
		std::vector<std::string> fullpath;
		std::string separator(1, splitKey); 
		Split2(fullpath_str, fullpath, separator);
		if (fullpath.size() <= 0)
		{
			return -1;
		}
	
		int ret;
		
		stringstream pathtmp;
		
		int size = fullpath.size();
		for (int i = 0; i < size; i++)
		{
			if("" == fullpath[i]){
				if(0 == i){
					pathtmp << "/";
				}
				continue;							
			}
			pathtmp << fullpath[i] << "/";
            std::string tgtDir = pathtmp.str();			

            if(!DirectoryExists(tgtDir))
			{
                std::cout << "不存在:" << tgtDir << std::endl;
				if ((ret = mkdir(tgtDir.c_str(), 0777)) != 0)
				{
                    std::cout << "创建失败:" << tgtDir << std::endl;
					return ret;
				}
				else
				{
                    std::cout << "创建成功:" << tgtDir << std::endl;
				}								
			}
			
			if (access(tgtDir.c_str(), 06) != 0)
			{
                std::cout << "没有读写权限:" << tgtDir << std::endl;
				/*return -1;*/
			}
		}
		return 0;
    }


	static void Split2(const string& srcStr, vector<string>& vec, const string& separator)
	{

		string::size_type posSubstringStart; // 子串开始位置
		string::size_type posSeparator;        //  分隔符位置

		posSeparator = srcStr.find(separator);
		posSubstringStart = 0;
		while (string::npos != posSeparator)
		{
			vec.push_back(srcStr.substr(posSubstringStart, posSeparator - posSubstringStart));

			posSubstringStart = posSeparator + separator.size();
			posSeparator = srcStr.find(separator, posSubstringStart);
		}

		// 截取最后一段数据
		if (posSubstringStart != srcStr.length()){
			vec.push_back(srcStr.substr(posSubstringStart));
		}else{
			vec.push_back("");
		}
	}

};
}